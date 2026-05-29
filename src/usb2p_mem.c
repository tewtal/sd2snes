/* sd2snes - SD card based universal cartridge for the SNES
   Copyright (C) 2009-2010 Maximilian Rehkopf <otakon@gmx.net>

   USB2P v3 memory transfer (README.usb2p.md §9).  Segmented READ and
   WRITE over the SNES/MSU/CMD/CONFIG spaces, replacing the legacy GET/VGET and
   PUT/VPUT.  segment_count==1 is the ordinary contiguous transfer; >1 is
   scatter/gather.  All FPGA SPI access is deferred to the scheduler steps, which
   run in main-loop context (see usb2p_sched.c).

   WRITE is two-phase: the REQ pins the op and validates the descriptor, then the
   host's DAT-OUT chunks are staged here and committed by usb2p_sched_step_write.
*/

#include <stdint.h>
#include <string.h>

#include "fpga_spi.h"
#include "memory.h"
#include "msu1.h"
#include "snes.h"
#include "usb2p.h"
#include "usb2p_internal.h"
#include "usbuser.h"

/* read_payload stages one outbound DAT chunk (READ/FREAD) and is reused as the
   READDIR batch buffer (usb2p_fs.c); pending_op_chunk stages one inbound
   WRITE/FWRITE chunk awaiting commit.  Both alias safely because the
   single-op-slot model never overlaps their users.  Their storage lives in the
   shared union in usbinterface.c (aliased onto the legacy USBA data buffers),
   exported there as pointers — see usb2p_internal.h. */
uint32_t pending_op_chunk_len;
uint8_t  pending_op_chunk_last;

/* Segmented READ/WRITE scatter/gather cursor. */
uint32_t pending_seg_offset[USB2P_MAX_SEGMENTS];
uint32_t pending_seg_length[USB2P_MAX_SEGMENTS];
uint8_t  pending_seg_count;
uint8_t  pending_seg_idx;
uint32_t pending_op_total_remaining;
uint32_t pending_op_next_seq;
uint8_t  pending_op_rsp_sent;
uint8_t  pending_op_write_status;
uint32_t pending_op_written;
enum usb2p_write_state pending_op_write_state;

uint8_t usb2p_memory_space_valid(uint8_t space) {
  return space == USB2P_SPACE_SNES
      || space == USB2P_SPACE_MSU
      || space == USB2P_SPACE_CMD
      || space == USB2P_SPACE_CONFIG;
}

uint8_t usb2p_memory_range_ok(uint8_t space, uint64_t offset, uint64_t length) {
  uint64_t size;

  switch (space) {
    case USB2P_SPACE_SNES:
      size = USB2P_SNES_SPACE_SIZE;
      break;
    case USB2P_SPACE_MSU:
    case USB2P_SPACE_CMD:
      size = USB2P_16BIT_SPACE_SIZE;
      break;
    case USB2P_SPACE_CONFIG:
      return length == 1 && offset < USB2P_16BIT_SPACE_SIZE;
    default:
      return 0;
  }

  return offset <= size && length <= size - offset;
}

uint32_t usb2p_memory_read(uint8_t space, uint32_t offset,
                           uint8_t *dst, uint32_t len) {
  switch (space) {
    case USB2P_SPACE_SNES:
      return sram_readblock(dst, offset, (uint16_t)len);
    case USB2P_SPACE_MSU:
      return msu_readblock(dst, (uint16_t)offset, (uint16_t)len);
    case USB2P_SPACE_CMD:
      return snescmd_readblock(dst, (uint16_t)offset, (uint16_t)len);
    case USB2P_SPACE_CONFIG:
      dst[0] = fpga_read_config((uint8_t)(offset >> 8), (uint8_t)offset);
      return 1;
  }
  return 0;
}

static uint32_t usb2p_memory_write(uint8_t space, uint32_t offset,
                                   const uint8_t *src, uint32_t len,
                                   uint8_t *status) {
  *status = USB2P_STATUS_OK;
  switch (space) {
    case USB2P_SPACE_SNES:
      return sram_writeblock((void *)src, offset, (uint16_t)len);
    case USB2P_SPACE_CMD:
      return snescmd_writeblock((void *)src, (uint16_t)offset, (uint16_t)len);
    case USB2P_SPACE_CONFIG:
      fpga_write_config((uint8_t)(offset >> 8), (uint8_t)offset, src[0], 0x00);
      return 1;
    case USB2P_SPACE_MSU:
      *status = USB2P_STATUS_EUNSUPPORTED;
      return 0;
  }

  *status = USB2P_STATUS_EINVAL;
  return 0;
}

/* Parse a segmented READ/WRITE descriptor into pending_seg_* and compute the
   total byte count.  Validates structure (segment_count 1..MAX, reserved=0,
   payload long enough), the memory space, and range-checks EVERY segment.
   On success fills pending_seg_offset/length[0..count-1], pending_seg_count,
   sets *total_out, and returns OK.  On failure returns the status to send.
   segment_count==1 is the common contiguous case and takes the same path. */
static uint8_t usb2p_decode_rw_desc(const uint8_t *payload, uint32_t payload_len,
                                    uint8_t space, uint32_t *total_out) {
  uint16_t segment_count;
  uint16_t i;
  uint64_t total = 0;

  if (payload_len < USB2P_RW_DESC_HDR_LEN) {
    return USB2P_STATUS_EINVAL;
  }
  segment_count = usb2p_rd16(payload + 0);
  if (usb2p_rd16(payload + 2) != 0 || segment_count == 0) {
    return USB2P_STATUS_EINVAL;
  }
  if (segment_count > USB2P_MAX_SEGMENTS) {
    return USB2P_STATUS_EMSGSIZE;  /* over the HELLO-advertised max_segments */
  }
  if (payload_len < USB2P_RW_DESC_HDR_LEN
                    + ((uint32_t)segment_count * USB2P_RW_SEG_LEN)) {
    return USB2P_STATUS_EINVAL;
  }
  if (!usb2p_memory_space_valid(space)) {
    return USB2P_STATUS_EINVAL;
  }

  for (i = 0; i < segment_count; i++) {
    const uint8_t *seg = payload + USB2P_RW_DESC_HDR_LEN
                         + (uint32_t)i * USB2P_RW_SEG_LEN;
    uint64_t offset = usb2p_rd64(seg + 0);
    uint64_t length = usb2p_rd64(seg + 8);

    if (length > UINT32_MAX) {
      return USB2P_STATUS_EMSGSIZE;
    }
    if (!usb2p_memory_range_ok(space, offset, length)) {
      return USB2P_STATUS_ERANGE;
    }
    total += length;
    if (total > UINT32_MAX) {
      return USB2P_STATUS_EMSGSIZE;  /* aggregate too large for 32-bit backend */
    }
    pending_seg_offset[i] = (uint32_t)offset;
    pending_seg_length[i] = (uint32_t)length;
  }

  pending_seg_count = (uint8_t)segment_count;
  *total_out = (uint32_t)total;
  return USB2P_STATUS_OK;
}

void usb2p_handle_read(const struct usb2p_header *h, const uint8_t *payload) {
  uint32_t total;
  uint8_t status;

  /* EBUSY before parse so a malformed REQ during an active op still NAKs
     EBUSY (single data-op slot), consistent with the contiguous path. */
  if (pending_op_kind != USB2P_OP_NONE) {
    usb2p_queue_nak(USB2P_STATUS_EBUSY, h->txn_id, h->seq);
    return;
  }

  status = usb2p_decode_rw_desc(payload, (uint32_t)h->length, h->space, &total);
  if (status != USB2P_STATUS_OK) {
    /* ERANGE on an otherwise valid request is a backend rejection -> RSP;
       structural faults -> NAK (§6.2 taxonomy). */
    if (status == USB2P_STATUS_ERANGE) {
      usb2p_queue_rsp_empty(h->opcode, h->space, status, h->txn_id, h->seq);
    } else {
      usb2p_queue_nak(status, h->txn_id, h->seq);
    }
    return;
  }

  /* Defer all FPGA SPI work to the scheduler step (main loop).
     Start at segment 0; the READ producer advances through pending_seg_*. */
  pending_op_kind = USB2P_OP_READ;
  pending_op_space = h->space;
  pending_op_opcode = h->opcode;
  pending_op_txn_id = h->txn_id;
  pending_op_seq = h->seq;
  pending_op_next_seq = h->seq + 1;
  pending_seg_idx = 0;
  pending_op_offset = pending_seg_offset[0];
  pending_op_remaining = pending_seg_length[0];
  pending_op_total = total;
  pending_op_rsp_sent = 0;
}

void usb2p_handle_write(const struct usb2p_header *h, const uint8_t *payload) {
  uint32_t total;
  uint8_t status;

  if (pending_op_kind != USB2P_OP_NONE) {
    usb2p_queue_nak(USB2P_STATUS_EBUSY, h->txn_id, h->seq);
    return;
  }

  status = usb2p_decode_rw_desc(payload, (uint32_t)h->length, h->space, &total);
  if (status != USB2P_STATUS_OK) {
    if (status == USB2P_STATUS_ERANGE) {
      usb2p_queue_rsp_u64(h->opcode, h->space, status, h->txn_id, h->seq, 0);
    } else {
      usb2p_queue_nak(status, h->txn_id, h->seq);
    }
    return;
  }
  /* REQ carries the descriptor ONLY — segment data follows as DAT frames.
     Reject any trailing bytes past the exact descriptor. */
  if (h->length != (uint64_t)(USB2P_RW_DESC_HDR_LEN
                              + pending_seg_count * USB2P_RW_SEG_LEN)) {
    usb2p_queue_nak(USB2P_STATUS_EINVAL, h->txn_id, h->seq);
    return;
  }

  pending_op_kind = USB2P_OP_WRITE;
  pending_op_space = h->space;
  pending_op_opcode = h->opcode;
  pending_op_txn_id = h->txn_id;
  pending_op_seq = h->seq;
  pending_seg_idx = 0;
  pending_op_offset = pending_seg_offset[0];
  pending_op_remaining = pending_seg_length[0];  /* remaining in current segment */
  pending_op_total = total;                       /* total across all segments */
  pending_op_total_remaining = total;             /* bytes still expected (DAT acct) */
  pending_op_write_status = USB2P_STATUS_OK;
  pending_op_written = 0;
  pending_op_write_state = USB2P_WRITE_AWAIT_DAT;
  pending_op_chunk_len = 0;
  pending_op_chunk_last = 0;

  /* Zero-length WRITE (all segments length 0): no DAT phase, RSP immediately. */
  if (total == 0) {
    pending_op_write_state = USB2P_WRITE_DONE;
  }
}

/* DAT-OUT during an active WRITE: stage one chunk for the main-loop commit.
   Validates the chunk against the remaining count, copies it, and lets
   usb2p_sched_step_write commit it on the next poll. */
void usb2p_handle_write_dat(const struct usb2p_header *h, const uint8_t *payload) {
  uint8_t is_last = (h->flags & USB2P_FLAG_LAST) ? 1 : 0;
  uint32_t chunk_len = (uint32_t)h->length;

  if (pending_op_kind != USB2P_OP_WRITE
      || pending_op_write_state != USB2P_WRITE_AWAIT_DAT
      || h->txn_id != pending_op_txn_id) {
    usb2p_queue_nak(USB2P_STATUS_EBADSTATE, h->txn_id, h->seq);
    return;
  }
  /* Chunk is validated against the TOTAL bytes still expected (it may span
     segment boundaries); the commit step splits it across segments. */
  if (chunk_len > pending_op_total_remaining) {
    usb2p_queue_nak(USB2P_STATUS_EMSGSIZE, h->txn_id, h->seq);
    return;
  }
  if (chunk_len > USB2P_MAX_RX_PAYLOAD) {
    usb2p_queue_nak(USB2P_STATUS_EMSGSIZE, h->txn_id, h->seq);
    return;
  }
  if (is_last && chunk_len != pending_op_total_remaining) {
    /* LAST must close the transfer exactly. */
    usb2p_queue_nak(USB2P_STATUS_EINVAL, h->txn_id, h->seq);
    return;
  }
  if (!is_last && chunk_len == pending_op_total_remaining) {
    /* Last bytes arrived without LAST flag — accept and treat as LAST so the
       transfer terminates; the spec requires LAST on the final DAT, but being
       lenient here avoids a stuck op if a host forgets to set it. */
    is_last = 1;
  }

  if (chunk_len) {
    memcpy(pending_op_chunk, payload, chunk_len);
  }
  pending_op_chunk_len = chunk_len;
  pending_op_chunk_last = is_last;
  pending_op_write_state = USB2P_WRITE_COMMIT;
}

/* One step of an active WRITE: commit a staged chunk and/or emit the final RSP.
   Returns PROGRESS if it did work, BLOCKED if tx_buf was full for the RSP,
   IDLE if the WRITE is waiting on the host for the next DAT (AWAIT_DAT). */
enum usb2p_sched_result usb2p_sched_step_write(void) {
  uint32_t written;
  uint8_t status;

  /* AWAIT_DAT: nothing to do until the parser stages the next chunk. */
  if (pending_op_write_state == USB2P_WRITE_AWAIT_DAT) {
    return USB2P_STEP_IDLE;
  }
  /* COMMIT: a chunk is staged in pending_op_chunk; push it to SPI.  The chunk
     may span multiple segments, so commit it piecewise: write min(chunk-left,
     current-segment-left) at the current segment's offset, then advance to the
     next segment when the current one is exhausted. */
  if (pending_op_write_state == USB2P_WRITE_COMMIT) {
    uint32_t chunk_pos = 0;
    while (chunk_pos < pending_op_chunk_len) {
      uint32_t piece = pending_op_chunk_len - chunk_pos;
      if (piece > pending_op_remaining) {
        piece = pending_op_remaining;   /* clamp to current segment */
      }
      written = usb2p_memory_write(pending_op_space, pending_op_offset,
                                   pending_op_chunk + chunk_pos,
                                   piece, &status);
      if (status != USB2P_STATUS_OK
          && pending_op_write_status == USB2P_STATUS_OK) {
        pending_op_write_status = status;
      }
      pending_op_written += written;
      if (written != piece
          && pending_op_write_status == USB2P_STATUS_OK) {
        pending_op_write_status = USB2P_STATUS_EINVAL;
      }
      pending_op_offset += piece;
      pending_op_remaining -= piece;
      pending_op_total_remaining -= piece;
      chunk_pos += piece;
      /* Current segment done — advance to the next one (if any). */
      if (pending_op_remaining == 0
          && (pending_seg_idx + 1) < pending_seg_count) {
        pending_seg_idx++;
        pending_op_offset = pending_seg_offset[pending_seg_idx];
        pending_op_remaining = pending_seg_length[pending_seg_idx];
      }
    }
    if (pending_op_chunk_last || pending_op_total_remaining == 0) {
      pending_op_write_state = USB2P_WRITE_DONE;
    } else {
      pending_op_write_state = USB2P_WRITE_AWAIT_DAT;
      pending_op_chunk_len = 0;
      return USB2P_STEP_PROGRESS;  /* committed a chunk; more to come */
    }
  }
  /* DONE: emit RSP and clear the slot. */
  if (!usb2p_queue_rsp_u64(pending_op_opcode, pending_op_space,
                           pending_op_write_status, pending_op_txn_id,
                           pending_op_seq, pending_op_written)) {
    return USB2P_STEP_BLOCKED;  /* tx_buf full; retry next step */
  }
  pending_op_kind = USB2P_OP_NONE;
  pending_op_chunk_len = 0;
  return USB2P_STEP_PROGRESS;
}

/* One step of an active READ: emit the RSP (first step) or one DAT chunk.
   Returns PROGRESS if it emitted a message, BLOCKED if tx_buf was full. */
enum usb2p_sched_result usb2p_sched_step_read(void) {
  uint32_t chunk;
  uint16_t flags;

  if (!pending_op_rsp_sent) {
    if (!usb2p_queue_rsp_u64(pending_op_opcode, pending_op_space,
                             USB2P_STATUS_OK, pending_op_txn_id,
                             pending_op_seq, pending_op_total)) {
      return USB2P_STEP_BLOCKED;
    }
    pending_op_rsp_sent = 1;
    if (!pending_op_total) {
      /* Zero-length READ (all segments empty): RSP only, no DAT. */
      pending_op_kind = USB2P_OP_NONE;
    }
    return USB2P_STEP_PROGRESS;
  }

  /* Skip over any empty segments to find the current one with bytes to send. */
  while (pending_op_remaining == 0
         && (pending_seg_idx + 1) < pending_seg_count) {
    pending_seg_idx++;
    pending_op_offset = pending_seg_offset[pending_seg_idx];
    pending_op_remaining = pending_seg_length[pending_seg_idx];
  }

  /* Emit exactly one DAT chunk from the current segment, then yield so the
     dispatcher re-checks the EVENT slot at this message boundary.  LAST is set
     only when this chunk finishes the LAST segment. */
  chunk = pending_op_remaining;
  if (chunk > USB2P_MAX_TX_DAT_PAYLOAD) {
    chunk = USB2P_MAX_TX_DAT_PAYLOAD;
  }
  {
    uint8_t finishes_segment = (chunk == pending_op_remaining);
    uint8_t is_last_segment = ((pending_seg_idx + 1) >= pending_seg_count);
    flags = (finishes_segment && is_last_segment) ? USB2P_FLAG_LAST : 0;
  }

  chunk = usb2p_memory_read(pending_op_space, pending_op_offset,
                            read_payload, chunk);
  if (!chunk) {
    pending_op_kind = USB2P_OP_NONE;
    return USB2P_STEP_PROGRESS;  /* op ended (SPI read returned 0) */
  }
  if (!usb2p_queue_frame(USB2P_TYPE_DAT, pending_op_opcode,
                         pending_op_space, USB2P_STATUS_OK, flags,
                         pending_op_txn_id, pending_op_next_seq,
                         read_payload, chunk)) {
    /* tx_buf full.  The SPI read above is repeated next step (wasteful but
       harmless); the offset/remaining were not advanced. */
    return USB2P_STEP_BLOCKED;
  }
  pending_op_next_seq++;
  pending_op_offset += chunk;
  pending_op_remaining -= chunk;
  /* Op is done only when the current segment is exhausted AND it's the last. */
  if (pending_op_remaining == 0
      && (pending_seg_idx + 1) >= pending_seg_count) {
    pending_op_kind = USB2P_OP_NONE;
  }
  return USB2P_STEP_PROGRESS;
}
