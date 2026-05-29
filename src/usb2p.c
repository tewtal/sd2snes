/* sd2snes - SD card based universal cartridge for the SNES
   Copyright (C) 2009-2010 Maximilian Rehkopf <otakon@gmx.net>

   USB2P v3 framing core: the RX byte-stream state machine, the HELLO attach
   handshake + INFO/TIME control responses, opcode dispatch, and the public
   entry points (usb2p_reset / usb2p_rx_bytes / usb2p_poll / usb2p_take_command).

   The feature modules do the work behind the dispatch:
     usb2p_codec.c  — byte I/O, header CRC, frame queue, tx_buf streaming
     usb2p_path.c   — UTF-8 <-> CP-1252 path transcode + normalization
     usb2p_mem.c    — memory READ/WRITE (segmented)
     usb2p_fs.c     — file + directory ops
     usb2p_watch.c  — address-watch subscriptions + EVENT emission
     usb2p_sched.c  — the IN-endpoint priority dispatcher

   This file delivers received bytes into the RX state machine, validates each
   header (codec), dispatches the framed message to the right handler, and pumps
   the scheduler from the main-loop poll.  All FPGA SPI / FatFs work is deferred
   to the scheduler steps so it never runs in the USB ISR.
*/

#include <stdint.h>
#include <string.h>

#include "cfg.h"
#include "config.h"
#include "fpga_spi.h"
#include "memory.h"
#include "rtc.h"
#include "snes.h"
#include "timer.h"
#include "usb2p.h"
#include "usb2p_internal.h"
#include "usbuser.h"
#include "version.h"

extern cfg_t CFG;

/* RX framing state machine. */
static enum usb2p_rx_state rx_state;
static uint8_t rx_header[USB2P_HEADER_WIRE_LEN];
static uint8_t rx_payload[USB2P_MAX_RX_PAYLOAD];
static uint32_t rx_header_pos;
static uint32_t rx_payload_pos;
static uint32_t rx_payload_len;
static tick_t rx_payload_deadline;
static struct usb2p_header rx_current;

/* Connection state: set by the HELLO handshake, cleared on reset/RESYNC. */
uint8_t usb2p_attached;

/* Deferred SNES command (RESET / MENU_RESET), drained by usb2p_take_command. */
int pending_cmd;

/* Deferred NAK.  A payload timeout or a NAK that could not be queued while
   mid-stream is parked here and flushed on the next rx-bytes entry. */
static uint8_t pending_nak;
static uint8_t pending_nak_status;
static uint32_t pending_nak_txn_id;
static uint32_t pending_nak_seq;

static void usb2p_rx_to_sync(void) {
  rx_state = USB2P_RX_SYNC;
  rx_header_pos = 0;
  rx_payload_pos = 0;
  rx_payload_len = 0;
  rx_payload_deadline = 0;
}

static uint8_t usb2p_flush_pending_nak(void) {
  if (!pending_nak) {
    return 1;
  }
  if (!usb2p_queue_nak(pending_nak_status, pending_nak_txn_id,
                       pending_nak_seq)) {
    return 0;
  }
  pending_nak = 0;
  return 1;
}

static uint8_t usb2p_mark_timeout_payload(void) {
  if (rx_state != USB2P_RX_PAYLOAD
      || !rx_payload_deadline
      || !time_after(getticks(), rx_payload_deadline)) {
    return 0;
  }

  pending_nak = 1;
  pending_nak_status = USB2P_STATUS_ETIMEDOUT;
  pending_nak_txn_id = rx_current.txn_id;
  pending_nak_seq = rx_current.seq;
  usb2p_rx_to_sync();
  return 1;
}

/* Returns 1 if a DAT(WRITE) dispatch would race with a still-pending commit of
   the previous chunk.  The parser then keeps the buffered payload and goes to
   USB2P_RX_HELD instead of dispatching; usb2p_try_dispatch_held() retries once
   the commit finishes. */
static uint8_t usb2p_dispatch_would_race(const struct usb2p_header *h) {
  return (h->type == USB2P_TYPE_DAT
          && h->opcode == USB2P_OPCODE_WRITE
          && pending_op_kind == USB2P_OP_WRITE
          && pending_op_write_state == USB2P_WRITE_COMMIT);
}

void usb2p_reset(void) {
  usb2p_rx_to_sync();
  memset(&rx_current, 0, sizeof(rx_current));
  usb2p_attached = 0;
  usb2p_codec_reset();
  pending_op_kind = USB2P_OP_NONE;
  pending_op_remaining = 0;
  pending_op_rsp_sent = 0;
  pending_op_write_state = USB2P_WRITE_AWAIT_DAT;
  pending_op_chunk_len = 0;
  pending_op_chunk_last = 0;
  pending_op_written = 0;
  pending_seg_count = 0;
  pending_seg_idx = 0;
  pending_op_total_remaining = 0;
  pending_file_complete = 0;
  pending_file_chunk_valid = 0;
  pending_dir_rsp_sent = 0;
  pending_dir_have_entry = 0;
  pending_dir_end = 0;
  pending_dir_batch_len = 0;
  pending_meta = 0;
  pending_cancel = 0;
  pending_nak = 0;
  pending_cmd = 0;
  usb2p_close_file_handles();
  usb2p_close_dir_handles();
  usb2p_clear_watches();
}

/* ---- HELLO / INFO / TIME builders ------------------------------------------*/

static void usb2p_write_hello_payload(uint8_t *payload) {
  memset(payload, 0, USB2P_HELLO_PAYLOAD_LEN);
  usb2p_wr16(payload + USB2P_HELLO_PAYLOAD_MIN_VERSION_OFFSET, USB2P_VERSION);
  usb2p_wr16(payload + USB2P_HELLO_PAYLOAD_MAX_VERSION_OFFSET, USB2P_VERSION);
  usb2p_wr32(payload + USB2P_HELLO_PAYLOAD_CAPABILITIES_OFFSET,
             USB2P_CAP_HEADER_CRC
             | USB2P_CAP_ACK_NAK_RESYNC
             | USB2P_CAP_MAX_OUTSTANDING1);
  usb2p_wr16(payload + USB2P_HELLO_PAYLOAD_MAX_RX_MESSAGE_PAYLOAD_OFFSET,
             USB2P_MAX_RX_PAYLOAD);
  usb2p_wr16(payload + USB2P_HELLO_PAYLOAD_MAX_TX_DAT_PAYLOAD_OFFSET,
             USB2P_MAX_TX_DAT_PAYLOAD);
  payload[USB2P_HELLO_PAYLOAD_MAX_OUTSTANDING_TXNS_OFFSET] = 1;
  payload[USB2P_HELLO_PAYLOAD_MAX_OPEN_FILES_OFFSET] = USB2P_MAX_OPEN_FILES;
  payload[USB2P_HELLO_PAYLOAD_MAX_WATCHES_OFFSET] = USB2P_MAX_WATCHES;
  payload[USB2P_HELLO_PAYLOAD_MAX_WATCH_BYTES_PER_TICK_OFFSET] =
      USB2P_MAX_WATCH_BYTES_PER_TICK;
  usb2p_wr16(payload + USB2P_HELLO_PAYLOAD_MAX_EVENT_BYTES_PER_TICK_OFFSET,
             USB2P_MAX_EVENT_BYTES_PER_TICK);
  payload[USB2P_HELLO_PAYLOAD_MAX_EVENT_BURST_OFFSET] = USB2P_MAX_EVENT_BURST;
  payload[USB2P_HELLO_PAYLOAD_MAX_SEGMENTS_OFFSET] = USB2P_MAX_SEGMENTS;
  /* bytes 20..23 reserved (= 0 from the memset). */
}

static void usb2p_copy_str(uint8_t *dst, uint32_t dst_len, const char *src) {
  uint32_t i;

  for (i = 0; i < dst_len; i++) {
    dst[i] = 0;
  }
  if (!src) return;
  for (i = 0; i + 1 < dst_len && src[i]; i++) {
    dst[i] = (uint8_t)src[i];
  }
}

void usb2p_write_info_payload(uint8_t *payload) {
  uint16_t cfg_flags = 0;
  const char *rom_name = current_filename;
  uint32_t rom_name_len;

  memset(payload, 0, USB2P_INFO_PAYLOAD_LEN);

  usb2p_wr32(payload + 0, CONFIG_FWVER);
  usb2p_wr16(payload + 4, current_features);
  if (CFG.enable_ingame_hook) cfg_flags |= 0x0001;
  if (CFG.enable_ingame_savestate && CFG.enable_ingame_hook) cfg_flags |= 0x0002;
  usb2p_wr16(payload + 6, cfg_flags);

  rom_name_len = strlen(rom_name);
  if (rom_name_len >= USB2P_INFO_CURRENT_ROM_LEN) {
    rom_name += rom_name_len - (USB2P_INFO_CURRENT_ROM_LEN - 1);
  }
  usb2p_copy_str(payload + 16, USB2P_INFO_CURRENT_ROM_LEN, rom_name);
  usb2p_copy_str(payload + 256, USB2P_INFO_VERSION_LEN, CONFIG_VERSION);
  usb2p_copy_str(payload + 320, USB2P_INFO_DEVICE_NAME_LEN, DEVICE_NAME);
}

uint8_t usb2p_queue_time_rsp(uint8_t space, uint32_t txn_id, uint32_t seq) {
  usb2p_wr64(meta_payload, get_bcdtime());
  return usb2p_queue_frame(USB2P_TYPE_RSP, USB2P_OPCODE_TIME, space,
                           USB2P_STATUS_OK, 0, txn_id, seq,
                           meta_payload, USB2P_TIME_RSP_LEN);
}

static uint8_t usb2p_handle_time(const struct usb2p_header *h,
                                 const uint8_t *payload) {
  if (h->length == USB2P_TIME_RSP_LEN) {
    set_bcdtime(usb2p_rd64(payload));
  } else if (h->length != 0) {
    return usb2p_queue_nak(USB2P_STATUS_EINVAL, h->txn_id, h->seq);
  }
  return usb2p_queue_time_rsp(h->space, h->txn_id, h->seq);
}

/* ---- frame dispatch --------------------------------------------------------*/

static void usb2p_handle_frame(const struct usb2p_header *h,
                               const uint8_t *payload) {
  uint8_t hello_payload[24];

  if (h->type == USB2P_TYPE_RESYNC
      || (h->type == USB2P_TYPE_REQ && h->opcode == USB2P_OPCODE_RESYNC)) {
    usb2p_attached = 0;
    usb2p_queue_header_only(USB2P_TYPE_ACK, USB2P_OPCODE_RESYNC,
                            USB2P_STATUS_OK, h->txn_id, h->seq);
    return;
  }

  /* DAT-OUT: payload for an in-flight host->device transfer.  Today only WRITE
     uses this; route to the WRITE state machine when it matches the pinned op. */
  if (h->type == USB2P_TYPE_DAT) {
    if (h->opcode == USB2P_OPCODE_WRITE) {
      usb2p_handle_write_dat(h, payload);
    } else {
      usb2p_queue_nak(USB2P_STATUS_EUNSUPPORTED, h->txn_id, h->seq);
    }
    return;
  }

  /* Bulk CANCEL (§6.7): target txn is the header txn_id.  Stash it; the
     scheduler clears the active op (if it matches) and emits the terminal
     RSP(ECANCELLED) at a message boundary.  Idempotent for a non-active txn.
     Accepted by type==CANCEL or REQ+opcode==CANCEL, mirroring RESYNC. */
  if (h->type == USB2P_TYPE_CANCEL
      || (h->type == USB2P_TYPE_REQ && h->opcode == USB2P_OPCODE_CANCEL)) {
    if (pending_cancel) {
      usb2p_queue_nak(USB2P_STATUS_EBUSY, h->txn_id, h->seq);
    } else {
      pending_cancel_txn_id = h->txn_id;
      pending_cancel_seq = h->seq;
      pending_cancel = 1;  /* publish last */
    }
    return;
  }

  if (h->type != USB2P_TYPE_REQ) {
    usb2p_queue_nak(USB2P_STATUS_EINVAL, h->txn_id, h->seq);
    return;
  }

  /* HELLO is the only REQ accepted before attach; everything else NAKs
     EBADSTATE until the handshake completes. */
  if (h->opcode != USB2P_OPCODE_HELLO && !usb2p_attached) {
    usb2p_queue_nak(USB2P_STATUS_EBADSTATE, h->txn_id, h->seq);
    return;
  }

  switch (h->opcode) {
    case USB2P_OPCODE_HELLO:
      /* HELLO is the session handshake: a (re)connecting client starts clean,
         so force-close any handles a previous/wedged session leaked.  With only
         one file slot this is the client's reliable "get me unstuck" path. */
      usb2p_close_file_handles();
      usb2p_close_dir_handles();
      usb2p_attached = 1;
      usb2p_write_hello_payload(hello_payload);
      (void)usb2p_queue_frame(USB2P_TYPE_RSP, USB2P_OPCODE_HELLO, h->space,
                              USB2P_STATUS_OK, 0, h->txn_id, h->seq,
                              hello_payload, sizeof(hello_payload));
      break;
    case USB2P_OPCODE_INFO:
      if (pending_op_kind != USB2P_OP_NONE) {
        /* A data transfer is in flight.  Don't answer inline — tx_buf may be
           full of its DATs and a dropped RSP would hang the host.  Stash the
           request; the scheduler emits the RSP at the next message boundary,
           interleaved between the transfer's DAT messages (§6.7). */
        if (pending_meta) {
          usb2p_queue_nak(USB2P_STATUS_EBUSY, h->txn_id, h->seq);
        } else {
          pending_meta_opcode = USB2P_OPCODE_INFO;
          pending_meta_space = h->space;
          pending_meta_txn_id = h->txn_id;
          pending_meta_seq = h->seq;
          pending_meta = 1;  /* publish last: fields valid before slot occupied */
        }
      } else {
        /* Idle: answer inline (fast path). */
        usb2p_write_info_payload(info_payload);
        (void)usb2p_queue_frame(USB2P_TYPE_RSP, USB2P_OPCODE_INFO, h->space,
                                USB2P_STATUS_OK, 0, h->txn_id, h->seq,
                                info_payload, USB2P_INFO_PAYLOAD_LEN);
      }
      break;
    case USB2P_OPCODE_TIME:
      if (pending_op_kind != USB2P_OP_NONE) {
        if (h->length != 0 || pending_meta) {
          usb2p_queue_nak(USB2P_STATUS_EBUSY, h->txn_id, h->seq);
        } else {
          pending_meta_opcode = USB2P_OPCODE_TIME;
          pending_meta_space = h->space;
          pending_meta_txn_id = h->txn_id;
          pending_meta_seq = h->seq;
          pending_meta = 1;
        }
      } else {
        (void)usb2p_handle_time(h, payload);
      }
      break;
    case USB2P_OPCODE_STAT:
      if (pending_op_kind != USB2P_OP_NONE) {
        usb2p_queue_nak(USB2P_STATUS_EBUSY, h->txn_id, h->seq);
      } else {
        usb2p_handle_stat(h, payload);
      }
      break;
    case USB2P_OPCODE_STATFS:
      if (pending_op_kind != USB2P_OP_NONE) {
        usb2p_queue_nak(USB2P_STATUS_EBUSY, h->txn_id, h->seq);
      } else {
        usb2p_handle_statfs(h, payload);
      }
      break;
    case USB2P_OPCODE_FOPEN:
      usb2p_handle_fopen(h, payload);
      break;
    case USB2P_OPCODE_FREAD:
      usb2p_handle_fread(h, payload);
      break;
    case USB2P_OPCODE_FWRITE:
      usb2p_handle_fwrite(h, payload);
      break;
    case USB2P_OPCODE_FTRUNCATE:
      usb2p_handle_ftruncate(h, payload);
      break;
    case USB2P_OPCODE_FSYNC:
      usb2p_handle_file_handle_only(h, payload, USB2P_OP_FSYNC);
      break;
    case USB2P_OPCODE_FCLOSE:
      usb2p_handle_file_handle_only(h, payload, USB2P_OP_FCLOSE);
      break;
    case USB2P_OPCODE_SETATTR:
      usb2p_handle_setattr(h, payload);
      break;
    case USB2P_OPCODE_SETTIMES:
      usb2p_handle_settimes(h, payload);
      break;
    case USB2P_OPCODE_MKDIR:
      usb2p_handle_path_only_op(h, payload, USB2P_OP_MKDIR);
      break;
    case USB2P_OPCODE_UNLINK:
      usb2p_handle_path_only_op(h, payload, USB2P_OP_UNLINK);
      break;
    case USB2P_OPCODE_RENAME:
      usb2p_handle_rename(h, payload);
      break;
    case USB2P_OPCODE_OPENDIR:
      usb2p_handle_opendir(h, payload);
      break;
    case USB2P_OPCODE_READDIR:
      usb2p_handle_readdir(h, payload);
      break;
    case USB2P_OPCODE_CLOSEDIR:
      usb2p_handle_closedir(h, payload);
      break;
    case USB2P_OPCODE_READ:
      usb2p_handle_read(h, payload);
      break;
    case USB2P_OPCODE_WRITE:
      usb2p_handle_write(h, payload);
      break;
    case USB2P_OPCODE_WATCH_ADD:
      (void)usb2p_handle_watch_add(h, payload);
      break;
    case USB2P_OPCODE_WATCH_REMOVE:
      (void)usb2p_handle_watch_remove(h, payload);
      break;
    case USB2P_OPCODE_WATCH_CLEAR:
      (void)usb2p_handle_watch_clear(h);
      break;
    case USB2P_OPCODE_RESET:
      usb2p_queue_rsp_empty(h->opcode, h->space, USB2P_STATUS_OK,
                            h->txn_id, h->seq);
      pending_cmd = SNES_CMD_RESET;
      break;
    case USB2P_OPCODE_MENU_RESET:
      usb2p_queue_rsp_empty(h->opcode, h->space, USB2P_STATUS_OK,
                            h->txn_id, h->seq);
      pending_cmd = SNES_CMD_RESET_TO_MENU;
      break;
    default:
      usb2p_queue_nak(USB2P_STATUS_EUNSUPPORTED, h->txn_id, h->seq);
      break;
  }
}

/* ---- RX byte-stream state machine ------------------------------------------*/

static uint8_t usb2p_magic_byte(uint32_t pos) {
  static const uint8_t magic[4] = {
    USB2P_MAGIC0, USB2P_MAGIC1, USB2P_MAGIC2, USB2P_MAGIC3
  };
  return magic[pos];
}

static void usb2p_rx_sync_byte(uint8_t byte) {
  if (byte == usb2p_magic_byte(rx_header_pos)) {
    rx_header[rx_header_pos++] = byte;
    if (rx_header_pos == 4) {
      rx_state = USB2P_RX_HEADER;
    }
    return;
  }

  rx_header_pos = (byte == USB2P_MAGIC0) ? 1 : 0;
  if (rx_header_pos) {
    rx_header[0] = byte;
  }
}

void usb2p_rx_bytes(const uint8_t *data, uint32_t len) {
  uint32_t i;

  if (usb2p_mark_timeout_payload()) {
    (void)usb2p_flush_pending_nak();
    return;
  }

  if (pending_nak) {
    (void)usb2p_flush_pending_nak();
    return;
  }

  for (i = 0; i < len; i++) {
    switch (rx_state) {
      case USB2P_RX_SYNC:
        usb2p_rx_sync_byte(data[i]);
        break;

      case USB2P_RX_HEADER:
        rx_header[rx_header_pos++] = data[i];
        if (rx_header_pos == USB2P_HEADER_WIRE_LEN) {
          uint8_t trusted;
          uint8_t status = usb2p_decode_header(rx_header, &rx_current, &trusted);
          if (status != USB2P_STATUS_OK) {
            usb2p_rx_to_sync();
            usb2p_queue_nak(status,
                            trusted ? rx_current.txn_id : 0,
                            trusted ? rx_current.seq : 0);
          } else if (rx_current.length == 0) {
            usb2p_rx_to_sync();
            usb2p_handle_frame(&rx_current, 0);
          } else {
            rx_payload_len = (uint32_t)rx_current.length;
            rx_payload_pos = 0;
            rx_payload_deadline = getticks() + MS_TO_TICKS(USB2P_RX_PAYLOAD_TIMEOUT_MS);
            rx_state = USB2P_RX_PAYLOAD;
          }
        }
        break;

      case USB2P_RX_PAYLOAD:
        rx_payload[rx_payload_pos++] = data[i];
        if (rx_payload_pos == rx_payload_len) {
          if (usb2p_dispatch_would_race(&rx_current)) {
            /* Hold the payload in rx_payload; the main-loop poll will retry
               dispatch once the previous chunk's commit completes.  No bytes
               are consumed in HELD state, so USB will back-pressure naturally. */
            rx_state = USB2P_RX_HELD;
          } else {
            struct usb2p_header h = rx_current;
            usb2p_rx_to_sync();
            usb2p_handle_frame(&h, rx_payload);
          }
        }
        break;

      case USB2P_RX_HELD:
        /* Drop further bytes silently — the caller should not be sending more
           data anyway since we have not signalled buffer space.  In practice
           the USB stack stops calling usb2p_rx_bytes for new packets while the
           EP OUT buffer is unread, so this branch should never execute. */
        (void)data[i];
        break;
    }
  }
}

static void usb2p_try_dispatch_held(void) {
  struct usb2p_header h;
  if (rx_state != USB2P_RX_HELD) return;
  if (usb2p_dispatch_would_race(&rx_current)) return;
  h = rx_current;
  usb2p_rx_to_sync();
  usb2p_handle_frame(&h, rx_payload);
}

/* ---- public poll / command API ---------------------------------------------*/

void usb2p_poll(void) {
  (void)usb2p_mark_timeout_payload();
  usb2p_poll_watches();
  usb2p_drain_pending_op();
  /* If the parser is holding a DAT chunk because the previous commit was still
     in flight, the drain just freed it — retry the dispatch now so the held
     chunk gets staged for the next drain pass. */
  usb2p_try_dispatch_held();
  /* If the held dispatch staged another COMMIT, drain it immediately so we
     don't sit a full main-loop cycle behind. */
  if (pending_op_write_state == USB2P_WRITE_COMMIT) {
    usb2p_drain_pending_op();
  }
}

int usb2p_take_command(void) {
  int cmd = pending_cmd;
  pending_cmd = 0;
  return cmd;
}
