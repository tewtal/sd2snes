/* sd2snes - SD card based universal cartridge for the SNES
   Copyright (C) 2009-2010 Maximilian Rehkopf <otakon@gmx.net>

   USB2P v3 codec: little-endian byte I/O, the header CRC + decode, frame
   queueing into tx_buf, and the byte-stream drain to the USB IN endpoint.

   tx_buf is a plain linear append buffer: frames are appended at tx_len and
   drained from tx_pos.  usb2p_tx_peek streams bytes out *without* stopping at
   frame boundaries — the host delimits messages by the header length prefix,
   not by USB packet boundaries (README.usb2p.md §12.3), so the tail of one frame
   and the head of the next can share a single 64 B USB packet.  That keeps the
   IN pipe continuous across a multi-DAT transfer: no short packet is emitted
   until the buffer genuinely runs dry, so one host dev.read pulls the whole
   transfer instead of terminating on each DAT.

   tx_buf lives in AHB SRAM (16 KB, lpc1754.ld §ahbram) to keep the tight 16 KB
   main RAM free.  It is sized (USB2P_TX_BUF_SIZE) to hold one RSP plus several
   max DAT chunks so the main loop can pre-queue DAT N+1 while DAT N streams out.
*/

#include <stdint.h>
#include <string.h>

#include "config.h"
#include "crc16.h"
#include "usb2p.h"
#include "usb2p_internal.h"
#include "usbuser.h"

/* tx_buf is shared with the USB ISR consumer (usb2p_tx_peek); the append in
   usb2p_queue_frame is guarded by the short USB-IRQ critical section. */
static IN_AHBRAM uint8_t tx_buf[USB2P_TX_BUF_SIZE];
static uint32_t tx_len;
static uint32_t tx_pos;

/* Response-payload scratch builders.  rsp_payload and meta_payload alias one
   backing array (sized to the larger): each is filled and immediately copied
   into tx_buf by usb2p_queue_frame within a single synchronous handler call, so
   the two never hold live data at the same time (single-op-slot model).
   info_payload is separate — it's larger and INFO can be stashed (pending_meta)
   while another response is built. */
uint8_t meta_payload[USB2P_FOPEN_RSP_LEN];   /* rsp_payload aliases this */
uint8_t info_payload[USB2P_INFO_PAYLOAD_LEN];

uint16_t usb2p_crc16(const uint8_t *data, uint32_t len) {
  uint16_t crc = USB2P_CRC16_INIT;
  uint32_t i;

  for (i = 0; i < len; i++) {
    crc = crc16_update(crc, data[i]);
  }
  return crc;
}

uint16_t usb2p_rd16(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

uint32_t usb2p_rd32(const uint8_t *p) {
  return (uint32_t)p[0]
       | ((uint32_t)p[1] << 8)
       | ((uint32_t)p[2] << 16)
       | ((uint32_t)p[3] << 24);
}

uint64_t usb2p_rd64(const uint8_t *p) {
  return (uint64_t)usb2p_rd32(p) | ((uint64_t)usb2p_rd32(p + 4) << 32);
}

void usb2p_wr16(uint8_t *p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xff);
  p[1] = (uint8_t)((v >> 8) & 0xff);
}

void usb2p_wr32(uint8_t *p, uint32_t v) {
  p[0] = (uint8_t)(v & 0xff);
  p[1] = (uint8_t)((v >> 8) & 0xff);
  p[2] = (uint8_t)((v >> 16) & 0xff);
  p[3] = (uint8_t)((v >> 24) & 0xff);
}

void usb2p_wr64(uint8_t *p, uint64_t v) {
  usb2p_wr32(p, (uint32_t)(v & 0xffffffffu));
  usb2p_wr32(p + 4, (uint32_t)(v >> 32));
}

uint8_t usb2p_queue_frame(uint8_t type, uint8_t opcode, uint8_t space,
                          uint8_t status, uint16_t flags,
                          uint32_t txn_id, uint32_t seq,
                          const uint8_t *payload, uint32_t payload_len) {
  uint16_t crc;
  uint32_t need = USB2P_HEADER_WIRE_LEN + payload_len;
  uint8_t *p;
  uint8_t lock;

  /* Guard the whole append in the short USB-IRQ critical section.  This is
     bounded by the payload memcpy (<=1 KB ~= a couple of us), NOT by any SPI
     read — the caller does its SPI read *before* calling us, with the IRQ
     enabled, so IN-complete IRQs ship buffered packets concurrently. */
  lock = usb2p_tx_lock();

  /* If the buffer is fully drained, snap back to the start so we have the
     maximum contiguous space available for the next append. */
  if (tx_pos >= tx_len) {
    tx_pos = 0;
    tx_len = 0;
  }

  if (need > sizeof(tx_buf) - tx_len) {
    usb2p_tx_unlock(lock);
    return 0;  /* not enough room to append this frame */
  }

  p = tx_buf + tx_len;
  p[0] = USB2P_MAGIC0;
  p[1] = USB2P_MAGIC1;
  p[2] = USB2P_MAGIC2;
  p[3] = USB2P_MAGIC3;
  p[4] = USB2P_VERSION;
  p[5] = USB2P_HEADER_LEN;
  p[USB2P_HEADER_TYPE_OFFSET] = type;
  p[USB2P_HEADER_OPCODE_OFFSET] = opcode;
  p[USB2P_HEADER_SPACE_OFFSET] = space;
  p[USB2P_HEADER_STATUS_OFFSET] = status;
  usb2p_wr16(p + USB2P_HEADER_FLAGS_OFFSET, flags);
  usb2p_wr32(p + USB2P_HEADER_TXN_ID_OFFSET, txn_id);
  usb2p_wr16(p + USB2P_HEADER_SEQ_OFFSET, (uint16_t)seq);
  usb2p_wr16(p + USB2P_HEADER_LENGTH_OFFSET, (uint16_t)payload_len);
  usb2p_wr32(p + USB2P_HEADER_RESERVED_OFFSET, 0);

  crc = usb2p_crc16(p, USB2P_HEADER_LEN);
  usb2p_wr16(p + USB2P_HEADER_LEN, crc);

  if (payload_len) {
    memcpy(p + USB2P_HEADER_WIRE_LEN, payload, payload_len);
  }

  tx_len += need;
  usb2p_tx_unlock(lock);
  return 1;
}

uint8_t usb2p_queue_header_only(uint8_t type, uint8_t opcode, uint8_t status,
                                uint32_t txn_id, uint32_t seq) {
  return usb2p_queue_frame(type, opcode, 0, status, USB2P_FLAG_HDR_ONLY,
                           txn_id, seq, 0, 0);
}

uint8_t usb2p_queue_rsp_empty(uint8_t opcode, uint8_t space, uint8_t status,
                              uint32_t txn_id, uint32_t seq) {
  return usb2p_queue_frame(USB2P_TYPE_RSP, opcode, space, status,
                           USB2P_FLAG_HDR_ONLY, txn_id, seq, 0, 0);
}

uint8_t usb2p_queue_rsp_u64(uint8_t opcode, uint8_t space, uint8_t status,
                            uint32_t txn_id, uint32_t seq, uint64_t value) {
  usb2p_wr64(rsp_payload, value);
  return usb2p_queue_frame(USB2P_TYPE_RSP, opcode, space, status, 0,
                           txn_id, seq, rsp_payload, USB2P_RW_RSP_LEN);
}

uint8_t usb2p_queue_rsp_u32(uint8_t opcode, uint8_t space, uint8_t status,
                            uint32_t txn_id, uint32_t seq, uint32_t value) {
  usb2p_wr32(rsp_payload, value);
  return usb2p_queue_frame(USB2P_TYPE_RSP, opcode, space, status, 0,
                           txn_id, seq, rsp_payload, USB2P_WATCH_ID_LEN);
}

uint8_t usb2p_queue_nak(uint8_t status, uint32_t txn_id, uint32_t seq) {
  return usb2p_queue_header_only(USB2P_TYPE_NAK, USB2P_OPCODE_NAK,
                                 status, txn_id, seq);
}

uint8_t usb2p_header_magic_ok(const uint8_t *h) {
  return h[0] == USB2P_MAGIC0
      && h[1] == USB2P_MAGIC1
      && h[2] == USB2P_MAGIC2
      && h[3] == USB2P_MAGIC3;
}

uint8_t usb2p_decode_header(const uint8_t *rx_header,
                            struct usb2p_header *out, uint8_t *trusted) {
  uint16_t want_crc;
  uint16_t got_crc;

  *trusted = 0;
  if (!usb2p_header_magic_ok(rx_header)) {
    return USB2P_STATUS_EBADHDR;
  }

  got_crc = usb2p_rd16(rx_header + USB2P_HEADER_LEN);
  want_crc = usb2p_crc16(rx_header, USB2P_HEADER_LEN);
  if (got_crc != want_crc) {
    return USB2P_STATUS_EBADHDR;
  }

  out->type = rx_header[USB2P_HEADER_TYPE_OFFSET];
  out->opcode = rx_header[USB2P_HEADER_OPCODE_OFFSET];
  out->space = rx_header[USB2P_HEADER_SPACE_OFFSET];
  out->status = rx_header[USB2P_HEADER_STATUS_OFFSET];
  out->flags = usb2p_rd16(rx_header + USB2P_HEADER_FLAGS_OFFSET);
  out->txn_id = usb2p_rd32(rx_header + USB2P_HEADER_TXN_ID_OFFSET);
  out->seq = usb2p_rd16(rx_header + USB2P_HEADER_SEQ_OFFSET);
  out->length = usb2p_rd16(rx_header + USB2P_HEADER_LENGTH_OFFSET);
  *trusted = 1;

  if (rx_header[USB2P_HEADER_VERSION_OFFSET] != USB2P_VERSION
      || rx_header[USB2P_HEADER_HEADER_LEN_OFFSET] != USB2P_HEADER_LEN
      || usb2p_rd32(rx_header + USB2P_HEADER_RESERVED_OFFSET) != 0) {
    return USB2P_STATUS_EBADHDR;
  }

  if (out->flags & USB2P_FLAGS_RESERVED) {
    return USB2P_STATUS_EINVAL;
  }
  if ((out->flags & USB2P_FLAG_HDR_ONLY) && out->length != 0) {
    return USB2P_STATUS_EINVAL;
  }
  if (out->length > USB2P_MAX_RX_PAYLOAD) {
    return USB2P_STATUS_EMSGSIZE;
  }
  return USB2P_STATUS_OK;
}

void usb2p_codec_reset(void) {
  tx_len = 0;
  tx_pos = 0;
}

/* Zero-copy drain: hand the consumer (CDC2P_KickTx) a pointer straight into
   tx_buf plus the byte count it may take, capped at max_len.  USB_WriteEP copies
   the bytes into the SIE FIFO synchronously, so the pointer only needs to stay
   valid for that one call; the consumer then calls usb2p_tx_consume to advance
   tx_pos.  This avoids the per-packet bounce buffer + memcpy the old
   usb2p_tx_read needed.

   Streams bytes out without regard to frame boundaries — the host delimits
   messages by the header length prefix (README.usb2p.md §12.3), so packing the
   tail of one frame and the head of the next into one 64 B USB packet is
   correct. */
uint32_t usb2p_tx_peek(uint8_t **data, uint32_t max_len) {
  uint32_t count;

  if (tx_pos >= tx_len) {
    /* No bytes ready; backend ops refill from usb2p_poll (main loop) so
       FPGA SPI never runs in the USB ISR. */
    tx_len = 0;
    tx_pos = 0;
    return 0;
  }

  count = tx_len - tx_pos;
  if (count > max_len) {
    count = max_len;
  }
  *data = tx_buf + tx_pos;
  return count;
}

void usb2p_tx_consume(uint32_t count) {
  tx_pos += count;
  if (tx_pos >= tx_len) {
    tx_len = 0;
    tx_pos = 0;
  }
}
