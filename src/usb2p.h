/* sd2snes - SD card based universal cartridge for the SNES
   Copyright (C) 2009-2010 Maximilian Rehkopf <otakon@gmx.net>

   USB2P v3 framing/parser core.  This module is intentionally transport
   agnostic: endpoint code feeds bytes in and drains serialized bytes out.
*/

#ifndef __USB2P_H__
#define __USB2P_H__

#include <stdint.h>

/* Shared protocol constants and fixed payload/header offsets are generated from
   ../usb2p_protocol_schema.py so firmware, host tools, and libusb2p consume the
   same wire definition. */
#include "usb2p_protocol_generated.h"

/* Decoded common header.  seq and length are u16 on the wire (v3): length is one
   message's payload, capped at the negotiated max (1024); seq is a per-txn message
   counter.  64-bit file offsets/sizes live in opcode payloads, not here. */
struct usb2p_header {
  uint8_t type;
  uint8_t opcode;
  uint8_t space;
  uint8_t status;
  uint16_t flags;
  uint32_t txn_id;
  uint16_t seq;
  uint16_t length;
};

void usb2p_reset(void);
void usb2p_rx_bytes(const uint8_t *data, uint32_t len);
void usb2p_poll(void);
int usb2p_take_command(void);
/* Zero-copy TX drain: usb2p_tx_peek returns a pointer into the internal tx_buf
   and the number of bytes available (capped at max_len); the consumer ships them
   then calls usb2p_tx_consume to advance.  The peeked pointer is valid only until
   the next consume.  Both must run under usb2p_tx_lock (USB-IRQ critical section). */
uint32_t usb2p_tx_peek(uint8_t **data, uint32_t max_len);
void usb2p_tx_consume(uint32_t count);

#endif /* __USB2P_H__ */
