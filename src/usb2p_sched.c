/* sd2snes - SD card based universal cartridge for the SNES
   Copyright (C) 2009-2010 Maximilian Rehkopf <otakon@gmx.net>

   USB2P v3 IN-endpoint scheduler (README.usb2p.md §12.2).  The IN endpoint is
   driven by an explicit dispatcher evaluated at each message boundary, rather
   than a single transfer "owning" the endpoint until done.  Each usb2p_sched_step
   emits AT MOST ONE message and returns, so the priority order is re-checked
   between every message:

     1. Pending watch EVENT        (usb2p_watch.c)
     2. Pending bulk CANCEL        (terminates the active op promptly)
     3. Pending metadata RSP       (INFO/TIME stashed while a data op is active)
     4. Active transfer            (READ/WRITE in usb2p_mem.c; file/dir ops in
                                    usb2p_fs.c)
     5. Idle

   This module also owns the pinned-op core state (pending_op_*) and the two
   interleave slots (CANCEL, metadata), and exposes usb2p_drain_pending_op, which
   the main-loop poll calls to flush messages until tx_buf is full or idle.
*/

#include <stdint.h>

#include "fileops.h"
#include "usb2p.h"
#include "usb2p_internal.h"

/* The pinned deferred op.  See enum usb2p_op_kind / the single-op-slot model in
   usb2p_internal.h. */
enum usb2p_op_kind pending_op_kind;
uint8_t pending_op_space;
uint8_t pending_op_opcode;
uint32_t pending_op_txn_id;
uint32_t pending_op_seq;
uint32_t pending_op_offset;
uint32_t pending_op_remaining;
uint32_t pending_op_total;

/* Metadata interleaving (§6.7).  Cheap metadata/control ops (INFO/TIME today)
   may be answered WHILE a data-bearing READ/WRITE is in flight.  When such a REQ
   arrives during an active data op, the framing core cannot answer inline
   reliably — tx_buf may be full of the active transfer's DATs, and a dropped
   response would hang the host.  So it stashes the request here (depth 1) and
   this scheduler emits the response at the next message boundary when tx_buf has
   room.  A second metadata REQ while the slot is occupied is NAK(EBUSY). */
uint8_t  pending_meta;
uint8_t  pending_meta_opcode;
uint8_t  pending_meta_space;
uint32_t pending_meta_txn_id;
uint32_t pending_meta_seq;

/* Bulk CANCEL (§6.7).  A CANCEL names a target txn in the header txn_id.  The
   framing core only stashes the target here (depth 1); the scheduler processes
   it at a step boundary so clearing the active op and emitting the terminal RSP
   happen in main-loop context with no race against an in-progress step. */
uint8_t  pending_cancel;
uint32_t pending_cancel_txn_id;
uint32_t pending_cancel_seq;

/* Process a stashed bulk CANCEL at a message boundary.  If the target txn is the
   active data op, clear it (the scheduler then stops emitting its DATs — this
   RSP becomes the last frame for that txn).  Always emit a terminal
   RSP(ECANCELLED, txn=target); cancel is idempotent for a non-active txn. */
static enum usb2p_sched_result usb2p_sched_step_cancel(void) {
  if (!pending_cancel) {
    return USB2P_STEP_IDLE;
  }
  /* Emit the terminal RSP first; only clear the slot/op once it's queued, so a
     full tx_buf just retries next boundary without losing the cancel. */
  if (!usb2p_queue_frame(USB2P_TYPE_RSP, USB2P_OPCODE_CANCEL, 0,
                         USB2P_STATUS_ECANCELLED, USB2P_FLAG_LAST,
                         pending_cancel_txn_id, pending_cancel_seq, 0, 0)) {
    return USB2P_STEP_BLOCKED;
  }
  /* If the cancelled txn is the active op, terminate it so no more DATs flow. */
  if (pending_op_kind != USB2P_OP_NONE
      && pending_op_txn_id == pending_cancel_txn_id) {
    if (pending_op_kind == USB2P_OP_FOPEN
        && pending_file_complete
        && pending_file_status == USB2P_STATUS_OK
        && file_handles[pending_file_slot].active) {
      (void)f_close(&file_handles[pending_file_slot].fil);
      file_handles[pending_file_slot].active = 0;
    }
    /* OPENDIR cancelled after a successful open: the host never received the
       dir_handle, so close it here or the slot leaks until reset. */
    if (pending_op_kind == USB2P_OP_OPENDIR
        && pending_file_complete
        && pending_file_status == USB2P_STATUS_OK
        && dir_handles[pending_file_slot].active) {
      (void)f_closedir(&dir_handles[pending_file_slot].dir);
      dir_handles[pending_file_slot].active = 0;
    }
    pending_op_kind = USB2P_OP_NONE;
  }
  pending_cancel = 0;
  return USB2P_STEP_PROGRESS;
}

/* Emit a stashed metadata response (§6.7) at a message boundary.  Builds the
   response payload now (cheap, no SPI for INFO/TIME) and queues it; clears the
   slot on success.  IDLE if no metadata is pending, BLOCKED if tx_buf is full. */
static enum usb2p_sched_result usb2p_sched_step_meta(void) {
  if (!pending_meta) {
    return USB2P_STEP_IDLE;
  }
  switch (pending_meta_opcode) {
    case USB2P_OPCODE_INFO:
      usb2p_write_info_payload(info_payload);
      if (!usb2p_queue_frame(USB2P_TYPE_RSP, USB2P_OPCODE_INFO,
                             pending_meta_space, USB2P_STATUS_OK, 0,
                             pending_meta_txn_id, pending_meta_seq,
                             info_payload, USB2P_INFO_PAYLOAD_LEN)) {
        return USB2P_STEP_BLOCKED;  /* tx_buf full; retry next boundary */
      }
      break;
    case USB2P_OPCODE_TIME:
      if (!usb2p_queue_time_rsp(pending_meta_space, pending_meta_txn_id,
                                pending_meta_seq)) {
        return USB2P_STEP_BLOCKED;
      }
      break;
    default:
      /* Should not happen — only interleavable opcodes are stashed. */
      if (!usb2p_queue_nak(USB2P_STATUS_EUNSUPPORTED,
                           pending_meta_txn_id, pending_meta_seq)) {
        return USB2P_STEP_BLOCKED;
      }
      break;
  }
  pending_meta = 0;
  return USB2P_STEP_PROGRESS;
}

/* One dispatcher step: emit at most one message in priority order. */
static enum usb2p_sched_result usb2p_sched_step(void) {
  enum usb2p_sched_result r;

  /* Priority 1: pending watch EVENT. */
  r = usb2p_sched_next_event();
  if (r != USB2P_STEP_IDLE) {
    return r;
  }

  /* Priority 2: pending bulk CANCEL — terminates the active op promptly so no
     further DATs are emitted after the cancel takes effect. */
  r = usb2p_sched_step_cancel();
  if (r != USB2P_STEP_IDLE) {
    if (r == USB2P_STEP_PROGRESS) usb2p_watch_burst_reset();
    return r;
  }

  /* Priority 3: pending metadata response (INFO/TIME stashed during a data op). */
  r = usb2p_sched_step_meta();
  if (r != USB2P_STEP_IDLE) {
    if (r == USB2P_STEP_PROGRESS) usb2p_watch_burst_reset();
    return r;
  }

  /* Priority 4: active transfer. */
  switch (pending_op_kind) {
    case USB2P_OP_WRITE:
      r = usb2p_sched_step_write();
      if (r == USB2P_STEP_PROGRESS) usb2p_watch_burst_reset();
      return r;
    case USB2P_OP_READ:
      r = usb2p_sched_step_read();
      if (r == USB2P_STEP_PROGRESS) usb2p_watch_burst_reset();
      return r;
    case USB2P_OP_FOPEN:
    case USB2P_OP_FREAD:
    case USB2P_OP_FWRITE:
    case USB2P_OP_FTRUNCATE:
    case USB2P_OP_FSYNC:
    case USB2P_OP_FCLOSE:
    case USB2P_OP_SETATTR:
    case USB2P_OP_SETTIMES:
    case USB2P_OP_MKDIR:
    case USB2P_OP_UNLINK:
    case USB2P_OP_RENAME:
      r = usb2p_sched_step_file();
      if (r == USB2P_STEP_PROGRESS) usb2p_watch_burst_reset();
      return r;
    case USB2P_OP_OPENDIR:
    case USB2P_OP_READDIR:
    case USB2P_OP_CLOSEDIR:
      r = usb2p_sched_step_dir();
      if (r == USB2P_STEP_PROGRESS) usb2p_watch_burst_reset();
      return r;
    case USB2P_OP_NONE:
    default:
      break;
  }

  /* Priority 5: idle. */
  return USB2P_STEP_IDLE;
}

/* Main-loop drain.  Runs with the USB IRQ ENABLED (usb2p_poll no longer masks
   it), so the SPI reads/writes in the steps overlap with USB packet shipping:
   while a step reads the next DAT chunk over SPI, IN-complete IRQs drain tx_buf
   to the host.  Concurrency:
   - tx_buf is guarded by the short critical section in usb2p_queue_frame.
   - READ: the framing core never mutates pending_op_* for an active READ (a new
     REQ just gets NAK(EBUSY)), so the READ state is single-threaded here.
   - WRITE: usb2p_handle_write_dat (ISR) and the commit step hand off via
     pending_op_write_state — the ISR only stages a chunk when state==AWAIT_DAT,
     the step only commits when state==COMMIT.  The byte-sized state writes are
     atomic on Cortex-M, so the handshake is race-free without masking.
   Drives the dispatcher until tx_buf is full (BLOCKED) or there is nothing to do
   (IDLE); the next IN-complete event re-enters us. */
void usb2p_drain_pending_op(void) {
  while (usb2p_sched_step() == USB2P_STEP_PROGRESS) {
    /* keep emitting: each step re-checks priorities (EVENT slot first) */
  }
}
