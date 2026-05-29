/* sd2snes - SD card based universal cartridge for the SNES
   Copyright (C) 2009-2010 Maximilian Rehkopf <otakon@gmx.net>

   USB2P v3 — internal interface shared between the protocol's implementation
   modules.  This header is NOT part of the public API (usb2p.h); it exists only
   so the framing core (usb2p.c) and the feature modules (codec, path, memory,
   filesystem, watch, scheduler) can share the small amount of state and the
   helper routines they pass between each other.

   The module split mirrors the wire protocol's structure:
     usb2p_codec.c  — byte I/O, header CRC, frame queue, tx_buf streaming
     usb2p_path.c   — UTF-8 <-> CP-1252 path transcode + normalization
     usb2p_mem.c    — memory spaces (SNES/MSU/CMD/CONFIG), segmented READ/WRITE
     usb2p_fs.c     — file + directory handles and their FatFs-backed ops
     usb2p_watch.c  — address-watch subscriptions and EVENT emission
     usb2p_sched.c  — the IN-endpoint priority dispatcher tying it together
     usb2p.c        — RX state machine, opcode dispatch, public entry points

   Single-op-slot model: at most one heavyweight (data-bearing) operation is
   pinned at a time via the pending_op_* block below.  HELLO advertises
   max_outstanding_txns=1, so the framing core NAK(EBUSY)s any overlap.  This is
   what lets all the deferred SPI/FatFs work run from the main-loop poll without
   locking against the USB ISR.
*/

#ifndef __USB2P_INTERNAL_H__
#define __USB2P_INTERNAL_H__

#include <stdint.h>

#include "fileops.h"   /* FIL, DIR, FILINFO, _MAX_LFN */
#include "timer.h"     /* tick_t */
#include "usb2p.h"

#define USB2P_SNES_SPACE_SIZE  0x1000000ull
#define USB2P_16BIT_SPACE_SIZE 0x10000ull

/* RX framing state machine (usb2p.c). */
enum usb2p_rx_state {
  USB2P_RX_SYNC = 0,
  USB2P_RX_HEADER,
  USB2P_RX_PAYLOAD,
  /* Full payload buffered, but dispatch is held because the previous WRITE
     chunk has not yet been committed to SPI.  No new bytes are consumed in
     this state — the USB OUT pipe naturally NAKs at the hardware level when
     the next EP buffer can't be accepted, back-pressuring the host without
     a logical NAK.  Cleared by usb2p_try_dispatch_held() from the main-loop
     poll once pending_op_write_state returns to AWAIT_DAT. */
  USB2P_RX_HELD
};

/* The deferred backend operation currently pinned (single slot).
   USB2P_OPCODE_READ/WRITE and every FS op touch the FPGA SPI bus or FatFs.
   Running those from the USB ISR races menu-loop SPI and wedges the bus, so the
   ISR only parses+validates+stashes the op here and the main-loop poll executes
   it.  HELLO advertises max_outstanding_txns=1 so any overlap is NAK(EBUSY). */
enum usb2p_op_kind {
  USB2P_OP_NONE = 0,
  USB2P_OP_READ,
  USB2P_OP_WRITE,
  USB2P_OP_FOPEN,
  USB2P_OP_FREAD,
  USB2P_OP_FWRITE,
  USB2P_OP_FTRUNCATE,
  USB2P_OP_FSYNC,
  USB2P_OP_FCLOSE,
  USB2P_OP_SETATTR,
  USB2P_OP_SETTIMES,
  USB2P_OP_MKDIR,
  USB2P_OP_UNLINK,
  USB2P_OP_RENAME,
  USB2P_OP_OPENDIR,
  USB2P_OP_READDIR,
  USB2P_OP_CLOSEDIR
};

/* WRITE is two-phase (README.usb2p.md §9): the REQ pins the op AWAIT_DAT, then
   each host DAT chunk is staged and committed (COMMIT) before the RSP (DONE). */
enum usb2p_write_state {
  USB2P_WRITE_AWAIT_DAT = 0, /* waiting for the next DAT-OUT chunk from host */
  USB2P_WRITE_COMMIT,        /* a DAT chunk is buffered in pending_op_chunk; commit + RSP if last */
  USB2P_WRITE_DONE           /* RSP queued (or pending); will clear pending_op */
};

/* The return contract of every scheduler step (usb2p_sched.c). */
enum usb2p_sched_result {
  USB2P_STEP_IDLE = 0,   /* nothing to do */
  USB2P_STEP_PROGRESS,   /* emitted a message / made committable progress */
  USB2P_STEP_BLOCKED     /* wanted to emit but tx_buf is full; retry later */
};

struct usb2p_file_handle {
  uint8_t active;
  uint8_t id;
  uint8_t mode;
  FIL fil;
};

/* Directory iteration (§10.2).  A dir handle pins a FatFs DIR across multiple
   READDIR calls; the cursor advances in-place, so the host re-issues READDIR to
   page through a directory.  Handles are force-closed on reset/disconnect.
   The LFN buffer is required by f_readdir to return long names. */
struct usb2p_dir_handle {
  uint8_t active;
  uint8_t id;
  DIR dir;
  char lfn[_MAX_LFN + 1];
};

/* An address-watch subscription (§6.5).  Polled by usb2p_poll_watches() in
   main-loop context so all FPGA SPI reads stay out of the ISR. */
struct usb2p_watch {
  uint8_t active;
  uint8_t id;
  uint8_t space;
  uint8_t length;
  uint8_t context_count;
  uint32_t offset;
  uint16_t interval_ms;
  uint32_t next_poll_cyc;   /* next poll deadline in DWT cycles (0 = not yet) */
  uint32_t event_seq;
  uint32_t pending_event_seq;
  uint32_t pending_event_tick;
  uint8_t pending_context_len;
  uint8_t baseline_valid;
  uint8_t pending_event;
  uint8_t pending_initial;
};

/* ===========================================================================
   Shared state.  Defined once in the listed owner module; declared extern here
   so the other modules can read/advance it.  Kept deliberately small: the
   single-op-slot model means at most one heavyweight op uses the pending_op_*
   and pending_file / pending_dir blocks at a time.
   =========================================================================== */

/* Connection state (usb2p.c): set by the HELLO handshake, cleared on
   reset/RESYNC.  Drives the per-opcode EBADSTATE gate and watch polling. */
extern uint8_t usb2p_attached;

/* Large CPU-only chunk buffers, shared across ops because only one op runs at a
   time.  read_payload doubles as the READDIR batch buffer (see dir_batch in
   usb2p_fs.c).  These are NOT in AHB SRAM: they alias the legacy USBA data
   buffers (recv/cmd/send) in main RAM via the union in usbinterface.c — the two
   protocols never run concurrently in a session.  Pointers (not arrays) for that
   reason; only ever indexed or offset, never sizeof/&. */
extern uint8_t * const read_payload;
extern uint8_t * const pending_op_chunk;

/* Scratch response-payload builders (codec/mem/fs share these; safe because a
   queued frame copies the payload before the next builder runs).  rsp_payload
   aliases meta_payload (the larger): the two never hold live data concurrently
   under the single-op-slot model, so they share one backing array. */
extern uint8_t meta_payload[USB2P_FOPEN_RSP_LEN];
#define rsp_payload meta_payload
extern uint8_t info_payload[USB2P_INFO_PAYLOAD_LEN];

/* Validated/transcoded path scratch (usb2p_path.c owns; fs/mem consume). */
extern char path_payload[USB2P_MAX_PATH_BYTES + 1];
extern char path_payload2[USB2P_MAX_PATH_BYTES + 1];

/* The pinned deferred op (usb2p_sched.c owns).  See enum usb2p_op_kind. */
extern enum usb2p_op_kind pending_op_kind;
extern uint8_t pending_op_space;
extern uint8_t pending_op_opcode;
extern uint32_t pending_op_txn_id;
extern uint32_t pending_op_seq;        /* base seq from the request (echoed in RSP) */
extern uint32_t pending_op_offset;     /* current SPI offset within the current segment */
extern uint32_t pending_op_remaining;  /* bytes still to transfer in the CURRENT segment */
extern uint32_t pending_op_total;      /* total length across all segments (for RSP) */

/* Segmented READ/WRITE scatter/gather cursor (§6.2.1). */
extern uint32_t pending_seg_offset[USB2P_MAX_SEGMENTS];
extern uint32_t pending_seg_length[USB2P_MAX_SEGMENTS];
extern uint8_t  pending_seg_count;
extern uint8_t  pending_seg_idx;       /* segment currently being transferred */
extern uint32_t pending_op_total_remaining; /* WRITE: bytes still expected across all segments */
extern uint32_t pending_op_next_seq;   /* next DAT seq (READ only) */
extern uint8_t  pending_op_rsp_sent;   /* READ: RSP queued.  WRITE: unused (state covers it). */
extern uint8_t  pending_op_write_status; /* WRITE: aggregate status (first non-OK wins) */
extern uint32_t pending_op_written;    /* WRITE: bytes successfully committed so far */
extern enum usb2p_write_state pending_op_write_state;
extern uint32_t pending_op_chunk_len;
extern uint8_t  pending_op_chunk_last; /* set when the buffered chunk carries LAST */

/* File-op staging (usb2p_fs.c owns).  pending_file_slot also carries the chosen
   dir slot index for OPENDIR/READDIR/CLOSEDIR steps. */
extern struct usb2p_file_handle file_handles[USB2P_MAX_OPEN_FILES];
extern uint8_t pending_file_slot;
extern uint8_t pending_file_mode;
extern uint8_t pending_file_rsp_sent;
extern uint8_t pending_file_complete;
extern uint8_t pending_file_status;
extern uint8_t pending_file_attr;
extern uint8_t pending_file_mask;
extern uint8_t pending_file_chunk_valid;
extern uint8_t pending_file_chunk_status;
extern uint16_t pending_file_chunk_flags;
extern uint32_t pending_file_offset;
extern uint32_t pending_file_length;
extern uint32_t pending_file_remaining;
extern uint32_t pending_file_next_seq;
extern uint32_t pending_file_result32;
extern uint32_t pending_file_mtime;
extern uint64_t pending_file_result64;

/* Directory iteration staging (usb2p_fs.c owns). */
extern struct usb2p_dir_handle dir_handles[USB2P_MAX_OPEN_DIRS];
extern uint32_t pending_dir_batch_len;
extern uint32_t pending_dir_max_bytes;
extern uint8_t  pending_dir_rsp_sent;
extern uint8_t  pending_dir_have_entry; /* a read-ahead entry is buffered */
extern uint8_t  pending_dir_end;        /* FatFs reported end-of-directory */
extern uint32_t pending_dir_next_seq;
extern FILINFO  pending_dir_fno;        /* lfname is bound to the handle's lfn */

/* Interleaved metadata response slot (usb2p_sched.c owns), §6.7. */
extern uint8_t  pending_meta;           /* 1 = slot occupied */
extern uint8_t  pending_meta_opcode;
extern uint8_t  pending_meta_space;
extern uint32_t pending_meta_txn_id;
extern uint32_t pending_meta_seq;

/* Bulk CANCEL slot (usb2p_sched.c owns), §6.7. */
extern uint8_t  pending_cancel;         /* 1 = slot occupied */
extern uint32_t pending_cancel_txn_id;
extern uint32_t pending_cancel_seq;

/* Deferred SNES command request (usb2p.c owns; drained by usb2p_take_command). */
extern int pending_cmd;

/* ===========================================================================
   Cross-module function interface.
   =========================================================================== */

/* usb2p_codec.c — byte I/O. */
uint16_t usb2p_rd16(const uint8_t *p);
uint32_t usb2p_rd32(const uint8_t *p);
uint64_t usb2p_rd64(const uint8_t *p);
void usb2p_wr16(uint8_t *p, uint16_t v);
void usb2p_wr32(uint8_t *p, uint32_t v);
void usb2p_wr64(uint8_t *p, uint64_t v);

/* usb2p_codec.c — header CRC + decode. */
uint16_t usb2p_crc16(const uint8_t *data, uint32_t len);
uint8_t  usb2p_header_magic_ok(const uint8_t *h);
uint8_t  usb2p_decode_header(const uint8_t *rx_header,
                             struct usb2p_header *out, uint8_t *trusted);

/* usb2p_codec.c — frame queueing into tx_buf. */
uint8_t usb2p_queue_frame(uint8_t type, uint8_t opcode, uint8_t space,
                          uint8_t status, uint16_t flags,
                          uint32_t txn_id, uint32_t seq,
                          const uint8_t *payload, uint32_t payload_len);
uint8_t usb2p_queue_header_only(uint8_t type, uint8_t opcode, uint8_t status,
                                uint32_t txn_id, uint32_t seq);
uint8_t usb2p_queue_rsp_empty(uint8_t opcode, uint8_t space, uint8_t status,
                              uint32_t txn_id, uint32_t seq);
uint8_t usb2p_queue_rsp_u64(uint8_t opcode, uint8_t space, uint8_t status,
                            uint32_t txn_id, uint32_t seq, uint64_t value);
uint8_t usb2p_queue_rsp_u32(uint8_t opcode, uint8_t space, uint8_t status,
                            uint32_t txn_id, uint32_t seq, uint32_t value);
uint8_t usb2p_queue_nak(uint8_t status, uint32_t txn_id, uint32_t seq);
void    usb2p_codec_reset(void);   /* clears tx_buf */

/* usb2p_path.c — validate + transcode + normalize a wire path.  Returns OK with
   dst filled, or EINVAL (structural) / EUNREPRESENTABLE (per §6.8). */
uint8_t usb2p_copy_path_status_to(char *dst, const uint8_t *payload,
                                  uint64_t length);
uint8_t usb2p_copy_path_status(const uint8_t *payload, uint64_t length);
uint32_t usb2p_fattime_from_filinfo(const FILINFO *fno);

/* usb2p_mem.c — memory spaces + segmented READ/WRITE. */
uint8_t  usb2p_memory_space_valid(uint8_t space);
uint8_t  usb2p_memory_range_ok(uint8_t space, uint64_t offset, uint64_t length);
uint32_t usb2p_memory_read(uint8_t space, uint32_t offset,
                           uint8_t *dst, uint32_t len);
void usb2p_handle_read(const struct usb2p_header *h, const uint8_t *payload);
void usb2p_handle_write(const struct usb2p_header *h, const uint8_t *payload);
void usb2p_handle_write_dat(const struct usb2p_header *h, const uint8_t *payload);
enum usb2p_sched_result usb2p_sched_step_read(void);
enum usb2p_sched_result usb2p_sched_step_write(void);

/* usb2p_fs.c — file/dir handles and ops. */
void usb2p_close_file_handles(void);
void usb2p_close_dir_handles(void);
uint8_t usb2p_file_handles_active(void);
void usb2p_handle_stat(const struct usb2p_header *h, const uint8_t *payload);
void usb2p_handle_statfs(const struct usb2p_header *h, const uint8_t *payload);
void usb2p_handle_fopen(const struct usb2p_header *h, const uint8_t *payload);
void usb2p_handle_fread(const struct usb2p_header *h, const uint8_t *payload);
void usb2p_handle_fwrite(const struct usb2p_header *h, const uint8_t *payload);
void usb2p_handle_ftruncate(const struct usb2p_header *h, const uint8_t *payload);
void usb2p_handle_file_handle_only(const struct usb2p_header *h,
                                   const uint8_t *payload,
                                   enum usb2p_op_kind kind);
void usb2p_handle_setattr(const struct usb2p_header *h, const uint8_t *payload);
void usb2p_handle_settimes(const struct usb2p_header *h, const uint8_t *payload);
void usb2p_handle_path_only_op(const struct usb2p_header *h,
                               const uint8_t *payload, enum usb2p_op_kind kind);
void usb2p_handle_rename(const struct usb2p_header *h, const uint8_t *payload);
void usb2p_handle_opendir(const struct usb2p_header *h, const uint8_t *payload);
void usb2p_handle_readdir(const struct usb2p_header *h, const uint8_t *payload);
void usb2p_handle_closedir(const struct usb2p_header *h, const uint8_t *payload);
enum usb2p_sched_result usb2p_sched_step_file(void);
enum usb2p_sched_result usb2p_sched_step_dir(void);

/* usb2p_watch.c — address watches + EVENT emission. */
void usb2p_clear_watches(void);
void usb2p_poll_watches(void);
enum usb2p_sched_result usb2p_handle_watch_add(const struct usb2p_header *h,
                                               const uint8_t *payload);
enum usb2p_sched_result usb2p_handle_watch_remove(const struct usb2p_header *h,
                                                  const uint8_t *payload);
enum usb2p_sched_result usb2p_handle_watch_clear(const struct usb2p_header *h);
enum usb2p_sched_result usb2p_sched_next_event(void);
void usb2p_watch_burst_reset(void);   /* clears the EVENT-burst counter */

/* usb2p_sched.c — the IN-endpoint priority dispatcher. */
void usb2p_drain_pending_op(void);

/* usb2p.c — INFO/TIME payload builders the scheduler's metadata step reuses. */
void usb2p_write_info_payload(uint8_t *payload);
uint8_t usb2p_queue_time_rsp(uint8_t space, uint32_t txn_id, uint32_t seq);

#endif /* __USB2P_INTERNAL_H__ */
