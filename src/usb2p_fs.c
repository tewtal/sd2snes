/* sd2snes - SD card based universal cartridge for the SNES
   Copyright (C) 2009-2010 Maximilian Rehkopf <otakon@gmx.net>

   USB2P v3 filesystem ops (README.usb2p.md §10).  Handle-based file I/O
   (FOPEN/FREAD/FWRITE/FTRUNCATE/FSYNC/FCLOSE), path ops (STAT/STATFS/SETATTR/
   SETTIMES/MKDIR/UNLINK/RENAME), and directory iteration (OPENDIR/READDIR/
   CLOSEDIR).  These are kept separate from memory READ/WRITE: a file is
   addressed by an open handle with its own offset domain and can short-read at
   EOF, whereas memory is {space, offset} with fixed extents.

   All FatFs calls touch the SD/SPI back-end, so the REQ handlers only validate
   and pin the op; the scheduler steps (usb2p_sched_step_file/_dir) run the
   actual FatFs work in main-loop context, one wire message per step.
*/

#include <stdint.h>
#include <string.h>

#include "fileops.h"
#include "usb2p.h"
#include "usb2p_internal.h"

extern FATFS fatfs;

/* Max bytes to extend a file per FTRUNCATE scheduler step.  Extending past EOF
   in write mode allocates the whole cluster chain (create_chain per cluster), so
   a multi-MB grow can take ~1 s — long enough that the client times out and the
   late RSP desyncs the stream.  Stepping the f_lseek target forward in bounded
   increments keeps each step short, so the main loop keeps spinning (USB stays
   alive, INFO/watches interleave) while the chain grows.  1 MB/step is a few
   tens of ms on SD — well under any sane client timeout. */
#define USB2P_FTRUNCATE_EXTEND_STEP 0x100000u

/* File-handle table.  max_open_files=1 today (mk2 budget), so the allocators
   below are trivial but written to scale if the cap grows. */
struct usb2p_file_handle file_handles[USB2P_MAX_OPEN_FILES];
static uint8_t file_next_id = 1;
uint8_t pending_file_slot;
uint8_t pending_file_mode;
uint8_t pending_file_rsp_sent;
uint8_t pending_file_complete;
uint8_t pending_file_status;
uint8_t pending_file_attr;
uint8_t pending_file_mask;
uint8_t pending_file_chunk_valid;
uint8_t pending_file_chunk_status;
uint16_t pending_file_chunk_flags;
uint32_t pending_file_offset;
uint32_t pending_file_length;
uint32_t pending_file_remaining;
uint32_t pending_file_next_seq;
uint32_t pending_file_result32;
uint32_t pending_file_mtime;
uint64_t pending_file_result64;

/* Directory-handle table + READDIR staging.  The batch reuses read_payload (the
   READ/FREAD DAT buffer) — a READDIR only runs while the single data-op slot is
   otherwise idle, so the two never overlap.  This avoids a second 1 KB AHB
   buffer (the region is near full on mk2). */
struct usb2p_dir_handle dir_handles[USB2P_MAX_OPEN_DIRS];
static uint8_t dir_next_id = 1;
#define dir_batch read_payload
uint32_t pending_dir_batch_len;
uint32_t pending_dir_max_bytes;
uint8_t  pending_dir_rsp_sent;
uint8_t  pending_dir_have_entry;
uint8_t  pending_dir_end;
uint32_t pending_dir_next_seq;
FILINFO  pending_dir_fno;          /* lfname is bound to the handle's lfn */

/* ---- FRESULT mapping -------------------------------------------------------*/

static uint8_t usb2p_fresult_status(FRESULT res) {
  switch (res) {
    case FR_OK:
      return USB2P_STATUS_OK;
    case FR_NO_FILE:
    case FR_NO_PATH:
      return USB2P_STATUS_ENOENT;
    case FR_DENIED:
    case FR_EXIST:
    case FR_WRITE_PROTECTED:
      return USB2P_STATUS_EACCES;
    case FR_LOCKED:
      return USB2P_STATUS_EBUSY;
    case FR_TOO_MANY_OPEN_FILES:
      return USB2P_STATUS_ENFILE;
    case FR_INVALID_NAME:
    case FR_INVALID_DRIVE:
    case FR_INVALID_PARAMETER:
      return USB2P_STATUS_EINVAL;
    case FR_INVALID_OBJECT:
      return USB2P_STATUS_EBADSTATE;
    case FR_DISK_ERR:
    case FR_INT_ERR:
    case FR_NOT_READY:
    case FR_NOT_ENABLED:
    case FR_NO_FILESYSTEM:
    case FR_MKFS_ABORTED:
    case FR_TIMEOUT:
    case FR_NOT_ENOUGH_CORE:
    default:
      return USB2P_STATUS_EIO;
  }
}

/* ---- handle tables ---------------------------------------------------------*/

void usb2p_close_file_handles(void) {
  uint8_t i;

  for (i = 0; i < USB2P_MAX_OPEN_FILES; i++) {
    if (file_handles[i].active) {
      (void)f_close(&file_handles[i].fil);
      file_handles[i].active = 0;
    }
  }
  file_next_id = 1;
}

void usb2p_close_dir_handles(void) {
  uint8_t i;

  for (i = 0; i < USB2P_MAX_OPEN_DIRS; i++) {
    if (dir_handles[i].active) {
      (void)f_closedir(&dir_handles[i].dir);
      dir_handles[i].active = 0;
    }
  }
  dir_next_id = 1;
}

static int8_t usb2p_find_dir_handle(uint32_t handle) {
  uint8_t i;

  if (handle == 0 || handle > 255) {
    return -1;
  }
  for (i = 0; i < USB2P_MAX_OPEN_DIRS; i++) {
    if (dir_handles[i].active && dir_handles[i].id == (uint8_t)handle) {
      return (int8_t)i;
    }
  }
  return -1;
}

static uint8_t usb2p_alloc_dir_slot(void) {
  uint8_t i;

  for (i = 0; i < USB2P_MAX_OPEN_DIRS; i++) {
    if (!dir_handles[i].active) {
      return i;
    }
  }
  return USB2P_MAX_OPEN_DIRS;
}

static uint8_t usb2p_alloc_dir_id(void) {
  uint8_t tries;
  uint8_t id;

  for (tries = 0; tries < 255; tries++) {
    id = dir_next_id++;
    if (dir_next_id == 0) dir_next_id = 1;
    if (usb2p_find_dir_handle(id) < 0) {
      return id;
    }
  }
  return 0;
}

uint8_t usb2p_file_handles_active(void) {
  uint8_t i;

  for (i = 0; i < USB2P_MAX_OPEN_FILES; i++) {
    if (file_handles[i].active) {
      return 1;
    }
  }
  for (i = 0; i < USB2P_MAX_OPEN_DIRS; i++) {
    if (dir_handles[i].active) {
      return 1;
    }
  }
  return 0;
}

static uint8_t usb2p_alloc_file_slot(void) {
  uint8_t i;

  for (i = 0; i < USB2P_MAX_OPEN_FILES; i++) {
    if (!file_handles[i].active) {
      return i;
    }
  }
  return USB2P_MAX_OPEN_FILES;
}

static int8_t usb2p_find_file_handle(uint32_t handle) {
  uint8_t i;

  if (handle == 0 || handle > 255) {
    return -1;
  }
  for (i = 0; i < USB2P_MAX_OPEN_FILES; i++) {
    if (file_handles[i].active && file_handles[i].id == (uint8_t)handle) {
      return (int8_t)i;
    }
  }
  return -1;
}

static uint8_t usb2p_alloc_file_id(void) {
  uint8_t tries;
  uint8_t id;

  for (tries = 0; tries < 255; tries++) {
    id = file_next_id++;
    if (file_next_id == 0) file_next_id = 1;
    if (usb2p_find_file_handle(id) < 0) {
      return id;
    }
  }
  return 0;
}

static uint8_t usb2p_fopen_mode_valid(uint8_t mode, uint8_t share) {
  uint8_t create_bits = mode & (FA_CREATE_NEW | FA_CREATE_ALWAYS | FA_OPEN_ALWAYS);
  uint8_t create_count = 0;

  if (share & ~0x03u) {
    return 0;
  }
  if (mode & ~(FA_READ | FA_WRITE | FA_CREATE_NEW
               | FA_CREATE_ALWAYS | FA_OPEN_ALWAYS)) {
    return 0;
  }
  if (!(mode & (FA_READ | FA_WRITE))) {
    return 0;
  }
  if (create_bits && !(mode & FA_WRITE)) {
    return 0;
  }
  if (create_bits & FA_CREATE_NEW) create_count++;
  if (create_bits & FA_CREATE_ALWAYS) create_count++;
  if (create_bits & FA_OPEN_ALWAYS) create_count++;
  return create_count <= 1;
}

/* ---- response builders -----------------------------------------------------*/

static uint8_t usb2p_queue_fopen_rsp(uint8_t status, uint32_t txn_id,
                                     uint32_t seq, uint32_t handle,
                                     uint64_t size, uint32_t mtime,
                                     uint8_t attr) {
  if (status != USB2P_STATUS_OK) {
    return usb2p_queue_rsp_empty(USB2P_OPCODE_FOPEN, USB2P_SPACE_FILE,
                                 status, txn_id, seq);
  }
  usb2p_wr32(meta_payload + 0, handle);
  usb2p_wr64(meta_payload + 4, size);
  usb2p_wr32(meta_payload + 12, mtime);
  meta_payload[16] = attr;
  meta_payload[17] = 0;
  meta_payload[18] = 0;
  meta_payload[19] = 0;
  return usb2p_queue_frame(USB2P_TYPE_RSP, USB2P_OPCODE_FOPEN,
                           USB2P_SPACE_FILE, USB2P_STATUS_OK, 0,
                           txn_id, seq, meta_payload, USB2P_FOPEN_RSP_LEN);
}

/* ---- STAT / STATFS ---------------------------------------------------------*/

void usb2p_handle_stat(const struct usb2p_header *h, const uint8_t *payload) {
  FILINFO fno;
  FRESULT res;
  uint8_t status;

  status = usb2p_copy_path_status(payload, h->length);
  if (status == USB2P_STATUS_EINVAL) {
    usb2p_queue_nak(status, h->txn_id, h->seq);
    return;
  }
  if (status != USB2P_STATUS_OK) {
    usb2p_queue_rsp_empty(h->opcode, h->space, status, h->txn_id, h->seq);
    return;
  }
#if _USE_LFN
  fno.lfname = 0;
#endif
  res = f_stat(path_payload, &fno);
  status = usb2p_fresult_status(res);
  if (status == USB2P_STATUS_OK) {
    usb2p_wr64(meta_payload + 0, fno.fsize);
    usb2p_wr32(meta_payload + 8, usb2p_fattime_from_filinfo(&fno));
    meta_payload[12] = fno.fattrib;
    meta_payload[13] = 0;
    meta_payload[14] = 0;
    meta_payload[15] = 0;
    usb2p_queue_frame(USB2P_TYPE_RSP, h->opcode, h->space, status, 0,
                      h->txn_id, h->seq, meta_payload, USB2P_STAT_RSP_LEN);
    return;
  }
  usb2p_queue_rsp_empty(h->opcode, h->space, status, h->txn_id, h->seq);
}

void usb2p_handle_statfs(const struct usb2p_header *h, const uint8_t *payload) {
  FATFS *ffs = &fatfs;
  DWORD free_clusters;
  FRESULT res;
  uint8_t status;
  const char *path = "0:";

  if (h->length) {
    status = usb2p_copy_path_status(payload, h->length);
    if (status == USB2P_STATUS_EINVAL) {
      usb2p_queue_nak(status, h->txn_id, h->seq);
      return;
    }
    if (status != USB2P_STATUS_OK) {
      usb2p_queue_rsp_empty(h->opcode, h->space, status, h->txn_id, h->seq);
      return;
    }
    path = path_payload;
  }

  res = f_getfree(path, &free_clusters, &ffs);
  status = usb2p_fresult_status(res);
  if (status == USB2P_STATUS_OK) {
    uint64_t cluster_bytes = (uint64_t)ffs->csize * 512ull;
    uint64_t total = ((uint64_t)ffs->n_fatent - 2ull) * cluster_bytes;
    uint64_t free = (uint64_t)free_clusters * cluster_bytes;
    usb2p_wr64(meta_payload + 0, total);
    usb2p_wr64(meta_payload + 8, free);
    usb2p_queue_frame(USB2P_TYPE_RSP, h->opcode, h->space, status, 0,
                      h->txn_id, h->seq, meta_payload, USB2P_STATFS_RSP_LEN);
    return;
  }
  usb2p_queue_rsp_empty(h->opcode, h->space, status, h->txn_id, h->seq);
}

/* ---- file-op REQ handlers --------------------------------------------------*/

void usb2p_handle_fopen(const struct usb2p_header *h, const uint8_t *payload) {
  uint64_t path_len;
  uint8_t mode;
  uint8_t share;
  uint8_t slot;
  uint8_t path_status;

  if (pending_op_kind != USB2P_OP_NONE) {
    usb2p_queue_nak(USB2P_STATUS_EBUSY, h->txn_id, h->seq);
    return;
  }
  if (h->length <= USB2P_FOPEN_REQ_HDR_LEN) {
    usb2p_queue_nak(USB2P_STATUS_EINVAL, h->txn_id, h->seq);
    return;
  }
  mode = payload[0];
  share = payload[1];
  if (usb2p_rd16(payload + 2) != 0 || !usb2p_fopen_mode_valid(mode, share)) {
    usb2p_queue_nak(USB2P_STATUS_EINVAL, h->txn_id, h->seq);
    return;
  }
  path_len = h->length - USB2P_FOPEN_REQ_HDR_LEN;
  path_status = usb2p_copy_path_status(payload + USB2P_FOPEN_REQ_HDR_LEN,
                                       path_len);
  if (path_status == USB2P_STATUS_EINVAL) {
    usb2p_queue_nak(path_status, h->txn_id, h->seq);
    return;
  }
  if (path_status != USB2P_STATUS_OK) {
    usb2p_queue_rsp_empty(h->opcode, h->space, path_status, h->txn_id, h->seq);
    return;
  }
  slot = usb2p_alloc_file_slot();
  if (slot >= USB2P_MAX_OPEN_FILES) {
    usb2p_queue_rsp_empty(h->opcode, h->space, USB2P_STATUS_ENFILE,
                          h->txn_id, h->seq);
    return;
  }

  pending_file_slot = slot;
  pending_file_mode = mode;
  pending_file_complete = 0;
  pending_op_kind = USB2P_OP_FOPEN;
  pending_op_space = USB2P_SPACE_FILE;
  pending_op_opcode = h->opcode;
  pending_op_txn_id = h->txn_id;
  pending_op_seq = h->seq;
}

void usb2p_handle_fread(const struct usb2p_header *h, const uint8_t *payload) {
  uint32_t handle;
  uint64_t offset;
  uint64_t length;
  uint32_t available;
  int8_t slot;

  if (pending_op_kind != USB2P_OP_NONE) {
    usb2p_queue_nak(USB2P_STATUS_EBUSY, h->txn_id, h->seq);
    return;
  }
  if (h->length != USB2P_FILE_IO_REQ_LEN || usb2p_rd32(payload + 4) != 0) {
    usb2p_queue_nak(USB2P_STATUS_EINVAL, h->txn_id, h->seq);
    return;
  }
  handle = usb2p_rd32(payload + 0);
  offset = usb2p_rd64(payload + 8);
  length = usb2p_rd64(payload + 16);
  slot = usb2p_find_file_handle(handle);
  if (slot < 0) {
    usb2p_queue_rsp_u64(h->opcode, h->space, USB2P_STATUS_EBADF,
                        h->txn_id, h->seq, 0);
    return;
  }
  if (!(file_handles[(uint8_t)slot].mode & FA_READ)) {
    usb2p_queue_rsp_u64(h->opcode, h->space, USB2P_STATUS_EACCES,
                        h->txn_id, h->seq, 0);
    return;
  }
  if (offset > UINT32_MAX || length > UINT32_MAX) {
    usb2p_queue_rsp_u64(h->opcode, h->space, USB2P_STATUS_ERANGE,
                        h->txn_id, h->seq, 0);
    return;
  }

  available = 0;
  if ((uint32_t)offset < file_handles[(uint8_t)slot].fil.fsize) {
    available = file_handles[(uint8_t)slot].fil.fsize - (uint32_t)offset;
    if (available > (uint32_t)length) {
      available = (uint32_t)length;
    }
  }

  pending_file_slot = (uint8_t)slot;
  pending_file_offset = (uint32_t)offset;
  pending_file_length = available;
  pending_file_remaining = available;
  pending_file_next_seq = h->seq + 1;
  pending_file_rsp_sent = 0;
  pending_file_chunk_valid = 0;
  pending_op_kind = USB2P_OP_FREAD;
  pending_op_space = USB2P_SPACE_FILE;
  pending_op_opcode = h->opcode;
  pending_op_txn_id = h->txn_id;
  pending_op_seq = h->seq;
}

void usb2p_handle_fwrite(const struct usb2p_header *h, const uint8_t *payload) {
  uint32_t handle;
  uint64_t offset;
  uint64_t length;
  int8_t slot;

  if (pending_op_kind != USB2P_OP_NONE) {
    usb2p_queue_nak(USB2P_STATUS_EBUSY, h->txn_id, h->seq);
    return;
  }
  if (h->length < USB2P_FILE_IO_REQ_LEN || usb2p_rd32(payload + 4) != 0) {
    usb2p_queue_nak(USB2P_STATUS_EINVAL, h->txn_id, h->seq);
    return;
  }
  handle = usb2p_rd32(payload + 0);
  offset = usb2p_rd64(payload + 8);
  length = usb2p_rd64(payload + 16);
  if (length != h->length - USB2P_FILE_IO_REQ_LEN
      || length > USB2P_FWRITE_MAX_INLINE) {
    usb2p_queue_nak(USB2P_STATUS_EINVAL, h->txn_id, h->seq);
    return;
  }
  slot = usb2p_find_file_handle(handle);
  if (slot < 0) {
    usb2p_queue_rsp_u64(h->opcode, h->space, USB2P_STATUS_EBADF,
                        h->txn_id, h->seq, 0);
    return;
  }
  if (!(file_handles[(uint8_t)slot].mode & FA_WRITE)) {
    usb2p_queue_rsp_u64(h->opcode, h->space, USB2P_STATUS_EACCES,
                        h->txn_id, h->seq, 0);
    return;
  }
  if (offset > UINT32_MAX || length > UINT32_MAX) {
    usb2p_queue_rsp_u64(h->opcode, h->space, USB2P_STATUS_ERANGE,
                        h->txn_id, h->seq, 0);
    return;
  }
  if (length) {
    memcpy(pending_op_chunk, payload + USB2P_FILE_IO_REQ_LEN, (uint32_t)length);
  }

  pending_file_slot = (uint8_t)slot;
  pending_file_offset = (uint32_t)offset;
  pending_file_length = (uint32_t)length;
  pending_file_complete = 0;
  pending_op_kind = USB2P_OP_FWRITE;
  pending_op_space = USB2P_SPACE_FILE;
  pending_op_opcode = h->opcode;
  pending_op_txn_id = h->txn_id;
  pending_op_seq = h->seq;
}

void usb2p_handle_ftruncate(const struct usb2p_header *h,
                            const uint8_t *payload) {
  uint32_t handle;
  uint64_t size;
  int8_t slot;

  if (pending_op_kind != USB2P_OP_NONE) {
    usb2p_queue_nak(USB2P_STATUS_EBUSY, h->txn_id, h->seq);
    return;
  }
  if (h->length != USB2P_FTRUNCATE_REQ_LEN || usb2p_rd32(payload + 4) != 0) {
    usb2p_queue_nak(USB2P_STATUS_EINVAL, h->txn_id, h->seq);
    return;
  }
  handle = usb2p_rd32(payload + 0);
  size = usb2p_rd64(payload + 8);
  slot = usb2p_find_file_handle(handle);
  if (slot < 0) {
    usb2p_queue_rsp_empty(h->opcode, h->space, USB2P_STATUS_EBADF,
                          h->txn_id, h->seq);
    return;
  }
  if (!(file_handles[(uint8_t)slot].mode & FA_WRITE)) {
    usb2p_queue_rsp_empty(h->opcode, h->space, USB2P_STATUS_EACCES,
                          h->txn_id, h->seq);
    return;
  }
  if (size > UINT32_MAX) {
    usb2p_queue_rsp_empty(h->opcode, h->space, USB2P_STATUS_ERANGE,
                          h->txn_id, h->seq);
    return;
  }

  pending_file_slot = (uint8_t)slot;
  pending_file_length = (uint32_t)size;
  pending_file_complete = 0;
  pending_op_kind = USB2P_OP_FTRUNCATE;
  pending_op_space = USB2P_SPACE_FILE;
  pending_op_opcode = h->opcode;
  pending_op_txn_id = h->txn_id;
  pending_op_seq = h->seq;
}

void usb2p_handle_file_handle_only(const struct usb2p_header *h,
                                   const uint8_t *payload,
                                   enum usb2p_op_kind kind) {
  uint32_t handle;
  int8_t slot;

  if (pending_op_kind != USB2P_OP_NONE) {
    usb2p_queue_nak(USB2P_STATUS_EBUSY, h->txn_id, h->seq);
    return;
  }
  if (h->length != USB2P_FILE_HANDLE_LEN) {
    usb2p_queue_nak(USB2P_STATUS_EINVAL, h->txn_id, h->seq);
    return;
  }
  handle = usb2p_rd32(payload);
  slot = usb2p_find_file_handle(handle);
  if (slot < 0) {
    usb2p_queue_rsp_empty(h->opcode, h->space, USB2P_STATUS_EBADF,
                          h->txn_id, h->seq);
    return;
  }

  pending_file_slot = (uint8_t)slot;
  pending_file_complete = 0;
  pending_op_kind = kind;
  pending_op_space = USB2P_SPACE_FILE;
  pending_op_opcode = h->opcode;
  pending_op_txn_id = h->txn_id;
  pending_op_seq = h->seq;
}

/* ---- path-op helpers -------------------------------------------------------*/

static uint8_t usb2p_mutating_path_op_busy(const struct usb2p_header *h) {
  if (pending_op_kind != USB2P_OP_NONE) {
    usb2p_queue_nak(USB2P_STATUS_EBUSY, h->txn_id, h->seq);
    return 1;
  }
  if (usb2p_file_handles_active()) {
    usb2p_queue_rsp_empty(h->opcode, h->space, USB2P_STATUS_EBUSY,
                          h->txn_id, h->seq);
    return 1;
  }
  return 0;
}

static void usb2p_start_path_op(const struct usb2p_header *h,
                                enum usb2p_op_kind kind) {
  pending_file_complete = 0;
  pending_op_kind = kind;
  pending_op_space = USB2P_SPACE_FILE;
  pending_op_opcode = h->opcode;
  pending_op_txn_id = h->txn_id;
  pending_op_seq = h->seq;
}

/* Copy + transcode a wire path into dst and, on failure, emit the response the
   §6.2/§6.3 taxonomy requires: structural fault -> NAK(EINVAL); a well-formed
   but unrepresentable/escaping path -> RSP(EUNREPRESENTABLE).  Returns OK on
   success so callers can `if (status != OK) return;`. */
static uint8_t usb2p_copy_path_or_respond(const struct usb2p_header *h,
                                          char *dst, const uint8_t *payload,
                                          uint64_t length) {
  uint8_t status = usb2p_copy_path_status_to(dst, payload, length);

  if (status == USB2P_STATUS_EINVAL) {
    usb2p_queue_nak(status, h->txn_id, h->seq);
  } else if (status != USB2P_STATUS_OK) {
    usb2p_queue_rsp_empty(h->opcode, h->space, status, h->txn_id, h->seq);
  }
  return status;
}

void usb2p_handle_setattr(const struct usb2p_header *h, const uint8_t *payload) {
  if (usb2p_mutating_path_op_busy(h)) {
    return;
  }
  if (h->length <= USB2P_SETATTR_REQ_HDR_LEN || usb2p_rd16(payload + 2) != 0) {
    usb2p_queue_nak(USB2P_STATUS_EINVAL, h->txn_id, h->seq);
    return;
  }
  if (usb2p_copy_path_or_respond(h, path_payload,
                                 payload + USB2P_SETATTR_REQ_HDR_LEN,
                                 h->length - USB2P_SETATTR_REQ_HDR_LEN)
      != USB2P_STATUS_OK) {
    return;
  }
  pending_file_attr = payload[0];
  pending_file_mask = payload[1];
  usb2p_start_path_op(h, USB2P_OP_SETATTR);
}

void usb2p_handle_settimes(const struct usb2p_header *h, const uint8_t *payload) {
  if (usb2p_mutating_path_op_busy(h)) {
    return;
  }
  if (h->length <= USB2P_SETTIMES_REQ_HDR_LEN || usb2p_rd32(payload + 4) != 0) {
    usb2p_queue_nak(USB2P_STATUS_EINVAL, h->txn_id, h->seq);
    return;
  }
  if (usb2p_copy_path_or_respond(h, path_payload,
                                 payload + USB2P_SETTIMES_REQ_HDR_LEN,
                                 h->length - USB2P_SETTIMES_REQ_HDR_LEN)
      != USB2P_STATUS_OK) {
    return;
  }
  pending_file_mtime = usb2p_rd32(payload);
  usb2p_start_path_op(h, USB2P_OP_SETTIMES);
}

void usb2p_handle_path_only_op(const struct usb2p_header *h,
                               const uint8_t *payload, enum usb2p_op_kind kind) {
  if (usb2p_mutating_path_op_busy(h)) {
    return;
  }
  if (usb2p_copy_path_or_respond(h, path_payload, payload, h->length)
      != USB2P_STATUS_OK) {
    return;
  }
  usb2p_start_path_op(h, kind);
}

void usb2p_handle_rename(const struct usb2p_header *h, const uint8_t *payload) {
  uint16_t old_len;
  uint16_t new_len;

  if (usb2p_mutating_path_op_busy(h)) {
    return;
  }
  if (h->length < USB2P_RENAME_REQ_HDR_LEN) {
    usb2p_queue_nak(USB2P_STATUS_EINVAL, h->txn_id, h->seq);
    return;
  }
  old_len = usb2p_rd16(payload + 0);
  new_len = usb2p_rd16(payload + 2);
  if (old_len == 0 || new_len == 0
      || h->length != (uint64_t)USB2P_RENAME_REQ_HDR_LEN + old_len + new_len) {
    usb2p_queue_nak(USB2P_STATUS_EINVAL, h->txn_id, h->seq);
    return;
  }
  if (usb2p_copy_path_or_respond(h, path_payload,
                                 payload + USB2P_RENAME_REQ_HDR_LEN, old_len)
      != USB2P_STATUS_OK) {
    return;
  }
  if (usb2p_copy_path_or_respond(h, path_payload2,
                                 payload + USB2P_RENAME_REQ_HDR_LEN + old_len,
                                 new_len) != USB2P_STATUS_OK) {
    return;
  }
  usb2p_start_path_op(h, USB2P_OP_RENAME);
}

/* ---- directory REQ handlers ------------------------------------------------*/

void usb2p_handle_opendir(const struct usb2p_header *h, const uint8_t *payload) {
  uint8_t slot;

  if (pending_op_kind != USB2P_OP_NONE) {
    usb2p_queue_nak(USB2P_STATUS_EBUSY, h->txn_id, h->seq);
    return;
  }
  if (usb2p_copy_path_or_respond(h, path_payload, payload, h->length)
      != USB2P_STATUS_OK) {
    return;
  }
  slot = usb2p_alloc_dir_slot();
  if (slot >= USB2P_MAX_OPEN_DIRS) {
    usb2p_queue_rsp_empty(h->opcode, h->space, USB2P_STATUS_ENFILE,
                          h->txn_id, h->seq);
    return;
  }
  /* path_payload holds the validated path; the scheduler runs f_opendir (SPI)
     and emits RSP{dir_handle}.  pending_file_slot reuses the file-op slot field
     to carry the chosen dir slot index into the scheduler step. */
  pending_file_slot = slot;
  pending_file_complete = 0;
  pending_op_kind = USB2P_OP_OPENDIR;
  pending_op_space = USB2P_SPACE_FILE;
  pending_op_opcode = h->opcode;
  pending_op_txn_id = h->txn_id;
  pending_op_seq = h->seq;
}

void usb2p_handle_readdir(const struct usb2p_header *h, const uint8_t *payload) {
  uint32_t handle;
  uint32_t max_bytes;
  int8_t slot;

  if (pending_op_kind != USB2P_OP_NONE) {
    usb2p_queue_nak(USB2P_STATUS_EBUSY, h->txn_id, h->seq);
    return;
  }
  if (h->length != USB2P_READDIR_REQ_LEN) {
    usb2p_queue_nak(USB2P_STATUS_EINVAL, h->txn_id, h->seq);
    return;
  }
  handle = usb2p_rd32(payload + 0);
  max_bytes = usb2p_rd32(payload + 4);
  slot = usb2p_find_dir_handle(handle);
  if (slot < 0) {
    usb2p_queue_rsp_u32(h->opcode, h->space, USB2P_STATUS_EBADF,
                        h->txn_id, h->seq, 0);
    return;
  }
  if (max_bytes > USB2P_MAX_TX_DAT_PAYLOAD) {
    max_bytes = USB2P_MAX_TX_DAT_PAYLOAD;
  }
  /* A batch must be able to hold at least one max-length entry, else a long
     name would never fit and the host would spin.  Reject a too-small cap. */
  if (max_bytes < USB2P_READDIR_ENTRY_HDR_LEN + USB2P_MAX_PATH_BYTES) {
    max_bytes = USB2P_READDIR_ENTRY_HDR_LEN + USB2P_MAX_PATH_BYTES;
    if (max_bytes > USB2P_MAX_TX_DAT_PAYLOAD) {
      max_bytes = USB2P_MAX_TX_DAT_PAYLOAD;
    }
  }

  pending_file_slot = (uint8_t)slot;
  pending_dir_max_bytes = max_bytes;
  pending_dir_rsp_sent = 0;
  pending_dir_batch_len = 0;
  pending_dir_end = 0;
  pending_dir_next_seq = h->seq + 1;
  pending_op_kind = USB2P_OP_READDIR;
  pending_op_space = USB2P_SPACE_FILE;
  pending_op_opcode = h->opcode;
  pending_op_txn_id = h->txn_id;
  pending_op_seq = h->seq;
}

void usb2p_handle_closedir(const struct usb2p_header *h, const uint8_t *payload) {
  uint32_t handle;
  int8_t slot;

  if (pending_op_kind != USB2P_OP_NONE) {
    usb2p_queue_nak(USB2P_STATUS_EBUSY, h->txn_id, h->seq);
    return;
  }
  if (h->length != USB2P_FILE_HANDLE_LEN) {
    usb2p_queue_nak(USB2P_STATUS_EINVAL, h->txn_id, h->seq);
    return;
  }
  handle = usb2p_rd32(payload);
  slot = usb2p_find_dir_handle(handle);
  if (slot < 0) {
    usb2p_queue_rsp_empty(h->opcode, h->space, USB2P_STATUS_EBADF,
                          h->txn_id, h->seq);
    return;
  }

  pending_file_slot = (uint8_t)slot;
  pending_file_complete = 0;
  pending_op_kind = USB2P_OP_CLOSEDIR;
  pending_op_space = USB2P_SPACE_FILE;
  pending_op_opcode = h->opcode;
  pending_op_txn_id = h->txn_id;
  pending_op_seq = h->seq;
}

/* ---- file scheduler step ---------------------------------------------------*/

enum usb2p_sched_result usb2p_sched_step_file(void) {
  struct usb2p_file_handle *fh = &file_handles[pending_file_slot];
  FRESULT res;
  uint8_t status;

  switch (pending_op_kind) {
    case USB2P_OP_FOPEN: {
      FILINFO fno;
      uint8_t id;

      if (!pending_file_complete) {
        res = f_open(&fh->fil, path_payload, pending_file_mode);
        status = usb2p_fresult_status(res);
        pending_file_status = status;
        pending_file_result32 = 0;
        pending_file_result64 = 0;
        pending_file_mtime = 0;
        pending_file_attr = 0;
        if (status == USB2P_STATUS_OK) {
          id = usb2p_alloc_file_id();
          if (!id) {
            (void)f_close(&fh->fil);
            pending_file_status = USB2P_STATUS_ENFILE;
          } else {
            fh->active = 1;
            fh->id = id;
            fh->mode = pending_file_mode;
            pending_file_result32 = id;
            pending_file_result64 = fh->fil.fsize;
#if _USE_LFN
            fno.lfname = 0;
#endif
            if (f_stat(path_payload, &fno) == FR_OK) {
              pending_file_mtime = usb2p_fattime_from_filinfo(&fno);
              pending_file_attr = fno.fattrib;
            }
          }
        }
        pending_file_complete = 1;
      }
      if (!usb2p_queue_fopen_rsp(pending_file_status, pending_op_txn_id,
                                 pending_op_seq, pending_file_result32,
                                 pending_file_result64, pending_file_mtime,
                                 pending_file_attr)) {
        return USB2P_STEP_BLOCKED;
      }
      pending_op_kind = USB2P_OP_NONE;
      return USB2P_STEP_PROGRESS;
    }

    case USB2P_OP_FREAD:
      if (!pending_file_rsp_sent) {
        res = f_lseek(&fh->fil, (DWORD)pending_file_offset);
        status = usb2p_fresult_status(res);
        if (status == USB2P_STATUS_OK
            && fh->fil.fptr != (DWORD)pending_file_offset) {
          status = USB2P_STATUS_ERANGE;
        }
        if (!usb2p_queue_rsp_u64(USB2P_OPCODE_FREAD, USB2P_SPACE_FILE,
                                 status, pending_op_txn_id, pending_op_seq,
                                 status == USB2P_STATUS_OK
                                   ? pending_file_length : 0)) {
          return USB2P_STEP_BLOCKED;
        }
        pending_file_rsp_sent = 1;
        if (status != USB2P_STATUS_OK || pending_file_remaining == 0) {
          pending_op_kind = USB2P_OP_NONE;
        }
        return USB2P_STEP_PROGRESS;
      } else {
        if (!pending_file_chunk_valid) {
          UINT bytes_read = 0;
          UINT request = pending_file_remaining;

          if (request > USB2P_MAX_TX_DAT_PAYLOAD) {
            request = USB2P_MAX_TX_DAT_PAYLOAD;
          }
          res = f_read(&fh->fil, read_payload, request, &bytes_read);
          pending_file_chunk_status = usb2p_fresult_status(res);
          pending_file_result32 = bytes_read;
          pending_file_chunk_flags =
              (pending_file_chunk_status != USB2P_STATUS_OK
               || bytes_read >= pending_file_remaining
               || bytes_read < request) ? USB2P_FLAG_LAST : 0;
          pending_file_chunk_valid = 1;
        }
        if (!usb2p_queue_frame(USB2P_TYPE_DAT, USB2P_OPCODE_FREAD,
                               USB2P_SPACE_FILE, pending_file_chunk_status,
                               pending_file_chunk_flags,
                               pending_op_txn_id, pending_file_next_seq,
                               read_payload,
                               pending_file_chunk_status == USB2P_STATUS_OK
                                 ? pending_file_result32 : 0)) {
          return USB2P_STEP_BLOCKED;
        }
        pending_file_next_seq++;
        pending_file_chunk_valid = 0;
        if (pending_file_chunk_status != USB2P_STATUS_OK
            || pending_file_chunk_flags || pending_file_result32 == 0) {
          pending_op_kind = USB2P_OP_NONE;
        } else {
          pending_file_remaining -= pending_file_result32;
        }
        return USB2P_STEP_PROGRESS;
      }

    case USB2P_OP_FWRITE: {
      UINT bytes_written = 0;

      if (!pending_file_complete) {
        res = f_lseek(&fh->fil, (DWORD)pending_file_offset);
        status = usb2p_fresult_status(res);
        if (status == USB2P_STATUS_OK
            && fh->fil.fptr != (DWORD)pending_file_offset) {
          status = USB2P_STATUS_ERANGE;
        }
        if (status == USB2P_STATUS_OK) {
          res = f_write(&fh->fil, pending_op_chunk, pending_file_length,
                        &bytes_written);
          status = usb2p_fresult_status(res);
        }
        pending_file_status = status;
        pending_file_result64 = bytes_written;
        pending_file_complete = 1;
      }
      if (!usb2p_queue_rsp_u64(USB2P_OPCODE_FWRITE, USB2P_SPACE_FILE,
                               pending_file_status, pending_op_txn_id,
                               pending_op_seq, pending_file_result64)) {
        return USB2P_STEP_BLOCKED;
      }
      pending_op_kind = USB2P_OP_NONE;
      return USB2P_STEP_PROGRESS;
    }

    case USB2P_OP_FTRUNCATE:
      if (!pending_file_complete) {
        DWORD target = (DWORD)pending_file_length;
        DWORD next;

        /* Step the file pointer toward the target in bounded increments,
           yielding between steps.  Extending past EOF (write mode) allocates the
           cluster chain in f_lseek — create_chain per cluster — so a multi-MB
           grow can take ~1 s; doing it all in one step blocks the main loop long
           enough that the client times out and the late RSP desyncs the stream.
           Each intermediate seek grows fsize and leaves fptr at the new offset,
           so fptr is its own progress cursor across re-entries.  A shrink
           (target <= fptr) seeks straight to target in one step (cheap). */
        next = fh->fil.fptr;
        if (target > next) {
          DWORD remaining = target - next;
          if (remaining > USB2P_FTRUNCATE_EXTEND_STEP) {
            remaining = USB2P_FTRUNCATE_EXTEND_STEP;
          }
          next += remaining;
        } else {
          next = target;
        }

        res = f_lseek(&fh->fil, next);
        status = usb2p_fresult_status(res);
        if (status != USB2P_STATUS_OK) {
          pending_file_status = status;
          pending_file_complete = 1;
        } else if (fh->fil.fptr != next) {
          /* Disk full / clipped before the requested offset. */
          pending_file_status = USB2P_STATUS_ERANGE;
          pending_file_complete = 1;
        } else if (fh->fil.fptr < target) {
          /* More to grow: yield and resume on the next scheduler step. */
          return USB2P_STEP_PROGRESS;
        } else {
          status = usb2p_fresult_status(f_truncate(&fh->fil));
          pending_file_status = status;
          pending_file_complete = 1;
        }
      }
      if (!usb2p_queue_rsp_empty(USB2P_OPCODE_FTRUNCATE, USB2P_SPACE_FILE,
                                 pending_file_status, pending_op_txn_id,
                                 pending_op_seq)) {
        return USB2P_STEP_BLOCKED;
      }
      pending_op_kind = USB2P_OP_NONE;
      return USB2P_STEP_PROGRESS;

    case USB2P_OP_FSYNC:
      if (!pending_file_complete) {
        status = USB2P_STATUS_OK;
        if (fh->mode & FA_WRITE) {
          status = usb2p_fresult_status(f_sync(&fh->fil));
        }
        pending_file_status = status;
        pending_file_complete = 1;
      }
      if (!usb2p_queue_rsp_empty(USB2P_OPCODE_FSYNC, USB2P_SPACE_FILE,
                                 pending_file_status, pending_op_txn_id,
                                 pending_op_seq)) {
        return USB2P_STEP_BLOCKED;
      }
      pending_op_kind = USB2P_OP_NONE;
      return USB2P_STEP_PROGRESS;

    case USB2P_OP_FCLOSE:
      if (!pending_file_complete) {
        status = USB2P_STATUS_OK;
        if (fh->mode & FA_WRITE) {
          status = usb2p_fresult_status(f_sync(&fh->fil));
        }
        res = f_close(&fh->fil);
        if (status == USB2P_STATUS_OK) {
          status = usb2p_fresult_status(res);
        }
        fh->active = 0;
        pending_file_status = status;
        pending_file_complete = 1;
      }
      if (!usb2p_queue_rsp_empty(USB2P_OPCODE_FCLOSE, USB2P_SPACE_FILE,
                                 pending_file_status, pending_op_txn_id,
                                 pending_op_seq)) {
        return USB2P_STEP_BLOCKED;
      }
      pending_op_kind = USB2P_OP_NONE;
      return USB2P_STEP_PROGRESS;

    case USB2P_OP_SETATTR:
      if (!pending_file_complete) {
        pending_file_status = usb2p_fresult_status(
            f_chmod(path_payload, pending_file_attr, pending_file_mask));
        pending_file_complete = 1;
      }
      if (!usb2p_queue_rsp_empty(USB2P_OPCODE_SETATTR, USB2P_SPACE_FILE,
                                 pending_file_status, pending_op_txn_id,
                                 pending_op_seq)) {
        return USB2P_STEP_BLOCKED;
      }
      pending_op_kind = USB2P_OP_NONE;
      return USB2P_STEP_PROGRESS;

    case USB2P_OP_SETTIMES: {
      FILINFO fno;

      if (!pending_file_complete) {
        memset(&fno, 0, sizeof(fno));
        fno.fdate = (WORD)(pending_file_mtime >> 16);
        fno.ftime = (WORD)pending_file_mtime;
        pending_file_status = usb2p_fresult_status(f_utime(path_payload, &fno));
        pending_file_complete = 1;
      }
      if (!usb2p_queue_rsp_empty(USB2P_OPCODE_SETTIMES, USB2P_SPACE_FILE,
                                 pending_file_status, pending_op_txn_id,
                                 pending_op_seq)) {
        return USB2P_STEP_BLOCKED;
      }
      pending_op_kind = USB2P_OP_NONE;
      return USB2P_STEP_PROGRESS;
    }

    case USB2P_OP_MKDIR:
      if (!pending_file_complete) {
        pending_file_status = usb2p_fresult_status(f_mkdir(path_payload));
        pending_file_complete = 1;
      }
      if (!usb2p_queue_rsp_empty(USB2P_OPCODE_MKDIR, USB2P_SPACE_FILE,
                                 pending_file_status, pending_op_txn_id,
                                 pending_op_seq)) {
        return USB2P_STEP_BLOCKED;
      }
      pending_op_kind = USB2P_OP_NONE;
      return USB2P_STEP_PROGRESS;

    case USB2P_OP_UNLINK:
      if (!pending_file_complete) {
        pending_file_status = usb2p_fresult_status(f_unlink(path_payload));
        pending_file_complete = 1;
      }
      if (!usb2p_queue_rsp_empty(USB2P_OPCODE_UNLINK, USB2P_SPACE_FILE,
                                 pending_file_status, pending_op_txn_id,
                                 pending_op_seq)) {
        return USB2P_STEP_BLOCKED;
      }
      pending_op_kind = USB2P_OP_NONE;
      return USB2P_STEP_PROGRESS;

    case USB2P_OP_RENAME:
      if (!pending_file_complete) {
        pending_file_status = usb2p_fresult_status(
            f_rename(path_payload, path_payload2));
        pending_file_complete = 1;
      }
      if (!usb2p_queue_rsp_empty(USB2P_OPCODE_RENAME, USB2P_SPACE_FILE,
                                 pending_file_status, pending_op_txn_id,
                                 pending_op_seq)) {
        return USB2P_STEP_BLOCKED;
      }
      pending_op_kind = USB2P_OP_NONE;
      return USB2P_STEP_PROGRESS;

    default:
      return USB2P_STEP_IDLE;
  }
}

/* ---- directory scheduler step ----------------------------------------------*/

/* Serialize the read-ahead entry in pending_dir_fno into dst, returning the
   entry byte length, or 0 if it does not fit in `room`.  Layout (§10.2):
     type:u8 attr:u8 reserved:u16=0 size:u64 mtime_fat:u32 name_len:u16 name[] */
static uint32_t usb2p_dir_entry_emit(uint8_t *dst, uint32_t room) {
  const char *name = pending_dir_fno.lfname && pending_dir_fno.lfname[0]
                     ? pending_dir_fno.lfname
                     : pending_dir_fno.fname;
  uint32_t name_len = (uint32_t)strlen(name);
  uint32_t total;

  if (name_len > USB2P_MAX_PATH_BYTES) {
    name_len = USB2P_MAX_PATH_BYTES;
  }
  total = USB2P_READDIR_ENTRY_HDR_LEN + name_len;
  if (total > room) {
    return 0;
  }
  dst[0] = (pending_dir_fno.fattrib & AM_DIR) ? 1 : 0;
  dst[1] = pending_dir_fno.fattrib;
  usb2p_wr16(dst + 2, 0);
  usb2p_wr64(dst + 4, pending_dir_fno.fsize);
  usb2p_wr32(dst + 12, usb2p_fattime_from_filinfo(&pending_dir_fno));
  usb2p_wr16(dst + 16, (uint16_t)name_len);
  usb2p_wr16(dst + 18, 0);
  memcpy(dst + USB2P_READDIR_ENTRY_HDR_LEN, name, name_len);
  return total;
}

/* Read the next directory entry into pending_dir_fno, skipping volume-label
   pseudo-entries.  Sets pending_dir_have_entry on success, pending_dir_end at
   end-of-directory, and returns the FRESULT.  The LFN buffer is rebound each
   call (FatFs requires it set per read). */
static FRESULT usb2p_dir_next(struct usb2p_dir_handle *dh) {
  FRESULT res;

  for (;;) {
#if _USE_LFN
    pending_dir_fno.lfname = dh->lfn;
    pending_dir_fno.lfsize = sizeof(dh->lfn);
    dh->lfn[0] = 0;
#endif
    res = f_readdir(&dh->dir, &pending_dir_fno);
    if (res != FR_OK) {
      return res;
    }
    if (pending_dir_fno.fname[0] == 0) {
      pending_dir_end = 1;          /* end of directory */
      return FR_OK;
    }
    if (pending_dir_fno.fattrib & AM_VOL) {
      continue;                     /* skip the volume label */
    }
    pending_dir_have_entry = 1;
    return FR_OK;
  }
}

/* One step of an active OPENDIR/READDIR/CLOSEDIR.  Mirrors usb2p_sched_step_file:
   all FatFs/SPI work runs here in main-loop context, one message per step. */
enum usb2p_sched_result usb2p_sched_step_dir(void) {
  struct usb2p_dir_handle *dh = &dir_handles[pending_file_slot];
  FRESULT res;
  uint8_t status;

  switch (pending_op_kind) {
    case USB2P_OP_OPENDIR:
      if (!pending_file_complete) {
        res = f_opendir(&dh->dir, path_payload);
        pending_file_status = usb2p_fresult_status(res);
        pending_file_result32 = 0;
        if (pending_file_status == USB2P_STATUS_OK) {
          uint8_t id = usb2p_alloc_dir_id();
          if (!id) {
            (void)f_closedir(&dh->dir);
            pending_file_status = USB2P_STATUS_ENFILE;
          } else {
            dh->active = 1;
            dh->id = id;
            pending_file_result32 = id;
          }
        }
        pending_file_complete = 1;
      }
      if (pending_file_status == USB2P_STATUS_OK) {
        if (!usb2p_queue_rsp_u32(USB2P_OPCODE_OPENDIR, USB2P_SPACE_FILE,
                                 USB2P_STATUS_OK, pending_op_txn_id,
                                 pending_op_seq, pending_file_result32)) {
          return USB2P_STEP_BLOCKED;
        }
      } else {
        if (!usb2p_queue_rsp_empty(USB2P_OPCODE_OPENDIR, USB2P_SPACE_FILE,
                                   pending_file_status, pending_op_txn_id,
                                   pending_op_seq)) {
          return USB2P_STEP_BLOCKED;
        }
      }
      pending_op_kind = USB2P_OP_NONE;
      return USB2P_STEP_PROGRESS;

    case USB2P_OP_READDIR:
      if (!pending_dir_rsp_sent) {
        /* Build the batch: pack whole entries from FatFs until the next one
           would overflow pending_dir_max_bytes or the directory is exhausted.
           A read-ahead entry that did not fit is carried in pending_dir_fno. */
        pending_dir_batch_len = 0;
        status = USB2P_STATUS_OK;
        while (!pending_dir_end) {
          uint32_t emitted;

          if (!pending_dir_have_entry) {
            res = usb2p_dir_next(dh);
            if (res != FR_OK) {
              status = usb2p_fresult_status(res);
              break;
            }
            if (pending_dir_end) {
              break;
            }
          }
          emitted = usb2p_dir_entry_emit(dir_batch + pending_dir_batch_len,
                                         pending_dir_max_bytes
                                           - pending_dir_batch_len);
          if (!emitted) {
            break;  /* does not fit this batch; keep it for the next READDIR */
          }
          pending_dir_batch_len += emitted;
          pending_dir_have_entry = 0;
        }
        if (!usb2p_queue_rsp_u32(USB2P_OPCODE_READDIR, USB2P_SPACE_FILE,
                                 status, pending_op_txn_id, pending_op_seq,
                                 status == USB2P_STATUS_OK
                                   ? pending_dir_batch_len : 0)) {
          return USB2P_STEP_BLOCKED;
        }
        pending_dir_rsp_sent = 1;
        if (status != USB2P_STATUS_OK) {
          pending_op_kind = USB2P_OP_NONE;
        }
        return USB2P_STEP_PROGRESS;
      } else {
        /* Emit the single DAT carrying this batch.  LAST when the directory is
           exhausted AND nothing remains buffered for a later batch. */
        uint16_t flags = (pending_dir_end && !pending_dir_have_entry)
                         ? USB2P_FLAG_LAST : 0;
        if (!usb2p_queue_frame(USB2P_TYPE_DAT, USB2P_OPCODE_READDIR,
                               USB2P_SPACE_FILE, USB2P_STATUS_OK, flags,
                               pending_op_txn_id, pending_dir_next_seq,
                               dir_batch, pending_dir_batch_len)) {
          return USB2P_STEP_BLOCKED;
        }
        pending_op_kind = USB2P_OP_NONE;
        return USB2P_STEP_PROGRESS;
      }

    case USB2P_OP_CLOSEDIR:
      if (!pending_file_complete) {
        pending_file_status = usb2p_fresult_status(f_closedir(&dh->dir));
        dh->active = 0;
        pending_file_complete = 1;
      }
      if (!usb2p_queue_rsp_empty(USB2P_OPCODE_CLOSEDIR, USB2P_SPACE_FILE,
                                 pending_file_status, pending_op_txn_id,
                                 pending_op_seq)) {
        return USB2P_STEP_BLOCKED;
      }
      pending_op_kind = USB2P_OP_NONE;
      return USB2P_STEP_PROGRESS;

    default:
      return USB2P_STEP_IDLE;
  }
}
