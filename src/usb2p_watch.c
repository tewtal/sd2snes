/* sd2snes - SD card based universal cartridge for the SNES
   Copyright (C) 2009-2010 Maximilian Rehkopf <otakon@gmx.net>

   USB2P v3 address watches (README.usb2p.md §11).  A watch is a low-rate
   polling subscription over the memory spaces: usb2p_poll_watches() samples each
   due watch once per firmware tick (so all FPGA SPI stays in main-loop context),
   diffs against the cached value, and stages an EVENT on change.  The scheduler
   (usb2p_sched_next_event) ships staged EVENTs at message boundaries, priority-
   interleaved into any active transfer, subject to the negotiated per-tick and
   burst budgets.  Overruns are reported via a single WATCH_DROPPED EVENT.
*/

#include <stdint.h>
#include <string.h>

#include "config.h"
#include "timer.h"
#include "usb2p.h"
#include "usb2p_internal.h"
#include "usbuser.h"   /* usb2p_hires_cycles() */

/* All watch scheduling math runs on raw DWT cycles (usb2p_hires_cycles), which
   wrap cleanly at 2^32, so the time_after()/time_before() signed-diff
   comparisons stay valid across the wrap.  A counter that wraps at anything
   other than a power of 2 (e.g. a µs value, which wraps at 2^32/cycles_per_us)
   would break those comparisons and freeze the fairness window — leaving the
   EVENT budget exhausted and the stream stuck flooding WATCH_DROPPED until the
   next accidental realign.  Deadlines are expressed in cycles via the
   conversion factors below. */
#define USB2P_CYCLES_PER_US (CONFIG_CPU_FREQUENCY / 1000000u)
#define USB2P_CYCLES_PER_MS (CONFIG_CPU_FREQUENCY / 1000u)

/* Upper clamp on a watch's poll interval.  interval_ms * CYCLES_PER_MS must stay
   well under 2^31 so the next_poll_cyc deadline is unambiguous to time_before()
   across the cycle-counter wrap.  10 s is far beyond any real watch cadence and
   leaves ~2x headroom under the 2^31-cycle (~22 s @ 96 MHz) limit.  Local, not a
   protocol constant — it only guards the internal deadline math. */
#define USB2P_WATCH_MAX_INTERVAL_MS 10000u

/* Length of the fairness window (§6.6): the period over which the EVENT byte
   budget (max_event_bytes_per_tick) is replenished and the EVENT `tick` field
   advances.  Independent of how fast individual watches poll — a watch may be
   sampled several times within one window.  ~10 ms keeps the budget cadence and
   the wire `tick` counter at roughly their historical (SysTick) rate. */
#define WATCH_FAIRNESS_WINDOW_CYCLES (10u * USB2P_CYCLES_PER_MS)

static struct usb2p_watch watches[USB2P_MAX_WATCHES];
static uint8_t watch_context_space[USB2P_MAX_WATCHES][USB2P_MAX_WATCH_CONTEXTS];
static uint8_t watch_context_len[USB2P_MAX_WATCHES][USB2P_MAX_WATCH_CONTEXTS];
static uint32_t watch_context_offset[USB2P_MAX_WATCHES][USB2P_MAX_WATCH_CONTEXTS];
static IN_AHBRAM uint8_t watch_cache[USB2P_MAX_WATCHES][USB2P_MAX_WATCH_LEN];
static IN_AHBRAM uint8_t watch_sample[USB2P_MAX_WATCH_LEN];
static IN_AHBRAM uint8_t watch_context_snapshot[USB2P_MAX_WATCHES][USB2P_MAX_WATCH_CONTEXT_BYTES];
static IN_AHBRAM uint8_t watch_event_payload[USB2P_WATCH_EVENT_MAX_LEN];
static uint8_t watch_total_bytes;
static uint8_t watch_next_id = 1;
static uint8_t watch_event_cursor;
static uint8_t watch_event_burst;
static uint32_t watch_window_start_cyc;  /* start of fairness window, DWT cycles */
static uint32_t watch_tick_counter;      /* fairness-window counter (EVENT `tick`) */
/* Overrun accounting (§6.6).  coalesced_count: trigger changes folded into an
   already-pending EVENT slot (intermediate value lost).  dropped_count: EVENTs
   the per-tick budget could not carry.  overrun_pending is set when either
   advances and clears when the WATCH_DROPPED report ships. */
static uint32_t watch_coalesced_count;
static uint32_t watch_dropped_count;
static uint8_t watch_overrun_pending;
static uint16_t watch_event_bytes_this_tick;
/* Set when the per-tick EVENT budget is exhausted: usb2p_sched_next_event then
   ships at most one drop report and yields (IDLE) instead of spinning on
   re-dropping the same still-pending events.  Cleared when the fairness window
   advances and replenishes the budget.  Without this, a watch changing every
   poll re-arms watch_overrun_pending inside the same drain pass (the window
   can't advance mid-drain), flooding the host with WATCH_DROPPED reports and
   never letting the real value through — the "never recovers" symptom. */
static uint8_t watch_budget_exhausted;

void usb2p_clear_watches(void) {
  memset(watches, 0, sizeof(watches));
  watch_total_bytes = 0;
  watch_next_id = 1;
  watch_event_cursor = 0;
  watch_event_burst = 0;
  watch_window_start_cyc = usb2p_hires_cycles();
  watch_tick_counter = 0;
  watch_coalesced_count = 0;
  watch_dropped_count = 0;
  watch_overrun_pending = 0;
  watch_event_bytes_this_tick = 0;
  watch_budget_exhausted = 0;
}

void usb2p_watch_burst_reset(void) {
  watch_event_burst = 0;
}

static int8_t usb2p_find_watch(uint32_t id) {
  uint8_t i;

  if (id == 0 || id > 255) {
    return -1;
  }
  for (i = 0; i < USB2P_MAX_WATCHES; i++) {
    if (watches[i].active && watches[i].id == (uint8_t)id) {
      return (int8_t)i;
    }
  }
  return -1;
}

static uint8_t usb2p_alloc_watch_slot(void) {
  uint8_t i;

  for (i = 0; i < USB2P_MAX_WATCHES; i++) {
    if (!watches[i].active) {
      return i;
    }
  }
  return USB2P_MAX_WATCHES;
}

static uint8_t usb2p_alloc_watch_id(void) {
  uint8_t tries;
  uint8_t id;

  for (tries = 0; tries < 255; tries++) {
    id = watch_next_id++;
    if (watch_next_id == 0) watch_next_id = 1;
    if (usb2p_find_watch(id) < 0) {
      return id;
    }
  }
  return 0;
}

/* Snapshot the watch's context blocks and stage an EVENT.  If a prior EVENT for
   this watch is still queued, this sample overwrites it (the intermediate
   trigger value is coalesced); event_seq still bumps per change so the host sees
   the gap, and the WATCH_DROPPED report makes the loss explicit (§6.6). */
static uint8_t usb2p_mark_watch_event(uint8_t idx, uint8_t initial,
                                      const uint8_t *sample) {
  struct usb2p_watch *w = &watches[idx];
  uint32_t pos = 0;
  uint8_t i;

  for (i = 0; i < w->context_count; i++) {
    uint8_t space = watch_context_space[idx][i];
    uint8_t len = watch_context_len[idx][i];
    uint32_t got;

    if (pos + len > USB2P_MAX_WATCH_CONTEXT_BYTES) {
      return 0;
    }
    got = usb2p_memory_read(space, watch_context_offset[idx][i],
                            watch_context_snapshot[idx] + pos, len);
    if (got != len) {
      return 0;
    }
    pos += len;
  }

  if (w->pending_event) {
    watch_coalesced_count++;
    watch_overrun_pending = 1;
  }

  memcpy(watch_cache[idx], sample, w->length);
  w->event_seq++;
  w->pending_event_seq = w->event_seq;
  w->pending_event_tick = watch_tick_counter;
  w->pending_context_len = (uint8_t)pos;
  w->pending_event = 1;
  if (initial) {
    w->pending_initial = 1;
  }
  return 1;
}

static uint16_t usb2p_build_watch_event(uint8_t idx) {
  struct usb2p_watch *w = &watches[idx];
  uint32_t pos = USB2P_WATCH_EVENT_BASE_LEN + w->length;
  uint32_t ctx_pos = 0;
  uint8_t i;

  if (pos > USB2P_WATCH_EVENT_MAX_LEN) {
    return 0;
  }

  usb2p_wr16(watch_event_payload + 0, w->id);
  watch_event_payload[2] = w->context_count;
  watch_event_payload[3] = 0;
  usb2p_wr32(watch_event_payload + 4, w->pending_event_seq);
  usb2p_wr32(watch_event_payload + 8, w->pending_event_tick);
  usb2p_wr64(watch_event_payload + 12, w->offset);
  usb2p_wr16(watch_event_payload + 20, w->length);
  usb2p_wr16(watch_event_payload + 22, 0);
  memcpy(watch_event_payload + USB2P_WATCH_EVENT_BASE_LEN,
         watch_cache[idx], w->length);

  for (i = 0; i < w->context_count; i++) {
    uint8_t space = watch_context_space[idx][i];
    uint8_t len = watch_context_len[idx][i];

    if (pos + USB2P_WATCH_CONTEXT_DESC_LEN + len > USB2P_WATCH_EVENT_MAX_LEN) {
      return 0;
    }
    if (ctx_pos + len > w->pending_context_len) {
      return 0;
    }

    watch_event_payload[pos + 0] = space;
    watch_event_payload[pos + 1] = 0;
    usb2p_wr16(watch_event_payload + pos + 2, len);
    usb2p_wr64(watch_event_payload + pos + 4, watch_context_offset[idx][i]);
    memcpy(watch_event_payload + pos + USB2P_WATCH_CONTEXT_DESC_LEN,
           watch_context_snapshot[idx] + ctx_pos, len);
    ctx_pos += len;
    pos += USB2P_WATCH_CONTEXT_DESC_LEN + len;
  }

  return (uint16_t)pos;
}

void usb2p_poll_watches(void) {
  uint32_t now;
  uint8_t idx;

  if (!usb2p_attached || !watch_total_bytes) {
    return;
  }

  /* Free-running DWT cycle counter, not the 100 Hz SysTick: this runs from the
     main loop, which spins far faster than 10 ms, so a watch can be sampled
     within a few ms of a trigger change and capture coherent context (§6.5).
     Cycles (not the µs helper) so time_after() stays valid across the wrap.
     Each watch carries its own next_poll_cyc deadline. */
  now = usb2p_hires_cycles();

  /* Advance the fairness window (§6.6) when it elapses: replenish the per-window
     EVENT byte budget and bump the EVENT `tick` counter.  Decoupled from the
     poll rate, so a fast-polling watch can't inflate `tick` or the budget. */
  if (time_after(now, watch_window_start_cyc + WATCH_FAIRNESS_WINDOW_CYCLES)) {
    watch_window_start_cyc = now;
    watch_tick_counter++;
    watch_event_bytes_this_tick = 0;
    watch_budget_exhausted = 0;
  }

  for (idx = 0; idx < USB2P_MAX_WATCHES; idx++) {
    struct usb2p_watch *w;
    uint32_t got;

    w = &watches[idx];
    if (!w->active) {
      continue;
    }
    if (w->next_poll_cyc && time_before(now, w->next_poll_cyc)) {
      continue;
    }

    w->next_poll_cyc = now + (uint32_t)w->interval_ms * USB2P_CYCLES_PER_MS;
    got = usb2p_memory_read(w->space, w->offset, watch_sample, w->length);
    if (got != w->length) {
      /* Backend became unreadable; leave the watch registered but skip this
         sample.  Range was validated at registration, so this should be rare. */
      continue;
    }

    if (!w->baseline_valid) {
      if (w->pending_initial) {
        if (usb2p_mark_watch_event(idx, 1, watch_sample)) {
          w->baseline_valid = 1;
        }
      } else {
        memcpy(watch_cache[idx], watch_sample, w->length);
        w->baseline_valid = 1;
      }
      continue;
    }

    if (memcmp(watch_cache[idx], watch_sample, w->length) != 0) {
      (void)usb2p_mark_watch_event(idx, 0, watch_sample);
    }
  }
}

static uint8_t usb2p_watch_event_queued(void) {
  uint8_t i;

  for (i = 0; i < USB2P_MAX_WATCHES; i++) {
    if (watches[i].active && watches[i].pending_event) {
      return 1;
    }
  }
  return 0;
}

static uint8_t usb2p_non_event_work_pending(void) {
  if (pending_cancel || pending_meta) {
    return 1;
  }
  if (pending_op_kind == USB2P_OP_READ) {
    return 1;
  }
  if (pending_op_kind == USB2P_OP_WRITE) {
    return pending_op_write_state != USB2P_WRITE_AWAIT_DAT;
  }
  return pending_op_kind != USB2P_OP_NONE;
}

/* WATCH_ADD/REMOVE/CLEAR return a sched_result so the frame dispatcher can treat
   a tx_buf-full append as BLOCKED and retry, rather than dropping the response. */
enum usb2p_sched_result usb2p_handle_watch_add(const struct usb2p_header *h,
                                               const uint8_t *payload) {
  uint64_t offset;
  uint16_t length;
  uint16_t interval_ms;
  uint8_t context_count;
  uint16_t context_total_len = 0;
  uint32_t expected_len;
  uint8_t slot;
  uint8_t id;
  uint8_t i;

  if (pending_op_kind != USB2P_OP_NONE) {
    return usb2p_queue_nak(USB2P_STATUS_EBUSY, h->txn_id, h->seq)
         ? USB2P_STEP_PROGRESS : USB2P_STEP_BLOCKED;
  }
  if (h->length < USB2P_WATCH_ADD_BASE_LEN) {
    return usb2p_queue_nak(USB2P_STATUS_EINVAL, h->txn_id, h->seq)
         ? USB2P_STEP_PROGRESS : USB2P_STEP_BLOCKED;
  }

  offset = usb2p_rd64(payload + 0);
  length = usb2p_rd16(payload + 8);
  interval_ms = usb2p_rd16(payload + 10);
  context_count = payload[12];
  if (payload[13] != 0 || payload[14] != 0 || payload[15] != 0
      || length == 0 || length > USB2P_MAX_WATCH_LEN
      || context_count > USB2P_MAX_WATCH_CONTEXTS) {
    return usb2p_queue_nak(USB2P_STATUS_EINVAL, h->txn_id, h->seq)
         ? USB2P_STEP_PROGRESS : USB2P_STEP_BLOCKED;
  }
  expected_len = USB2P_WATCH_ADD_BASE_LEN
               + (uint32_t)context_count * USB2P_WATCH_CONTEXT_DESC_LEN;
  if (h->length != expected_len) {
    return usb2p_queue_nak(USB2P_STATUS_EINVAL, h->txn_id, h->seq)
         ? USB2P_STEP_PROGRESS : USB2P_STEP_BLOCKED;
  }
  if (!usb2p_memory_space_valid(h->space)) {
    return usb2p_queue_nak(USB2P_STATUS_EINVAL, h->txn_id, h->seq)
         ? USB2P_STEP_PROGRESS : USB2P_STEP_BLOCKED;
  }
  if (!usb2p_memory_range_ok(h->space, offset, length)) {
    return usb2p_queue_rsp_empty(h->opcode, h->space, USB2P_STATUS_ERANGE,
                                h->txn_id, h->seq)
         ? USB2P_STEP_PROGRESS : USB2P_STEP_BLOCKED;
  }
  if (watch_total_bytes + length > USB2P_MAX_WATCH_BYTES_PER_TICK) {
    return usb2p_queue_nak(USB2P_STATUS_EMSGSIZE, h->txn_id, h->seq)
         ? USB2P_STEP_PROGRESS : USB2P_STEP_BLOCKED;
  }
  for (i = 0; i < context_count; i++) {
    const uint8_t *ctx = payload + USB2P_WATCH_ADD_BASE_LEN
                       + (uint32_t)i * USB2P_WATCH_CONTEXT_DESC_LEN;
    uint8_t ctx_space = ctx[0];
    uint16_t ctx_len = usb2p_rd16(ctx + 2);
    uint64_t ctx_offset = usb2p_rd64(ctx + 4);

    if (ctx[1] != 0 || ctx_len == 0 || ctx_len > USB2P_MAX_WATCH_LEN
        || !usb2p_memory_space_valid(ctx_space)) {
      return usb2p_queue_nak(USB2P_STATUS_EINVAL, h->txn_id, h->seq)
           ? USB2P_STEP_PROGRESS : USB2P_STEP_BLOCKED;
    }
    if (!usb2p_memory_range_ok(ctx_space, ctx_offset, ctx_len)) {
      return usb2p_queue_rsp_empty(h->opcode, h->space, USB2P_STATUS_ERANGE,
                                  h->txn_id, h->seq)
           ? USB2P_STEP_PROGRESS : USB2P_STEP_BLOCKED;
    }
    context_total_len += ctx_len;
    if (context_total_len > USB2P_MAX_WATCH_CONTEXT_BYTES) {
      return usb2p_queue_nak(USB2P_STATUS_EMSGSIZE, h->txn_id, h->seq)
           ? USB2P_STEP_PROGRESS : USB2P_STEP_BLOCKED;
    }
  }
  if ((uint32_t)USB2P_WATCH_EVENT_BASE_LEN + length
      + (uint32_t)context_count * USB2P_WATCH_CONTEXT_DESC_LEN
      + context_total_len > USB2P_MAX_EVENT_BYTES_PER_TICK) {
    return usb2p_queue_nak(USB2P_STATUS_EMSGSIZE, h->txn_id, h->seq)
         ? USB2P_STEP_PROGRESS : USB2P_STEP_BLOCKED;
  }

  slot = usb2p_alloc_watch_slot();
  if (slot >= USB2P_MAX_WATCHES) {
    return usb2p_queue_nak(USB2P_STATUS_EBUSY, h->txn_id, h->seq)
         ? USB2P_STEP_PROGRESS : USB2P_STEP_BLOCKED;
  }
  id = usb2p_alloc_watch_id();
  if (!id) {
    return usb2p_queue_nak(USB2P_STATUS_EBUSY, h->txn_id, h->seq)
         ? USB2P_STEP_PROGRESS : USB2P_STEP_BLOCKED;
  }

  if (!usb2p_queue_rsp_u32(h->opcode, h->space, USB2P_STATUS_OK,
                           h->txn_id, h->seq, id)) {
    return USB2P_STEP_BLOCKED;
  }

  memset(&watches[slot], 0, sizeof(watches[slot]));
  watches[slot].active = 1;
  watches[slot].id = id;
  watches[slot].space = h->space;
  watches[slot].length = (uint8_t)length;
  watches[slot].context_count = context_count;
  watches[slot].offset = (uint32_t)offset;
  /* 0 = device default; otherwise clamp to [MIN, MAX].  MIN keeps a host from
     requesting a poll rate that would starve transfers / the SPI bus (§6.5).
     MAX keeps interval_ms * CYCLES_PER_MS well under 2^31 cycles so the
     next_poll_cyc deadline stays within time_before()'s unambiguous half-range
     (at 96 MHz, 2^31 cycles ≈ 22 s; cap at 10 s). */
  if (interval_ms == 0) {
    watches[slot].interval_ms = USB2P_WATCH_DEFAULT_INTERVAL_MS;
  } else if (interval_ms < USB2P_WATCH_MIN_INTERVAL_MS) {
    watches[slot].interval_ms = USB2P_WATCH_MIN_INTERVAL_MS;
  } else if (interval_ms > USB2P_WATCH_MAX_INTERVAL_MS) {
    watches[slot].interval_ms = USB2P_WATCH_MAX_INTERVAL_MS;
  } else {
    watches[slot].interval_ms = interval_ms;
  }
  watches[slot].next_poll_cyc = 0;
  watches[slot].pending_initial = (h->flags & USB2P_FLAG_INITIAL) ? 1 : 0;
  watch_total_bytes += (uint8_t)length;
  for (i = 0; i < context_count; i++) {
    const uint8_t *ctx = payload + USB2P_WATCH_ADD_BASE_LEN
                       + (uint32_t)i * USB2P_WATCH_CONTEXT_DESC_LEN;
    watch_context_space[slot][i] = ctx[0];
    watch_context_len[slot][i] = (uint8_t)usb2p_rd16(ctx + 2);
    watch_context_offset[slot][i] = (uint32_t)usb2p_rd64(ctx + 4);
  }

  return USB2P_STEP_PROGRESS;
}

enum usb2p_sched_result usb2p_handle_watch_remove(const struct usb2p_header *h,
                                                  const uint8_t *payload) {
  uint32_t id;
  int8_t idx;

  if (pending_op_kind != USB2P_OP_NONE) {
    return usb2p_queue_nak(USB2P_STATUS_EBUSY, h->txn_id, h->seq)
         ? USB2P_STEP_PROGRESS : USB2P_STEP_BLOCKED;
  }
  if (h->length != USB2P_WATCH_ID_LEN) {
    return usb2p_queue_nak(USB2P_STATUS_EINVAL, h->txn_id, h->seq)
         ? USB2P_STEP_PROGRESS : USB2P_STEP_BLOCKED;
  }
  id = usb2p_rd32(payload);
  idx = usb2p_find_watch(id);
  if (idx < 0) {
    return usb2p_queue_rsp_empty(h->opcode, h->space, USB2P_STATUS_ENOENT,
                                h->txn_id, h->seq)
         ? USB2P_STEP_PROGRESS : USB2P_STEP_BLOCKED;
  }

  if (!usb2p_queue_rsp_empty(h->opcode, h->space, USB2P_STATUS_OK,
                             h->txn_id, h->seq)) {
    return USB2P_STEP_BLOCKED;
  }
  watch_total_bytes -= watches[idx].length;
  memset(&watches[idx], 0, sizeof(watches[idx]));
  return USB2P_STEP_PROGRESS;
}

enum usb2p_sched_result usb2p_handle_watch_clear(const struct usb2p_header *h) {
  if (pending_op_kind != USB2P_OP_NONE) {
    return usb2p_queue_nak(USB2P_STATUS_EBUSY, h->txn_id, h->seq)
         ? USB2P_STEP_PROGRESS : USB2P_STEP_BLOCKED;
  }
  if (h->length != 0) {
    return usb2p_queue_nak(USB2P_STATUS_EINVAL, h->txn_id, h->seq)
         ? USB2P_STEP_PROGRESS : USB2P_STEP_BLOCKED;
  }
  if (!usb2p_queue_rsp_empty(h->opcode, h->space, USB2P_STATUS_OK,
                             h->txn_id, h->seq)) {
    return USB2P_STEP_BLOCKED;
  }
  usb2p_clear_watches();
  return USB2P_STEP_PROGRESS;
}

/* Emit the single WATCH_DROPPED overrun report (§6.6): watch_id=0, txn_id=0,
   status=WATCH_DROPPED, payload {coalesced_count:u32, dropped_count:u32}.
   Clears the counters and the pending flag once queued. */
static enum usb2p_sched_result usb2p_sched_emit_drop_report(void) {
  uint8_t payload[USB2P_WATCH_DROPPED_LEN];

  usb2p_wr32(payload + 0, watch_coalesced_count);
  usb2p_wr32(payload + 4, watch_dropped_count);
  if (!usb2p_queue_frame(USB2P_TYPE_EVENT, USB2P_OPCODE_WATCH_ADD, 0,
                         USB2P_STATUS_WATCH_DROPPED, 0, 0, 0,
                         payload, sizeof(payload))) {
    return USB2P_STEP_BLOCKED;
  }
  watch_coalesced_count = 0;
  watch_dropped_count = 0;
  watch_overrun_pending = 0;
  return USB2P_STEP_PROGRESS;
}

/* Emit one pending watch EVENT if any is due, into tx_buf.  Watches are polled
   by usb2p_poll_watches(); this step only ships already-cached changes so EVENT
   injection adds no SPI work at each READ/DAT boundary.  Returns PROGRESS if it
   emitted one, BLOCKED if it wanted to but tx_buf was full, IDLE if none due. */
enum usb2p_sched_result usb2p_sched_next_event(void) {
  uint8_t scanned;

  if (!usb2p_attached) {
    watch_event_burst = 0;
    return USB2P_STEP_IDLE;
  }
  if (!usb2p_watch_event_queued() && !watch_overrun_pending) {
    watch_event_burst = 0;
    return USB2P_STEP_IDLE;
  }
  if (watch_event_burst >= USB2P_MAX_EVENT_BURST
      && usb2p_non_event_work_pending()) {
    return USB2P_STEP_IDLE;
  }

  /* A drop report ships ahead of fresh EVENTs so a tracker learns the stream is
     lossy before it tries to interpret the next (post-gap) sample. */
  if (watch_overrun_pending) {
    enum usb2p_sched_result r = usb2p_sched_emit_drop_report();
    if (r == USB2P_STEP_PROGRESS) {
      watch_event_burst++;
    }
    /* If we're dropping because the budget is spent, yield after the report
       instead of letting the drain loop re-enter and re-drop the same pending
       events this window (which would flood the host with WATCH_DROPPED and
       starve real events).  The window advance in usb2p_poll_watches clears
       watch_budget_exhausted and lets the backlog drain next window. */
    if (watch_budget_exhausted && r == USB2P_STEP_PROGRESS) {
      return USB2P_STEP_IDLE;
    }
    return r;
  }

  /* Budget already spent this window: don't scan/drop pending events (that just
     re-arms watch_overrun_pending and spins).  Yield until the window advances. */
  if (watch_budget_exhausted) {
    return USB2P_STEP_IDLE;
  }

  for (scanned = 0; scanned < USB2P_MAX_WATCHES; scanned++) {
    uint8_t idx = watch_event_cursor;
    struct usb2p_watch *w;
    uint16_t event_len;

    watch_event_cursor++;
    if (watch_event_cursor >= USB2P_MAX_WATCHES) {
      watch_event_cursor = 0;
    }

    w = &watches[idx];
    if (!w->active || !w->pending_event) {
      continue;
    }
    event_len = usb2p_build_watch_event(idx);
    if (!event_len) {
      w->pending_event = 0;
      w->pending_initial = 0;
      return USB2P_STEP_PROGRESS;
    }

    /* Per-tick EVENT budget (§6.6): if shipping this EVENT would exceed the
       negotiated max_event_bytes_per_tick, drop it instead of shipping — count
       it and let the WATCH_DROPPED report carry the loss.  Bounds how much one
       tick's worth of watch traffic can starve an active transfer. */
    if ((uint32_t)watch_event_bytes_this_tick + event_len
        > USB2P_MAX_EVENT_BYTES_PER_TICK) {
      w->pending_event = 0;
      w->pending_initial = 0;
      watch_dropped_count++;
      watch_overrun_pending = 1;
      /* No room left this window: gate further event/drop processing until the
         next fairness window replenishes the budget (see watch_budget_exhausted).
         The next call ships the drop report, then yields. */
      watch_budget_exhausted = 1;
      return USB2P_STEP_PROGRESS;
    }

    if (!usb2p_queue_frame(USB2P_TYPE_EVENT, USB2P_OPCODE_WATCH_ADD,
                           w->space, USB2P_STATUS_OK,
                           w->pending_initial ? USB2P_FLAG_INITIAL : 0,
                           0, w->pending_event_seq,
                           watch_event_payload,
                           event_len)) {
      return USB2P_STEP_BLOCKED;
    }
    w->pending_event = 0;
    w->pending_initial = 0;
    watch_event_bytes_this_tick += event_len;
    watch_event_burst++;
    return USB2P_STEP_PROGRESS;
  }

  return USB2P_STEP_IDLE;
}
