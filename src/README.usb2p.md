# USB2P — sd2snes / FXPAK PRO USB protocol v3

USB2P is a length-prefixed, binary, little-endian protocol carried over the
CDC-ACM data endpoint. A session starts as the legacy CDC (usb2snes) byte
protocol and *upgrades* into USB2P framing in place via a one-way upgrade command
(the legacy `CDC2P` opcode); there is no separate vendor interface. This document
defines the protocol and the firmware that implements it; it is a reference, not
a tutorial.

The wire constants quoted here are authoritative in
[usb2p_protocol_generated.h](usb2p_protocol_generated.h), which is generated from
[../usb2p_protocol_schema.py](../usb2p_protocol_schema.py). Firmware, host tools,
and any host binding consume the same generated definitions.

---

## 1. Transport and layering

Two independent layers; do not conflate them.

- **USB transport (hardware).** Fixed 64-byte full-speed bulk packets on the
  CDC-ACM data endpoint. The protocol never sees individual packets: a byte run
  is streamed across as many 64-byte packets as the link needs. Both supported
  MCUs (LPC175x, STM32F401) are USB 2.0 Full-Speed only, so 64 bytes is the
  hardware maximum.
- **Protocol (this spec).** A **message** is a 26-byte frame
  `[24-byte header][2-byte header CRC]` optionally followed by a `length`-byte
  payload. One header per logical message — *not* per 64-byte packet.

The receiver delimits messages **solely** by the header `length` field. USB
short packets and zero-length packets may occur as the link requires, but
receivers must not depend on them for message boundaries. This is what makes the
stream self-delimiting and immune to the desync-freeze of the legacy protocol.

A large transfer is a *sequence* of bounded `DAT` messages that share one
`txn_id`, not one giant message — so other traffic (watch `EVENT`s, interleaved
metadata responses) can be spliced in at message boundaries without corrupting
the transfer. The host demultiplexes by `type` + `txn_id`.

Chunk size is a per-message choice bounded by the negotiated
`max_tx_dat_payload` ceiling (1024 bytes). The sender may vary the size of each
`DAT` freely under that ceiling with no renegotiation, because each `DAT` carries
its own `length`.

---

## 2. Message header

24-byte fixed header followed by a 2-byte CRC (26 bytes on the wire,
`USB2P_HEADER_WIRE_LEN`).

```
off size field       notes
  0   4  magic        bytes 'U' '2' 'P' '2' (0x55 0x32 0x50 0x32)
  4   1  version      = 3 (USB2P_VERSION)
  5   1  header_len   = 24 (USB2P_HEADER_LEN); receiver rejects any other value
  6   1  type         REQ/RSP/DAT/ACK/NAK/EVENT/RESYNC/CANCEL (§3)
  7   1  opcode       see §4
  8   1  space        FILE/SNES/MSU/CMD/CONFIG (§5)
  9   1  status        0 = OK, else an error code (§6); meaningful on RSP/NAK
 10   2  flags        see §2.1
 12   4  txn_id       logical operation id; host-allocated for REQ, 0 for EVENT
 16   2  seq          per-txn message sequence
 18   2  length       payload byte count of THIS message; must be <= the
                       negotiated per-message maximum (§7)
 20   4  reserved     = 0
 -- 24-byte fixed header ends --
 24   2  hdr_crc16    CRC-16 (init 0xFFFF) over header bytes 0..23
```

The header CRC is validated before any header field is acted on.

Transfer-specific fields (segment offsets, file handles, etc.) live in the
opcode-specific payload, never in the common header.

**On the width of `length` and `seq`.** Both are u16. `length` is *one message's*
payload, capped at the HELLO-negotiated `max_rx_message_payload` /
`max_tx_dat_payload` (1024 today) — so u16 is ample. Big transfers are *sequences*
of bounded `DAT` messages (§1), each with its own small `length`; the total is
never expressed in one header. 64-bit file offsets/sizes that genuinely need the
width live in opcode payloads (`FREAD offset:u64`, `STAT size:u64`), past the
framing layer, and the FatFs backend is 32-bit regardless (over-4 GB →
`RSP(ERANGE)`). **The header's forward-compatibility knob is `header_len`**: a
future version may grow the fixed header and bump `header_len`, and a receiver
rejects a `header_len` it does not recognize — so the common header can be
extended without a hard version break (the 4-byte reserved tail leaves room).

### 2.1 Flags (`flags`, u16)

```
bit 4  LAST         final DAT of a transfer
bit 5  INITIAL      EVENT is a watch's initial baseline snapshot (§7)
bit 6  HDR_ONLY     message has no payload (length must be 0)
all other bits      reserved (= 0)
```

Any reserved flag bit set (`USB2P_FLAGS_RESERVED` = every bit except LAST/INITIAL/
HDR_ONLY) is rejected with `EINVAL`. `HDR_ONLY` with a non-zero `length` is
`EINVAL`.

---

## 3. Message types

```
REQ     host request
RSP     device response to a REQ (status carries the result)
DAT     bulk data chunk belonging to a transaction
ACK     acknowledgement (RESYNC handshake)
NAK     transport/framing rejection (§6)
EVENT   device-initiated, unsolicited (watches; txn_id = 0)
RESYNC  flush logical state to idle on either side
CANCEL  abort an in-flight transaction (§8)
```

---

## 4. Opcodes

Memory access uses segmented `READ`/`WRITE`; file access uses the handle-based
file ops. Each opcode's payload is a fixed little-endian struct defined in the
schema.

| Group     | Opcodes |
|-----------|---------|
| Memory    | `READ`, `WRITE` (segmented, §6.x of schema) |
| File      | `FOPEN`, `FREAD`, `FWRITE`, `FTRUNCATE`, `FSYNC`, `FCLOSE` |
| Path      | `STAT`, `STATFS`, `SETATTR`, `SETTIMES`, `MKDIR`, `UNLINK`, `RENAME` |
| Directory | `OPENDIR`, `READDIR`, `CLOSEDIR` |
| Watch     | `WATCH_ADD`, `WATCH_REMOVE`, `WATCH_CLEAR` |
| Control   | `RESET`, `MENU_RESET`, `INFO`, `TIME` |
| Protocol  | `HELLO`, `ACK`, `NAK`, `RESYNC`, `CANCEL` |

`BOOT` and `POWER_CYCLE` opcode values are reserved in the schema but not yet
implemented; the device answers them `NAK(EUNSUPPORTED)`.

---

## 5. Address spaces

```
FILE     0   handle/path filesystem ops only
SNES     1   24-bit SNES/SRAM space (0x1000000)
MSU      2   16-bit MSU space (read-only on WRITE → EUNSUPPORTED)
CMD      3   16-bit SNES command space
CONFIG   4   FPGA config registers (single-byte access)
```

Memory `READ`/`WRITE` and the watch trigger/context blocks accept SNES, MSU,
CMD, and CONFIG. FILE-space watches are rejected (`EINVAL`).

---

## 6. Error model

The header `status` field carries the result of every RSP/NAK (0 = OK). The
split between `NAK` and `RSP(status != OK)` is strict:

- **`NAK`** — a transport/framing fault: the request could not be parsed or
  admitted. Bad header CRC, unknown opcode, invalid enum, wrong state, queue
  full, or `length` over the negotiated maximum.
- **`RSP(status != OK)`** — a well-formed request the backend rejected: file not
  found, permission denied, or an offset/size beyond the 32-bit FatFs backend
  (`ERANGE`). A `FREAD(offset = 5 GB)` is well-formed, so it is `RSP(ERANGE)`,
  not a NAK.

Status taxonomy: `OK, EMSGSIZE, ERANGE, ENOENT, EACCES, EBUSY, ENFILE,
EUNREPRESENTABLE, EBADHDR, EQUEUEFULL, ECANCELLED, ETIMEDOUT, EINVAL,
EUNSUPPORTED, EBADSTATE, WATCH_DROPPED, EIO, EBADF`. Every FatFs
`FRESULT` maps to exactly one of these.

### 6.1 Resync

Either side may send `RESYNC`; both flush logical state to idle and the device
replies `ACK(RESYNC)`. RESYNC also clears the attached state, so the host must
re-`HELLO`.

On a corrupt header the receiver scans the byte stream for the 4-byte magic and
then *validates* the candidate by its header CRC before accepting it — a pattern
match alone is insufficient, which is why the magic is 4 exact bytes and the
header CRC is mandatory. A header that fails CRC yields no trusted `txn_id`, so a
courtesy `NAK(EBADHDR, txn_id=0)` is only emitted after stream state is safe.

### 6.2 Payload timeout

If a host sends a valid header then stalls mid-payload, an OUT payload timeout
(`USB2P_RX_PAYLOAD_TIMEOUT_MS`, 1000 ms) aborts the transaction with
`NAK(ETIMEDOUT)` and returns the parser to idle. The device never hangs
mid-payload.

---

## 7. HELLO handshake and negotiated limits

`HELLO` is the only REQ accepted before attach; every other opcode returns
`NAK(EBADSTATE)` until the handshake completes. The device replies with a
24-byte `HELLO` RSP advertising versions, a capability bitmap, and the hard
limits below. A host must treat these as the contract for the session.

Advertised capabilities: `HEADER_CRC`, `ACK_NAK_RESYNC`, `MAX_OUTSTANDING1`.

| Limit | Value | Meaning |
|-------|-------|---------|
| `max_rx_message_payload`        | 1024 | largest payload the device accepts (OUT) |
| `max_tx_dat_payload`            | 1024 | largest DAT chunk the device emits (IN) |
| `max_outstanding_txns`          | 1    | one heavyweight op at a time |
| `max_open_files`                | 1    | concurrent file handles |
| `max_watches`                   | 8    | concurrent address watches |
| `max_watch_bytes_per_tick`      | 240  | total watched trigger bytes |
| `max_event_bytes_per_tick`      | 576  | EVENT bytes shipped per tick |
| `max_event_burst`               | 2    | consecutive EVENTs before a non-event message |
| `max_segments`                  | 8    | segments per READ/WRITE descriptor |

A header whose `length` exceeds the relevant maximum is rejected with
`NAK(EMSGSIZE)` before the device enters payload-receive, so the wide `length`
field cannot be used to force an unbounded allocation.

---

## 8. Transactions, ordering, and cancellation

Every REQ carries a host-allocated `txn_id` and a per-txn `seq`; the device
echoes both in every RSP/DAT/NAK for that transaction. With
`max_outstanding_txns = 1`, only one heavyweight (data-bearing) operation is
active at a time:

- A second heavyweight REQ while one is active is `NAK(EBUSY)`.
- Cheap metadata/control requests (`INFO`, read-only `TIME`) that arrive while a
  data op is active are *interleaved*: stashed and answered at the next message
  boundary, between the transfer's `DAT` messages, rather than dropped.
- Watch `EVENT`s are independent of the request queue (device-originated,
  `txn_id = 0`).

**Cancellation.** `CANCEL(txn_id)` (or `REQ` + opcode `CANCEL`) names the target
transaction. The device emits a terminal `RSP(ECANCELLED, txn = target)` at a
message boundary; if the target is the active op it is torn down (open file/dir
handles opened by that op are closed) so no further `DAT` flows. CANCEL is
idempotent for a non-active txn.

---

## 9. Memory transfer (READ / WRITE)

A descriptor lists 1..`max_segments` `{offset:u64, length:u64}` segments;
`segment_count == 1` is the ordinary contiguous transfer, `> 1` is
scatter/gather. The device range-checks every segment against the space size.

```
READ / WRITE request descriptor:
  segment_count : u16   (1..max_segments)
  reserved      : u16 = 0
  segment[segment_count]: { offset:u64, length:u64 }

READ  response: RSP(total_length:u64) then DAT chunks carrying the segments
                back-to-back in request order; final DAT has LAST.
WRITE request : descriptor only; segment data follows as DAT messages.
WRITE response: RSP(written:u64).
```

`WRITE` is two-phase: the REQ pins the operation and validates the descriptor,
then each host `DAT` chunk is staged and committed before the final RSP. A chunk
may span segment boundaries; the commit step splits it across segments. The LAST
`DAT` must close the transfer exactly.

A segment offset/length the backend cannot satisfy (e.g. beyond the 32-bit
backend) is a backend rejection: `RSP(ERANGE)`, not a NAK. Structural faults in
the descriptor are `NAK(EINVAL/EMSGSIZE)`.

---

## 10. Filesystem

Handle-based file I/O over the bundled FatFs. `FOPEN(path, mode, share)` returns
a non-zero handle plus `{size, mtime_fat, attr}`; `FREAD`/`FWRITE` carry an
explicit `offset:u64` and use the same DAT-stream framing as memory transfer;
`FTRUNCATE`, `FSYNC`, and `FCLOSE` take a handle. `FCLOSE` fsyncs before closing.
Path ops (`STAT`, `STATFS`, `SETATTR`, `SETTIMES`, `MKDIR`, `UNLINK`, `RENAME`)
take a validated path. Directory iteration is stateful:
`OPENDIR` → handle, `READDIR(handle, max_bytes)` → next batch of entries,
`CLOSEDIR`.

Semantics:

- **Single mediated writer.** The device's FatFs is the only FAT writer; host,
  menu, and SNES file access serialize through the firmware, so the volume stays
  consistent. This is why USB2P file ops are preferred over raw mass storage.
- **Handle invalidation.** All file and dir handles are force-closed on
  reset/disconnect; an op on a stale handle returns `EBADF`.
- **Partial writes.** `FWRITE` returns `{written}`; a short write (card full) is
  a successful RSP with `written < length`, not a NAK.
- **Wide types, 32-bit backend.** `offset`/`size` are u64 on the wire but FatFs
  is 32-bit; a valid request beyond 4 GB returns `RSP(ERANGE)`.
- **Times.** FAT stores only a reliable modified time; a host filesystem layer
  should synthesize create/access times.

### 10.1 Path encoding

All path/filename fields on the wire are **UTF-8**. The bundled FatFs uses OEM
code page 1252, so the device transcodes UTF-8 ⇄ CP-1252 at the protocol
boundary:

- Characters representable in CP-1252 round-trip; anything else, malformed
  UTF-8, embedded NUL, control characters, or the reserved `:` and `\` →
  `EUNREPRESENTABLE`.
- `/` is the only separator. Paths are normalized in place: `//` and `.`
  segments collapse, `..` pops the previous segment, and any `..` that would
  escape the SD root is rejected.
- Full-path length is capped at `USB2P_MAX_PATH_BYTES` (255).

The `EINVAL` (structural) vs `EUNREPRESENTABLE` (well-formed but un-encodable)
split lets callers map to NAK vs RSP correctly.

---

## 11. Address watches

A watch is a polling subscription over the memory spaces: the device samples a
registered trigger block on its own schedule, diffs against the cached value,
and emits an `EVENT` on change. There is no hardware watchpoint — every sample
is an FPGA SPI read, so the feature is capped and budgeted.

```
WATCH_ADD request:
  offset         : u64
  length         : u16   (1..64)
  interval_ms    : u16   (poll period; 0 = device default of 4 ms, floored at 1 ms)
  context_count  : u8    (0..4)
  reserved       : u8[3] = 0
  context[context_count]: { space:u8, reserved:u8=0, length:u16, offset:u64 }
WATCH_ADD response / WATCH_REMOVE request:
  watch_id : u32   (device-allocated, non-zero)

EVENT payload:
  watch_id:u16, context_count:u8, reserved:u8, event_seq:u32, tick:u32,
  offset:u64, length:u16, reserved:u16, trigger_bytes[length],
  context[context_count]: { space:u8, reserved:u8, length:u16, offset:u64, bytes[length] }
```

- **Context reads.** A watch may attach context descriptors. Only the trigger
  block is polled; when it changes, the EVENT carries the new trigger bytes plus
  a snapshot of each context block, read in the **same poll pass** (the same SPI
  burst, microseconds apart) — so "when A changes, also return B/C/D" needs no
  host follow-up and the context is coherent with the trigger *at sample time*.
  The residual skew is across poll passes, not within one: the trigger may have
  flipped up to `interval_ms` before the poll that detects it, so if the context
  mutates shortly after the flip, a shorter `interval_ms` is needed to sample
  before that mutation. Polling cannot catch a change that reverts between two
  polls; for guaranteed flip-time capture, latch the values game-side and watch
  the latched copy.
- **`INITIAL`.** With the `INITIAL` flag, one EVENT is emitted immediately after
  the first successful poll as a baseline; otherwise only post-registration
  changes are reported.
- **Pacing.** Polling is driven from the main loop against a free-running clock
  (the Cortex-M DWT cycle counter, with a SysTick fallback where DWT does not
  count standalone), not the 100 Hz system tick, so `interval_ms` is honored down
  to its 1 ms floor everywhere including the menu — subject to main-loop
  responsiveness (a blocking menu-loop path delays the next poll).
- **Lifecycle.** Watches are cleared on disconnect/reset.

### 11.1 Fairness and overrun

Events and bulk data share one IN endpoint, so the protocol bounds how they
contend:

- Each EVENT carries a per-watch `event_seq` and a device `tick`, so the host can
  detect coalescing and gaps. `tick` counts a ~10 ms *fairness window*, which is
  independent of the (possibly faster) per-watch poll rate — a fast-polling watch
  cannot inflate `tick` or the budget below.
- A per-window byte budget (`max_event_bytes_per_tick`) and a coalescing rule (a
  fresh trigger overwrites an unsent EVENT for the same watch) bound watch
  traffic. Losses are made explicit by a single `EVENT(status=WATCH_DROPPED)`
  carrying `{coalesced_count, dropped_count}`, shipped ahead of fresh EVENTs so a
  tracker learns the stream is lossy before interpreting the next sample.
- `max_event_burst` bounds how many consecutive EVENTs may precede servicing at
  least one non-event message, so events can never wholly starve an active
  transfer.

---

## 12. Firmware architecture

The implementation is transport-agnostic: endpoint code feeds received bytes in
and drains serialized bytes out, knowing nothing about which endpoint delivered
them. The protocol layer is split to mirror the wire structure
([usb2p_internal.h](usb2p_internal.h)):

| Module | Responsibility |
|--------|----------------|
| [usb2p.c](usb2p.c)           | RX byte-stream state machine, opcode dispatch, public entry points |
| [usb2p_codec.c](usb2p_codec.c) | byte I/O, header CRC, frame queueing, the tx_buf stream-out |
| [usb2p_path.c](usb2p_path.c) | UTF-8 ⇄ CP-1252 transcode + normalization |
| [usb2p_mem.c](usb2p_mem.c)   | memory spaces, segmented READ/WRITE |
| [usb2p_fs.c](usb2p_fs.c)     | file + directory handles and FatFs ops |
| [usb2p_watch.c](usb2p_watch.c) | watch subscriptions and EVENT emission |
| [usb2p_sched.c](usb2p_sched.c) | the IN-endpoint priority dispatcher |

### 12.1 Single-op-slot model

At most one heavyweight (data-bearing) operation is pinned at a time. Every
`READ`/`WRITE` and every FS op touches the FPGA SPI bus or FatFs, and running
those from the USB ISR would race the menu loop's SPI access and wedge the bus.
So the ISR only parses, validates, and stashes the operation; the main-loop poll
(`usb2p_poll`) executes the deferred SPI/FatFs work. Because `HELLO` advertises
`max_outstanding_txns = 1`, any overlapping heavyweight REQ is `NAK(EBUSY)`,
which keeps the deferred state single-threaded against the ISR without locking.

### 12.2 IN-endpoint scheduler

The IN endpoint is driven by an explicit dispatcher evaluated at each message
boundary rather than a transfer "owning" the endpoint until done. Each step
emits at most one message and re-checks priority:

```
1. Pending watch EVENT
2. Pending bulk CANCEL        (terminates the active op promptly)
3. Pending metadata RSP       (INFO/TIME stashed during a data op)
4. Active transfer            (READ/WRITE, file/dir ops)
5. Idle
```

This is what lets a long `READ` stream continuously while watch `EVENT`s and
interleaved metadata responses splice in cleanly at message boundaries.

### 12.3 tx_buf and OUT back-pressure

Outbound bytes accumulate in `tx_buf`, a linear append buffer in AHB SRAM
(`USB2P_TX_BUF_SIZE`, sized to hold one RSP plus several max DAT chunks). It is
drained without regard to frame boundaries — the host delimits by header length,
so the tail of one frame and the head of the next may share one 64-byte USB
packet, keeping the IN pipe continuous across a multi-DAT transfer. The append in
`usb2p_queue_frame` is guarded by a short USB-IRQ critical section; the SPI read
that produces a chunk runs *before* the queue call with the IRQ enabled, so USB
packet shipping overlaps the next chunk's SPI read.

For host→device `WRITE`, the parser holds a fully-received `DAT` chunk in
`USB2P_RX_HELD` while the previous chunk's commit is still in flight. No bytes are
consumed in that state, so the USB OUT pipe NAKs at the hardware level and the
host is back-pressured without a logical NAK; the main-loop poll retries the
dispatch once the commit completes.

---

## 13. Coexistence with legacy CDC

USB2P shares the single CDC-ACM (usb2snes) data endpoint with the legacy
protocol: a connection speaks legacy CDC until a client issues the upgrade
command, after which the same endpoint carries USB2P framing for the rest of the
session (reset by CDC close / config reset). A client that never upgrades sees an
unchanged usb2snes device, so existing usb2snes / QUsb2Snes / SNI clients keep
working. USB2P does not raise raw bandwidth — both MCUs are full-speed only and
the real limiters are the SD card and FPGA SPI back-ends. The wins are stability
(self-delimiting framing with resync), a typed asynchronous event channel, and
the richer filesystem surface — all without a second USB interface or a WinUSB
driver bind.
