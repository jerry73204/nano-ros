---
id: 814
title: "The whole zero-copy surface sits behind `feature = \"lending\"`, which only
  a posix test crate ever enables"
status: open
type: test-gap
area: rmw
related: [issue-0812, issue-0813, phase-391]
---

## Problem

`SlotLending` (publish) and `SlotBorrowing` (subscribe) are both
`#[cfg(feature = "lending")]`. The only crate in the tree that turns the
feature on:

```toml
# packages/testing/nros-tests/Cargo.toml:98
nros-rmw-zenoh = { path = "...", features = ["lending", "platform-posix"], optional = true }
```

`platform-posix`. So the zero-copy path is exercised **only on a desktop host**,
against the posix platform, and never on any embedded target — which is the
only place the feature's stated benefit (RAM-tight, copy-count-sensitive)
applies. A scan of the mr_canhubk3/s32k344 image finds one loan-related symbol
and no live path.

## Why it is not merely "untested"

Two defects were found by *reading* the code during a memory-allocation review,
not by any test:

- [issue 0812](archived/0812-publisher-loan-heap-allocates-per-loan.md) — a `Box::new`
  per loan, i.e. a malloc on the zero-copy path
- [issue 0813](archived/0813-zenoh-tx-buf-hardcoded-and-unpriced.md) — a hardcoded 1 KiB
  ceiling that the feature's own use cases exceed

Both are the kind of thing an embedded lane would have surfaced immediately.
Neither is visible from a posix host with a large heap and no size budget.

## What the gap actually is

`RFC-0010` records that loan/borrow are **exclusively raw** — the lent slot is
`len` bytes, because CDR length is not known before encoding, so
`Publisher<M>` has no typed `loan()` dual. That makes the feature's users
precisely the byte-oriented embedded backends (uORB on PX4, and any
POD-struct transport), and those are exactly the targets no lane builds with
`lending` on.

Minimum useful coverage: one embedded lane (qemu tier is enough) building with
`lending` enabled, exercising loan -> commit and try_borrow -> drop.

---

## Research: is raw-byte the right shape? (2026-09-03)

The brief was to question the shape, not the lane. Three vendored backends were read
(zenoh-pico 1.7.2, Cyclone DDS **0.10.5** — the version ROS ships, not 11.x — and
Micro-XRCE-DDS Client 3.0.1 / microCDR 2.0.2), plus the upstream `rmw` contract and
our own ABI. Submodule citations were read in the primary checkout, where the
submodules are populated; paths are repo-relative.

Short answer: **the byte-ness is right and the LOAN/TOKEN LIFECYCLE is wrong.** No
backend we ship can hand back a flat writable `&mut [u8]` into its transport buffer
and later be told to forget it, which is precisely what `try_loan` / `commit` /
`discard` promises. The tree already ships a differently-shaped spelling of the same
capability that two of three backends fill natively, ungated by any feature —
`publish_streamed` and `process_raw_in_place`.

### 1. Upstream's loan is TYPED, and that is the whole divergence

The six loan symbols are all in the derived 88-symbol implementation contract —
every `librmw_*_cpp.so` defines them (`docs/reference/rmw-implementation-contract.txt:14,63,72,73,92,93`).
Their signatures are typed and carry no length:

```
rmw_borrow_loaned_message  const rmw_publisher_t *, const rosidl_message_type_support_t *, void **
rmw_take_loaned_message    const rmw_subscription_t *, void **, bool *, rmw_subscription_allocation_t *
```
(`docs/reference/rmw-implementation-signatures.txt:7,151`)

The typesupport pointer *is* the size: upstream never asks "how many bytes", because
the loaned object is a sample of a known type in a known layout. Ours asks for a
length and hands back a span plus an opaque token
(`packages/core/nros-rmw-abi/include/nros/rmw_vtable.h:536-539`), a deviation the
header already declares
(`packages/core/nros-rmw-abi/include/nros/rmw_vtable.h:587-589`).

That deviation was not a mistake — rosidl typesupport is declined ABI-wide, so a
typed loan in *upstream's* spelling was never available to us. But it changes what
the slot can mean: upstream's loan is "give me a `T`", ours is "give me `n` bytes",
and only the first is a thing a DDS implementation with shared memory can serve.

`take_loaned_message` is carried and filled by nobody
(`docs/reference/rmw-api-map.toml:412-415`; NULL in Cyclone at
`packages/rmw/cyclonedds/nros-rmw-cyclonedds/src/vtable.cpp:318,366`, NULL in XRCE at
`packages/rmw/xrce/nros-rmw-xrce/src/vtable.c:74`), decided KEPT in issue 0781 for a
reason that still holds: it is the only shape in this ABI that can hand a view to a
caller which outlives the call, which is what `nros-c` / `nros-cpp` `try_borrow` do.

### 2. Per backend: what zero-copy actually exists

**zenoh-pico 1.7.2 — no loan API; the alias saves one copy at the container level and
the transport copies anyway.**

`z_loan()` / `z_loan_mut()` are a handle-borrow idiom (`&obj->_val`,
`include/zenoh-pico/api/olv_macros.h:120-123`) and have nothing to do with payload
lending. The payload-aliasing family is `z_bytes_from_static_buf`
(`include/zenoh-pico/api/primitives.h:658`), which stores the caller pointer in a
slice with a NULL deleter (`src/api/api.c:279-283`) — no copy *there*, which is what
`zpico_publish_with_attachment_aliased` uses
(`packages/rmw/zenoh/zpico-sys/c/zpico/zpico.c:2743-2775`).

But the encode step always memcpys. `_z_buf_encode` takes its no-copy `wrap` branch
only for an *expandable* wbuf (`src/protocol/codec.c:236-245`), and every transport's
TX buffer is created non-expandable — `_z_wbuf_make(mtu, false)`
(`src/transport/unicast/transport.c:60`, `src/transport/multicast/transport.c:77`),
which sets `_expansion_step = 0` (`src/protocol/iobuf.c:202-210`). The fragmented path
*can* wrap, then siphons with a `memcpy` into the fixed buffer that is actually sent
(`src/protocol/iobuf.c:478-506`). **So on the RTOS path the payload is copied into the
wire buffer regardless of whether we aliased it.** What `lending` saves on zenoh is
one copy from a user buffer into our own 1 KiB `LendArena` — a saving against
`publish_raw`'s `z_bytes_copy_from_buf`, not against the transport.

Receive side is worse than the RFC records. `SlotBorrowing for ZenohSubscriber` views
*our own* SPSC ring (`packages/rmw/zenoh/nros-rmw-zenoh/src/shim/subscriber.rs:1471-1497`),
which the C `sample_handler` already filled by copy
(`packages/rmw/zenoh/zpico-sys/c/zpico/zpico.c:978`, `z_bytes_reader_read`). It saves
the *second* copy (ring → user buffer), which `process_raw_in_place` already saves
without any feature (`.../shim/subscriber.rs:1269-1286`). The genuine borrow of
zenoh's own receive buffer is `z_bytes_get_contiguous_view`, wired into
`zpico_subscribe_zero_copy` (`packages/rmw/zenoh/zpico-sys/c/zpico/zpico.c:2260`) —
behind `Z_FEATURE_UNSTABLE_API`, which is off unless `unstable-zenoh-api` is on
(`packages/rmw/zenoh/nros-zpico-build/src/runner.rs:596`;
`packages/rmw/zenoh/zpico-sys/Cargo.toml:30,103` — not in `default`), so the compiled
arm is the stub returning `ZPICO_ERR_GENERIC`
(`.../zpico-sys/c/zpico/zpico.c:2313-2320`). **And it has no Rust caller anywhere.**

No shared memory: `Z_FEATURE_SHARED_MEMORY` and `z_shm_` have zero hits in the
vendored tree. SHM is a zenoh-c/Rust feature; zenoh-pico does not carry even a stub.

**Cyclone DDS 0.10.5 — a real loan, TYPED, and unreachable on every target we ship.**

`dds_loan_sample(dds_entity_t writer, void **sample)`
(`third-party/dds/cyclonedds/src/core/ddsc/include/dds/ddsc/dds_loan_api.h:90`;
version confirmed `project(CycloneDDS VERSION 0.10.5 …)` at
`third-party/dds/cyclonedds/CMakeLists.txt:13`). The pointer is sized
`m_stype->iox_size` (`src/core/ddsc/src/dds_loan.c:186`), which is the topic
descriptor's `m_size`, i.e. `sizeof(TopicType)`
(`src/core/ddsi/src/ddsi_sertype_default.c:305`) — **typed struct storage, not CDR
bytes.** `dds_request_loan` does not exist in this version.

It requires all three of: `DDS_HAS_SHM` compiled in, a live iceoryx pub/sub on that
entity, and a `fixed_size` type (`src/core/ddsc/src/dds_loan.c:48-80,185-192`).
Missing any one is `DDS_RETCODE_UNSUPPORTED` — there is **no heap fallback**
(`src/core/ddsc/src/dds_loan.c:168-172`). `ENABLE_SHM` defaults to `AUTO` and is
forced off when `find_package(iceoryx_binding_c)` fails
(`third-party/dds/cyclonedds/src/CMakeLists.txt:48,58-59`); iceoryx is not vendored
here at all, and the fork's carried commits touch none of this
(`docs/reference/cyclonedds-fork-delta.md`). iceoryx needs POSIX shared memory and a
separate RouDi daemon — there is no RTOS build of it in this tree and no gating that
contemplates one. Our Cyclone vtable NULLs every loan slot
(`packages/rmw/cyclonedds/nros-rmw-cyclonedds/src/vtable.cpp:315-318`), correctly.

Note the trap the word "loan" sets here: the *classic* DDS reader loan
(`dds_return_loan` after a `dds_read`/`dds_take` with a NULL buffer,
`src/core/ddsc/src/dds_read.c:512-556`) is a plain reader-owned heap buffer with no
SHM involvement, and is a different mechanism sharing the name.

**Micro-XRCE-DDS 3.0.1 — the one genuine transport-buffer loan, and it cannot be a
flat slice.**

`uxr_prepare_output_stream(session, stream, entity, ucdrBuffer *ub, uint32_t len)`
(`packages/rmw/xrce/xrce-sys/micro-xrce-dds-client/include/uxr/client/core/session/write_access.h:106-111`)
hands back a `ucdrBuffer` whose iterator points **into the session's own output
stream buffer** (`.../src/c/core/session/stream/output_reliable_stream.c:62,81,96`),
and those bytes reach `send()` with no further copy
(`.../src/c/core/session/session.c:614,632,765-774` →
`.../src/c/profile/transport/ip/udp/udp_transport_posix.c:76`). No shared memory is
involved: the optional SHM *transport* profile defaults OFF
(`.../micro-xrce-dds-client/CMakeLists.txt:68`) and its macro expands to nothing when
disabled (`.../src/c/core/session/stream/shared_memory_internal.h:37-45`).

Two properties make the loan/commit/discard shape wrong for it:

* **The length is required up front, and upstream's own example computes it from the
  TYPE** — `uint32_t topic_size = HelloWorld_size_of_topic(&topic, 0);` immediately
  before the prepare call
  (`.../micro-xrce-dds-client/examples/PublishHelloWorld/main.c:133-136`). So
  "up-front length" is idiomatic here, not a concession.
* **The granted region is not contiguous past one block.** A `len` spanning several
  history blocks is chained, and microCDR re-points the same `ucdrBuffer` at the next
  block through an `on_full_output_buffer` callback
  (`.../src/c/core/session/stream/output_reliable_stream.c:100-165,287-305`). Default
  transport MTU is 512 (`.../micro-xrce-dds-client/CMakeLists.txt:60-65`). A flat
  `&mut [u8]` cannot express that; a refill callback can.
* **There is no cancel.** Our own port records it: "the slot is COMMITTED: `uxr` has
  no cancel for a prepared output stream"
  (`packages/rmw/xrce/nros-rmw-xrce/src/publisher.c:288-296`). So XRCE can never
  implement `return_loaned_message_from_publisher`, and its vtable NULLs the loan trio
  (`packages/rmw/xrce/nros-rmw-xrce/src/vtable.c:71-74`) while filling
  `publish_streamed` (`.../src/vtable.c:90`).

### 3. The tree already has the right shape, ungated

`publish_streamed` (`packages/core/nros-rmw-abi/include/nros/rmw_vtable.h:713-758`)
takes a `size_cb` + a repeated `chunk_cb` and no token. It is filled natively by XRCE
(`packages/rmw/xrce/nros-rmw-xrce/src/vtable.c:90`) and by zenoh, where it assembles
the payload **inside zenoh's own `z_owned_bytes_t` via `z_bytes_writer`** rather than
in a separate arena (`packages/rmw/zenoh/nros-rmw-zenoh/src/shim/publisher.rs:354-410`)
— strictly better than what `lending` does. Cyclone leaves it NULL
(`packages/rmw/cyclonedds/nros-rmw-cyclonedds/src/vtable.cpp:335`) and gets the
4 KiB-stack staging default (`packages/core/nros-rmw/src/traits.rs:1755-1800`). The
user-facing entry point is public and behind no feature at all:
`EmbeddedPublisher::publish_streamed(total_len, writer)`
(`packages/core/nros-node/src/executor/handles.rs:179`).

Its receive twin is `process_raw_in_place`, always on for zenoh
(`packages/rmw/zenoh/nros-rmw-zenoh/src/shim/subscriber.rs:1269-1286`).

So the capability `lending` exists to deliver is already delivered, on more backends,
with no feature, no arena, no 1 KiB ceiling and no token to leak. The difference is
scope: `publish_streamed` / `process_raw_in_place` end when the callback returns; the
loan pair does not. That difference is real and is exactly why 0781 kept the slots —
but it is a difference about *who may hold the view*, not about copies.

### 4. Does CDR-length-unknown force raw? No — and the tree already computes the bound

RFC-0010 D7 argues a typed `loan()` is impossible because the CDR length is only known
after encoding. That was true when it was written (2026-04). Since phase-380 it is
false for bounded types:

* `Message::MAX_SERIALIZED_SIZE_XCDR1` / `_XCDR2` are `const Option<usize>` computed
  from `FIELDS` (`packages/core/nros-serdes/src/schema.rs:158-163`).
* `Message::IS_PLAIN` — "no variable-length member anywhere: the size above is EXACT
  and **the type is loan-eligible**" (`packages/core/nros-serdes/src/schema.rs:166-171`).
* `size::is_loan_eligible::<M>()` exists, is `M::IS_PLAIN`, and phase-380 W5 says in as
  many words that "wiring it into `borrow_loaned_message` … is left to whoever owns
  those slots" (`packages/core/nros-serdes/src/size.rs:569`;
  `docs/roadmap/archived/phase-380-serialized-size-bound.md:228-231`). Nothing outside
  a test calls it.
* `size::serialized_size(&value, version)` gives the EXACT length of *this* message by
  running the real writer with stores disabled
  (`packages/core/nros-serdes/src/size.rs:454`).

And D7's second objection — that over-reserving `MAX_SIZE` "wastes wire bandwidth" —
does not hold either: the commit already carries an `actual_len` and truncates
(`packages/rmw/zenoh/nros-rmw-zenoh/src/shim/publisher.rs:574`;
`nros_publisher_commit(actual_len)`), so over-reservation costs arena RAM, not wire.

Worth noting how exactly `IS_PLAIN` matches Cyclone's own gate: Cyclone loans only
`fixed_size` types (`third-party/dds/cyclonedds/src/core/ddsc/src/dds_loan.c:62,69`).
The two predicates are the same predicate. That is the strongest available evidence
that a typed loan for plain types is the *natural* shape — and also that its only real
consumer is a Cyclone-with-iceoryx deployment, which is not a target we ship.

One cost to record against a typed dual: not every message type has a
`schema::Message`. `examples/native/rust/custom-msg` hand-writes `RosMessage` +
`Serialize` + `Deserialize` and has no schema; phase-380 W4 was reverted the same day
for breaking exactly that
(`docs/roadmap/archived/phase-380-serialized-size-bound.md:245-256`). A typed loan
keyed on `Message` would silently exclude hand-written types.

### 5. What 0812 and 0813 imply

**Both are symptoms of the shape, not merely bugs in it — but they are honest bugs too.**

* **0812 (a malloc per loan) is a symptom.** A scoped callback needs no token, so it
  has nothing to allocate; a token that outlives the call must be manufactured, and on
  a `void **` it either boxes or encodes an integer. The fix encoded an integer, which
  works only because the arena is single-slot and the publisher comes back as a
  parameter — i.e. it works because *there is no real backend loan to describe*. A
  backend with genuine per-loan state (a `ucdrBuffer` cursor; an iceoryx chunk pointer)
  could not be described by a tagged length, and the malloc would come back.
  The class is also not closed: the cffi **fallback** still does
  `Box::new(ArenaStaging { buf: vec![0u8; len] })` per loan
  (`packages/rmw/cffi/src/lib.rs:2765-2779`) — two allocations, on the path every
  backend except zenoh takes — and `nros-c`'s borrow still boxes a view per call
  (`packages/api/nros-c/src/subscription.rs:754`).
  Worse, without `alloc` that fallback returns `Ok(None)`
  (`packages/rmw/cffi/src/lib.rs:2788-2792`), which `try_loan` maps to
  `LoanError::WouldBlock` (`packages/core/nros-node/src/executor/handles.rs:777`) —
  so on a heap-free image with Cyclone or XRCE, `try_loan` is a **permanent
  WouldBlock** rather than an error, and `loan_with_timeout` spins to its timeout.
  That is a silent-hang shape, and it is a genuine bug regardless of the shape debate.

* **0813 (the 1 KiB ceiling) is a bug in the implementation.** The ceiling is ours
  (`packages/rmw/zenoh/nros-rmw-zenoh/src/shim/publisher.rs:492,536`), not the
  backend's; zenoh-pico imposes no such limit on a put. But the ceiling only exists
  because the loan needs a flat contiguous region reserved before the first byte is
  written, and a flat region on an MCU has to be a fixed static array. `publish_streamed`
  has no ceiling on zenoh at all, because it never needs the whole payload contiguous
  on our side. So the *number* is an implementation bug; the *existence* of a
  per-publisher static arena is the shape.

### 6. How unreachable it actually is (measured, beyond the issue's original claim)

* `nros-rmw-zenoh-staticlib` — the crate every C/C++ and RTOS image links — declares
  **no `lending` forwarder at all** (`packages/rmw/zenoh/nros-rmw-zenoh-staticlib/Cargo.toml:26-98`).
* The string `lending` appears **zero times** in `cmake/` and `zephyr/`. No
  CMake-driven build can turn it on.
* The committed cbindgen header declares `nros_publisher_loan` unconditionally
  (`packages/api/nros-c/include/nros/nros_generated.h:5560`) with no `#ifdef` guard,
  while the Rust symbol is `#[cfg(feature = "lending")]`
  (`packages/api/nros-c/src/publisher.rs:552,598,629`). A C user who reads the shipped
  header and calls it gets an undefined-symbol link error, not a compile error.
* `can_loan_messages` — the ABI's own capability flag, mirroring upstream's
  `rmw_publisher_t` field — is written `false` by Cyclone
  (`packages/rmw/cyclonedds/nros-rmw-cyclonedds/src/publisher.cpp:117`) and XRCE
  (`packages/rmw/xrce/nros-rmw-xrce/src/publisher.c:103`), and is **never written by
  the Rust adapter** (`packages/rmw/cffi/src/rust_adapter.rs:541-548`). So even a
  `lending` zenoh build, whose vtable does fill the loan slots
  (`packages/rmw/zenoh/nros-rmw-zenoh/src/lib.rs:428-431`), reports "cannot loan". The
  only `true` in the tree is a test backend
  (`packages/rmw/cffi/tests/loan_native.rs:67`).
* **No example, anywhere, calls `try_loan` / `try_borrow` / `nros_publisher_loan`.**
* There is no book chapter — RFC-0010's own 99.K deliverable was never written.

What coverage *does* exist, all posix: `loan_fallback`, `loan_native` and
`lending_traits` in `just check required-features-tests`
(`just/check.just:669-675`), and `loan_e2e` — which does run, in `just test` and
`just test-all`, through `test-zpico-multisession` (`justfile:1163,1191-1195`). The
issue's "only a posix test crate enables the feature" is accurate; "never exercised"
is slightly stronger than the truth, because the posix E2E is real and asserts a
zero-alloc window.

## Recommendation

**(C), and it is closer to "stop paying for A" than to B.** Keep the byte-span loan
ABI — 0781's reason for it is sound and unchanged, and the byte-ness is not the defect
— but stop treating `feature = "lending"` as the tree's zero-copy story, because
`publish_streamed` / `process_raw_in_place` already are that story on more backends
with a better shape. Do **not** add a typed dual now: its natural consumer is
`IS_PLAIN` over Cyclone-with-iceoryx, which no target we ship can reach.

Concretely, in the order the evidence supports:

1. **Make the unreachability loud rather than silent.** Either guard
   `nros_publisher_loan` / `_commit` / `_discard` / `nros_subscription_borrow` in the
   generated header behind an `NROS_LENDING` macro the build defines, or accept the
   symbols are undefined and say so in the header. A shipped header advertising a
   symbol no shipped library defines is the same "reads as coverage" failure the
   vacuous-test gate exists for, one layer out.
2. **Fix the heap-free WouldBlock.** `try_lend_slot`'s no-`alloc` arm returning
   `Ok(None)` should be an error (`Unsupported`), not a transient. As written, the
   heap-free tier's `loan_with_timeout` burns its whole budget for a condition that
   can never clear.
3. **Make `can_loan_messages` agree with the vtable**, in whichever direction is true.
   Today it is `false` on the one backend that can loan.
4. **Close the fallback's remaining per-loan allocations** (cffi `ArenaStaging`,
   `nros-c` borrow box) — 0812's own "what this does NOT yet buy" list, still open.
5. **Wire `is_loan_eligible::<M>()` into the one place it was written for**, if and
   when a typed loan is ever built. Until then, leave RFC-0010 D7 amended rather than
   acted on: record that the length objection now holds only for *unbounded* types,
   and that a bounded type's bound is a compile-time constant.
6. **Amend RFC-0010's per-backend matrix**, which is stale in three rows: XRCE has no
   `lending` feature at all (`packages/rmw/xrce/nros-rmw-xrce/` declares none) though
   the matrix says "landed"; zenoh's native receive is behind a non-default
   `unstable-zenoh-api` and has no caller; and "dds — gated on SHM transport" should
   say that 0.10.5's SHM loan is typed and iceoryx-only, which is a different
   statement from "not yet".

### Per-backend one-liners

* **zenoh-pico 1.7.2** — no loan API; `z_bytes_from_static_buf` aliases at the
  container level, but the transport memcpys into a non-expandable wire buffer before
  the socket either way. Receive-side true borrow exists (`z_bytes_get_contiguous_view`)
  behind a non-default flag, with no caller. No SHM at all.
* **Cyclone DDS 0.10.5** — a real, TYPED loan (`dds_loan_sample` → `sizeof(TopicType)`),
  requiring `DDS_HAS_SHM` + a live iceoryx endpoint + a `fixed_size` type, with no heap
  fallback. Unreachable on any RTOS we target; iceoryx is not even vendored.
* **Micro-XRCE-DDS 3.0.1** — the only genuine loan into a transport buffer
  (`uxr_prepare_output_stream`), bytes not typed, length mandatory up front, no cancel,
  and non-contiguous past one block. It fits `publish_streamed` exactly and cannot fit
  loan/commit/discard, which is why our port fills the former and NULLs the latter.

### On the proposed test lane

The issue proposes "one qemu-tier lane building with `lending`, exercising
loan → commit and try_borrow → drop". That is **necessary but not sufficient, and not
buildable today**: no CMake or Zephyr path enables the feature, and
`nros-rmw-zenoh-staticlib` has no forwarder, so the lane would first need a build
plumbing change — which this task deliberately does not make.

More importantly, say what it would prove. It would prove our own 1 KiB arena and the
aliased put work on an RTOS, and that the token encoding survives a 32-bit target. It
would **not** prove a copy was avoided, because zenoh-pico copies into the wire buffer
regardless (§2). A lane that green-lights "zero-copy works on hardware" while every
byte is still copied is the wrong claim to buy.

Two cheaper things buy more, and both are stated as options for a separate decision,
not enabled here:

* **A compile ratchet.** `cargo check -p nros-rmw-cffi --no-default-features --features
  lending --target thumbv7em-none-eabi` and `cargo check -p nros-node
  --no-default-features --features rmw-cffi,rmw-lending --target thumbv7em-none-eabi`
  both pass today (measured, this session). That is a no_std no_alloc gate costing
  seconds, and it is the gate that would have caught the two files 0779 found not
  compiling. It belongs beside the existing `lending` entries in
  `check::required-features-tests`.
* **A copy-count assertion, not a delivery assertion.** `loan_e2e` already installs a
  counting allocator; the thing worth measuring is memcpy'd bytes on the publish path,
  and the honest place to measure it is `publish_streamed` versus `publish_raw`, on the
  cells that already exist. If that measurement shows no difference on zenoh — which
  §2 predicts — then the right follow-up is to retire `lending`'s publish half rather
  than to build a lane for it.

---

## What landed (2026-09-03) — recommendation (C)

Implemented as argued above: the byte-span ABI is kept, the honesty defects are
fixed, and `publish_streamed` / `process_raw_in_place` are named as the supported
zero-copy surface. No typed dual, no ABI change, no lane enabling `lending`.

Two things the study did not know, both found by trying to build what it
described:

* **The C loan surface had not COMPILED since issue 0812 landed.**
  `cargo check -p nros-c --features rmw-cffi,lending` failed with seven errors on
  `main`. Three were the loan trio: 0812 retyped the backend token from
  `*mut c_void` to `*mut rmw_loan_token_t` (`packages/rmw/cffi/src/lib.rs:2963,3012,3029`)
  and left the three FFI call sites behind. Four were the receive half using
  `alloc::boxed::Box` while `extern crate alloc` is `#[cfg(feature = "alloc")]`
  — so `lending` without `alloc` was not a build with a missing capability, it
  was not a build. `nros-cpp` is broken the same way and is NOT fixed here; its
  fifth blocker is the deliberate `T::BackendDynamic` tripwire at
  `packages/api/nros-cpp/src/lib.rs:802`, whose own comment predicts exactly
  this and whose resolution is a cross-crate feature-observation question with
  issue 0586's exhaustiveness policy attached.
* **`can_loan_messages` had drifted in BOTH directions at once**, not just the
  one the study measured. The under-claim is as described (nothing outside a
  test writes `true`; `RustBackendAdapter`'s generic trampoline structurally
  cannot). The over-claim is its mirror: a backend writing `true` with a NULL
  slot was believed, and the test stub did precisely that. It is now DERIVED
  from the vtable slot, so the two spellings cannot disagree.

### On 0812 and 0813 — judged, not fixed

Both are already `resolved` (`5af7e1e44`); what the study flagged is the residue
0812 itself listed under "what this does NOT yet buy". **Neither residue is worth
closing in place, for the same reason:**

* The cffi `ArenaStaging` `Box` + `vec![]` per loan
  (`packages/rmw/cffi/src/lib.rs:2765-2779`) is on the FALLBACK path — taken only
  when the backend has no loan slot, i.e. Cyclone / XRCE / uORB. On those the
  "zero-copy" loan is already a staging buffer plus a memcpy at commit; it is not
  a zero-copy path. Removing the malloc means a static arena in the cffi layer,
  which is issue 0813's shape reintroduced one layer DOWN — a per-publisher
  ceiling priced into every image on every backend, instead of one backend's.
  That is a worse trade than the malloc.
* `nros-c`'s per-borrow `RecvView` box has the same alternative and the same
  cost. It is now DECLARED instead: the `cfg` on `nros_subscription_borrow` /
  `_release` says `all(lending, alloc)`, so the heap requirement is visible
  rather than a compile error.

The honest move — taken — is that `publish_streamed` is the supported path, and
it is now said where a caller will actually read it: in the committed C/C++
headers, and in the error text a refused heap-free loan produces. It has no
token, no arena, no ceiling and no heap, and XRCE and zenoh fill it natively
while all three backends NULL the loan trio.

### The ratchet, and what is still open

`just check no-std` gained the heap-free lending slice on
`thumbv7m-none-eabi` and `riscv32imc-unknown-none-elf`;
`just check workspace-features` gained `nros-c`'s two lending rows (host — its
build script needs a cross C toolchain, which the `no-std` lane excludes). Both
rows passed before they were written; this is a ratchet, and what it buys is
that they cannot quietly stop. It is what would have caught the `nros-c` break.

**This issue stays `open`.** The ratchet COMPILES the lending surface on
embedded targets; nothing RUNS it, which is the gap in this issue's title. The
study's argument against the proposed QEMU lane still stands — zenoh-pico
memcpys into a non-expandable wire buffer either way, so such a lane would be
green while every byte is still copied — so the useful successor is the
copy-count measurement in §"On the proposed test lane", not a delivery test.
