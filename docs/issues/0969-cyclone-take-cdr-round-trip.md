---
id: 969
title: "The Cyclone RMW deserializes every received sample and re-serializes it, so `take_serialized` costs a decode, an encode and two heap allocations per take"
status: open
area: [rmw, memory]
severity: high
related: [0958, 0781, 0896, phase-391, phase-403, rfc-0035, 0038]
---

# We take CDR off the wire, decode it, encode it again, and hand back the second copy

## What the code does

`nros-rmw-cyclonedds`'s `subscription_take` (`src/subscriber.cpp:134`) runs this
sequence for every sample:

```cpp
void* sample = ddsrt_calloc(1, state->desc->m_size);   // typed sample, heap
dds_return_t taken = dds_take(state->reader, samples, si, 1, 1);
// ... Cyclone deserializes wire CDR into `sample`, allocating for every
//     variable-length member as it goes ...
dds_ostream_t os;
dds_ostream_init(&os, 0, 1 /*xcdr1*/);                 // starts empty, grows by realloc
bool ok = dds_stream_write_sample(&os, sample, state->st->as_sertype());
// ... re-serializes the typed sample back to CDR ...
uint32_t paylen = os.m_index;
uint32_t total = paylen + 4;
if (buf_len < total) { /* NROS_RMW_RET_BUFFER_TOO_SMALL */ }
```

The caller asked for bytes. The bytes were already there. We decoded them,
allocated a typed struct plus one block per variable-length member, allocated and
grew an output stream, encoded the struct back to bytes, copied those into the
caller's buffer, and freed everything.

The publish direction is the mirror image: CDR bytes → `dds_stream_read_sample` →
typed buffer → `dds_write` → Cyclone serializes again.

## This is deliberate, and the reason it was accepted no longer holds

`src/sertype_min.hpp:6-29` states the tradeoff outright:

> Cyclone's `dds_writecdr` / `dds_takecdr` raw-CDR API needs a real
> `ddsi_sertype *` linked to a `ddsi_domaingv`, which our backend can't get to
> without reaching into Cyclone's private struct layout. We sidestep that path
> entirely: […]
> **Cost: a 2× CDR roundtrip per publish + per recv.** Acceptable for an in-tree
> smoke; low-throughput control loops on Cortex-A/R safety MCUs run well under the
> headroom. A future zero-copy fast path can replace this once Cyclone exposes
> `dds_writer_lookup_serdatatype` upstream.

Two things are wrong with that reasoning as it applies to **receive**.

**The stated blocker is transmit-only.** `dds_takecdr` takes no sertype:

```c
dds_return_t dds_takecdr(dds_entity_t reader_or_condition,
                         struct ddsi_serdata **buf, uint32_t maxs,
                         dds_sample_info_t *si, uint32_t mask);
```

(`third-party/dds/cyclonedds/src/core/ddsc/include/dds/dds.h:3783`.) The reader
already owns its sertype from `dds_create_topic(desc)`. `dds_writer_lookup_serdatatype`
is what the *publish* path would need, to build a serdata from raw bytes. Receive
never needed it. The blocker was written once and applied to both directions.

**"Acceptable for an in-tree smoke" is no longer the deployment.** The
autoware-safety-island an536 lane is a real consumer running a control loop over
this backend, and it is the lane that motivated issues 0896, 0917 and 0958.

## What the reference implementation does

`ros2/rmw_cyclonedds`'s `rmw_take_ser_int` (`rmw_cyclonedds_cpp/src/rmw_node.cpp:3572`)
is the shape we should have:

```cpp
while (dds_takecdr(sub->enth, &d, 1, &info, DDS_ANY_STATE) == 1) {
  size_t size = ddsi_serdata_size(d);
  rmw_serialized_message_resize(serialized_message, size);
  ddsi_serdata_to_ser(d, 0, size, serialized_message->buffer);
  serialized_message->buffer_length = size;
  ddsi_serdata_unref(d);
```

One `memcpy` out of the serdata. No typed sample, no member allocations, no
ostream, no re-serialization. `ddsi_serdata_to_ser` on Cyclone's default sertype
reads the CDR the serdata is already holding.

## Three costs, not one

**Throughput and jitter.** Per take: two-plus heap allocations, a full decode, a
full encode. On a Cortex-R safety MCU this sits directly in the receive path of a
control loop.

> **Now measured** (`tests/codec_bench.cpp`, section "The CPU cost: MEASURED"
> below). The decode+encode pair costs a **~46 ns floor on every message**
> regardless of size, rising to 176 ns at a 16 KB payload; the new path's whole
> per-message payload work is a `memcpy` at 0.8-76 ns over the same range. The
> allocations are the NULL RESULT below — count unchanged, bytes crossing over
> at ~6 KB. So of the three costs asserted here, the CPU one is real at every
> size and the allocation one is a trade, not a win.

**Real-time bound.** [phase 391](../roadmap/phase-391-allocation-unification-and-tier-model.md) argues
the heap holds infrastructure while payload buffers stay static, and derives a
Robson bound from that. Every Cyclone take allocates a *payload-sized* block, and
the ostream's growth-by-realloc allocates a second one whose size depends on the
sample. The bound as stated does not describe a Cyclone image.

**Correctness of the bytes, not only their cost.** `dds_ostream_init(&os, 0, 1)`
requests **XCDR1 in native byte order**. A ROS 2 publisher emits XCDR2, and a
big-endian peer emits big-endian. So the caller does not receive the wire
representation — it receives a re-encoding, with an encapsulation header we
synthesized. This is why the sizing work has to reason about
`MAX_SERIALIZED_SIZE_XCDR1` for this backend while the wire carries XCDR2
(see [#0964](archived/0964-two-different-sizes-for-the-same-type.md)). Taking the serdata's
own bytes removes the discrepancy rather than documenting it.

## Direction

Rewrite `subscription_take` to the `dds_takecdr` + `ddsi_serdata_to_ser` shape:
take the serdata, read `ddsi_serdata_size`, refuse with
`NROS_RMW_RET_BUFFER_TOO_SMALL` when the caller's buffer is smaller, otherwise one
`memcpy` into it, then `ddsi_serdata_unref`. The caller-owns-the-buffer contract
costs exactly one copy; everything above that copy is removable today, inside this
backend, with no change to Cyclone.

`subscription_take_multi` (`src/subscriber.cpp:281`) has the same body and gets
the same treatment. The request path in `src/service.cpp:657` mirrors it and
should be checked, not assumed.

**Not in scope here:** the publish direction, which genuinely needs the sertype we
do not own — that is [#0970](archived/0970-cyclone-rmw-should-own-its-sertype.md), and it
subsumes this fix if it lands first.

**Also not in scope:** filling the ABI's `take_loaned_message` slot for this
backend. Upstream returns `RMW_RET_UNSUPPORTED` for it without shared-memory
support, and even its SHM path ends in a `memcpy`; see the amendment to
[design 0038](../design/0038-zero-copy-data-transport.md). Removing the round trip
is the whole of the available win on the buffered path.

## Verification — measured

**Allocation count: 9.93 per message → 2.00.** `data_roundtrip` gained
`NROS_ROUNDTRIP_ITERS`; two runs at different counts under valgrind cancel
session and entity setup and give the per-message cost as a slope. Same harness,
this backend against `origin/main`'s:

| | allocs @1 | allocs @200 | per message |
| --- | ---: | ---: | ---: |
| before | 984 | 2,960 | **9.93** |
| after | 968 | 1,366 | **2.00** |

The remaining 2 are one serdata object and its payload buffer — on the loopback
path Cyclone hands the same serdata to the local reader by reference, so one
message costs one serdata.

The before figure is not an integer, and that is information: the old path's
ostream grew by `realloc`, so its allocation count depended on the sample. Part
of what the round trip cost varied with the message, which is the property a
real-time budget most dislikes. The after figure is exactly 2.00 across 199
messages.

**Correcting this section as first written:** it said the
[phase 394](../roadmap/phase-394-memory-campaign-ledger.md) ledger could report
allocation count per take. It cannot — that instrument reads static RAM out of an
ELF symbol table. Runtime allocation needed an instrument and did not have one.

**Byte-identity: confirmed, with a qualification that matters for sizing.**
`ros2_pubsub_e2e` now prints what a real ROS 2 Humble peer over stock
`rmw_cyclonedds` actually delivers (the payload was widened to 16 characters
first, because the old one came to exactly 24 bytes of CDR — already 4-aligned,
so it could not tell wire bytes from a padded re-encode):

```
WIRE=len:28 hdr:00010000 cdr:25
```

The backend adds nothing — `get_size` returns exactly what `from_ser` was handed.
But **transparent is not unpadded**: the three extra bytes are the RTPS
submessage's 4-byte alignment applied by the SENDER, and the encapsulation
options read `0000` rather than `0003`, so the pad length is not recoverable from
the header either. Two consequences survive this issue rather than being removed
by it — a deserialiser must tolerate trailing bytes (nros-serdes reads by
position, so it does), and a receive buffer cut to a type's exact
`MAX_SERIALIZED_SIZE` can be up to 3 bytes short of what a remote peer delivers.
That belongs to [#0964](archived/0964-two-different-sizes-for-the-same-type.md).

## The third site was CHECKED 2026-09-03 — still unconverted, and the reason is 0976

This issue said `src/service.cpp:657` "mirrors it and should be checked, not
assumed". Checked. It is NOT converted:

* `subscriber.cpp` uses `dds_takecdr` in 8 places — both `subscription_take` and
  `subscription_take_multi` carry the fix.
* `service.cpp`'s `take_typed_wire` (now line 671) still runs the full
  `dds_take` -> `dds_ostream_init(&os, 0, 1 /*xcdr1*/)` ->
  `dds_stream_write_sample` round trip this issue exists to remove. It is reached
  from the request path (line 1022) and the reply path (line 1412).

**And the interesting part: converting it would REMOVE adapters, not conflict
with them.** The first read of this is that the five action adapters
([#0976](archived/0976-service-action-adapters-tested-only-against-ourselves.md)) block the
change, because they reshape bytes the typed path produces. The direction matters:

* `strip_goal_id_len_at` and `strip_nested_cdr_at` correct bytes WE generate.
  A raw `dds_takecdr` returns the PEER's bytes, which a conforming ROS 2 peer
  already emits correctly — so on receive there is nothing to correct.
* `take_fibonacci_get_result_response_wire` exists because
  `dds_stream_read_sample` CRASHES on that type (phase 171.0.b). Taking the
  serdata never calls the stream reader, so the crash path is not on the route.

So the receive half of 0976's adapter set looks like it falls out of this fix
rather than standing in its way. That is a claim about a byte-exact path in an
action protocol, and it is NOT verified here — it needs the Cyclone action E2E
fixtures built and `ros2_pubsub_e2e`'s witness extended to the action types, which
is a fixture build this session did not have room for.

**DONE 2026-09-03 — the third site is converted, and the prediction held.**

`take_typed_wire` now runs `dds_takecdr` -> `ddsi_serdata_size` ->
`ddsi_serdata_to_ser` into the caller's buffer, the same shape `subscriber.cpp`
already had. The typed sample, the `dds_ostream_t`, the re-encode and the two
heap allocations are gone from the request and reply paths.

The prediction above was that converting would REMOVE a receive adapter rather
than conflict with it. It did: `take_fibonacci_get_result_response_wire` existed
because `dds_stream_read_sample` crashes on that type (phase 171.0.b), and there
is no stream read on this path any more, so it is deleted. `write_fibonacci_get_
result_response` stays — it is on the WRITE side, which still decodes, and that
is issue 0970's half.

Acceptance, on fixtures rebuilt against the converted path:

| check | result |
| --- | --- |
| backend ctest suite | 23/23 |
| `ros2_action_e2e`, both directions, real ROS 2 peer | 2 passed |
| `test_native_cyclonedds_rust_action` (nros to nros) | passed |

One red appeared in the first suite run — `ros2_pubsub_e2e`, a lane this change
does not touch. It passed solo (11.4 s) and the whole suite passed 23/23 on
re-run, which is this repo's own guidance applied: retest a QEMU/e2e red SOLO
before believing it.

**This was only checkable because `ros2_action_e2e` exists** (issue 0976). Before
that witness, converting this path would have changed the action wire format with
nothing in the tree able to tell.

**Not measured, and not expected to move:** delivery rate at the fragment sizes
in [#0917](0917-an536-fragmented-sample-never-syncs.md). That cliff is the
LAN9118's RX FIFO capacity and has nothing to do with serialisation. What should
move on that lane is per-message CPU and allocation, so the rate below the cliff
and the jitter — an an536 measurement still owed.

## The third site, mapped — 2026-09-03

Read end to end and written down rather than converted. Everything below is
either quoted from the tree or marked as unverified; the one assumption that
decides whether the conversion is a half-hour job or a redesign is named at the
bottom.

### The conversion, concretely

`take_typed_wire` (`src/service.cpp:671`) mirrors what `subscription_take`
(`src/subscriber.cpp:236-268`) already does:

```cpp
struct ddsi_serdata* d = nullptr;
dds_sample_info_t si[1];
for (;;) {                                   // skip invalid_data, as the
    dds_return_t taken =                     // converted subscriber does
        dds_takecdr(reader, &d, 1, si, DDS_ANY_STATE);
    if (taken < 0)  return wire_status(NROS_RMW_RET_ERROR);
    if (taken == 0) return wire_status(NROS_RMW_RET_NO_DATA);
    if (si[0].valid_data) break;
    ddsi_serdata_unref(d);
    d = nullptr;
}
const uint32_t total = ddsi_serdata_size(d);  // counts the 4-byte CDRHeader
if (out_cap < total) { ddsi_serdata_unref(d); return wire_status(NROS_RMW_RET_BUFFER_TOO_SMALL); }
ddsi_serdata_to_ser(d, 0, total, out_buf);    // header + payload, wire form
ddsi_serdata_unref(d);
return static_cast<int32_t>(total);
```

That deletes, in one move:

* the `dds_ostream_init(&os, 0, 1 /*xcdr1*/)` + `dds_stream_write_sample` pair
  (`:689-691`) — the re-encode this issue is named for, and the reason the path
  emits **XCDR1 native-endian** rather than the peer's own representation. That
  is a correctness question, not only a cost one: the endianness bytes at
  `:701-707` are written from the HOST's `__BYTE_ORDER__`, so the bytes handed
  to `split_wire_header` are re-labelled rather than passed through;
* the `_SendGoal_*` / `_GetResult_Request_` memcpy fallback (`:692-714`), which
  exists only because `dds_stream_write_sample` returns false for those types;
* the `Fibonacci_GetResult_Response_` special case (`:682-687` calling
  `take_fibonacci_get_result_response_wire` at `:503`), which exists only
  because `dds_stream_read_sample` **crashes** on that type. A serdata take
  never enters either function.

### Two things that check out

**`split_wire_header` is transparent to this.** It already takes *wire* CDR —
encap, then the 8-byte request header, then user fields (`:585-604`) — and it is
called on `take_typed_wire`'s output at `:1022` (server request) and `:1412`
(client reply). A serdata take hands it the peer's bytes in the same shape the
reserialiser was reconstructing, so the strip logic is unaffected.

**The write-side half of the adapter hypothesis is now measured.** The
2026-09-03 section above guessed that converting this path would DELETE 0976's
adapters rather than have to preserve them. For the write side that is no longer
a guess: instrumenting both branches of `write_typed` and running the Rust, C
and C++ action clients against a stock ROS 2 server shows `strip_goal_id_len_at`
and `strip_nested_cdr_at` declining in all three with identical counts.

### The assumption that has to be checked first

`subscriber.cpp` creates its topic with `dds_create_topic_sertype`
(`subscriber.cpp:164`). `service.cpp` creates its with `dds_create_topic(desc)`
(`:946-947`, `:1221-1222`) and allocates a `SertypeMin` alongside (`:976-977`,
`:1248-1249`).

**Whether `dds_takecdr` yields usable serdata on a reader whose topic came from
a descriptor rather than a sertype is NOT verified here.** It is the kind of
thing that ought to work — serdata is Cyclone's internal representation either
way, and this issue's own argument for the receive path is that `dds_takecdr`
takes no sertype — but "ought to" is what this file exists to distrust. 0970's
step 4, checking this path's request/reply handling against upstream's
`cdds_request_wrapper_t`, is recorded as undone and is the same question from
the other side.

If it holds, the conversion is the block above plus deleting three helpers. If
it does not, the topics must move to `dds_create_topic_sertype` first — which is
0970's service half, a larger change, and the reason `sertype_min.{hpp,cpp}`
still exists.

### Why it was not converted here

Blast radius is services AND actions, and the direction with no automated test
is exactly the one the deleted helpers serve:
`take_fibonacci_get_result_response_wire` fires only when nano-ros is the CLIENT
taking a result (`:1412`), and the action witness (`ros2_action_e2e.rs`) runs
the server direction only. A rewrite of this path with no test that would catch
a regression is the shape this repo keeps retracting.

### Order for whoever takes it

1. Answer the `dds_takecdr`-on-a-desc-topic question — a throwaway assertion in
   `tests/` is enough, and it decides which of the two changes this is.
2. Land the client-direction witness 0976 asks for (nros action client against a
   stock `ros2` action server), so the three deletions have something watching
   them.
3. Convert, delete the three helpers, and re-run the allocation harness
   (`tests/data_roundtrip.cpp`, `NROS_ROUNDTRIP_ITERS`) — the message path went
   9.93 → 2.00 allocations/message and this path still carries three sites, one
   per request and two per reply.

## The allocation measurement, done 2026-09-03 — and it is a NULL RESULT

Step 3 of the order above said "re-run the allocation harness". Done, both ways,
and the answer corrects something I wrote when the migration landed.

`service_roundtrip.cpp` and the `ros2_srv_{client,server}` pair gained
`NROS_ROUNDTRIP_ITERS`, the same knob and the same slope method
`data_roundtrip.cpp` uses — deliberately the same, because a service number
measured a different way would not be comparable to the message path's
9.93 -> 2.00.

**Single process (client and server in one, the loopback case):**

| | iters=1 | iters=200 | per exchange |
| --- | ---: | ---: | ---: |
| before the migration | 1,200 | 1,996 | **4.00** |
| after | 1,178 | 1,974 | **4.00** |

**Two processes (the case a control loop actually runs), CLIENT half only:**

| | iters=1 | iters=200 | per exchange |
| --- | ---: | ---: | ---: |
| before | 1,445 | 2,269 | **4.14** |
| after | 1,429 | 2,255 | **4.15** |

**The migration did not change measured per-exchange allocations.** Not in
loopback, not across processes.

### What that corrects

The commit that landed the sertype migration said "per-message allocation is
otherwise off the service and action data path". That is NOT supported. What was
measured then was `check-rmw-alloc-sites` going 2 -> 1 — a count of SOURCE
SITES, not of runtime allocations. Letting a static ledger stand in for a runtime
number is the substitution this campaign catches elsewhere, and it happened here.

What the migration did do, and these still hold:

* removed a decode and an encode per message on the service path (structural,
  visible in the source);
* removed one steady-state allocation SITE, gated;
* removed five per-type adapters that existed only to work around the round trip.

What it did not do is reduce the allocation COUNT.

### The loopback harness under-reports, which is worth keeping

The two-process CLIENT HALF ALONE costs 4.15, while the single-process run
measuring BOTH halves costs 4.00. So a same-process harness prices roughly half
of what a deployment pays — Cyclone can hand a sample to a local reader by
reference, and the write path may never fully serialise. Any future service or
action number should come from the two-process pair.

Note the message path's 9.93 -> 2.00 was itself a single-process measurement, so
it carries the same caveat about absolute value. It is still good evidence that
the METHOD detects real change: the same harness style that moved by 7.93 there
moves by 0.01 here, so this null result is not the instrument failing to see.

### EXPLAINED, by DHAT — the harness payload is too small to show the defect

`valgrind --tool=dhat` at both counts, differenced, attributes every growing
allocation:

| allocation | before | after |
| --- | ---: | ---: |
| our typed sample, write path (`maybe_flush_request`) | 1.01 | — |
| our typed sample, take path (`take_typed_wire`) | 1.01 | — |
| our serdata x2 (`serdata_from_sample` / `serdata_from_ser`) | — | 2.02 |
| Cyclone `ddsrt_malloc` payload x2 | 1.99 | 2.02 |
| **total** | **4.01** | **4.15** |

**The migration SUBSTITUTED allocations rather than removing them.** Before, we
allocated a typed sample per direction and Cyclone allocated the serdata; after,
we allocate the serdata and there is no typed sample. One for one.

**And the count was never going to move on THIS payload.** The prediction that
the old path cost more assumed the `dds_ostream_t` allocated separately from the
typed sample. It does not, for a small message: `dds_ostream_init(&os, 0, ...)`
starts empty and the first write allocates once — for a 12-byte `AddTwoInts`
reply it never reallocs, so the entire re-encode costs ONE allocation, the same
one the new path spends on a serdata.

That is what this issue already recorded about the message path, read from the
other end: **9.93 was NON-INTEGRAL because the ostream grew by realloc, so its
allocation count depended on the sample.** A 12-byte reply produces no growth, so
it produces no delta. The harness exchanges `req[20]` and `reply[12]`, sizes
chosen for an interop assertion rather than for this measurement.

So the honest statement is narrower than either "no saving" or "a saving":

* on a SMALL fixed-size type the migration is allocation-neutral (measured);
* the saving the round-trip removal buys lives in the REALLOC TAIL, which appears
  only once a payload outgrows the ostream's first block — the regime where the
  message path measured 9.93 and where the count stops being an integer;
* what it removes unconditionally is a decode and an encode of CPU work, plus the
  per-type adapters. Those hold at any payload size.

**Next measurement**: the same slope with a large, variable-length reply — a
sequence long enough to force ostream growth. That is where a service-path number
comparable to 9.93 -> 2.00 would come from, and this harness's payload cannot
produce one.

## The size sweep — the migration is a CROSSOVER at ~8 KB, measured 2026-09-03

Allocation COUNT is neutral at every size (4.14 -> 4.15 at 12 B, 4.20 -> 4.26 at
16 KB), so counting allocations was the wrong instrument. BYTES is where the
behaviour lives. Client half, two-process, bytes allocated per exchange:

| reply payload | before | after | delta |
| ---: | ---: | ---: | ---: |
| 16 B | 8,329 | 270 | **-8,059** |
| 104 B | 8,329 | 276 | -8,054 |
| 488 B | 8,428 | 844 | -7,584 |
| 1,008 B | 8,347 | 1,186 | -7,161 |
| 2,008 B | 8,350 | 2,260 | -6,089 |
| 4,008 B | 8,354 | 4,342 | -4,011 |
| **8,008 B** | 8,325 | 8,219 | **-106** |
| 16,008 B | 8,444 | 16,260 | **+7,816** |

**The old path is FLAT — ~8,350 bytes per exchange whatever the payload, from
16 bytes to 16 KB.** It pays a fixed cost for a typed sample plus a re-encode
buffer, and a one-field reply costs the same as a 2,000-element sequence.

**The new path is LINEAR — about 1.02x the payload, on a ~260-byte floor.** The
sample IS the bytes, so what it allocates is what the message weighs.

So the migration is neither a win nor a regression; it is a change of SHAPE, and
the two curves cross at roughly **8 KB of reply payload**:

* below it, the saving is large and grows as the message shrinks — **30x fewer
  bytes** on a small reply, which is the regime a control loop actually runs in;
* above it, the new path allocates more, and the gap widens linearly.

### Why this matters beyond a number

Flat-versus-linear is the more useful half. A fixed ~8.4 KB per exchange is easy
to bound and impossible to shrink; a payload-proportional cost is the opposite.
For [phase 391](../roadmap/phase-391-allocation-unification-and-tier-model.md)'s
Robson bound the linear one is the better shape — it is derivable from the type's
own `MAX_SERIALIZED_SIZE`, which is exactly what issue 0896 makes available —
whereas the old constant was a number nobody could attribute.

But a service whose replies routinely exceed 8 KB now allocates MORE, and nothing
warns about it. That is worth knowing before someone quotes the round-trip
removal as an unconditional improvement, which is what the migration commit did
and what this measurement corrects.

### Method and limits

`NROS_PROBE_SEQ_LEN` was a SCRATCH change (an `int64[] sums` reply on
`AddTwoInts`) used to sweep payload size, and it is reverted — the committed
harness keeps only `NROS_ROUNDTRIP_ITERS`. Two points per size (n=1, n=50), slope
over 49 exchanges, client half only. The server half and the CPU cost of the
removed decode/encode are not measured. The 8 KB crossover is where these two
curves meet on THIS type and this transport; treat it as an order of magnitude,
not a threshold to encode.

## The SERVER half, and the whole exchange — measured 2026-09-03

The sweep above was the client. The server, same method (valgrind on the server
process, client untraced), bytes allocated per exchange:

| reply payload | SERVER before | after | delta | CLIENT before | after | delta |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 16 B | 4,328 | 299 | -4,030 | 8,329 | 270 | -8,059 |
| 1,008 B | 4,223 | 1,299 | -2,924 | 8,347 | 1,186 | -7,161 |
| 8,008 B | 4,307 | 8,284 | **+3,977** | 8,325 | 8,219 | -106 |
| 16,008 B | 4,232 | 16,282 | +12,051 | 8,444 | 16,260 | +7,816 |

**The server's old cost is flat too, but at HALF the client's** — ~4,270 versus
~8,350 bytes per exchange. Both are payload-independent; they differ by a factor
of two, which is what the two directions cost asymmetrically in the old typed
path. The new cost is the same on both sides, ~1.02x payload, because both now
allocate exactly the message.

**Combined, per exchange:**

| reply | before | after | delta |
| ---: | ---: | ---: | ---: |
| 16 B | 12,657 | 569 | **-12,089** |
| 1,008 B | 12,570 | 2,485 | -10,085 |
| 8,008 B | 12,632 | 16,503 | +3,871 |
| 16,008 B | 12,676 | 32,542 | +19,867 |

**So the WHOLE-EXCHANGE crossover is ~6 KB of reply, not the ~8 KB the client
half alone suggested.** Measuring one side and doubling would have put it in the
wrong place: the server's flat cost is half the client's, so the combined old
curve sits at ~12,635 rather than ~16,700.

Corrected summary of the migration, both halves, whole exchange:

* **a 22x reduction** on a small reply (12,657 -> 569 bytes);
* break-even at **~6 KB**;
* **2.6x more** at 16 KB, growing linearly beyond it.

The shape argument stands and is stronger with both halves: flat-and-unattributable
becomes linear-and-derivable, on both sides, from the same
`MAX_SERIALIZED_SIZE` issue 0896 exposes.

Limits unchanged: two points per size, one type, one transport, allocation BYTES
only — the CPU cost of the removed decode/encode is still unmeasured, and it is
the half that does not cross over.

## The CPU cost: ATTEMPTED, NOT MEASURED — and why the instrument failed

The decode and encode this change removes are a saving at every payload size,
unlike the byte curve, so the number is worth having. `valgrind --tool=callgrind`
was the right instrument to reach for and it did not produce a trustworthy figure.
Recorded so the next attempt starts past these two walls rather than into them.

**Wall 1 — process-level instruction counts are dominated by POLLING, and vary
about 2x run to run.** `call_blocking` spins on `take_response` with a 5 ms
sleep, so the client's instruction total tracks how long a reply happened to
take, not what the codec did. Two runs of the identical 100-exchange workload
collected 3,853,244 and 7,686,322 instructions. A slope taken across iteration
counts is meaningless against that spread, and the raw per-exchange numbers came
out non-monotonic in payload size (65,835 at 16 B, 17,835 at 1 KB, 82,011 at
16 KB), which is the noise showing rather than a trend.

**Wall 2 — the codec functions are not attributable.** `dds_stream_write_sample`
is present in the binary (`nm` finds it three times) and never appears in
`callgrind_annotate` output, inclusive or flat: it is reached through
Cyclone's static, heavily-inlined cdrstream, so its cost is folded into callers
that annotate as `???`. What IS visible — `write_sample_eot`, `write_sample_gc`,
`serdata_default_from_ser_nokey` — is Cyclone's own write and serdata path, not
the re-encode being removed.

So a number here would have to come from differencing two noisy process totals
whose spread exceeds the effect. That is how a plausible wrong figure gets into
an issue, and this campaign has enough of those already.

**What would work**, for whoever wants it:

* a bench harness with NO poll loop — call the codec directly on a prepared
  sample, in-process, N times, so the instruction count is the codec's and
  nothing else's. `dds_stream_write_sample` against a `SertypeMin` is callable
  in isolation; that is the shape `nros-bench` exists for;
* or `perf stat` on a pinned core with the sleep removed, comparing wall
  instructions across trees, which trades determinism for not needing a harness.

Both are real work. Neither is a five-minute follow-up to this measurement, which
is why this section says "not measured" instead of estimating.

## The CPU cost: MEASURED — the first option above, built

`tests/codec_bench.cpp` is that harness: no poll loop, no network, no DDS
entities. It builds a wire buffer, then times a `memcpy` (the new path's whole
per-message payload work) against `dds_istream_init` + `dds_stream_read_sample`
+ `dds_ostream_init` + `dds_stream_write_sample` (the pair `take_typed_wire`
ran on every reply). 50,000 iterations per point; run-to-run spread is roughly
10-15%, so read these as two significant figures.

| reply payload | memcpy (new) | decode+encode (old) | removed |
| --- | --- | --- | --- |
| 36 B | 0.8 ns | 46.3 ns | **45.5 ns** (60x) |
| 1,028 B | 8.0 ns | 56.5 ns | **48.5 ns** (7.0x) |
| 8,028 B | 47.4 ns | 130.3 ns | **82.9 ns** (2.7x) |
| 16,028 B | 75.9 ns | 252.1 ns | **176.2 ns** (3.3x)

The shape: a **~46 ns fixed floor** the old path paid on every reply regardless
of size — that is the per-message cost this change removes outright — plus a
size-dependent term. Both paths grow with payload, so the *ratio* falls as the
message gets bigger while the *absolute* saving rises.

Put beside the byte curve measured above, the two do not point the same way and
the difference is the whole point: allocation **bytes** favour the old path
below the ~6 KB crossover, while CPU favours the new path at **every** size
measured. A decision that reads only one of the two curves gets a different
answer depending on which one it happened to read.

### Three artifacts this harness produced before it produced a number

Each looked like a result. None was. Worth the space, because a bench that is
wrong is worse than no bench — it answers confidently.

1. **Hand-written CDR decoded nothing.** Laying the buffer out by hand — length
   then elements — gave `_length = 0` at every size, so the "decode" cost was
   the constant cost of bailing on a malformed buffer, and the sweep came out
   flat (~47 ns at both 8 KB and 16 KB). Fixed by having the codec **serialise
   its own input** rather than guessing at DHEADERs, alignment and
   extensibility.
2. **The sequence is not at sample offset 0.** A reply sample is
   `cdds_request_header_t` (client GUID + sequence number, 16 B) followed by the
   payload struct. Seeding at offset 0 writes into the header: the payload stays
   zeroed (20 B serialised for any length) *and* the verification read the
   seeded values straight back out of the decoded header, so the check passed on
   a sample carrying no elements. A check reading the same wrong offset as the
   seed cannot fail.
3. **A verification that reads a length is not a verification.** Artifact 2
   passed the `_length` check while the serialised size stayed 20 B for every
   requested length. The size and the length disagreeing is what exposed it —
   one number could not.

The check now prints the decoded length at every point and every run above shows
it matching. Timings collected before it matched were discarded, not adjusted.
