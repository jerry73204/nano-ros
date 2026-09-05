# Phase 393 — what is left of the RMW contract, and what deliberately is not

**Status (2026-08-30). W1, W2 and W2a DONE; W3 holds. The contract work this
doc scoped is finished — what remains is VERIFICATION, which is a different
thing and is called out below.** Measured with `just check rmw-slot-producers`:
produced 53, default 7, unimplemented 0, **inert 14**, and
`check-rmw-api-parity` reports 0 gap / 0 unclassified.

* **W1 — issue 0823, the QoS read-back.** RESOLVED. It was the one correctness
  item here: six slots where the runtime did not merely lack the reading but
  asserted the wrong answer, reporting the REQUESTED QoS as GRANTED.
* **W2.** `publisher_count_matched_subscriptions`,
  `subscription_count_matched_publishers` and `get_gid_for_publisher` are
  `produced`.
* **W2a.** Decided: the identity pair stays NULL, and `rmw_vtable.h` says so
  rather than promising a fallback nobody wrote.
* **W3.** The 14 inert slots are all deliberate reservations with recorded
  reasons, held by `INERT_FAMILIES` in the gate.
* The struck-through graph-query entry in W3 was answered by **phase-381**,
  which shipped the twelve graph slots and proved two of them against a live
  `rmw_zenoh_cpp` peer.

**What is NOT finished, and it is this doc's own closing warning turned on
itself.** "A slot's EXISTENCE reads as coverage" is the trap recorded at the
bottom of this file, and `produced` is not immune to it: it means something
writes the slot and something reads it, NOT that either was ever exercised
against a real peer. Phase-381 is the demonstration — twelve slots `produced`,
mutation-tested, parity-clean, and the feature did not work at all until issue
0903 met a live talker. Ten of those twelve are still unproven live, Cyclone's
graph reader has never run live, and `graph_interop.rs` — the committed form of
the one comparison that matters — has never executed. That is the next work in
this area, and it is verification rather than contract.

**Implements.** RFC-0054 (the C headers are the ABI SSoT). Continues phase-376
(archived) and issue 0800 (archived). Issue 0791 owns the graph-query family and
is NOT superseded by this doc. Not to be confused with phase-379, which
is the USER API (rclc/rclcpp/rclrs) one layer up.

## Where the contract stands

The contract is EMPIRICAL: the 88 `rmw_*` symbols every `librmw_*_cpp.so`
defines, not the 177 the headers declare. Measured 2026-08-27 by
`just check rmw-api-parity`:

| bucket | n | meaning |
| --- | --- | --- |
| vtable | 70 | a slot exists |
| layer | 5 | answered elsewhere, named |
| declined | 13 | deliberately absent, with an RTOS reason |
| gap | 0 | — |

`vtable` counts a SLOT, not a backend that fills one, which is the overstatement
issue 0800 found. Of those 70: **34 answered by a slot something writes and
reads, 36 by an INERT one** (no producer, no consumer). So the honest headline
is **34 of 88 working**, plus 5 at another layer, 13 declined and 0 missing.

`just check rmw-slot-producers` keeps that split honest in both directions, and
`rmw-api-parity` now prints it beside the buckets so 70 cannot read as 70
working.

## W1 — the QoS read-back (the one CORRECTNESS item)

Issue **0823**. Six inert slots, and unlike the rest the runtime does not merely
lack the reading — it asserts the wrong one, reporting the REQUESTED QoS as
granted. Hides the most common cause of a silent ROS 2 pair.

Do this one first. It is the only item here that is a bug.

## W2 — cheap and genuinely useful

| slots | why |
| --- | --- |
| `publisher_count_matched_subscriptions`, `subscription_count_matched_publishers` | "why is nothing arriving" answered in one call; cyclone has `dds_get_matched_*` |
| `get_gid_for_publisher` | GIDs travel in the attachment today; an out-of-band read costs little |
| ~~`get_implementation_identifier`, `get_serialization_format`~~ | **WRONG — corrected 2026-08-29, see W2a below.** "The runtime answers both" is not true: no such fallback was ever written. |

Each is small, each has a live backend primitive, none changes a contract.

## W2a — what the identity item actually was (done 2026-08-29)

The row above claimed the runtime answers both slots and that filling them buys
a bridge image a per-backend answer. Grepping for that fallback found **only the
sentence promising it**. Nothing in the tree calls either slot, so a NULL one is
not "answered elsewhere" — it is unanswerable, and the header said otherwise for
two phases.

Filling them would also have been the campaign's own trap. `check-rmw-slot-
producers` classifies any slot with a producer as `produced` **whether or not
anything reads it** (`if s in produced` precedes the consumer test), so two
bodies nothing calls would have moved inert 27 -> 25 and changed nothing
observable. That is issue 0800's overstatement, re-created by the doc written to
prevent it.

What was real, in the same area:

**Cyclone spelled its identity twice** — `nros_rmw_cffi_register_named(
"cyclonedds", ...)` and `gid->implementation_identifier = "cyclonedds"`, in one
file, with nothing checking they agreed. `rmw_compare_gids_equal` compares that
STRING before the bytes, and `register_named` admits several backends in one
image precisely so the comparison has work to do. Renaming one and missing the
other does not fail to build and raises no error: two gids naming the same
publisher start comparing unequal, silently. Now one `kImplementationIdentifier`.

`get_serialization_format` has no such half. Every backend speaks CDR, nothing
asks, no divergence is possible — it is parity shape, and it stays NULL.

Both slot docs in `rmw_vtable.h` now say they are reserved and unanswerable
rather than claiming a fallback, and say what filling them would need: a caller.

## W3 — reservations that should STAY reserved

Recorded so nobody spends a week closing a "gap" that is a decision. Reasons
live in `check-rmw-slot-producers.py`'s `INERT_FAMILIES` and the gate holds them.

* ~~**graph queries + entity counts (11).**~~ **WRONG — corrected 2026-08-27.**
  This said they should stay reserved because reading the graph back means
  holding a discovered view of every peer, unbounded on a target. Issue **0791**
  had already refuted that with better evidence, and this doc was written
  without reading it.

  The refutation: we are ALREADY visible in the graph — cyclone publishes
  `ros_discovery_info` and the zenoh shim declares `@ros2_lv` liveliness tokens
  — so a nano-ros node appears in `ros2 node list` while being unable to answer
  "is anyone subscribed to this topic" in any of the three languages. That
  ASYMMETRY is the defect, and "unbounded memory" was an argument against a
  persistent cache that nobody proposed; a query-on-demand needs no cache.

  0791 also measured the real cost, and it is not small: zenoh has
  `liveliness_get_check` — a BOOLEAN "does anything match", not an enumeration —
  so a slot needs a new zpico shim entry point that collects reply keyexprs, a
  parser for `rmw_zenoh_cpp`'s `@ros2_lv/...` grammar that must match exactly,
  then the slot, a runtime wrapper and three language surfaces. Cyclone needs a
  READER for the info it only writes. That is a phase, which is what 0791 asks
  for — not a decline, and not a W3 item here.

  Recorded rather than quietly edited: the reasoning was checkable and I did not
  check it, which is the failure mode this whole campaign has been about.
* **`publisher_wait_for_all_acked` (1).** It BLOCKS, and this ABI decomposes
  waiting into `has_data` / `drive_io` / `next_deadline_ms` precisely so one
  executor can drive several backends. A slot that blocks inside one backend
  does not fit, and that is a design property rather than a missing body.
* **on-new-* callbacks (3).** `set_wake_callback` is this ABI's answer and is
  per-SESSION, which is the shape a multi-backend executor can use.
* **with-info takes (2).** The runtime gets GID and timestamps off the
  attachment of the message it already took.
* **content filter (2), network flow (2), `feature_supported` (1).** Parity
  shape, no consumer, no demand.

## Acceptance

* `check-rmw-api-parity` still 0 gap, 0 unclassified.
* `check-rmw-slot-producers` inert count DROPS by exactly the slots W1/W2 fill,
  and the families lose exactly those names — the gate fails both ways, so a
  wired slot left in a family is caught.
* Issue 0823 has a test that fails against today's code.

## Notes for whoever picks this up

The trap in this area is that a slot's EXISTENCE reads as coverage. It caught
phase-376 (issue 0800), it caught issue 0785's enclave grouping, and it caught
`set_log_severity`, which shipped with a slot, a dispatcher and stub tests and
no backend body for two phases. Before claiming any symbol here, run
`check-rmw-slot-producers` and look at which column it lands in.

## Issues homed here (survey 2026-09-03)
Every open issue was checked for a home phase; these had none, or were
mentioned here only in passing. A mention is not an owner — an issue with
no work item is an issue nobody is accountable for, which is the same shape
as a gate sitting in a lane no CI job runs. Each row is a work item: the issue
holds the evidence, the item is *close it*.

| issue | why it belongs here |
| --- | --- |
| [#0970](../issues/archived/0970-cyclone-rmw-should-own-its-sertype.md) | the Cyclone backend borrows Cyclone's generated sertype instead of registering its own — the contract gap behind the CDR round trip. **RESOLVED**; the removed decode+encode is worth ~46 ns/message (176 ns at 16 KB), measured in 0969. Its allocation-site ledger reads 2 -> 1, which is SITES and not runtime calls — the runtime count did not move |
| [#0971](../issues/archived/0971-take-sequence-cannot-say-why-it-stopped.md) | `take_sequence` cannot say why a drain stopped, and its two implementations disagree about it |
| [#0976](../issues/archived/0976-service-action-adapters-tested-only-against-ourselves.md) | five action adapters in the Cyclone service path reshape the CDR to match ROS 2, exercised only by nano-ros talking to itself. This is the VERIFICATION remainder W3 names |


## Adopted issues (2026-09-04) — a backend that does not build in a shipped configuration

Two open issues had no phase and are the same statement: a backend must compile
in the configurations the platform layer actually selects, and neither of these
does.

* **[#1023](../issues/archived/1023-sertype-hosted-includes-break-freestanding.md)** —
  `nros_sertype.cpp` includes `<memory>` and `<string>` unconditionally, so
  cyclonedds cannot compile freestanding. Archived issue 0112's class at a new
  site, with the sibling TU in the same directory already carrying the lesson.
* **[#1021](../issues/1021-zenoh-pico-1-8-0-matching-off-build-break.md)** —
  zenoh-pico 1.8.0 does not compile with `Z_FEATURE_MATCHING=0`, which is
  exactly what nano-ros passes on Zephyr. Upstream's, carried on our patch line.

They sit here rather than with the platform work because the contract they break
is the RMW's: *the backend builds for every platform that declares it*. A
configuration nobody can build is a contract nobody can check.
