# Phase 381 — read the ROS graph, which we are already visible in

**Status (2026-08-30). W1–W7 LANDED, and LIVE INTEROP now PASSES — the phase
meets its own acceptance.** Twelve `rmw` graph slots produced, reachable from
Rust, C and C++; zenoh answers all twelve, Cyclone answers `get_node_names`;
XRCE degrades to `UNSUPPORTED` and that is TESTED rather than asserted.
`check-rmw-slot-producers`: produced 42 -> 53, inert 25 -> 14.
`check-api-parity`: every divergence carries a ledger entry.

Measured against a stock `demo_nodes_cpp talker` on `rmw_zenoh_cpp`, in the
repo's `ros2` distrobox on its own tree: our `get_node_names()` reports
`/talker`, our `get_topic_names_and_types()` reports `/chatter`,
`/parameter_events` and `/rosout`, and `ros2 node list` reports ours — both
directions, `probe_rc=0`. `graph_interop.rs` is the committed form of that
comparison.

Getting there took issue 0903, and the lesson is the one the phase doc's own
"NOT done" paragraph predicted: every check up to that point tested our code
against our own builders, our own parser and our own vtable, and the feature did
not work. The mechanism was wrong underneath the defects — `z_liveliness_get` is
an INTEREST, and a token reaches a get's callback only when the router tags its
declaration with that interest id, so a sweep saw an arbitrary handful of the
domain's tokens. A standing liveliness SUBSCRIBER with history replaced it.

**Still open, beyond acceptance:**

* **Cyclone's graph reader has never run live.** `native-graph-rust-cyclone-r2n`
  exists in `interop::CELLS` and has never been executed; W5's reader is
  verified against our own builders only, which is exactly the state zenoh was
  in before 0903.
  **Update 2026-09-06 (phase-433 W1, issue 1137).** It ran, and failed —
  `get_node_names` enumerated only the probe. The reader is NOT at fault: the
  harness had pinned the stock talker's Cyclone bus to loopback (issue 1009,
  landed 2026-09-04) and left our side on Cyclone's default interface, so the
  two participants never discovered each other. This cell had PASSED live on
  2026-08-30 (issue 0927); the regression is in the test harness and dated. The
  pin is now applied to both halves, and `check-dds-isolation-symmetry` keeps
  it that way — but **the cell has still not been seen GREEN since, so W5's
  reader remains live-unproven**, which is the same sentence this bullet
  carried before.
  W5's SCOPE, measured rather than restated: Cyclone's vtable fills
  `get_node_names` and eleven `nullptr`s
  (`nros-rmw-cyclonedds/src/vtable.cpp:380-391`), so ten declared
  `Unsupported`s in that run are correct answers and not ten defects.
  `check-rmw-slot-producers` now prints the per-backend split, so the number
  no longer lives only in this paragraph.
* **Ten of the twelve slots are unproven live.** Only `get_node_names` and
  `get_topic_names_and_types` have been measured against a real peer. The
  service forms, the counts and the six by-node forms are still self-verified.

**Superseded status (2026-08-29): UNBLOCKED — the bounded-memory decision does
not have to be made, because the buffer it was about ALREADY EXISTS and is
already ABI. See Design note 2a.**

**Superseded status (2026-08-27): NOT STARTED — BLOCKED on the deferred bounded-memory
decision in Design note 2, which is ABI-shaped and cannot be walked back.
Design notes added 2026-08-27 from reading the zpico shim — W1 is smaller than scoped (the primitive is already start/poll),
but the acceptance criteria need a warm-up window. See "Design notes" below.**
Split from issue 0791 because it is several times the size of the other W3
coverage gaps and needs live interop verification the others do not.

**Implements.** RFC-0035 (RMW vtable ABI). Amends RFC-0036, whose "no dynamic
discovery" line is the reason this was never attempted.

## Why

A nano-ros node appears in `ros2 node list` and `ros2 topic info` — the zenoh
shim declares `@ros2_lv` liveliness tokens, `nros-rmw-cyclonedds/src/graph.cpp`
publishes `ros_discovery_info` — and the node itself cannot answer "is anyone
subscribed to this topic", "did the peer I need come up", or "what is on this
topic" in any of the three languages.

The asymmetry is the defect. An operator who can see the node reasonably expects
it to behave like a participant, and code that would branch on the graph has to
be written blind on a system where it need not be.

Phase 379's `graph` stage recorded **37 `gap` rows** on this, against 15
`declined`. The declines that survive are the rclcpp `Event` /
`wait_for_graph_change` shape (allocator, listener thread, and a blocking wait
that does not drive the executor) and rclrs's `notify_on_graph_change`.

## What already exists — the easy half

`nros/rmw_vtable.h` has carried the family as optional slots since phase-376 W4:
`get_node_names`, the four `*_by_node` forms, `get_topic_names_and_types`,
`get_service_names_and_types`, `get_publishers_info_by_topic`,
`get_subscriptions_info_by_topic`, `count_publishers`, `count_subscribers`,
`node_get_graph_guard_condition` — 12 slots for upstream's 15 names, plus
`rmw_topic_endpoint_info_t` in `rmw_entity.h`.

**The result-carrier design is already settled and is the part that would
otherwise be hard.** Upstream returns `rcutils_string_array_t` and
`rmw_names_and_types_t`, which allocate two levels deep. There is no allocator
at this seam, and a caller-provides-the-buffer shape is worse than it looks: the
graph has no bound the CALLER can know. So enumeration is a **visitor** —
`rmw_node_visit_fn`, `rmw_names_and_types_visit_fn`,
`rmw_topic_endpoint_info_visit_fn`. Peak extra RAM is one entry, the backend
streams from state it already holds, and a caller with a bound stops early by
returning `false`. Every string is borrowed for the call only.

Every one of the 12 is `None` in `packages/rmw/cffi/src/lib.rs`. No runtime
wrapper exists above them, and no user entry point in C, C++ or Rust.

## What does not exist — the reason this is a phase

**zenoh has a boolean liveliness CHECK, not an enumeration.**
`packages/rmw/zenoh/nros-rmw-zenoh/src/zpico.rs:920-940` is
`liveliness_get_start(keyexpr, timeout_ms)` / `liveliness_get_check(handle)` —
"does anything match this keyexpr", used for peer-alive detection. It collects
no replies, and the crate has no keyexpr parser.

So "zenoh already queries the tokens, only the reading half is missing" is right
about the protocol and wrong about the distance.

## Work items

**W1 — zenoh enumeration.** A zpico shim entry point that COLLECTS liveliness
reply keyexprs rather than counting them. Note the shim's config is ABI-coupled
to the zenoh-pico library (CLAUDE.md's zpico rule, issue 0135): the generated
config must be shared or the TUs disagree silently.

**W2 — the keyexpr parser.** `@ros2_lv/<domain>/<zid>/<nid>/<eid>/…/<namespace>/<node>`
is `rmw_zenoh_cpp`'s grammar and must match it **exactly** — a graph query that
returns a plausible wrong answer is worse than one that returns none. Pin the
grammar against the `rmw_zenoh_cpp` the recorded router links (RFC-0075), and
expect it to move with the distro.

**W3 — fill the vtable slots for zenoh.** The easy part; the visitor contract is
designed and needs no allocator.

**W4 — the runtime wrapper and user entry points**, in all three languages. Note
the `subscription` / `subscriber` split found in phase-379: rclrs says
`get_subscription_names_and_types_by_node`, rcl says `subscriber`. Whichever we
pick, one lane is not a drop-in — settle it with issue 0788's sweep, not
separately.

**W5 — Cyclone.** Needs a READER for `ros_discovery_info`, which
`nros-rmw-cyclonedds/src/graph.cpp` currently only writes.

**W6 — degrade per backend.** XRCE has no graph. The slots are optional
precisely so a backend can decline; the user-facing answer for "this backend
cannot tell you" must be distinguishable from "nothing is there", or callers
will read absence as emptiness.

**W7 — amend RFC-0036.** Its "no dynamic discovery — peers static" line caused
phase-379 to decline six rows across two stages that had to be re-verdicted. The
accurate statement is narrower: no discovery-driven *entity matching*, but the
graph is observable.

## Design notes from reading the shim (2026-08-27)

Three findings that change W1's shape. Recorded here rather than in a second
phase doc, because this doc already owns the work.

### 1. The primitive is ALREADY non-blocking — W1 is smaller than it reads

The slot contract in `rmw_vtable.h` is strict: *"NONE of these may block on the
wire, and none takes a timeout"*, on the premise that no background transport
thread is assumed. That looked like a collision with a liveliness GET.

It is not. `zpico_liveliness_get_start(keyexpr, timeout_ms)` returns a SLOT
HANDLE immediately and `liveliness_get_check(handle)` polls it — start/poll, not
a blocking call. So W1 does not have to invent an async shape; it has one.

### 2. What is missing is storage, not asynchrony — **DEFERRED 2026-08-27**

**Decision: the bounded-memory question is deferred, and this phase does not
start until it is answered.** Not because it is hard, but because it is the one
part that cannot be walked back: a reply buffer sized into `get_reply_ctx_t` is
a struct BOTH the shim and the zenoh-pico library see, so its width becomes ABI
(issue 0135) and shrinking it later is a break, not a tweak. Everything else in
W1-W7 is additive.

What has to be settled before W1 opens:

* the per-query reply-buffer width, and whether it is a count of entries or a
  byte budget — a `@ros2_lv` keyexpr is variable-length, so a fixed entry count
  still needs a per-entry cap;
* whether the knob is separate from `ZPICO_MAX_PENDING_GETS` (default 4) or
  derived from it;
* what a 128 KiB image sets it to, and whether zero MUST leave the slots NULL
  rather than returning an empty enumeration — the two are different answers to
  the caller and W6 already says they must stay distinguishable;
* whether truncation is reportable. A graph query that silently drops the
  entries past the buffer is the "plausible wrong answer" W2 warns about, in a
  second form.

Until then the rest of this section is background, not a plan.



`get_reply_ctx_t` (`zpico.c`) holds `received`, `done` and `reply_count`. The
reply handler increments the count and DISCARDS the keyexpr. Enumeration needs
those strings kept.

That reintroduces a bounded-memory question, but a much smaller one than the
"graph cache" the vtable header warns about: it is one query's replies, not a
standing view of every peer. Size it with a knob beside the existing
`ZPICO_MAX_PENDING_GETS` (default 4, `CONFIG_NROS_MAX_PENDING_GETS`), so a
128 KiB image can set it to zero and leave the slots NULL.

**The zpico rule applies with force here** (CLAUDE.md, issue 0135): the shim's
config is ABI-coupled to the zenoh-pico library, and `get_reply_ctx_t` is a
struct both TUs see. A field added under a config flag that the two halves do
not agree on is a silent ABI break, which is exactly how the queryables went
session-local-only.

### 2a. The buffer already exists — measured 2026-08-29

**Design note 2's decision does not need to be made.** It was scoped on the
premise that keeping reply keyexprs needs a NEW sized field in `get_reply_ctx_t`,
a struct both the shim and the zenoh-pico library see, making its width ABI
(issue 0135) and therefore irreversible. That premise is false.

The slot already carries the field:

```c
typedef struct {
    uint8_t buf[ZPICO_GET_REPLY_BUF_SIZE];   /* default 4096 */
    size_t  len;
    _Atomic bool received;
    _Atomic bool done;
    uint32_t reply_count;
    ...
} /* pending get slot, zpico.c */
```

`ZPICO_GET_REPLY_BUF_SIZE` is already a knob (`CONFIG_NROS_GET_REPLY_BUF_SIZE`,
`nros-zpico-build`), already sized per image, and already part of the ABI both
TUs agree on. Nothing has to be added, so nothing becomes newly irreversible.

And on a LIVELINESS query it is dead weight today. `get_reply_handler` copies the
first reply's PAYLOAD into `buf` and **discards the keyexpr**, which is where all
the graph information lives. The shim says so itself, at
`zpico_liveliness_get_start`:

> Reuses the same `s->pending_gets` slot pool as `zpico_get_start` — a slot is
> just a (received_flag, dropper_done_flag, payload_buf) triple, agnostic to
> whether the caller will read the payload. **The reply handler still copies the
> (typically empty) liveliness token bytes into the slot's buffer; we never read
> them.** Only `received` matters.

So 4096 bytes are allocated, written with nothing, and never read, on precisely
the query this phase needs. Accumulating NUL-separated keyexprs there instead
changes no struct layout, no config flag, and no ABI.

What survives from Design note 2's four questions:

* **width, and count-vs-bytes** — answered. It is a BYTE budget, it is
  `ZPICO_GET_REPLY_BUF_SIZE`, and it already exists.
* **separate knob or derived from `ZPICO_MAX_PENDING_GETS`** — neither; the
  existing per-slot buffer is the budget, and the two knobs already multiply
  exactly as they do today.
* **what a 128 KiB image sets, and zero-MUST-leave-the-slots-NULL** — still
  open, but no longer ABI-shaped: it is a runtime predicate, additive, and
  reversible. W6 keeps "absent" and "empty" distinguishable.
* **is truncation reportable** — still open and still worth getting right; now
  cheap, because `reply_count` already counts EVERY reply while the buffer holds
  what fit, so "replies seen" vs "entries stored" is a subtraction, not a new
  mechanism. That is the "plausible wrong answer" guard W2 asks for.

Recorded rather than quietly restarting the phase: the blocker was real given
what was known, and it dissolved on reading the struct rather than on a
decision. The lesson is the campaign's own — the cost was assumed, not measured.

### 3. The slot is single-shot; the primitive is start/poll. That has a
### USER-VISIBLE consequence the acceptance criteria must state

`get_node_names(session, visit, ctx)` takes no timeout and may not block, so it
can only report what has ALREADY arrived. The backend therefore needs a standing
or periodic liveliness query fed by `drive_io`, and **the first call after
startup legitimately returns a partial graph**.

That differs from `rmw_zenoh_cpp`, whose cache is warm because its background
thread has been filling it. So the acceptance line *"`ros2 node list` and a
nano-ros node's own `get_node_names()` agree"* is only true after settling, and
written as-is it is a flaky test. It needs either a bounded wait for the counts
to match, or an explicit statement of the warm-up window.

The alternative — letting the slot block for a timeout — is available and should
be REJECTED deliberately rather than by omission: it would stall the executor's
only thread inside an introspection call, on a runtime whose whole premise is
that there is no other thread to do the work.

## Acceptance

* `ros2 node list` and a nano-ros node's own `get_node_names()` agree, on zenoh,
  in a live interop cell.
* A backend with no graph reports "unsupported", not an empty list.
* Phase-379's `graph.json` rows flip from `gap` to `same`, and
  `just check api-parity` stays green — which is the mechanical proof the
  surface landed rather than a hand-check.

## Adjacent, cheap, and not blocked on any of this

`get_transition_graph`: `nros-node/src/lifecycle_services.rs` serves our full
`ALL_TRANSITIONS` table over `~/get_transition_graph`, so a remote peer can read
the lifecycle state machine over the wire while the node's own code cannot read
it in-process in any language. The table is already `const`. That is a local
accessor, not a graph query, and could land any time.

## Issues homed here (survey 2026-09-03)
Every open issue was checked for a home phase; these had none, or were
mentioned here only in passing. A mention is not an owner — an issue with
no work item is an issue nobody is accountable for, which is the same shape
as a gate sitting in a lane no CI job runs. Each row is a work item: the issue
holds the evidence, the item is *close it*.

| issue | why it belongs here |
| --- | --- |
| [#0788](../issues/0788-api-verbs-disagree-across-our-three-languages.md) | the same API verb is spelled differently in our C, C++ and Rust — the sweep this phase's lane choice depends on |

