---
id: 1190
title: "the arena derivation prices a subscription's QoS history at three slots when the ROS default is eleven, so a one-subscription image derives 8,192 bytes for a 12,144-byte registration"
status: resolved
type: bug
area: core, rmw, api
severity: high
found: 2026-09-07
resolved: 2026-09-07
related: [0810, 0900, 1137, 0146, phase-403]
---

# `interop_e2e::case_2_zenoh_pubsub_ros2_to_nano`, measured live

Found by phase-433 W2's after-run, in the `ros2` distrobox against ROS 2
Humble, box tree at `605d667eb`, all fixtures freshly built.

```
FAIL [0.923s] interop::case_2_zenoh_pubsub_ros2_to_nano
nros listener did not become ready: ProcessFailed(
  "native-rs-listener did not print `Subscriber created for topic:` within 5s")
```

**Not a timeout and not a flake.** Three solo re-runs, 0.923 s / 0.925 s /
0.929 s, all FAIL — the process exits almost immediately. Its own output
carries the cause:

```
[ERROR] nros: arena exhausted: 3952 more bytes needed, 0/8192 in use.
nros: application error: NodeRegister("native_rs_listener")
[ERROR] nros: node declaration failed — NodeError::BufferTooSmall
```

`want` in that line is the shortfall PAST capacity, not the request
(`report_arena_exhausted(new_used - arena.len(), …)`), so the single allocation
asked for **8,192 + 3,952 = 12,144 bytes into an empty 8,192-byte arena**.

## Root cause: the model prices a buffer nothing creates

`packages/core/nros-node/build.rs` billed a pub/sub arena slot at

```
PUBSUB_SUB_BUFS * rx_buf + PUBSUB_ENTRY_OVERHEAD  =  3 * 1024 + 512  =  3,584
```

`3` is `TripleBuffer::SLOT_COUNT`, and a triple buffer is what
`buffered_region_size` returns **only for `depth <= 1`**. Every deeper history
is an `SpscRing`: `(depth + 1)` slots plus a `usize` length beside each. The
ROS default is `rmw_qos_profile_default` = KEEP_LAST(**10**)
(`QoSProfile::QOS_PROFILE_DEFAULT`), which every subscription created without an
explicit QoS gets — including the listener's.

The model carried no depth term at all, so it was short by a factor of ~3.7 on
the most ordinary entity in ROS.

**The 12,144 decomposes exactly**, on x86_64 with the cffi backend:

| term | bytes |
| --- | ---: |
| `size_of::<SubBufferedRawEntry<F>>()` — the entry the declarative runtime registers | 640 |
| its callback closure: a `heapless::String<128>` callback id (136) + the cell handle | 152 |
| `buffered_region_size(10, 1024)` = `11 * 1024 + 11 * 8` | 11,352 |
| **total** | **12,144** |

against **3,584** modelled. Both halves of the model were short: the region term
by 8,280 and the entry allowance by 280 — 512 is below *every* concrete entry
this crate defines (`SubBufferedRawEntry<()>` 640, `SubBufferedEntry<(), ()>`
and `SubBufferedRawCEntry` 656, `SubBufferedTypedCEntry` 672,
`SubBufferedRawInfoCEntry` 872), most of that being the backend handle
(`CffiSubscription` alone is 584, two 256-byte name buffers of it).

## Why it survived four phases, and it is not the floor

`ARENA_FLOOR` = 8,192 was **not** what hid this. `.max()` only raises: the
listener's declared sum was `1 * 3,584 + 2,048` = 5,632 and the floor lifted it
to 8,192 — the right direction, and still 3,952 short. Raising the floor to "one
entry" would tax timer-only and publisher-only images 6,232 bytes for a
guarantee they cannot use (8,192 holds 128 timers), so the floor stays where it
is; the fix belongs in the per-entry model, and the model's terms are now
asserted to be upper bounds rather than trusted.

What hid it is the **action-client term's margin**. An image that does not
declare its entities budgets every slot at the action-client worst case, 18,048
bytes, and that term is genuinely an over-estimate: measured, the entry is 5,312
bytes and its feedback region `buffered_region(8, 1024)` is 9,288, so 14,600
against 18,048. 3,448 bytes of slack per slot is more than enough to absorb a
pub/sub term short by 8,560 when the slot count is 4 and the subscriptions are
2 — which is every image in the tree that predates the entity inventory.

So the bug only becomes fatal on the images phase-403 step 3 made possible: the
ones whose **declared** entity counts drive the per-kind sum. Across the 1,547
`nros_node_config.rs` files in a fully-built developer tree, 1,524 read
`ARENA_SIZE = 74,240` (the undeclared worst case) and 5 read 8,192. This is a
regression introduced by step 3 (landed 2026-09-03), not a long-standing defect
that nobody triggered — the derivation only became reachable-and-wrong when an
image could declare one subscription.

## The fix

`packages/core/nros-node/build.rs`

* `buffered_region(depth, slot)` — a `const fn` mirroring
  `executor::arena::buffered_region_size`, the function the allocator itself
  calls. Every term that stands for a buffered receive region goes through it.
* `PUBSUB_QOS_DEPTH = 10`, restated from `QOS_PROFILE_DEFAULT.depth` because a
  build script cannot read a const out of the crate it is building, and held to
  it by `the_modelled_qos_depth_is_the_runtime_default`.
* `PUBSUB_ENTRY_STRUCT` 512 → 1,024, which covers every concrete entry above
  plus the declarative runtime's closure.
* The service term is modelled on its OWN shape now (`2 * rx_buf + 1,024`)
  rather than as `pubsub_entry + rx_buf`. A service server has no trailing
  region — both buffers are inline and it reaches the arena through
  `arena_alloc` — so leaving it hitched to the pub/sub entry would have billed
  it 13,400 bytes against the 2,592 `SrvRawEntry<1024, 1024>` measures.
* The model is EMITTED as `config::arena_model`, so nothing retypes it. The
  compatibility test `the_default_derivation_is_unchanged` had six of its
  constants copied out of the build script; it reads them now.

`packages/core/nros-node/src/executor/arena.rs`

* Compile-time assertions, gated on `rmw-cffi` (the configuration a real image
  is built in), that each modelled term is an upper bound on the entry types it
  stands for. This is the half a build script structurally cannot do: it has no
  `size_of`, and the dominant term is the linked backend's handle. The
  unit-test configuration substitutes `MockSubscriber`, whose 8-deep queue of
  canned 256-byte takes makes it ~2.1 KiB — a fixture, not a handle any image
  links, so sizing the model to cover it would spend 1.4 KiB of every slot on a
  mock.
* `DEFAULT_ACTION_FEEDBACK_DEPTH`, named so the `8u16` literal the action client
  registers with can be held to the number `build.rs` prices it at.
* Four unit tests in `arena_model_tests`. The load-bearing one,
  `the_modelled_pubsub_region_covers_a_default_qos_subscription`, fails on the
  old derivation with the numbers in it:

  ```
  the arena derivation budgets 3072 bytes for a subscription's buffered region
  and the allocator claims 11352 for it (11 slots of 1024 at the default
  KEEP_LAST(10)) — every image that derives its arena from a declared
  subscription count is short by 8280 bytes per subscription
  ```

## What this costs

Nothing, on 1,542 of the 1,547 built configs measured. The corrected pub/sub
term multiplies `MAX_CBS - ARENA_ACTION_CLIENTS`, which is **zero** at the
shipped defaults, so the default arena stays at exactly 74,240 — asserted, so a
future move has to be a decision. It moves only images that declare entities,
and only by what those entities actually claim: the listener goes
8,192 → 14,424 for a registration that wants 12,144.

The action-client term is left **byte-identical** on purpose. It is depth-blind
in the same way the pub/sub term was, but unlike that one it is still an upper
bound and by 3,448 bytes; rewriting it would move 1,524 images upward by 24,864
each if the fiction of a 4,096-byte `pending_request` per service client is
kept, or downward by 10,464 if it is not, and neither is a change this issue
measured a need for. `the_modelled_action_client_entry_covers_its_feedback_region`
is what will say so if that margin ever closes.

## What is still owed

`PUBSUB_QOS_DEPTH` is the DEFAULT depth, not a bound over every depth, so a
subscription built with an explicit `QoS(N > 10)` still out-runs the derivation
and still needs `NROS_EXECUTOR_ARENA_SIZE`. The right answer is phase-403 step
2's remaining wiring: `NROS_ENTITY_DECLARED_DEPTHS` and
`NROS_ENTITY_UNDECLARED_DEPTH_COUNT` reach cmake and stop there, never entering
the cargo environment this build script reads. Carrying them across would both
close that gap and let a declared KEEP_LAST(1) image reclaim the 8,280 bytes per
subscription it is now budgeted and does not use. Recorded in the phase-403
roadmap beside step 3's correction, not opened as a separate issue.

## Not verified here

The live cell. The fix was proven by arithmetic and by unit tests on a host with
no ROS; `interop_e2e::case_2_zenoh_pubsub_ros2_to_nano` in the box is
outstanding.

## Corrections to the original report

* `NodeRegister(…)` was read as possibly a keyexpr-length bound
  (`node.rs:337` via `tx_writer`, or the three producers in `action_core.rs`).
  It is not: the producer is `arena_alloc_with_trailing`, and the keyexpr DEBUG
  lines immediately before it are the successful backend-side creation, not the
  thing that failed. The arena advisory that named the real cause was already in
  the output.
* Whether it is a regression was recorded as unestablished. It is one — see the
  argument above, which needs no baseline run: the declared-count branch that
  produces an 8,192-byte arena for a one-subscription image landed on
  2026-09-03.
