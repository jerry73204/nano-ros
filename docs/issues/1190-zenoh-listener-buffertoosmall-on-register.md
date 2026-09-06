---
id: 1190
title: "the zenoh ROS2→nano listener dies at startup with `NodeError::BufferTooSmall` from `NodeRegister` — deterministic, 3/3 solo"
status: open
type: bug
area: rmw, api
severity: high
found: 2026-09-07
related: [1137, 0146]
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
0.929 s, all FAIL — the process exits almost immediately rather than failing to
start within the 5 s budget. Its own output carries the cause:

```
[DEBUG] Subscriber data keyexpr: 0/chatter/std_msgs::msg::dds_::String…
[DEBUG] liveli…
error: NodeRegister(…)
failed — NodeError::BufferTooSmall
```

So `Node::register` (or a callee) hits a fixed buffer while building the
subscriber's keyexpr or its liveliness token, and the failure surfaces four
collapses away as an opaque `NodeRegister(…)` — the collapse
`packages/api/nros/src/node.rs:53` already documents.

## Root cause — measured (PR #692)

| | bytes |
| --- | ---: |
| modelled per pub/sub entry, `3 * rx_buf + 512` | 3,584 |
| **real** | **12,144** |

It decomposes exactly, and the total is `8192 + 3952` — the shortfall this
issue opened with:

    size_of::<SubBufferedRawEntry<F>>() with a ZST callback      640
    the declarative runtime's closure (heapless::String<128>)    152
    buffered_region_size(10, 1024) = 11 * 1024 + 11 * 8       11,352
                                                             ------
                                                             12,144

**`PUBSUB_SUB_BUFS = 3` is `TripleBuffer::SLOT_COUNT`, and the allocator uses a
triple buffer only for `depth <= 1`.** Deeper is an `SpscRing` — `depth + 1`
slots plus a `usize` beside each. The ROS default is `rmw_qos_profile_default`,
KEEP_LAST(**10**). The model carried **no depth term at all**:
`grep -c depth build.rs` returns 0.

phase-403 step 2 added declared depths for exactly this; step 3 shipped without
reading them, and its own design doc calls defaulting to 1 "the unsafe
direction".

## IT IS A REGRESSION — established, contrary to this issue's first revision

`fac0c8173`, phase-403 step 3, **2026-09-03**. No baseline run was needed. The
first revision said the question was open because a truncated `tail -50` had
destroyed the before/after comparison — and the answer was in the history the
whole time. Losing the baseline was careless; concluding "unknowable" from it
was the larger error.

## Two things this issue got wrong about its own cause

1. **"The floor hides it" — no.** `ARENA_FLOOR`'s `.max()` only raises,
   5,632 → 8,192: the right direction, still 3,952 short. What hid it is the
   **action-client term's margin** — 18,048 modelled against 14,600 measured,
   3,448 B of slack per slot, absorbing a pub/sub term short by 8,560 on any
   4-slot image, and every image budgets every slot at the action-client worst
   case. The floor stays 8,192; raising it to one entry would tax timer-only
   and publisher-only images 6,232 B for a guarantee they cannot use.
2. **"The arena is too small" — the MODEL was wrong**, which points at a
   different fix. A floor is safe exactly when the per-entry model is right.

## Why the interesting suspects are probably not it

* **Issue 1137's pin** (`apply_to_command`) is a DDS profile; this is a zenoh
  cell and the failure is a length, not discovery.
* **Load** is excluded by the three solo runs at sub-second, identical timing.
* **Fixture staleness** is excluded: every fixture in this run was built minutes
  earlier from this tree, and `build-workspace-fixtures` plus all three row
  coordinate sets returned rc=0.

## Direction

1. Name the buffer. `NodeError::BufferTooSmall` has several producers
   (`node.rs:337` via `tx_writer`, three in `action_core.rs`); the keyexpr line
   printed immediately before the error points at the subscriber-registration
   path.
2. The keyexpr is long by construction —
   `<domain>/<topic>/<type with `::` separators>` — so the bound may simply be
   too small for a `std_msgs::msg::dds_::String` on `/chatter`, which is the
   most ordinary topic in ROS 2. If so, this is not an edge case.
3. `NodeRegister(…)` should carry which buffer overflowed. A caller today gets
   an opaque variant and has to read DEBUG output to learn what happened, which
   is what made a sub-second deterministic failure look like a 5 s readiness
   timeout.
