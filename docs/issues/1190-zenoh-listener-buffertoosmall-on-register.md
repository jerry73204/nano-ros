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

## What is NOT established

**Whether this is a regression.** The before/after comparison this run existed
to make cannot answer it: the baseline capture was `tail -50`, which truncated
nextest's summary line, and the two visible failures were reported as `(9/10)`
and `(10/10)` — nextest COMPLETION counters, not case numbers. So the baseline
does not say whether `case_2` passed on `91f3ce7c9`.

That is a harness mistake worth recording on its own: truncating a baseline
costs exactly the comparison the baseline was for. Capture full output when the
run's purpose is a diff.

To settle it, run `case_2` against `91f3ce7c9` in the box and compare.

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
