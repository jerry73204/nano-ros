---
id: 1195
title: "`Promise::wait` polls on a fixed 10 ms grid and overshoots timeouts shorter than that"
status: open
type: enhancement
area: executor
related: [issue-1192, phase-436]
---

## Problem

`Promise::wait` ignores the wake infrastructure and polls:

```rust
let spin_interval = core::time::Duration::from_millis(DEFAULT_SPIN_INTERVAL_MS);
let timeout_ms = timeout.as_millis().min(u64::MAX as u128) as u64;
let max_spins = (timeout_ms / DEFAULT_SPIN_INTERVAL_MS).max(1);
```
(`packages/core/nros-node/src/executor/handles.rs:2328`, with
`DEFAULT_SPIN_INTERVAL_MS = 10` at `handles.rs:13`)

Two consequences:

1. **Reply latency is quantized to 10 ms** regardless of when the reply lands,
   on backends that *do* have an async wake source. Cyclone installs a real
   `dds_set_listener`/`on_data_available` (`vtable.cpp:322`,
   `session.cpp:105`), so the wake exists and this path does not use it.

2. **A timeout shorter than 10 ms overshoots.** `max_spins` floors at 1, so
   `wait(Duration::from_millis(1))` still runs one full `spin_once(10ms)` —
   the caller's 1 ms deadline is exceeded by up to 10x before `WaitBudget`
   is consulted.

This does not violate the "never blocks forever" property — the loop is
bounded — but it is the one place in the executor where the waker
infrastructure exists and is bypassed.

## Fix direction

Spin with `min(remaining_budget, DEFAULT_SPIN_INTERVAL_MS)` so the last
iteration cannot overshoot the caller's deadline, and let the underlying
`spin_once` block on the wake primitive rather than returning on a fixed grid.
The existing `WaitBudget` already tracks the deadline; it is consulted one
spin too late.
