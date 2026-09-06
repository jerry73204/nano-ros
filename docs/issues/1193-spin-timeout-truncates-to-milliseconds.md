---
id: 1193
title: "A sub-millisecond spin timeout truncates to zero and silently becomes a busy loop"
status: open
type: bug
area: executor
related: [issue-0515, issue-1192, issue-1194, phase-436]
---

## Problem

`spin_once` converts the caller's `Duration` with `as_millis()`, which
**truncates**:

```rust
pub fn spin_once(&mut self, timeout: core::time::Duration) -> SpinOnceResult {
    let timeout_ms = timeout.as_millis().min(i32::MAX as u128) as i32;
```
(`packages/core/nros-node/src/executor/spin.rs:6374`)

`Duration::from_micros(500).as_millis()` is `0`. A zero `timeout_ms` selects
the non-blocking path — `primary_drive_timeout_ms = 0`, no wake wait — so:

```rust
executor.spin(core::time::Duration::from_micros(500));
```

is a **100 % CPU busy loop**, with no error and no warning. `spin()` simply
loops `spin_once` (`spin.rs:7351`), so nothing throttles it.

The one diagnostic that would have caught it is gated off precisely here:

```rust
if !self.spin_quantization_checked && timeout_ms > 0 {
```
(`spin.rs:6590`) — `audit_spin_quantization` is skipped when `timeout_ms == 0`,
i.e. exactly in the case where the quantization is total.

This is the same class as issue 0709 (a `spin_period` with no clock would have
free-run silently), which was fixed by *refusing* rather than degrading. This
path still degrades.

## Reach across the API surface

Three public spellings, three units, one truncating core:

| API | Unit accepted | Precision reaching the wait |
|---|---|---|
| `rclc_executor_spin_period` (C) | **nanoseconds** (`executor.rs:2960`) | ms |
| `nros::spin_once` (C++) | **milliseconds** (`nros.hpp:88`, `int32_t timeout_ms = 10`) | ms |
| `Executor::spin_once` (Rust) | `Duration` | ms |

The C API advertises nanosecond resolution and paces itself correctly in ns
(`invocation_time_ns += period_ns`, `sleep_ns`) — but the value it hands the
blocking wait is truncated to ms. The C++ API cannot express sub-ms at all.

## Why nothing is broken today

* nros-cpp's tier loops clamp with `period_us.max(1_000)`, so a declared
  sub-ms tier silently becomes 1 ms rather than a busy loop.
* ASI's controller declares `spin_period_us = 5000`.

So this is currently a **latent** defect — but `spin_period_us` is a
microsecond field in `NativeTierSpec`, and it accepts values it cannot honour.

## Fix direction

The floor is the platform layer: all five `nros_platform_wake_wait_ms`
implementations take `uint32_t timeout_ms` (`packages/platform/*/src/platform.c`),
so microseconds cannot currently reach the primitive on any target. A real fix
is a `wake_wait_us` (or `_ns`) slot with the ms one kept as the fallback for
platforms whose primitive is genuinely ms-granular.

Minimum acceptable interim: **round up, never down**, and refuse or warn on a
timeout that rounds to zero rather than converting it into a spin. Truncating
a 1500 us request to 1 ms is a 33 % short wait that no caller asked for.
