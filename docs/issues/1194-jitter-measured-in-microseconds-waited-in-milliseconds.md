---
id: 1194
title: "Release jitter is measured in microseconds against a cadence the executor can only wait in milliseconds"
status: open
type: tech-debt
area: executor
related: [issue-1193, phase-436]
---

## Problem

The same quantity is read at two different granularities inside one function.

`record_release_jitter` takes the cadence in **microseconds**:

```rust
let nominal_us = ... timeout.as_micros().min(u64::MAX as u128) as u64;
```
(`packages/core/nros-node/src/executor/spin.rs:2261`)

while the wait that is supposed to realise that cadence takes it in
**milliseconds**, truncated (`spin.rs:6374`, see issue 1193).

So `spin_once(Duration::from_micros(1500))`:

* is judged against a **1500 us** nominal by `release-jitter-runtime`,
* but actually waits **1 ms**.

The rule then measures lateness against a period the executor never attempted.
On a loop that meets its 1 ms wait exactly, the statistic reads as 500 us
early — or, once the caller's own pacing pushes the interval past 1500 us, as
lateness that no scheduling decision caused.

## Why it matters beyond tidiness

`release-jitter-runtime` is a **safety monitor**: it exists to say whether a
tier meets its declared cadence. A monitor whose yardstick and whose mechanism
disagree by up to a millisecond cannot support that claim. The measurement is
more precise than the thing being measured, which reads as rigour and is not.

## Fix direction

Bound to issue 1193 — one granularity for the wait and the measurement, chosen
as the finer of the two. Until the platform wake primitives accept sub-ms
(1193's fix direction), the honest interim is to record the nominal at the
granularity actually achievable and say so, rather than reporting µs precision
the wait cannot deliver.

Do **not** resolve this by coarsening the jitter statistic to ms: 1 ms is
already the whole budget of a 1 kHz tier, and cyclictest-class numbers are
reported in µs for that reason.
