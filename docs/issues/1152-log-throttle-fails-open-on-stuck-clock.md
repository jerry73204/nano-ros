---
id: 1152
title: "log throttle fails OPEN on a stuck clock — the `wrapping_sub` that protects against a wrapping clock admits every record when the clock does not move"
status: open
type: bug
area: log
related: [phase-428]
---

## Problem

`packages/core/nros-log/src/throttle.rs`: `NEVER = 0` (`:43`), `storable(0)`
returns `1` (`:133`), and `throttle_admits` is
`now_ns.wrapping_sub(last_ns) >= interval_ns` (`:56`).

With a clock pinned at 0: first call admits and stores `1`; every later call
computes `0u64.wrapping_sub(1) == u64::MAX >= interval_ns` and **admits
forever**. The throttle becomes a no-op exactly when a port's clock is broken,
which is when a log flood is most likely.

The doc at `:50-53` explains `wrapping_sub` as protection for a WRAPPING clock
and does not consider a STUCK one. The safety measure is the failure mode.

## Why the existing affordance cannot see it

`timestamp_available()` is `cfg!(feature = "platform-clock")` (`lib.rs:633`) —
a compile-time check that returns `true` for a linked-but-broken port. The
`compile_error!` covers only the ABSENT-clock case.

## Fix

Treat `now == last` as "throttled", or detect the sentinel returned twice and
degrade loudly. Add a test with a pinned clock.
