---
id: 1152
title: "log throttle fails OPEN on a stuck clock — the `wrapping_sub` that protects against a wrapping clock admits every record when the clock does not move"
status: resolved
type: bug
area: log
severity: medium
found: 2026-09-06
resolved: 2026-09-06
resolved_in: "fix/1152-throttle-stuck-clock"
related: [phase-417, phase-428, 0160]
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

## Resolution

The sentinel moved OUT of the timestamp domain. The defect was not
`wrapping_sub` — it was that "never emitted" was spelled as a timestamp value
(`0`), which forced a real reading of `0` to be displaced to `1` so the
sentinel stayed unambiguous, and a clock stuck on `0` then never equalled what
was stored. Any sentinel in the timestamp domain has this hole at the value it
occupies; the fix is a flag.

The C storage is one caller-owned, zero-initialised `uint64_t`
(`static uint64_t x = 0;` in every `NROS_LOG_*_THROTTLE` expansion), so the
flag could not become a second word without an ABI change. Instead the state
word is `(last_ns << 1) | 1`: bit 0 is "has emitted", the 63 bits above hold
the reading verbatim (mod 2^63), and `0` is still the fresh state a C site
carries with no initialiser. Elapsed time is `(now - last) mod 2^63`; a clock
that wraps at 2^64 also wraps at 2^63, so a real wrap still measures real
elapsed time, and a stuck clock (any value, `0` included) reads as zero elapsed
and stays shut. The only thing given up is that a jump of exactly 2^63 ns
(292 years) between two records reads as no time at all.

One pure function, `nros_log::throttle_decide(word, now_ns, interval_ns) ->
Option<u64>`, replaces `throttle_admits`: it returns the NEXT word rather than
a `bool`, so a storage cannot admit with the shared rule and then arm with its
own spelling of "last emitted" — which is exactly what the C shim did
(`nros_log_throttle_admit` carried a second copy of the `0 -> 1` skew). Both
storages (`ThrottleState` under its one-bit guard; the C `uint64_t`) now store
what `throttle_decide` hands back and interpret nothing.
`ThrottleState::last_emitted_ns` returns `Option<u64>`.

Tests in `throttle.rs`: stuck at 0 (the negative control — it FAILED on the
pre-fix code at the second record, `a clock that never moves is a window that
never elapses`), stuck at nonzero (five values including `u64::MAX`, which
already passed pre-fix: the hole was only at the sentinel's value), stuck at
the word's own boundaries (`1 << 63`, `u64::MAX >> 1`), normal 1 kHz cadence
against a 100 ms window (exactly 10 of 1000), and the same cadence walked
through `u64::MAX` and through `2^63` (10 of 1000 each — the wrap costs neither
a lost nor an extra record).

Behavioural consequence worth stating: a build without `platform-clock` now
emits the FIRST record at each throttled C site and then NONE, where it used to
emit every record. The once-only WARN in `nros_log_throttle_admit` says so; the
Rust clock-reading macros still refuse to compile without the feature. The C++
`RCLCPP_*_THROTTLE` path (`nros-cpp/include/nros/log.hpp`) is REFUSE-LOUD
(issue 1019) and has no arithmetic of its own, so there was no third copy to
align.
