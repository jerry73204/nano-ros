---
rfc: 0073
title: "Platform clock: one nanosecond symbol plus a resolution query"
status: Draft
since: 2026-08
last-reviewed: 2026-08
implements-tracked-by: []
supersedes: []
superseded-by: null
---

# RFC-0073 — Platform clock: one nanosecond symbol plus a resolution query

## Summary

Replace the two fixed-unit monotonic symbols (`nros_platform_clock_ms`,
`nros_platform_clock_us`) with **one** symbol in nanoseconds plus a
**resolution query**, and demote the unit conversions to `static inline`
header wrappers that no port implements.

```c
uint64_t nros_platform_clock_ns(void);
uint64_t nros_platform_clock_resolution_ns(void);
```

The ABI stops asserting a precision it cannot keep and starts stating
the precision it has.

## Motivation

The current surface fixes a *unit* but cannot express *resolution*, so
every port either lies or truncates:

- **Microseconds are a lie** where the source is a tick. ThreadX returns
  `tx_time_get() * MS_PER_TICK * 1000` — 10 ms steps at its default
  100 Hz tick, under a microsecond signature. The MPS2-AN385 bare-metal
  port is literally `clock_ms() * 1000`.
- **Nanoseconds are discarded** where the source is finer. POSIX gets a
  `timespec` and divides the ns away. On mps2-an385 the SysTick counter
  resolves 40 ns at 25 MHz (measured: consecutive `SYST_CVR` reads step
  by 0–1 cycle) while the ABI reports 1 µs. STM32F4 has DWT at 168 MHz —
  6 ns — behind a `MonotonicClock` documented as "Resolution is 1 ms".
- **The header already contradicts itself**: the wall clock advertises
  `nros_platform_time_since_epoch_nanos()` while monotonic stops at µs.

Three filed defects are the same missing fact:
[#0502](../issues/archived/0502-freertos-threadx-clock-us-ms-quantized.md)
(µs that was really ms), [#0515](../issues/archived/0515-period-not-multiple-of-spin-quantizes-silently.md)
(a period the spin cadence cannot express) and
[#0531](../issues/archived/0531-zephyr-cortex-m-clock-us-returns-zero.md)
(a clock that returned 0 forever on every Cortex-M Zephyr board under
60 MHz, undetected because nothing asserts a clock advances).

### The cost objection, measured and refuted

The standing objection to nanoseconds is "finer unit, more expensive on
a 32-bit MCU". Measured on the mps2-an385 lane — each candidate an
`#[inline(never)]` function reading `LOAD`/`VAL` plus the tick, 1000
calls under `qemu -icount shift=0`, where one guest instruction is 1 ns
of guest time:

| candidate | instructions/read |
|---|---|
| `us`, as the port does today (`cycles * 1_000 / (load+1)`) | 91 |
| `ns`, naive (`cycles * 1_000_000_000 / (load+1)`) | 94 |
| `ns`, exact multiplier (`cycles * 40`) | **37** |
| raw counter + frequency, no conversion | 34 |

The unit costs nothing (94 vs 91). The **runtime division** costs
everything (91 → 37). And nanoseconds make the division avoidable *more
often* than microseconds do: ns-per-cycle is an integer at 25, 50, 100,
125, 200 and 250 MHz (40, 20, 10, 8, 5, 4), while µs-per-cycle is never
an integer above 1 MHz. A microsecond ABI is forced into a divide
precisely where a nanosecond ABI gets a multiply.

Today's µs path is therefore the most expensive option measured, and
the executor reads it every spin.

## The interface

In `<nros/platform.h>`, replacing the `Clock (monotonic)` block:

```c
/* ---- Clock (monotonic) ---- */

/** Monotonic nanoseconds since a platform-defined epoch (boot, program
 *  start, ...). Never decreases. Wraps after ~584 years.
 *
 *  Must be backed by a hardware counter or the OS tick — never by a
 *  software counter that only advances when polled.
 *
 *  Available immediately after platform init, before any other nros
 *  subsystem. SHOULD be callable from an ISR; a port whose clock is not
 *  ISR-safe must say so in its port documentation (FreeRTOS ports must
 *  use the `FromISR` tick accessor when appropriate). */
uint64_t nros_platform_clock_ns(void);

/** Granularity of `nros_platform_clock_ns`, in nanoseconds: the
 *  smallest non-zero difference two successive reads can report.
 *
 *  Examples: 1000000 for a 1 kHz tick, 40 for a 25 MHz cycle counter,
 *  1000 for a microsecond hardware timer.
 *
 *  Must be non-zero, and constant for the lifetime of the program after
 *  platform init. A port whose underlying rate is only known at runtime
 *  (Zephyr `CONFIG_TIMER_READS_ITS_FREQUENCY_AT_RUNTIME`) returns the
 *  resolved value; one whose rate can change under it (a scaling APB
 *  clock) returns the COARSEST value it may exhibit. There is no
 *  "unknown" encoding: a port that cannot answer honestly is reporting
 *  a clock it cannot honestly offer. */
uint64_t nros_platform_clock_resolution_ns(void);

/* Unit conversions. Header-only: no port implements these, and the ABI
 * mirror gate skips `static inline` by design. */
#ifndef NROS_PLATFORM_LEGACY_CLOCK_UNITS
static inline uint64_t nros_platform_clock_us(void) {
    return nros_platform_clock_ns() / 1000u;
}
static inline uint64_t nros_platform_clock_ms(void) {
    return nros_platform_clock_ns() / 1000000u;
}
#endif
```

`NROS_PLATFORM_LEGACY_CLOCK_UNITS` is the deprecation escape hatch: an
out-of-tree port that still *defines* `clock_ms`/`clock_us` as real
symbols sets it to suppress the wrappers, keeping the old shape working
for one release.

### Rust mirror

`PlatformClock` keeps two required methods and provides the rest, so a
Rust port implements exactly what the C ABI requires:

```rust
pub trait PlatformClock {
    /// Monotonic nanoseconds. See the C ABI for the full contract.
    fn clock_ns() -> u64;

    /// Granularity of `clock_ns`, in nanoseconds. Non-zero.
    fn clock_resolution_ns() -> u64;

    fn clock_us() -> u64 { Self::clock_ns() / 1_000 }
    fn clock_ms() -> u64 { Self::clock_ns() / 1_000_000 }
}
```

`nros_platform_export_clock!` emits only the two ABI symbols.

## Port implementation guidance

The measurement above is the whole guidance, and it belongs in the ABI
text rather than being rediscovered per port:

1. **Frequency divides 1e9 evenly** (25/50/100/125/200/250 MHz):
   multiply by a compile-time `NS_PER_CYCLE`. This is the fast path —
   37 instructions on mps2 versus 91 today.
2. **It does not** (12 MHz `qemu_cortex_m3`, 168 MHz STM32F4): use a
   fixed-point reciprocal — multiply by `2^32 / freq`, then shift — to
   stay division-free at the cost of a wider multiply. Worth measuring
   before mandating; a plain divide is acceptable and is still no worse
   than today.
3. **Tick-only sources** (ThreadX, non-Cortex-M FreeRTOS): `ticks *
   (1e9 / tick_hz)`, a compile-time constant multiply, and report the
   matching resolution. These ports get *cheaper* than today, which
   divides.

A port must not invent precision: a 100 Hz tick reports
`resolution_ns = 10000000` even though its return value is in
nanoseconds.

## Conformance

`nros-platform-cffi`'s port tests gain three cases. The second is the
one that would have caught #0531:

- **Monotonic** — over N reads, no value is less than its predecessor.
- **Advances** — two reads separated by a platform sleep of 10 ms differ
  by at least 5 ms. A clock stuck at zero fails here; nothing in the
  tree asserts this today.
- **Resolution is honest** — over many back-to-back reads, no non-zero
  delta is smaller than `clock_resolution_ns()`.

## Migration

One change across the tree, because the header wrappers and the port
definitions cannot coexist:

- Header + Rust mirror + `generated.rs` + the export macro.
- Six C ports (posix, zephyr, freertos, threadx, esp-idf, and the cffi
  test stubs) and three Rust ports (mps2-an385, stm32f4, esp32-qemu).
- Callers: `clock_us` has the load-bearing ones (executor timer
  accounting, `nros::time`, `nros-log` timestamps, `nros-c`'s
  `get_time_ns`, which becomes a direct call instead of `us * 1000`);
  `clock_ms` has three (two in the CycloneDDS shim, one XRCE
  session-timeout loop). All keep compiling via the wrappers.
- Out-of-tree ports: `NROS_PLATFORM_LEGACY_CLOCK_UNITS` for one
  release, then the symbols go.

## Rejected alternatives

- **Linux-style clock ids.** Un-implementable with fidelity on four of
  six backends: FreeRTOS, ThreadX and ESP-IDF have no clock ids at all,
  and NuttX has five but its `clock_getres` returns the same
  tick-derived number for every one of them. The multiplexing exists in
  Linux for namespaces, vDSO variants and per-process CPU time — none
  of which apply here. Wall-clock vs monotonic, the one distinction
  that does apply, is already two separate ABI groups.
- **Raw counter + frequency** (the NuttX `perf_gettime`/`perf_getfreq`
  shape). Saves 3 instructions over exact-ns (34 vs 37, ~8%) while
  pushing wrap handling, frequency plumbing and conversion onto every
  caller. Right answer for a profiling counter with genuinely unknown
  units; wrong trade here.
- **Keeping both `clock_ms` and `clock_us` as ABI symbols.** Two entry
  points are two chances to disagree, and they already have: on Zephyr
  `clock_ms` is `k_uptime_get()` (tick) while `clock_us` is the cycle
  counter — different hardware sources with no enforced epoch relation,
  despite the header claiming "the same epoch".
- **A cheap/coarse second symbol** (the `CLOCK_MONOTONIC_COARSE`
  analogue). Deferred, not refused. The gap is real — a bare tick read
  is ~7 instructions against exact-ns's ~37 — but small in absolute
  terms once the division is gone, and no caller has yet been shown to
  read the clock often enough to care. Add it when one is.

## Open questions

- Whether the executor's per-spin read is frequent enough to justify
  the deferred coarse variant. Measurable on the existing lane.
- Whether `resolution_ns` should be permitted to change after init for
  a port with a scaling clock, instead of the conservative
  worst-case rule above.
- The same instruction-count measurement on silicon: QEMU `icount`
  counts instructions, not cycles, so the ratios should hold but the
  absolute microseconds should not be quoted.

## Changelog

- 2026-08 — initial draft, from the clock-capability survey in
  [#0532](../issues/archived/0532-platform-clock-abi-unit-and-resolution.md).
