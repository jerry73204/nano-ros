# Phase 436 — The poll/wake revision: a deadline-driven, platform-agnostic executor

**Status (2026-09-07). ALL FIVE WORK ITEMS IMPLEMENTED.** W1–W5 landed
test-first; the executor now computes a park deadline rather than accepting a
timeout. Two things the work changed about the phase as written:

* **W5's issue was wrong about its own evidence.** Issue 1196 said the
  unbounded `condvar_wait` had no callers. It has two — the grep that produced
  that claim searched `nros_platform_cond_wait`, and the symbol is
  `nros_platform_condvar_wait`. W5's gate found them on its first run. Both are
  correct as written (they bridge zenoh-pico's own unbounded primitive), so the
  fix became "confine by path and mark the header" rather than "delete".
* **The `wake_wait_ns` platform slot was NOT built.** Adding a required symbol
  to five ports buys less than the rounding contract does: carrying µs through
  the core and rounding UP at the boundary fixes issue 1193's actual harm — the
  truncation to zero, and the 33 % short wait — without making every port grow
  a slot. A port that gains a finer primitive can lower `park_granularity_us()`
  and nothing above it changes. §1 below describes the ns slot as the eventual
  shape; it is deferred, not done.

Also unbuilt, and deliberately: making the wait loops in `handles.rs` wake off
the backend's listener rather than a fixed grid. W4 fixes the overshoot half of
issue 1195; the waker half is the larger change and stays open.

## The one change this phase is about

Today the executor asks:

> **How long am I allowed to block?**

The answer comes from one place — the caller's `timeout` argument — and is then
narrowed by exactly one other input, `session.next_deadline_ms()`.

It should ask:

> **When is the next thing I owe anybody?**

The answer is a `min` over a set of *declared deadlines*, of which the caller's
budget is only one member. That is the whole design. Everything below is a
consequence:

* a timer deadline can shorten the sleep (issue 1192),
* the wake time is a function of declared periods rather than an arbitrary
  caller constant, so it is **analyzable** — you can enumerate the sources and
  their periods without running anything,
* the bare-metal case stops being a degraded mode and becomes the same
  mechanism with a smaller source set,
* "no busy wait" becomes structural rather than incidental: there is always a
  deadline to park until, even when it is far away.

## What is already right, and must not be rebuilt

The review found the wake machinery in good shape. This phase extends it; it
does not replace it.

* **`NodeWake` is already a platform-agnostic waker.** `nros_platform_wake_{wait_ms,signal,signal_from_isr}`
  is implemented on five ports — Zephyr (`k_sem_take`/`k_sem_give`), ThreadX
  (`tx_semaphore_get`), FreeRTOS, POSIX, ESP-IDF — each with an **ISR-safe**
  signal variant. The core never names an RTOS.
* **Selection is a runtime probe, not a `cfg`.** `has_async_wake` comes from
  `supports_wake_callback()` (`spin.rs:2994`), which for C backends is
  "the vtable slot is non-NULL" (`cffi/src/lib.rs:2591`). A port that gains an
  async source gets the fast path without a core change. This is the extension
  point the phase builds on.
* **Degradation is already correct in kind.** No wake primitive, or a poll-only
  backend, means blocking in the transport's own `recv` for the full timeout —
  not a spin.
* **`spin_period` refuses to run without a clock** (`spin.rs:8343`, issue 0709)
  rather than silently free-running. That is the standard this phase applies to
  the remaining degradations: refuse, or declare, but never silently degrade.
* **Priority inheritance holds on all three RTOSes.** ThreadX `TX_INHERIT`
  (`platform.c:611`), Zephyr `k_mutex`, and — checked because recursive mutexes
  are a common exception — FreeRTOS recursive mutexes do inherit, via
  `xQueueTakeMutexRecursive` → `xQueueSemaphoreTake` → `xTaskPriorityInherit`
  (`third-party/freertos/kernel/queue.c:1791`). No work item here.

## The design

### 1. One time base, nanoseconds, no truncation

`spin_once` currently does `timeout.as_millis()` — a **truncation**
(`spin.rs:6374`). `Duration::from_micros(500)` becomes `0`, which selects the
non-blocking path and turns `spin()` into a 100 % CPU loop with no warning
(issue 1193). The floor is real: all five `nros_platform_wake_wait_ms` slots
take `uint32_t timeout_ms`, so sub-ms cannot reach the primitive on any target.

The revision:

* carry `u64` nanoseconds from the public API to the platform slot,
* add `nros_platform_wake_wait_ns`, keeping `_ms` as the fallback for ports
  whose primitive is genuinely ms-granular,
* add `nros_platform_wake_granularity_ns()` — each port **declares** what it
  can achieve,
* round deadlines **up** to that granularity, never down, and expose the
  achieved value.

The last point is what converts issue 1194 from a defect into a contract. A
1500 us request on a 1 ms-granular port is then reported as *achieving* 2000 us,
rather than silently waiting 1 ms while the jitter rule judges against 1500.
A monitor whose yardstick and whose mechanism disagree cannot support a safety
claim; declaring the rounding makes them agree.

### 2. `WakeSource`: the platform-agnostic seam

```rust
/// Something that knows when it will next need the executor.
pub trait WakeSource {
    /// Absolute time of this source's next event in the executor's clock.
    /// `None` means "nothing pending" — the source does not shorten the park.
    fn next_deadline_ns(&self) -> Option<u64>;

    /// Whether this source can also signal ASYNCHRONOUSLY (ISR or another
    /// thread) via the wake primitive. A source may be deadline-only
    /// (a timer on a tickless port), signal-only (a socket), or both.
    fn is_async(&self) -> bool;
}
```

Core sources, always present, no platform knowledge:

| Source | `next_deadline_ns` | Fixes |
|---|---|---|
| `CallerBudget` | the `spin_once` timeout | — |
| `TimerSource` | `min(period_us - elapsed_us)` over live timer entries | **1192** |
| `SessionSource` | today's `session.next_deadline_ms()`, promoted | — |

Platform sources, registered by the port, never named by the core:

| Port | Natural source |
|---|---|
| Zephyr | `k_poll` event set (sockets, queues, semaphores) |
| POSIX | `epoll` / `eventfd` |
| FreeRTOS | queue set |
| ThreadX | event flags group |
| **Bare metal** | hardware timer compare + `WFI`/`WFE`, ISR calls `wake_signal_from_isr` |

The bare-metal row is why the seam is a *deadline* and not a *sleep*. On a
Cortex-R/M with no RTOS, "park until the earlier of a hardware timer compare
and any interrupt" is precisely `WFI` with the compare programmed — zero CPU,
no scheduler, no busy loop. Expressed as `sleep(duration)` that idiom is not
reachable; expressed as `park_until(deadline)` it is the *primary* case rather
than a degraded one.

### 3. The wait

```
deadline = min over sources of next_deadline_ns()      // never empty:
                                                       // CallerBudget is a member
park_until(deadline)                                   // platform primitive
drive_io(0)                                            // drain what arrived
```

Three-tier realisation, chosen at runtime from the platform probe:

| Port capability | Primitive | Result |
|---|---|---|
| Async wake **and** timed park | `wake_wait_ns` | Parks until deadline **or** signal. |
| Timed park only | `platform_sleep_ns` then `drive_io(0)` | Parks until deadline. |
| Neither (bare metal) | `WFI`/`WFE` + timer compare | Parks until interrupt or deadline. |

No row busy-waits. That is the point: "avoid busy waiting" stops being a
property each port has to remember and becomes a property of the only wait the
core knows how to perform.

### 4. Analyzability

Two additions, both cheap, both turning runtime behaviour into data:

* **Attribution.** Record which source won each park (a `u8` index plus a
  per-source counter). "Why did we wake" becomes a number instead of a guess,
  and it feeds the existing jitter and execution-time reporting.
* **A stated bound.** `spin_once`'s duration is `park + scan + dispatch`. Park
  is bounded by construction. Scan is `O(entries)`. **Dispatch is user code and
  the executor cannot bound it** — it can only measure it, which the
  execution-time high-water probe already does. The contract should say this
  plainly: *the budget is a floor on the wait, not a bound on the call.*
  Today `spin_period` silently absorbs an overrun by skipping its sleep, so a
  tier that chronically overruns looks fine and shows up only in the jitter
  counter.

## The `spin*()` API surface

The review's second half. The surface has grown three units and two meanings.

### Two meanings wearing one shape

```rust
executor.spin_once(timeout);   // timeout = a BLOCKING BOUND
executor.spin_period(period);  // period  = a CADENCE
```

Identical shape, different quantity. This is not hypothetical: nros-cpp's tier
loops paced with `platform_sleep_us(period_us)` while passing a hardcoded
`spin_once(…, 10)`, so `release-jitter-runtime` judged every tier against 10 ms
whatever the contract declared — a 1 kHz tier could run nine periods late and
register as on time. Fixed by making the cadence **declared**
(`set_spin_nominal_us`) rather than inferred from the timeout. That fix is
step one of this phase; the rest is making the distinction impossible to
confuse again, rather than merely corrected in two call sites.

### Three units

| API | Accepts | Reaches the wait as |
|---|---|---|
| `rclc_executor_spin_period` (C) | **nanoseconds** (`executor.rs:2960`) | ms |
| `nros::spin_once` (C++) | **milliseconds** (`nros.hpp:88`, `int32_t timeout_ms = 10`) | ms |
| `Executor::spin_once` (Rust) | `Duration` | ms |

The C API advertises nanoseconds and paces itself correctly in nanoseconds
(`invocation_time_ns += period_ns`, `sleep_ns`) — then truncates at the wait.
The C++ API cannot express sub-millisecond at all. §1 collapses this to one
unit.

### What the usage sites teach

* **The generated workspace entry is the good shape.**
  `actuation_entry_nros_main_generated.cpp` declares the cadence as *data* —
  `NativeTierSpec{ …, 5000ull, … }` — and calls `run_tiers`. It never calls
  `spin_once`. The cadence is declared, the mechanism is the runtime's choice,
  and a resolver can check it before the image is built. **This is the model
  the hand-written paths should converge on**, and the reason the tier-loop bug
  was invisible: the generated entry was right, and the runtime beneath it was
  not.
* **Hand-written C is the best of the three spellings.**
  `rclc_executor_spin_period(&app.executor, 100000000ULL)` — ns, absolute
  deadline accumulation, no drift.
* **Hand-written C++ taught the wrong idiom.** `while (…) { nros::spin_once(100); }`
  across ten examples: a bare bound, no declared cadence, no sleep. `nros::spin()`
  hardcodes `10` (`nros.hpp:176`). These are what the tier loops were modelled on.
* **The bridges sit exactly on the truncation floor.**
  `exec.spin_once(Duration::from_millis(1))` — one step finer and it busy-loops
  (issue 1193).

### Direction

Make the two quantities syntactically distinct rather than documented apart —
a cadence is declared once (as the generated entry already does) and a bound is
passed per call. Keep `spin_once(bound)` as the primitive; every cadence-shaped
wrapper should route through a declaration, so that "which number is this?"
cannot be answered wrongly by a caller who never reads the doc comment.

## Work items

Each is a filed issue; the issue holds the evidence.

* **W1 — [issue 1192](../issues/1192-executor-wait-ignores-next-timer-deadline.md):
  the wait is not bounded by the next timer deadline.** The `TimerSource` of
  §2, and the highest-value item — it is the one place the executor sleeps past
  a deadline it owns. Distinct from resolved issues 0505 (overrun policy) and
  0515 (grid quantization), both of which take the spin boundary as fixed;
  this moves the boundary.
* **W2 — [issue 1193](../issues/1193-spin-timeout-truncates-to-milliseconds.md):
  sub-ms timeouts truncate to a busy loop.** §1. Latent today — nros-cpp clamps
  `max(1_000)` and ASI runs `spin_period_us = 5000` — but `spin_period_us` is a
  microsecond field accepting values it cannot honour.
* **W3 — [issue 1194](../issues/1194-jitter-measured-in-microseconds-waited-in-milliseconds.md):
  jitter measured in µs, waited in ms.** Falls out of W2; the deliverable is
  the declared-granularity contract, not a coarser statistic.
* **W4 — [issue 1195](../issues/1195-promise-wait-polls-instead-of-using-the-waker.md):
  `Promise::wait` polls a 10 ms grid and overshoots shorter timeouts.** The one
  place the waker exists and is bypassed.
* **W5 — [issue 1196](../issues/1196-platform-condvar-wait-is-unbounded.md):
  the unbounded `condvar_wait` in the platform API.** Latent — no callers on
  the executor path — but "no unbounded wait" is only a system property if it
  is an API property.

## Sequencing

W2 before W3 (W3's contract needs W2's granularity probe). W1 is independent
and should go first — it is the largest real-time win and touches only the
core. W4 and W5 are independent of everything.

## What this phase does not do

* It does not make dispatch preemptive. Within one executor, dispatch stays
  non-preemptive; multi-tier preemption comes from the OS scheduler via
  `open_threaded`, unchanged.
* It does not bound user callbacks. It measures them and states that they are
  unbounded.
* It does not change `spin_period`'s drift compensation, which is already
  correct (absolute `next_us` accumulation, not `now + period`).
