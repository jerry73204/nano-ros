---
id: 1196
title: "`nros_platform_condvar_wait` blocks forever by construction, beside a bounded variant nothing forces you to prefer"
status: open
type: tech-debt
area: platform
related: [issue-1192, phase-436]
---

## Problem

The platform API exposes an unbounded condvar wait on every RTOS port:

```c
int8_t nros_platform_condvar_wait(void *cv, void *m) {
    if (cv == NULL || m == NULL) return -1;
    nros_platform_mutex_unlock(m);
    UINT rc = tx_semaphore_get((TX_SEMAPHORE *) cv, TX_WAIT_FOREVER);
```
(`packages/platform/nros-platform-threadx/src/platform.c:683`; the FreeRTOS and
ESP-IDF ports use `portMAX_DELAY` at `platform.c:702` and `platform.c:423`)

A bounded `nros_platform_condvar_wait_until(cv, m, abstime_ms)` sits directly
beneath it, and the executor's own wait path correctly uses the *bounded*
`nros_platform_wake_wait_ms` instead.

## Correction (2026-09-07): it HAS callers

**The original claim here — "no callers" — was wrong, and the error was in the
grep rather than the reading.** The search run was for
`nros_platform_cond_wait`; the symbol is `nros_platform_condvar_wait`. The typo
produced a clean false negative, and this issue was filed calling a live call
site latent. Phase-436 W5's gate found the real callers on its first run:

```
packages/rmw/zenoh/zpico-sys/c/zpico/platform_aliases.c:302  _z_condvar_wait
packages/rmw/zenoh/zpico-sys/c/zpico/platform_aliases.c:320  _z_condvar_wait_until, NULL deadline
```

Both are **correct as written**, which is why this stays an issue about the API
rather than becoming a bug about those lines. `_z_condvar_wait` is zenoh-pico's
own primitive and its contract is an unbounded wait; mapping an unbounded
upstream call onto the unbounded platform call is the honest bridge, and
narrowing it there would silently change zenoh-pico's semantics. The `_until`
variant falls back only when handed a NULL deadline.

The accurate statement: the executor's **own** wait path is bounded
(`nros_platform_wake_wait_ms`), and the unbounded primitive is reached only
through a shim implementing an upstream API that is itself unbounded.

## Why file it anyway

nano-ros claims real-time properties, and "no unbounded wait" is only a
property of the system if it is a property of the **API**, not of the current
call graph. An unbounded primitive sitting in the platform header with a
neutral name is an invitation: the next port, or the next backend, reaches for
`condvar_wait` because it is the obvious spelling, and the bound is lost
silently and without review.

The mutex waits in the same files (`K_FOREVER`, `portMAX_DELAY`,
`TX_WAIT_FOREVER`) are **not** part of this issue — those are short critical
sections and all three ports have priority inheritance: ThreadX creates with
`TX_INHERIT` (`platform.c:611`), Zephyr's `k_mutex` inherits, and FreeRTOS
recursive mutexes route through `xQueueSemaphoreTake` → `xTaskPriorityInherit`
(`third-party/freertos/kernel/queue.c:1791`). That was checked because
recursive mutexes are a common priority-inheritance exception; here they are
not one.

## Fix direction — DONE in phase-436 W5

Deleting the symbol was rejected: it is exported, out-of-tree ports link it,
and one in-tree caller legitimately needs unbounded semantics.

1. `include/nros/platform.h` marks it **NOT REAL-TIME — this wait is
   UNBOUNDED**, names the bounded alternatives, and says why it was kept.
2. `scripts/check-no-unbounded-condvar-wait.sh` confines it to the zenoh-pico
   alias shim **by path**. A call site anywhere else fails the gate and has to
   be argued for, rather than inheriting a blanket allowance.

Verified by adding a call outside the shim and watching the gate reject it.
