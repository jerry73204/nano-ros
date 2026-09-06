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

## Current exposure: latent, not live

Grepping `packages/core/` and `packages/rmw/` for `nros_platform_condvar_wait`
(excluding generated bindings) returns no callers. Nothing on the executor
path can block forever today.

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

## Fix direction

Either delete `nros_platform_condvar_wait` in favour of the `_until` variant,
or keep it and mark it non-RT in `include/nros/platform.h` with the reason —
so that choosing it is a decision someone made, not a spelling someone picked.
