---
id: 1131
title: "Fifteen knob-sized C arrays have no ruling on whether zero is a legal size"
status: open
area: rmw, memory
severity: medium
related: [1015, 1033, 0815, phase-392, phase-403, phase-412]
---

# The wider question issue 1015 left open, now enumerated

## What is known

`check-c-array-pool-floors` (issue 1015) reports the whole class:

```
$ python3 scripts/check-c-array-pool-floors.py --audit | tail -1
  21 knob-sized arrays, 0 problem(s)
```

21 fixed C arrays are sized by a macro a build can set (an `#ifndef` fallback,
i.e. a `-D` overrides it). Three carry a floor guard, three are ruled
zero-legal with a measurement behind them (issue 1033), and **fifteen have no
ruling at all**:

| knob | file |
| --- | --- |
| `NROS_COMPONENT_MAX_TIMERS` | `packages/api/nros-cpp/include/nros/component_node.hpp` |
| `NROS_RMW_UORB_PX4_MAX_CALLBACKS` | `packages/rmw/uorb/nros-rmw-uorb/src/px4_callback_glue.cpp` |
| `NROS_THREADX_MAX_TIMERS` | `packages/platform/nros-platform-threadx/src/timer.c` |
| `NROS_ZEPHYR_MAX_THREADS`, `NROS_ZEPHYR_MAX_TIERS` | `zephyr/nros_platform_zephyr_shims.c` |
| `STRESS_SIZE` | `packages/testing/nros-bench/stress-zenoh-zephyr/src/main.c` |
| `XRCE_BUFFER_SIZE`, `XRCE_MAX_PENDING_REPLIES`, `XRCE_SERVICE_REQUEST_RING_DEPTH` | `packages/rmw/xrce/nros-rmw-xrce/src/internal.h` |
| `ZPICO_GET_REPLY_BUF_SIZE`, `ZPICO_MAX_LIVELINESS`, `ZPICO_MAX_PENDING_GETS`, `ZPICO_MAX_PENDING_REPLIES`, `ZPICO_MAX_SESSIONS` | `packages/rmw/zenoh/zpico-sys/c/zpico/zpico.c` |
| `Z_TASK_STACK_SIZE` | `packages/rmw/zenoh/zpico-sys/c/platform/threadx/platform.h` |

They are in the gate's `UNCLASSIFIED` table, which may only shrink.

## Why this is not urgent, and not nothing

**Not urgent:** none of the fifteen is DERIVED today. Each is reachable only by
a person stating a number — a `.conf`, a `[env]` row, a `-D` — so the failure
issue 1015 measured (a derivation computing 0 on its own and nobody noticing)
cannot arrive through them. `ZPICO_MAX_LIVELINESS` and `ZPICO_MAX_PENDING_GETS`
are the closest: `config-knob-census` already classes them "derived — phase-392
is deciding", so the moment that lands they become 1015's shape exactly.

**Not nothing:** the five `zpico.c` entries are SIBLINGS OF THE ARRAY THAT BIT,
in the same struct. `ZPICO_MAX_SESSIONS = 0` gives `g_sessions[0]` and an image
that can open no session at all. Whether that is a build error, a loud runtime
refusal, or the same silence as 1015 is unknown, and "unknown" is the answer for
all fifteen.

## What closing this looks like

Per knob, one of two, and both are cheap:

* a guard beside the array —
  `#if <KNOB> < 1` / `#error "... it sizes a C array (issue 1015)"` — plus the
  floor in every producer that could derive it; or
* a `ZERO_LEGAL` entry with what MEASURED that zero is legal *for that array*.
  Issue 1033 is the worked example: the arrays are plain mid-struct members, not
  flexible ones; `arr[0]` compiles on gnu99/c11/gnu11; every walk is
  `for (i = 0; i < MAX; ++i)`, which at 0 does not run. That reasoning is per
  array, not per backend — the zenoh pools failed the same test on hardware.

Reading the uses is not enough on its own: 1015's board went silent at 0 with no
diagnostic and the mechanism was never isolated, so a knob whose uses "look
bounded" still wants a build or a run behind the claim.
