---
rfc: 0016
title: "RTOS Scheduling Features for nano-ros"
status: Stable
since: 2026-03
last-reviewed: 2026-06
implements-tracked-by: [phase-227, phase-228]
supersedes: []
superseded-by: null
superseded-in-part-by: [rfc-0079]
---

# RTOS Scheduling Features for nano-ros

> **Config-home note (2026-06).** This RFC's design sections still reference
> `config.toml [scheduling]`. That file is **retired** (Phase 172.K.6): scheduling
> config now lives in `nros.toml` `[node.rt]` for direct-mode embedded apps
> (RFC-0004 §7). Multi-node RT is **decided** (2026-06): node declares callback
> groups; `system.toml [tiers.<name>.<rtos>]` + `[[node_overrides]]` reassignment
> own tier config (RFC-0015 reconciliation); schema/codegen tracked by phase-227 /
> Phase 94. The per-RTOS priority-mapping survey below is unaffected and current.

## Overview

This document surveys real-time scheduling features available in each RTOS platform
supported by nano-ros, maps the runtime task architecture, and proposes a
platform-agnostic scheduling customization API.

## Runtime Task Architecture

A nano-ros application creates multiple tasks/threads at runtime. Understanding
these is essential for scheduling customization:

```
┌─────────────────────────────────────────────────────────────┐
│                    nano-ros Application                      │
│                                                              │
│  ┌──────────────┐  ┌──────────────┐  ┌───────────────────┐  │
│  │  App Task     │  │  Zenoh-pico  │  │  Zenoh-pico       │  │
│  │              │  │  Read Task   │  │  Lease Task       │  │
│  │  • Executor  │  │              │  │                   │  │
│  │  • Callbacks │  │  • Socket RX │  │  • Keep-alive TX  │  │
│  │  • spin_*() │  │  • Decode    │  │  • Lease monitor  │  │
│  └──────────────┘  └──────────────┘  └───────────────────┘  │
│                                                              │
│  ┌──────────────┐  ┌──────────────────────────────────────┐  │
│  │  Net Poll     │  │  Network Stack Task                  │  │
│  │  (FreeRTOS    │  │  (tcpip_thread / IP thread / kernel) │  │
│  │   only)       │  │                                      │  │
│  │  • RX FIFO   │  │  • TCP/IP processing                 │  │
│  │  • Frame feed│  │  • ARP, ICMP, DNS                    │  │
│  └──────────────┘  └──────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

### Task Inventory by Platform

| Task | FreeRTOS | ThreadX | NuttX | Zephyr |
|------|----------|---------|-------|--------|
| **App task** | xTaskCreate, pri=3, 64KB | tx_thread_create, pri=4, 64KB | main() context | main thread |
| **Net poll** | xTaskCreate, pri=4, 1KB | — (NetX handles) | — (kernel) | — (kernel) |
| **Zenoh read** | xTaskCreate, pri=4, 5KB | tx_thread_create, pri=14, 8KB | pthread_create | pthread_create |
| **Zenoh lease** | xTaskCreate, pri=4, 5KB | tx_thread_create, pri=14, 8KB | pthread_create | pthread_create |
| **Net stack** | tcpip_thread, pri=4, 4KB | IP thread, pri=1, 4KB | kernel-managed | kernel-managed |
| **System** | Idle(0), Timer(2) | — | kernel threads | idle, workqueue |

### Priority Maps (current defaults)

**FreeRTOS** (higher number = higher priority, max=7):
```
Pri 0: Idle
Pri 2: Timer
Pri 3: App task
Pri 4: Net poll, tcpip_thread, Zenoh read, Zenoh lease
```

**ThreadX** (lower number = higher priority):
```
Pri 1:  IP thread (NetX Duo)
Pri 4:  App task
Pri 14: Zenoh read, Zenoh lease
```

**NuttX** (POSIX, lower nice = higher priority):
```
Default: All threads at default SCHED_OTHER priority
Available: SCHED_FIFO, SCHED_RR, SCHED_SPORADIC
```

**Zephyr** (negative = cooperative, positive = preemptive):
```
Default: Main thread at default priority
Available: K_PRIO_COOP(n), K_PRIO_PREEMPT(n), deadline scheduling
```

## RTOS Scheduling Features Survey

### FreeRTOS

| Feature | API | Status in nano-ros |
|---------|-----|--------------------|
| Preemptive scheduling | `configUSE_PREEMPTION=1` | **Used** — enabled in FreeRTOSConfig.h |
| Static priorities (0–7) | `xTaskCreate(..., priority, ...)` | **Used** — hardcoded per task |
| Dynamic priority change | `vTaskPrioritySet(handle, new_pri)` | Not used |
| Task notifications | `xTaskNotifyGive()` / `ulTaskNotifyTake()` | Not used |
| Time slicing | `configUSE_TIME_SLICING` | Not used (disabled) |
| Stack watermark | `uxTaskGetStackHighWaterMark(handle)` | Not used |
| Per-task preemption disable | `configUSE_TASK_PREEMPTION_DISABLE` | Not used (FreeRTOS 11+) |
| Task suspend/resume | `vTaskSuspend()` / `vTaskResume()` | Not used |
| CPU usage stats | `configGENERATE_RUN_TIME_STATS` | Not used |

### ThreadX

| Feature | API | Status in nano-ros |
|---------|-----|--------------------|
| Priority-based preemption | `tx_thread_create(..., priority, ...)` | **Used** — hardcoded per thread |
| Preemption-threshold | `tx_thread_create(..., preempt_threshold, ...)` | **Set but ineffective** — threshold = priority |
| Dynamic priority change | `tx_thread_priority_change()` | Not used |
| Time slicing | `time_slice` parameter | Not used (`TX_NO_TIME_SLICE`) |
| Thread suspend/resume | `tx_thread_suspend()` / `tx_thread_resume()` | Not used |
| Runtime thread info | `tx_thread_info_get()` | Not used |
| Event flags | `tx_event_flags_*()` | Not used (could replace polling) |

### NuttX

| Feature | API | Status in nano-ros |
|---------|-----|--------------------|
| SCHED_FIFO | `sched_setscheduler(SCHED_FIFO)` | Not used (runs at kernel default) |
| SCHED_RR | `sched_setscheduler(SCHED_RR)` | Not used |
| SCHED_SPORADIC | NuttX extension | Not used |
| Priority inheritance mutexes | `PTHREAD_PRIO_INHERIT` | Not used |
| pthread priority | `pthread_attr_setschedparam()` | Not used |
| CPU affinity (SMP) | `sched_setaffinity()` | Not used |
| pthread barriers | `pthread_barrier_*()` | Not used |

### Zephyr

| Feature | API | Status in nano-ros |
|---------|-----|--------------------|
| Cooperative threads | `K_PRIO_COOP(n)` | Not used |
| Preemptive threads | `K_PRIO_PREEMPT(n)` | Not used (default) |
| Deadline scheduling (EDF) | `k_thread_deadline_set()` | Not used |
| Meta-IRQ priorities | `CONFIG_NUM_METAIRQ_PRIORITIES` | Not used |
| Thread runtime stats | `CONFIG_SCHED_THREAD_USAGE` | Not used |
| Thread analyzer | `CONFIG_THREAD_ANALYZER` | Not used |
| CPU affinity (SMP) | `k_thread_cpu_pin()` | Not used |

## Customizable Scheduling Properties

### Platform-Agnostic Properties

These properties exist across all RTOS platforms and can be abstracted:

| Property | FreeRTOS | ThreadX | NuttX (POSIX) | Zephyr |
|----------|----------|---------|---------------|--------|
| **Task priority** | 0–7 (higher=more) | 0–31 (lower=more) | nice / sched_param | K_PRIO levels |
| **Stack size** | words at create | bytes at create | pthread_attr | K_THREAD_STACK |
| **Stack usage** | `uxTaskGetStackHighWaterMark` | `tx_thread_info_get` | — | thread analyzer |
| **Scheduling policy** | preemptive only | preemptive only | FIFO/RR/sporadic | coop/preempt/EDF |

### Platform-Specific Properties

| Property | Platform | API |
|----------|----------|-----|
| Preemption-threshold | ThreadX only | `preempt_threshold` param |
| Time slicing | FreeRTOS, ThreadX | `configUSE_TIME_SLICING`, `time_slice` |
| Deadline | Zephyr only | `k_thread_deadline_set()` |
| CPU affinity | NuttX SMP, Zephyr SMP | `sched_setaffinity()`, `k_thread_cpu_pin()` |
| Priority inheritance | NuttX, Zephyr | `PTHREAD_PRIO_INHERIT`, `K_MUTEX_PRIO_INHERIT` |

## Customizable Tasks in nano-ros

The following tasks are candidates for user-controlled scheduling:

### 1. Application Task (executor + callbacks)

The task that runs `Executor::open()`, creates nodes, and calls `spin_*()`.
This is the most important task for user customization since it directly
determines callback latency.

**Current control:** Priority and stack size hardcoded in board crates.

**Desired control:**
- Priority (all platforms)
- Stack size (all platforms)
- Scheduling policy (NuttX: SCHED_FIFO vs SCHED_RR; Zephyr: coop vs preempt)
- CPU affinity (SMP platforms)

### 2. Network Poll Task

Platform-specific task that feeds received frames into the network stack.
Only exists on FreeRTOS (LAN9118 driver). Other platforms handle this in
the kernel or network stack.

**Current control:** Priority and interval hardcoded in board crate.

**Desired control:**
- Priority (must be ≥ zenoh-pico task priority)
- Poll interval (currently 5ms)

### 3. Zenoh-pico Read Task

Internal zenoh-pico task that reads from the transport socket and processes
incoming messages. Created automatically when a session opens.

**Current control:** Defaults from platform shim headers (`Z_TASK_PRIORITY`,
`Z_TASK_STACK_SIZE`). Compile-time only.

**Desired control:**
- Priority (relative to app task — should be ≥ app task)
- Stack size

### 4. Zenoh-pico Lease Task

Internal zenoh-pico task that monitors session keep-alive. Less
latency-sensitive than the read task.

**Current control:** Same defaults as read task.

**Desired control:**
- Priority (can be lower than read task)
- Stack size

### 5. Network Stack Task

OS-managed task (tcpip_thread on FreeRTOS/lwIP, IP thread on ThreadX/NetX,
kernel thread on NuttX/Zephyr). Generally not directly configurable from
nano-ros, but some platforms expose priority settings.

**Current control:** Hardcoded in network stack config headers.

**Desired control:**
- Priority (via platform config, not nano-ros API)

## Scheduling Constraints

Regardless of user customization, these invariants must hold:

1. **Net poll ≥ zenoh read priority** (FreeRTOS): The poll task must be able
   to preempt zenoh-pico's blocking `recv()` to feed frames. Otherwise the
   read task starves the RX FIFO.

2. **Zenoh read ≥ app task priority**: If the app task runs at higher priority
   than the read task, incoming messages may be delayed, causing lease
   timeouts and session drops.

3. **Zenoh lease ≥ idle priority**: The lease task must run periodically to
   prevent session expiration. It can be lower priority than read/app but
   must not be starved.

4. **Stack size minimums**: App task needs enough stack for the executor arena
   (configurable via `NROS_EXECUTOR_ARENA_SIZE`, default 4KB) plus zenoh-pico
   call depth. Minimum ~16KB for simple apps, ~64KB for action servers.

## Design: Scheduling Configuration via config.toml

### Decisions

- **Priority model**: Normalized 0–31 range (higher = more important), mapped per
  platform. Provides fine-grained control while the core library handles
  direction mapping (FreeRTOS: higher=more, ThreadX: lower=more).

  > **RETIRED as a user-facing concept — RFC-0079.** The normalized scale was
  > issue 0623's defect rather than a convenience over it: tier priorities and
  > transport priorities land in ONE kernel scheduler, so a tier written `5`
  > against a transport band reading `16` was ABOVE it, not below, and the two
  > numbers looked comparable while meaning opposite things. `[tiers.*]` now
  > carries no number at all — the user declares a timing contract and the
  > realizer allocates (RFC-0079 §3, §5). `to_freertos_priority` survives only
  > as the single conversion for callers still supplying normalized values;
  > nothing in the FreeRTOS default path calls it.
  >
  > The platform capability tables below are NOT retired. They are the input to
  > RFC-0079 §4's per-port address plans, which is the first thing that has
  > ever consumed them systematically.
- **Configuration source**: `config.toml` file, matching existing pattern.
  Board crates parse the `[scheduling]` section via `Config::from_toml()`.
- **Zenoh-pico task control**: Platform-shim globals set before `zpico_open()`.
  No zenoh-pico source patches required.
- **Implementation order**: FreeRTOS first, then port to other platforms.

### config.toml Format

```toml
[network]
ip = "10.0.2.20"
mac = "02:00:00:00:00:00"
gateway = "10.0.2.2"
netmask = "255.255.255.0"

[zenoh]
locator = "tcp/10.0.2.2:7451"
domain_id = 0

# Optional — all fields have platform-specific defaults
[scheduling]
# Normalized priority 0–31 (higher = more important).
# Mapped to platform-native scale by the board crate.
app_priority = 12           # Application task (executor + callbacks)
zenoh_read_priority = 16    # Zenoh-pico socket read task
zenoh_lease_priority = 16   # Zenoh-pico session keep-alive task
poll_priority = 16           # Network poll task (FreeRTOS only)

# Stack sizes in bytes
app_stack_bytes = 65536      # 64 KB. Sample, not a recommendation: the
                             # FreeRTOS default is 128 KiB, measured (issue
                             # 1146). It does NOT have to fit the executor
                             # arena — that has been a borrowed slice, placed
                             # by the caller, since phase-271.
zenoh_read_stack_bytes = 5120
zenoh_lease_stack_bytes = 5120

# Platform-specific (ignored on other platforms)
poll_interval_ms = 5         # Network poll interval (FreeRTOS only)
```

### Priority Mapping (0–31 Normalized → Platform-Native)

The core library defines the 0–31 normalized scale. Each board crate maps it:

```
Normalized   Meaning              FreeRTOS(0–7)  ThreadX(0–31)  NuttX(POSIX)   Zephyr
─────────────────────────────────────────────────────────────────────────────────────────
 0           Idle                  0              31             nice 19        K_PRIO_PREEMPT(15)
 4           Low                   1              24             nice 10        K_PRIO_PREEMPT(12)
 8           Below normal          2              20             nice 5         K_PRIO_PREEMPT(9)
12           Normal (default app)  3              16             nice 0         K_PRIO_PREEMPT(7)
16           Above normal          4              12             nice -5        K_PRIO_PREEMPT(5)
20           High                  5              8              nice -10       K_PRIO_PREEMPT(3)
24           Very high             6              4              nice -15       K_PRIO_PREEMPT(1)
28–31        Critical              7              1–0            nice -20       K_PRIO_COOP(0–3)
```

Intermediate values are linearly interpolated within the platform's usable range.

**Mapping function** (in each board crate):

```rust
/// Map normalized priority (0–31, higher=more) to FreeRTOS priority (0–7, higher=more).
fn to_freertos_priority(normalized: u8) -> u32 {
    // Linear map: 0→0, 31→7 (configMAX_PRIORITIES - 1)
    let clamped = normalized.min(31) as u32;
    (clamped * (CONFIG_MAX_PRIORITIES - 1) + 15) / 31
}

/// Map normalized priority (0–31, higher=more) to ThreadX priority (0–31, lower=more).
fn to_threadx_priority(normalized: u8) -> u32 {
    // Invert: 0→31, 31→0
    31 - normalized.min(31) as u32
}
```

### Implementation Status

| Component | Status | Files |
|-----------|--------|-------|
| **zpico C API** (`zpico_set_task_config`) | Done | `zpico-sys/c/zpico/zpico.c`, `zpico.h` |
| **zpico Rust FFI** | Done | `zpico-sys/src/lib.rs`, `ffi.rs` |
| **FreeRTOS board crate Config** | Done | `nros-board-mps2-an385-freertos/src/config.rs` |
| **FreeRTOS board crate wiring** | Done | `nros-board-mps2-an385-freertos/src/node.rs` |
| **CMake config parser** | Done | `nros-c/cmake/NanoRosReadConfig.cmake` |
| **FreeRTOS Rust examples** | Done | talker + listener have `[scheduling]` |
| **FreeRTOS C example** | Done (CMake wired) | talker has `[scheduling]` + `APP_*` defs |
| **E2E validation** | Done | `just freertos test` passes; broken-value test confirms config is applied |
| **ThreadX board crate** | Not started | Future work |
| **NuttX board crate** | Not started | Future work |
| **Zephyr board crate** | Not started | Future work |

### zpico Task Config API

`zpico_set_task_config()` stores read/lease task attributes in static globals.
`zpico_open()` passes them to `zp_start_read_task()` / `zp_start_lease_task()`
instead of `NULL`. Platform-specific handling:

| Platform | `z_task_attr_t` | Fields set |
|----------|-----------------|------------|
| FreeRTOS (lwIP) | struct | `.name`, `.priority`, `.stack_depth` |
| POSIX / NuttX / Zephyr | `pthread_attr_t` | stack size via `pthread_attr_setstacksize` |
| ThreadX / Generic | `void*` | No-op (zenoh-pico ignores attr on these platforms) |

### Board Crate Config (FreeRTOS — implemented)

The `Config` struct in `nros-board-mps2-an385-freertos` has 8 scheduling fields
parsed from `[scheduling]` in config.toml. `Config::to_freertos_priority()`
maps normalized 0–31 → FreeRTOS 0–7 linearly. `run()` and `app_task_entry()`
use config values instead of hardcoded constants. `zpico_set_task_config()`
is called before `Executor::open()`.

### CMake Config Parser

`nano_ros_read_config()` in `NanoRosReadConfig.cmake` parses `[scheduling]`
fields and exports `NROS_CONFIG_APP_PRIORITY`, `NROS_CONFIG_APP_STACK_BYTES`,
etc. Defaults match the Rust board crate. C/C++ examples wire these as
`APP_*` compile definitions.

### Control Flow: How Config Reaches Each Task

```
config.toml
  └─ include_str!() at compile time
      └─ Config::from_toml()
          │
          ├─ run() uses config.app_priority, config.app_stack_bytes
          │   └─ nros_freertos_create_task("nros_app", stack, priority)
          │
          ├─ app_task_entry() uses config.poll_priority, config.poll_interval_ms
          │   └─ nros_freertos_create_task("net_poll", stack, priority)
          │
          └─ app_task_entry() sets zpico globals before Executor::open()
              └─ zpico_set_task_config(read_pri, read_stack, lease_pri, lease_stack)
                  └─ zpico_open() passes z_task_attr_t to zp_start_read/lease_task()
```

### Zenoh-pico Platform Shim: Global Task Config

Add to `zpico.c` (the nano-ros zpico wrapper, not zenoh-pico itself):

```c
// Global task attributes, settable before zpico_open().
// Board crate calls zpico_set_task_config() to override defaults.
static z_task_attr_t g_read_task_attr = {
    .name = "zp_read",
    .priority = configMAX_PRIORITIES / 2,  // platform default
    .stack_depth = 5120,
};
static z_task_attr_t g_lease_task_attr = {
    .name = "zp_lease",
    .priority = configMAX_PRIORITIES / 2,
    .stack_depth = 5120,
};
static bool g_task_config_set = false;

void zpico_set_task_config(
    uint32_t read_priority, uint32_t read_stack,
    uint32_t lease_priority, uint32_t lease_stack
) {
    g_read_task_attr.priority = read_priority;
    g_read_task_attr.stack_depth = read_stack;
    g_lease_task_attr.priority = lease_priority;
    g_lease_task_attr.stack_depth = lease_stack;
    g_task_config_set = true;
}
```

Then in `zpico_open()`, pass the attrs instead of NULL:

```c
// Before:
zp_start_read_task(z_session_loan_mut(&g_session), NULL);
zp_start_lease_task(z_session_loan_mut(&g_session), NULL);

// After:
z_task_attr_t *read_attr = g_task_config_set ? &g_read_task_attr : NULL;
z_task_attr_t *lease_attr = g_task_config_set ? &g_lease_task_attr : NULL;
zp_start_read_task(z_session_loan_mut(&g_session), read_attr);
zp_start_lease_task(z_session_loan_mut(&g_session), lease_attr);
```

The board crate calls `zpico_set_task_config()` via FFI before `Executor::open()`:

```rust
// In app_task_entry(), before running the user closure:
unsafe {
    zpico_set_task_config(
        to_freertos_priority(config.zenoh_read_priority),
        config.zenoh_read_stack_bytes,
        to_freertos_priority(config.zenoh_lease_priority),
        config.zenoh_lease_stack_bytes,
    );
}
```

### CMake Config Parser Update

`cmake/NanoRosConfig.cmake`'s `nano_ros_read_config()` adds `[scheduling]` parsing:

```cmake
# New variables set by nano_ros_read_config():
# NROS_CONFIG_APP_PRIORITY, NROS_CONFIG_APP_STACK_BYTES,
# NROS_CONFIG_ZENOH_READ_PRIORITY, NROS_CONFIG_ZENOH_READ_STACK_BYTES,
# NROS_CONFIG_ZENOH_LEASE_PRIORITY, NROS_CONFIG_ZENOH_LEASE_STACK_BYTES,
# NROS_CONFIG_POLL_PRIORITY, NROS_CONFIG_POLL_INTERVAL_MS
```

C examples pass these as compile definitions, same as network config.

### Portability Considerations

The normalized 0–31 scale is the **only** scheduling concept that crosses
crate boundaries. Board crates own the mapping to platform-native values.

**What stays in the core library (`nros-node`):** Nothing. The core executor
is scheduling-agnostic — it doesn't create tasks or set priorities. This is
intentional: the executor runs within whatever task the board crate creates.

**What stays in the board crate:** All priority mapping, task creation, and
platform-specific scheduling logic. The board crate is the natural owner
because it already creates the tasks.

**What stays in `zpico.c`:** The `zpico_set_task_config()` global, called
by board crates via FFI. This avoids patching zenoh-pico upstream.

**Portability contract:** A `config.toml` with `[scheduling]` fields works
on any platform. Platform-specific fields (e.g., `poll_interval_ms`) are
silently ignored on platforms that don't use them.

### Implementation Plan (FreeRTOS First)

1. Add `zpico_set_task_config()` to `zpico.c` + `zpico.h`
2. Add scheduling fields to FreeRTOS board crate `Config` + `Default`
3. Add `[scheduling]` parsing to `Config::from_toml()`
4. Add `to_freertos_priority()` mapping function
5. Wire `run()` and `app_task_entry()` to use config values instead of consts
6. Add FFI call to `zpico_set_task_config()` before `Executor::open()`
7. Add `[scheduling]` section to one example's `config.toml` (talker)
8. Update CMake `nano_ros_read_config()` for C examples
9. Test with `just freertos test`

### Future Platform Ports

Once FreeRTOS works, porting to other platforms means:
- Add `to_<platform>_priority()` mapping function to each board crate
- Wire the config fields into each platform's task creation calls
- Add platform-specific fields where needed (e.g., ThreadX preemption-threshold)

## Trace Visualization (Tonbandgeraet)

The scheduling config was validated using [Tonbandgeraet](https://github.com/schilkp/Tonbandgeraet),
an open-source embedded tracer that hooks FreeRTOS trace macros and outputs to
[Perfetto](https://ui.perfetto.dev). Integration is opt-in via `NROS_TRACE=1`.

### Validation Results

Two traces were captured with the FreeRTOS talker example + zenohd:

**Default scheduling** (`app_priority=12, poll_priority=16`):
- `net_poll` task created at FreeRTOS priority 3 (`to_freertos_priority(16) = 3`)
- Task switches: IDLE → net_poll (5ms poll) → app → tcpip_thread → zenoh read/lease
- 16 KB snapshot buffer filled with ~500+ events during 10-message publish run

**Modified scheduling** (`app_priority=20, poll_priority=20`):
- `net_poll` task created at FreeRTOS priority 4 (`to_freertos_priority(20) = 4`)
- Higher priority for both app and poll tasks — different preemption pattern

The traces confirm that `[scheduling]` config values flow through to FreeRTOS
`xTaskCreate()` and `zpico_set_task_config()`, producing measurably different
task priorities. The Perfetto timeline view shows task execution slices, context
switches, and queue operations.

### Usage

```bash
just freertos trace talker     # Capture + convert
# Open test-logs/freertos-trace/trace.pf in https://ui.perfetto.dev
```

## References

- [Executor fairness analysis](../reference/executor-fairness-analysis.md)
- [QEMU icount clock synchronization](../reference/qemu-icount.md)
- [FreeRTOS LAN9118 debugging](../guides/freertos-lan9118-debugging.md)
- [Schedulability analysis (archived)](archived/schedulability-analysis.md)
- [WCET analysis (archived)](archived/wcet-analysis.md)
- [Real-time analysis (book)](../../book/src/internals/realtime-analysis.md)
