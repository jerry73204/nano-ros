---
rfc: 0002
title: "RT Execution Model — Live Design Doc"
status: Draft
since: 2026-05
last-reviewed: 2026-06
implements-tracked-by: []
supersedes: []
superseded-by: null
---

# RT Execution Model — Live Design Doc

**Status:** Draft (in progress) · **Owner:** TBD · **Last updated:** 2026-05-04

> **Live doc.** Sections below are filled iteratively as design + RTOS research lands. RTOS-specific subsections marked _PENDING_ are populated by parallel research forks (`tmp/sched-research-*.md`).

## 1. Goals & Non-Goals

### Goals

- **Mixed-criticality callbacks on one executor** — let the user mix critical (RT-bounded), normal, and best-effort callbacks on a single nano-ros `Executor`.
- **Decouple scheduling _policy_ from scheduling _execution model_** — the executor abstracts a `ReadySet` so PiCAS / EDF / CIL-EDF / HSE / FIFO can all live behind one trait.
- **Avoid OS-priority-slot starvation** — N callbacks must NOT consume N OS priority slots. Default model: scheduling decisions in user space, **1 OS priority slot per executor thread**.
- **C / C++ / Rust API parity** — same execution model expressible in all three surfaces. C uses cbindgen-generated vtable + opaque storage; C++ uses thin polymorphic or template wrapper; Rust uses traits.
- **Rust no_std + heapless** — all built-in `ReadySet` impls must work without an allocator.
- **Stable user API across scheduler swaps** — user code (`add_subscription`, `add_timer`, `spin_once`) does not change when scheduler changes (OSEK / ERIKA precedent).
- **Single-executor binary on RTOS** — multiple ROS nodes share one executor in a combined binary; user manually orchestrates today (launch-file orchestration tracked in [`0015-rtos-orchestration.md`](0015-rtos-orchestration.md), out-of-scope here).

### Non-Goals

- **Chain abstraction** — chains are causal relationships discovered _externally_ (e.g. `~/repos/play_launch` from launch files + callback source). nano-ros does NOT model chains; it consumes per-callback hints (`period`, `deadline`, `budget`) that an external tool can derive.
- **Launch-file orchestration** — covered by [`0015-rtos-orchestration.md`](0015-rtos-orchestration.md).
- **Per-callback OS thread priority assignment (PiCAS-style native)** — supported as one optional `Scheduler::OsPriority` mode but not the foundation. Default model uses user-space scheduling with 1 OS pri slot per thread.
- **Hard formal WCRT bounds** — analysis tooling can be built on top, but the core doesn't bake in any specific RTA.

## 2. Relationship to Existing Docs

| Doc                                                          | Layer                                                                                | Status                     |
|--------------------------------------------------------------|--------------------------------------------------------------------------------------|----------------------------|
| [`0016-rtos-scheduling-features.md`](0016-rtos-scheduling-features.md) | OS-thread-level config (zpico / network / app-task priorities via `config.toml`)     | Phase 76 (FreeRTOS landed) |
| [`0015-rtos-orchestration.md`](0015-rtos-orchestration.md)             | Tier-task spawn from launch manifests; cross-tier mutex; multi-tier executor binding | Future phase               |
| **This doc**                                                 | **Intra-executor scheduling** — how `spin_once` selects + dispatches ready callbacks | **Phase 110 (proposed)**   |

The three layers stack:

```
launch manifest                      ← 0015-rtos-orchestration.md (future)
   │
   ▼
tier task (OS thread)                ← 0016-rtos-scheduling-features.md
   │ pri=N, sched_policy=FIFO, stack=4K
   ▼
Executor (this doc)                  ← 0002-rt-execution-model.md
   │ ReadySet<EDF>, SchedContext{...}
   ▼
Callbacks (subscriptions, timers, …)
```

## 3. Current State

Source: `packages/core/nros-node/src/executor/spin.rs:1498..1731`.

**Observed shape:**

```
spin_once(timeout):
  1. drive_io(timeout_ms)            // pump transport
  2. for entry in entries:           // registration order
       bits |= 1 << i if has_data(entry)
  3. trigger evaluation               // Any/All/AnyOf/...
  4. for entry in entries:           // registration order again
       if bits & (1<<i): try_process(entry, dt)
```

**Properties:**

- Single-thread, non-preemptive run-to-completion within a cycle.
- Dispatch order = registration order. **No criticality, priority, deadline.**
- One bitmap stores readiness; no queue.
- `drive_io` pumps the transport (zenoh-pico / xrce-dds), wakes on data via `_z_condvar_wait_until` or platform equivalent (Phase 77 wake primitives).
- Timers, subscriptions, services, clients, action_servers, action_clients, guard_conditions all flow through one `EntryMeta` table indexed by `HandleId`.

**Gaps:**

- High-criticality callbacks may be delayed by registration-order predecessors (HSE _real-time_ side completely unaddressed).
- No way to express "this callback has a 20 ms deadline" or "this callback is best-effort".
- No way to bind an OS scheduling policy (e.g. NuttX `SCHED_SPORADIC`) at the executor level.

## 4. Proposed Execution Model

Four orthogonal abstractions.

### 4.1 `JobDescriptor` (static, registration-time)

Per callback, computed once when the callback is added.

**Rust surface (preferred):**

```rust
pub struct JobDescriptor {
    pub handle_id: HandleId,
    pub kind: EntryKind,
    pub period: Option<NonZeroU32>,       // microseconds; None = aperiodic
    pub wcet_hint: Option<NonZeroU32>,    // microseconds; None = unknown (for analysis)
    pub sched_context_id: SchedContextId,
}
```

No `chain_id`. No baked-in `Criticality` enum — criticality derived from bound `SchedContext`. **No per-callback deadline** — deadline is SC-level (see § 4.2). This avoids the conflict between `JobDescriptor.rel_deadline_us` and `SchedContext.deadline_us` and matches seL4 MCS semantics: callbacks under one SC share one deadline.

**C / C++ ABI surface (sentinel-backed, layout-stable):**

```c
typedef struct {
    nros_handle_id_t handle_id;
    nros_entry_kind_e kind;
    uint32_t period_us;          // 0 = aperiodic
    uint32_t wcet_hint_us;       // 0 = unknown
    nros_sched_context_id_t sched_context_id;
} nros_job_descriptor_t;
```

Convention: **`0` is the sentinel for "absent"** for all `*_us` time fields. Physically meaningful: a 0-period would mean infinite frequency; a 0-budget means unbounded; a 0-deadline means no deadline. Rust side wraps the raw u32 in a newtype:

```rust
#[repr(transparent)]
#[derive(Copy, Clone, Default, Eq, PartialEq)]
pub struct OptUs(u32);

impl OptUs {
    pub const NONE: Self = Self(0);
    pub fn from_us(us: NonZeroU32) -> Self { Self(us.get()) }
    pub fn get(self) -> Option<NonZeroU32> { NonZeroU32::new(self.0) }
    pub fn raw_us(self) -> u32 { self.0 }
}
```

`#[repr(transparent)]` ⇒ same ABI as `u32`; cbindgen emits `uint32_t` directly. Rust API uses `Option<Duration>`-style ergonomics; C sees plain integers w/ documented sentinel.

### 4.2 `SchedContext` (first-class scheduling capability)

Inspired by **seL4 MCS** Scheduling Contexts. Decouples "thread of control" from "right to run for X of every Y".

**Rust surface:**

```rust
pub struct SchedContext {
    pub id: SchedContextId,
    pub class: SchedClass,
    pub period: Option<NonZeroU32>,       // µs; replenishment period (Sporadic) or chain period
    pub budget: Option<NonZeroU32>,       // µs; per-period budget (Sporadic); None = unbounded
    pub deadline: Option<NonZeroU32>,     // µs; relative deadline (Edf); None = no deadline
    pub deadline_policy: DeadlinePolicy,
    pub os_thread: Option<ThreadHandle>,  // None = inherit from current Executor's OS thread
}

pub enum SchedClass {
    Fifo,        // run-to-completion, registration order (default — current behaviour)
    Edf,         // user-space deadline ordering, 1 OS pri slot per thread
    Sporadic,    // budget+period replenishment (server semantics)
    BestEffort,  // background, runs only when nothing else ready
}

pub enum DeadlinePolicy {
    Released,    // abs_deadline = last_release_tick + relative deadline (timers, periodic chains)
    Activated,   // abs_deadline = activation_tick + relative deadline (event-triggered subs)
    Inherited,   // abs_deadline carried in message header (latency-aware; requires header support)
}
```

**C / C++ ABI surface (same sentinel pattern):**

```c
typedef struct {
    nros_sched_context_id_t id;
    nros_sched_class_e class_;
    uint32_t period_us;              // 0 = absent
    uint32_t budget_us;              // 0 = unbounded
    uint32_t deadline_us;            // 0 = no deadline
    nros_deadline_policy_e deadline_policy;
    nros_thread_handle_t os_thread;  // NROS_THREAD_HANDLE_NONE sentinel
} nros_sched_context_t;
```

A single `SchedContext` may be shared by N callbacks. Crucially: **two callbacks sharing one `SchedContext::Edf` consume zero extra OS priority slots vs. one callback** — deadline ordering is in user-space `EdfReadySet`, not in the OS scheduler. Callbacks sharing an SC share its deadline; if per-callback deadlines are needed, create separate SCs.

### 4.3 `ReadySet` (queue + selection abstraction)

**ReadySet stores `(SortKey, DescIdx)` pairs only**, never full `ActiveJob` payloads. Full job info reconstructed from `JobDescriptor[desc_idx]` lookup at dispatch time. Keeps capacity small (4–8 bytes per entry) and makes the bitmap impl trivial.

```rust
pub type DescIdx = u8;

#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd)]
pub struct SortKey(u64);  // EDF: abs_deadline_us; FIFO: insertion-counter; Bucketed: bucket | deadline

pub trait ReadySet {
    /// Idempotent. If `desc_idx` is already ready, this is a no-op
    /// (sort key may update for EDF — implementation-defined).
    /// Returns Err if at capacity (drop-newest semantics; see § 4.7 overflow).
    fn insert(&mut self, key: SortKey, desc_idx: DescIdx) -> Result<(), Overflow>;

    /// Returns the next ready callback in scheduler order. Clears its ready bit.
    fn pop_next(&mut self) -> Option<DescIdx>;

    fn is_empty(&self) -> bool;
    fn len(&self) -> usize;
    fn capacity(&self) -> usize;
}
```

`ActiveJob` is **ephemeral** — emitted by `Activator`, consumed by `ReadySet::insert`, never stored:

```rust
struct ActiveJob {
    pub desc_idx: DescIdx,
    pub key: SortKey,
}
```

Built-in implementations:

| Impl                 | Storage                                       | Selection order       | OS pri slots used | Maps to                         |
|----------------------|-----------------------------------------------|-----------------------|-------------------|---------------------------------|
| `FifoReadySet<N>`    | bitmap (N bits) + insertion order            | registration order    | 1                 | current nano-ros, default ROS 2 |
| `EdfReadySet<N>`     | bitmap (presence) + `heapless::BinaryHeap<N>` | abs_deadline asc      | **1**             | CIL-EDF                         |
| `BucketedFifoSet<N>` | N bitmaps                                     | bucket asc            | 1                 | HSE fair side                   |
| `BucketedEdfSet<N>`  | N (bitmap + EDF heap)                         | bucket asc → deadline | 1                 | HSE critical side               |
| `OsPrioritySet`      | per-OS-pri queue                              | OS scheduler          | up to N           | PiCAS (degenerate)              |

**Idempotency.** `insert(_, desc_idx)` is a *set* operation, not a *queue push*. If `desc_idx` is already ready, the second insert is a no-op. This matches default ROS 2 semantics: one "ready" bit per callback regardless of how many messages arrived. The callback drains the per-handle queue (subscription QoS) on each invocation.

**Capacity & overflow.** All built-ins are heapless w/ const-generic `N`. On capacity exhaustion: drop-newest, increment `executor.dropped_activations` counter. Sized to `MAX_HANDLES` so overflow only happens if every handle becomes simultaneously ready (rare).

### 4.4 `Activator` + `Dispatcher`

**No `dyn Transport` trait.** Transports (zenoh-pico, xrce-dds, dust-dds, uorb, ros2-rmw) have wildly different shapes; forcing a common dyn trait costs vtable + alloc-style indirection (we want to remain alloc-free) without buying anything. Instead, the existing `session.drive_io(timeout_ms)` path stays — it pumps the wire in the platform-native way. The `Activator` runs *after* `drive_io` returns and only consults handle metadata via per-entry `has_data` callbacks. This keeps the design no_std + alloc-free and isolates transport concerns from scheduling concerns.

```rust
pub struct ActivatorCtx<'a> {
    pub entries: &'a [JobDescriptor],
    pub arena: *mut u8,
    pub now_tick: u64,
    pub trigger: &'a Trigger,   // composition: Any/All/AnyOf/AllOf/One/Predicate
    // (no transport ref — drive_io already returned)
}

pub trait Activator {
    /// Called once per spin cycle, after `drive_io` returns.
    /// Scans entries' `has_data` flags, applies the configured `Trigger`,
    /// computes a `SortKey` (per the active SchedClass), and pushes
    /// at most one `ActiveJob` into `ReadySet` per entry per cycle.
    fn scan(&mut self, ctx: &ActivatorCtx<'_>, ready: &mut dyn ReadySet);
}

pub trait Dispatcher {
    /// `arena` is bound at construction (not per-call) — Dispatcher owns the
    /// stable pointer to the executor arena.
    fn run(&mut self, desc_idx: DescIdx, delta_us: u32) -> DispatchResult;
}
```

Note: the `Trigger` (current `Trigger::Any | All | AnyOf | AllOf | One | Predicate | RawPredicate`) lives inside `Activator::scan`, NOT in `Dispatcher`. Trigger composition determines whether a `desc_idx` becomes ready; ReadySet selection picks among ready callbacks.

Refactored `spin_once`:

```rust
fn spin_once<R: ReadySet, A: Activator, D: Dispatcher>(
    session: &mut Session,
    activator: &mut A,
    ready: &mut R,
    dispatcher: &mut D,
    timeout: Duration,
    drain_mode: DrainMode,    // Latched (default) or Greedy
) -> CycleResult {
    let _ = session.drive_io(timeout.as_millis());

    // SPSC ring drain — ISR-deposited jobs flushed into ReadySet
    while let Some(job) = isr_ring.try_pop() {
        let _ = ready.insert(job.key, job.desc_idx);
    }

    // Activator scan: emits ready jobs from polled handle metadata + Trigger
    let ctx = ActivatorCtx { entries, arena, now_tick, trigger: &self.trigger };
    activator.scan(&ctx, ready);

    let snapshot_len = ready.len();   // for Latched mode
    let mut dispatched = 0;

    while let Some(desc_idx) = ready.pop_next() {
        dispatcher.run(desc_idx, delta_us);
        dispatched += 1;
        if drain_mode == DrainMode::Latched && dispatched >= snapshot_len { break; }
        if let Some(budget) = cycle_budget && clock_now() >= cycle_deadline { break; }
    }

    CycleResult { ... }
}

pub enum DrainMode {
    Latched,  // default — drain only the snapshot at scan time; new activations roll to next cycle
    Greedy,   // keep draining until ready set empty; risk unbounded cycle
}
```

### 4.4a Periodic-timer overrun policy (issue 0505)

A periodic timer whose tier is preempted past several of its periods has a
backlog. What happens to it is a semantics decision, and it belongs here rather
than in whichever line of the dispatcher happens to implement it.

```rust
pub enum TimerOverrunPolicy {
    /// DEFAULT. Coalesce the backlog: fire ONCE, drop the missed periods, count
    /// them in `overruns`. Phase-preserving — the sub-period remainder carries
    /// over, so activations stay on the declared cadence grid.
    Skip,
    /// Replay every missed activation back-to-back. For counters and
    /// accumulators, where each tick carries state that must not be lost.
    CatchUp,
}
```

**The default is `Skip`, and the reason is not "bursts are surprising".** It is
that `CatchUp` makes the runtime rate monitor report health during a fault.
Measured A/B on the FreeRTOS mps2-an385 QEMU lane, same tree, only the default
flipped, 2 kHz inbound flood, 2 runs each:

| default  | achieved ctrl rate | replay activations | stalls/run |
| --- | --- | --- | --- |
| CatchUp | **100.03 Hz** (declared 100) | 275/run | 13.0 |
| Skip    | 90.8–93.0 Hz | 8/run | 12.0 |

Under `CatchUp` the island stalls for up to 611 ms at a stretch and the achieved
publish rate is still exactly nominal, because every missed activation is
replayed later inside the same counting window. `check_rate` counts publishes
over ~5 s, so `rate-hierarchy-runtime` is STRUCTURALLY blind to a stall under
`CatchUp` — the one runtime rule that ought to see the failure cannot. Under
`Skip` the same stalls surface as a 7–9 % rate deficit.

Precedent agrees: rclcpp advances `next_call_time` past "now", and Zephyr's
`k_timer` coalesces expiries into one callback carrying an expiry count. Replay
is the surprising behaviour, not the conservative one.

Caveat, so the rate rule is not oversold: a 7–9 % deficit only trips
`rate-hierarchy-runtime` if the declared minimum sits within ~10 % of nominal,
and a single isolated stall (20 missed activations of a 100 Hz loop in a 5 s
window = 0.4 %) will not trip it at any sane threshold. The rate rule catches
sustained degradation; it is not a substitute for the explicit overrun signal
below.

**Overrun is counted and reported, not just handled.** Each timer carries a
saturating `overruns`, and dropped activations reach the same violation ring as
the rate/age/latency/deadline rules as `timer-overrun-runtime` (`fqn` = the bound
SchedContext/tier name, `measured` = overruns accrued in the window, `declared` =
tolerated count, 0 by default). The check runs after dispatch, so a stalling tier
reports in the same cycle. Without it a replayed burst SATISFIES a rate check
while being exactly the failure that check exists for.

**Surface.** `Executor::set_timer_overrun_policy` / `timer_overruns`; the
callback signature stays `FnMut()`, so the counter is queryable rather than
pushed into user code (Zephyr passes an expiry count into the handler; taking
that route here would have been a breaking change for no gain the counter does
not already give). Making the policy declarable in launch-level scheduling
metadata — a tier dim, `real_time` class implying `Skip` — is deliberately NOT
decided here; it belongs with the contract schema.

**Resolution.** Periods are microseconds end to end. They were milliseconds at
three boundaries, each silent: `TimerDuration` stored ms (`from_micros(500)`
produced a zero period that free-runs, `from_micros(1_500)` a 50 % rate error),
the declarative path round-tripped through `EntityMetadata.period_ms`, and
`nros-c` took ns on the ABI then truncated to ms — so the C surface promised
nanoseconds, delivered milliseconds, and the two language paths disagreed about
what a 500 µs timer does (C rejected it, Rust silently free-ran). Measured on the
FreeRTOS lane, a mid tier declared at 33.333 ms: rate error −0.99 % before,
+0.01 % after.

Still true and deliberately not fixed: a period that is not a multiple of its
tier's spin period quantizes to the spin grid (33 ms on a 5 ms spin alternates
35/30 ms, mean 33.07). The long-run rate is right and no activation is dropped,
so no rule fires — correctly — but the jitter is predictable at resolve time and
would be better as a resolver warning than a surprise on target.

### 4.4b Arena placement: a named static, not an anonymous allocation

**Decision.** The executor's backing — and therefore the arena carved from it —
is a **named static in `.bss`** on every entry path, so the largest single RAM
reservation in an image has a symbol.

**Implemented 2026-09-06, phase-392 W6.** The section's original text said the
arena was *"an inline `[MaybeUninit<u8>; ARENA_SIZE]` field of `Executor`"* that
*"lands on whichever task calls `spin`"*. **That had not been true since
phase-271 (issue 0110)**, and it mattered: it aimed the fix at the wrong memory.
Corrected here with the measurement that settled it.

`Executor` holds `arena: &'s mut [MaybeUninit<u8>]`, a slice carved by
`executor::storage::carve` out of caller-supplied backing. **Placement is
therefore the caller's**, and it decided everything:

* **C** — all 34 in-tree `nros_executor_t` objects are file-scope statics, so
  the backing is carved from `_opaque` in `.bss`.
* **C++** — `Node::GlobalStorageHolder<0>::storage`, also `.bss`.
* **Rust** — every board entry (linux, zephyr, freertos, nuttx, threadx,
  esp32-qemu, mps2-an385) reached an `alloc` convenience constructor, which
  `Box::leak`ed the backing. **A heap allocation has no symbol**, so this was the
  invisible one, on every Rust image, on every platform.

Two consequences, both working against the properties this RFC exists to
provide:

* **It was invisible to the tools that are supposed to bound it.** `mem-report`
  reads symbols. Measured on the native zenoh talker: the largest `nros_node`
  RAM symbol in the whole ELF was **1 byte**, and the crate did not appear in
  the by-crate table. After: `nros_node::executor::backing::EXECUTOR_BACKING`,
  **21,560 bytes**, 5.1 % of the image's RAM, fourth-largest symbol.
* **It charged the allocator instead of the linker.** On a hosted target that is
  free — the OS heap has no fixed reservation. On an RTOS the allocator arena is
  *itself* a fixed static (`CONFIG_COMMON_LIBC_MALLOC_ARENA_SIZE` on Zephyr,
  `configTOTAL_HEAP_SIZE` on FreeRTOS), sized to hold this backing, so the bytes
  were reserved statically anyway — just under a name that says "heap".

`nros_node::executor::backing::EXECUTOR_BACKING` is that static.
`NROS_EXECUTOR_BACKING_SECTION` places it in a named section (a `NOLOAD` one:
the static is uninitialised) for parts with tightly-coupled memory;
`NROS_EXECUTOR_BACKING_U64S` resizes it, or removes it entirely at `0`, because
the RTOS half of the move — lowering the allocator arena by the same amount — is
per-image and only its author can measure it (issue 1145). The heap arm remains
for a second executor (tiered boot opens one per tier) and for an entry sized
past the reservation.

**What this does NOT buy, contrary to the original text.** It does not shrink a
task stack. The claim that raising `NROS_SUBSCRIPTION_BUFFER_SIZE` "lands on
that stack", and that a five-subscription image reserves ~320 KiB of *stack*,
followed from the inline-field premise and went with it: phase-271 took the
arena out of the caller's frame, so no task stack has carried a
subscription-proportional term since. The FreeRTOS app-task stack is a separate
number that has still never been derived (issue 1146).

**Orthogonal, and deliberately not decided here: whether payload buffers
themselves become heap-backed.** Phase 392 amendment B reopened that as a
measurement wave — pools are each sized for their own worst case and then
SUMMED, and an arena sized for the aggregate peak could be the larger win. This
decision is about WHERE the arena lives, not what backs the buffers inside it,
and it does not pre-empt that wave: a `.bss` arena is a better starting point
for it, because the thing being measured becomes visible to `mem-report`.

The standing case against, which amendment B must answer rather than assume:
timing is not the objection — rlsf is O(1) (phase 391 W2) — but a heap-backed
payload converts a compile-time constant into a bound that depends on worst-case
concurrent in-flight samples across every topic, widens the heap's block-size
spread (what Robson's bound scales with, and what phase 391 says makes TLSF
sizeable at all), and puts an allocation failure on the receive path, where the
existing failure mode is already "received, ACKed, then silently discarded"
(`report_dropped_take`, issue 0757).

**Rejected alternative: one buffer shared by all subscriptions.** LET sampling
settles it: `SubInfoEntry::sampled_len` records pre-sampled data that must
survive the LET window, so the buffer cannot be reused by another subscription
in the meantime. Sharing would also require mutual exclusion across
receive→deserialize, which couples the WCET of unrelated topics and introduces
a priority-inversion channel between tiers — the opposite of § 4.2's intent. And
it does not even pay: sized to the largest type it saves the difference between
`max` and `sum`, which for a typical island graph is a few KiB against the loss
of per-topic independence.

**What makes the static affordable** is sizing each subscription to its own
type rather than to a global default — `MAX_SERIALIZED_SIZE_XCDR{1,2}` (phase
380) via `rx_buffer_for!`. That is a separate lever (phase 392 W3a) and this
decision does not depend on it, but the two together are what turn the arena
from a stack-resident unknown into a static the linker can print.

### 4.5 ISR ↔ Executor SPSC Ring

ReadySet is single-threaded (no `Sync`). When an ISR (transport RX, hardware timer, GuardCondition fire) needs to activate a callback, it writes into a **lock-free SPSC ring** read by the executor at the top of `spin_once`. Mirrors FreeRTOS `xQueueSendFromISR` / `xQueueReceive`.

```rust
pub struct IsrRing<const N: usize> {
    storage: [UnsafeCell<MaybeUninit<ActiveJob>>; N],
    head: AtomicUsize,   // producer (ISR)
    tail: AtomicUsize,   // consumer (executor)
}
// SAFETY: SPSC contract; producer-only writes head, consumer-only reads tail.
```

On Cortex-M0/M0+ (no LDREX/STREX): atomics fall back to `critical_section` (PRIMASK disable, ~10 cycles).

Already exists in tree as `executor/spsc_ring.rs` — reuse.

### 4.6 Single-Thread Limitations (CRITICAL READ)

**Single-thread, non-preemptive execution cannot guarantee hard deadlines across SchedClasses.** Once a callback enters `try_process`, it runs to completion. A `Fifo` callback w/ 50 ms WCET will block an `Edf` callback w/ 1 ms deadline → **deadline miss**.

This is not a bug — it's the cost of "1 OS pri slot per executor". Within one OS thread, no kernel can preempt user-space code mid-callback without per-callback stacks (which `no_std` Rust can't provide).

**Mitigations within a single executor:**

1. **`wcet_hint` budget check** — compute "would this callback's WCET fit before the next pending deadline?" before dispatch. If not, defer. Soft. Still allows missed deadlines if no callback fits.
2. **Cycle wall-time budget** — cap total cycle time; surplus rolls to next spin. Soft.
3. **Single SchedClass per executor** — one Executor = one SC class. No mixing. Hard but limiting.

**The only hard solution: multi-executor (§ 4.7).** Different SchedClasses → different OS threads → OS handles preemption. **Mandatory for any hard-RT use case** (drone flight controller, watchdog, control-loop @ kHz).

When single-thread is acceptable:
- Soft-RT mobile robot 10 Hz pipelines (sense-plan-act, no overlap)
- Combined-binary multi-node where all callbacks share criticality
- MCU sensor aggregator w/ low rates and known-bounded WCET

When NOT acceptable (use multi-executor):
- Mixed criticality (Critical + BestEffort on same executor)
- Watchdog liveness
- Hard real-time control (>100 Hz period, < period WCET)

### 4.7 Multi-Executor Binding (Embassy `InterruptExecutor` analogue)

For threaded RTOSes, an _Executor_ instance owns one OS thread, one ReadySet, and one set of SchedContexts.

```rust
let exec_rt = Executor::open_threaded(ExecutorConfig {
    os_thread_pri: 90,
    scheduler: Scheduler::Edf,
})?;
let exec_be = Executor::open_threaded(ExecutorConfig {
    os_thread_pri: 30,
    scheduler: Scheduler::Fifo,
})?;
exec_rt.add_subscription_in(sc_critical, "/lidar", |m| { ... })?;
exec_be.add_subscription_in(sc_logging, "/log", |m| { ... })?;
```

Two OS pri slots regardless of callback count. The OS preempts low-pri executor on high-pri executor's wake.

On single-thread MCU: `Executor::open()` (current API) returns one in-tree executor; multi-executor not used.

## 5. Per-RTOS Scheduling Primitives & Fit Check

> Detailed primitives per RTOS — research is parallelised; sections fill in as forks return.

### 5.1 POSIX (Linux + glibc)

**Policies available.** `SCHED_OTHER` (CFS/EEVDF), `SCHED_FIFO`, `SCHED_RR`, `SCHED_BATCH`, `SCHED_IDLE`, `SCHED_DEADLINE` (3.14+, global EDF + CBS via `sched_setattr`). `SCHED_SPORADIC` is POSIX-spec but **not implemented in mainline Linux**.

**Priority space.** `SCHED_FIFO/RR` 1..99, higher = more important. `SCHED_DEADLINE` has no priority — ordered by absolute deadline; conceptually preempts FIFO 99.

**`SCHED_DEADLINE` constraints (`sched_attr`):**
- `runtime > 0`, `runtime ≤ deadline ≤ period`
- Per-CPU bandwidth admission via `dl_bw`; rejects with `-EBUSY` if Σ runtime/period exceeds threshold (default `950ms / 1s`).
- Overrun behaviour: deadline pushed forward (task isn't killed); `SCHED_FLAG_DL_OVERRUN` makes `SIGXCPU` fire on overrun. `SCHED_FLAG_RECLAIM` enables GRUB (preferred for soft RT).
- No `glibc` wrapper — must call `syscall(SYS_sched_setattr, ...)`.

**Refill / wake primitives.**
- `timerfd_create` + `timerfd_settime` — fits epoll/poll, ns resolution. **Best fit for budget-refill events in the executor's I/O loop.**
- `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ...)` — wake exactly at next deadline.
- `eventfd` — cheap thread-to-thread wake.
- POSIX `timer_create` w/ `SIGEV_THREAD` — heavy (thread per fire); avoid.

**Mutex.** `PTHREAD_PRIO_NONE | INHERIT | PROTECT`. Per-mutex via `pthread_mutexattr_setprotocol`. Default = NONE; must opt in. Robust mutex via `PTHREAD_MUTEX_ROBUST`.

**Mixed-criticality features.** PREEMPT_RT mainline (≥6.12). cgroup v2 cpu/cpuset for partitioning. `isolcpus=` + `nohz_full=` boot args. PSI for back-pressure feedback.

**Production gotchas.**
1. `mlockall(MCL_CURRENT|MCL_FUTURE)` — page faults break WCET.
2. RT throttling: `/proc/sys/kernel/sched_rt_runtime_us` defaults to 950 000 (95 %); set to `-1` on dedicated RT systems.
3. `pthread_create` defaults to `PTHREAD_INHERIT_SCHED` — child inherits parent policy. Use `PTHREAD_EXPLICIT_SCHED` or boost via `pthread_setschedparam` after spawn.
4. `prctl(PR_SET_TIMERSLACK, 1)` — defaults to 50 µs.
5. `CAP_SYS_NICE` (or `RLIMIT_RTPRIO`) for FIFO/RR/DEADLINE.

**Fit check.**

| Component                           | POSIX mapping                                                                                                              |
|-------------------------------------|----------------------------------------------------------------------------------------------------------------------------|
| `SchedClass::Fifo`                  | Executor thread on `SCHED_FIFO`, prio 50; user-space FIFO queue. ✅ native.                                                |
| `SchedClass::Edf` (user-space)      | Executor on `SCHED_FIFO`, EDF in `EdfReadySet`, deadlines via `clock_gettime(CLOCK_MONOTONIC)`. ✅ native (1 OS pri slot). |
| `SchedClass::Edf` (native OS)       | Whole executor under `SCHED_DEADLINE` for kernel bandwidth cap; per-callback ordering still in user space. ✅ optional.    |
| `SchedClass::Sporadic` (user-space) | SC tracks budget + refill heap; `timerfd` integrated into `drive_io` to wake on refill. ✅ native.                         |
| `SchedClass::Sporadic` (native OS)  | ❌ No `SCHED_SPORADIC` on Linux; user-space only.                                                                          |
| Multi-executor (per-thread OS pri)  | Each Executor = pthread on `SCHED_FIFO` w/ distinct prio. ✅ native.                                                       |
| Budget refill timer                 | `timerfd` (preferred), `clock_nanosleep`, POSIX timer. ✅ many options.                                                    |
| Deadline timer source               | `CLOCK_MONOTONIC` w/ hrtimer (~1 µs). ✅ native.                                                                           |

**Verdict.** Full design fits cleanly. Linux is the most flexible target — `SCHED_DEADLINE` available as opt-in for kernel-side bandwidth enforcement on top of user-space EDF. Recommend default = `SCHED_FIFO` + user-space EDF; `SCHED_DEADLINE` opt-in via `ExecutorConfig::os_policy`.

### 5.2 NuttX

**Policies available.** `SCHED_FIFO` (always), `SCHED_RR` (`CONFIG_RR_INTERVAL > 0`, system-wide quantum in ms), `SCHED_SPORADIC` (`CONFIG_SCHED_SPORADIC`). `SCHED_OTHER` aliases `SCHED_FIFO`. **No `SCHED_DEADLINE` / EDF.** No partition scheduler.

**Priority space.** 1..255 (`CONFIG_SCHED_PRIORITY_MAX` default 255), higher = more important. Idle = 0.

**`SCHED_SPORADIC`.** Full POSIX 1003.1b §13 implementation:
- `sched_param` extended w/ `sched_priority`, `sched_ss_low_priority`, `sched_ss_repl_period`, `sched_ss_init_budget`, `sched_ss_max_repl`.
- Thread runs at `sched_priority` while in budget; drops to `sched_ss_low_priority` on exhaustion.
- Replenishment of size `consumed` scheduled for `t_arrival + repl_period`; FIFO of pending replenishments capped at `sched_ss_max_repl` (≤ `CONFIG_SCHED_SPORADIC_MAXREPL`, default 8).
- Replenishment granularity = system tick (1 ms default; finer w/ `CONFIG_SCHED_TICKLESS`).
- Replenish callback `nxsched_sporadic_replenish()` runs in tick handler.

**This is the only RTOS we target where the kernel natively implements sporadic-server semantics.** Direct match for `SchedClass::Sporadic` if used at executor-thread level.

**Refill / wake primitives.**
- `timer_create` + `SIGEV_SIGNAL` (kernel POSIX timer; tick or hrtimer granularity).
- `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ...)`.
- `mq_timedreceive` — block on message or absolute deadline.
- For executor: keep sorted refill heap, check at top of `spin_once` against `clock_gettime(CLOCK_MONOTONIC)`. Avoids signal infrastructure.

**Mutex.** Same POSIX API as Linux. `CONFIG_PRIORITY_INHERITANCE=y` enables PI globally; `CONFIG_SEM_PREALLOCHOLDERS` sizes PI bookkeeping (must be ≥ max concurrent PI chains; insufficient sizing silently disables PI on overflow). `sem_setprotocol(&sem, SEM_PRIO_INHERIT)` on counting semaphores (NuttX extension).

**Configurability gotchas.**
- `CONFIG_RR_INTERVAL` system-wide (not per-thread).
- `CONFIG_SCHED_TICKLESS=y` for fine-grained timing (10 ms default tick otherwise).
- Stack in **bytes** (vs FreeRTOS words).
- `pthread_create` honours POSIX attr; `task_create` uses NuttX-native defaults.
- Address environments (`CONFIG_BUILD_KERNEL` / `BUILD_PROTECTED`) for memory partitioning.

**Fit check.**

| Component                           | NuttX mapping                                                                                                           |
|-------------------------------------|-------------------------------------------------------------------------------------------------------------------------|
| `SchedClass::Fifo`                  | Executor on `SCHED_FIFO` mid-prio. ✅ native.                                                                           |
| `SchedClass::Edf` (user-space)      | EDF in `EdfReadySet`, `clock_gettime(CLOCK_MONOTONIC)`. ✅.                                                             |
| `SchedClass::Edf` (native OS)       | ❌ No kernel EDF. User-space only.                                                                                      |
| `SchedClass::Sporadic` (user-space) | Refill heap, executor-tick. ✅.                                                                                         |
| `SchedClass::Sporadic` (native OS)  | ✅ **Native — `SCHED_SPORADIC` per executor thread**. Map directly.                                                     |
| Multi-executor                      | One thread per Executor instance, each pinned to its `SCHED_FIFO`/`SCHED_SPORADIC` prio. ✅.                            |
| Budget refill timer                 | POSIX `timer_create` or in-loop heap. ✅.                                                                               |
| Deadline timer                      | `clock_gettime(CLOCK_MONOTONIC)` — tick-granular w/o tickless. Recommend `CONFIG_SCHED_TICKLESS=y` for executor builds. |

**Verdict.** Excellent fit. NuttX is the platform where `SchedClass::Sporadic` can map natively — Phase 110.E's `Sporadic` executor on NuttX should use kernel `SCHED_SPORADIC` directly (Budget-Micro-ROS RTSS '21 already proved this works). Set `CONFIG_SCHED_TICKLESS=y` + `CONFIG_PRIORITY_INHERITANCE=y` + `CONFIG_SCHED_SPORADIC=y` in nano-ros NuttX board defconfigs.

### 5.3 Zephyr

**Policies available.** Preemptive priority-based (default). Cooperative threads at negative priority — never preempted by other threads (only meta-IRQ). Time slicing via `CONFIG_TIMESLICING=y` + `CONFIG_TIMESLICE_SIZE=N` ms (same-priority preemptive only). **EDF as a tiebreaker** via `CONFIG_SCHED_DEADLINE=y` + `k_thread_deadline_set(tid, delta_cycles)` — among threads of *equal static priority*, earlier deadline runs first; kernel does not enforce, no admission control.

**Priority space.** Cooperative `K_PRIO_COOP(N)..K_PRIO_COOP(0)` (negative), preemptive `K_PRIO_PREEMPT(0)..K_PRIO_PREEMPT(M)` (non-negative). **Lower numeric = higher pri** (opposite of POSIX/FreeRTOS). Default: 16 coop + 15 preempt + idle.

**Scheduler implementation.** `CONFIG_SCHED_DUMB` (linked list, default), `CONFIG_SCHED_SCALABLE` (red-black tree, log N), `CONFIG_SCHED_MULTIQ` (per-priority queues).

**Refill / wake primitives.**
- `K_TIMER_DEFINE(name, expiry_fn, stop_fn)` — `expiry_fn` runs in **IRQ context** (limited APIs). Don't run user callbacks here.
- **Work queues (`k_work_q`)** — first-class deferred-dispatch threads. System work queue auto-created at `CONFIG_SYSTEM_WORKQUEUE_PRIORITY` (default −1, cooperative). Custom: `k_work_queue_init` + `k_work_queue_start(q, stack, size, prio, NULL)`. **Recommended path for nano-ros:** timer expiry → `k_work_submit` to executor's dedicated work queue → handler pushes `JobActivation` into ReadySet.
- `k_sem_take(&sem, K_MSEC(timeout))` — block until semaphore available or timeout. Wakes set via `k_sem_give` (ISR-safe).
- `k_msgq_*` — message queue for activation events.

**Sporadic / budget enforcement.** None native. Emulate via `k_timer` for budget tick + `k_thread_priority_set(tid, lower_prio)` on exhaust + replenishment timer to restore. `k_uptime_get_32` (ms) and `k_cycle_get_32` (raw cycles) for accounting.

**Mutex.** `k_mutex` w/ built-in priority inheritance (automatic). Recursive locking supported. No explicit ceiling protocol API.

**Tickless idle.** `CONFIG_TICKLESS_KERNEL=y` (default since ~2019). Eliminates periodic tick; kernel programs next-event timer based on soonest pending. Budget refill timers piggy-back. `k_cycle_get_32` and `k_uptime_get` remain monotonic.

**SMP.** `CONFIG_SMP=y` + `CONFIG_MP_MAX_NUM_CPUS=N`. Affinity via `k_thread_cpu_pin`, `k_thread_cpu_mask_*` (requires `CONFIG_SCHED_CPU_MASK=y`).

**Mixed-criticality.** `CONFIG_USERSPACE=y` + memory partitions (`k_mem_domain_*`). Spatial isolation only — no temporal CBS. Not safety-certified out of box.

**Meta-IRQ threads.** `CONFIG_NUM_METAIRQ_PRIORITIES` — preempt all other threads (incl. coops + `k_sched_lock`). Designed for "interrupt bottom-half". Severe API restrictions.

**Fit check.**

| Component                           | Zephyr mapping                                                                                                           |
|-------------------------------------|--------------------------------------------------------------------------------------------------------------------------|
| `SchedClass::Fifo`                  | One Zephyr thread at preemptive prio. ✅.                                                                                |
| `SchedClass::Edf` (user-space)      | `EdfReadySet` heap on executor thread. ✅.                                                                               |
| `SchedClass::Edf` (native OS)       | ⚠️ Only as tiebreaker among equal-static-prio threads. Useful only for multi-executor at same prio; otherwise irrelevant. |
| `SchedClass::Sporadic` (user-space) | Atomic budget + `k_timer` for refill. ✅.                                                                                |
| `SchedClass::Sporadic` (native OS)  | ❌ Emulate.                                                                                                              |
| Multi-executor                      | One Zephyr thread per Executor; pin via `k_thread_cpu_pin` on SMP. ✅.                                                   |
| Best-effort tier                    | `BestEffort` SC binds to a low-priority `k_work_q` thread. ✅ clean.                                                     |
| Budget refill timer                 | `k_timer` (ISR-context expiry) → `k_work_submit` to executor wq. ✅.                                                     |
| Deadline timer source               | `k_cycle_get_32` (cycle-precision). ✅.                                                                                  |

**Verdict.** Excellent fit. Zephyr's `k_work_q` is the cleanest "best-effort tier sink" of any RTOS we target — `BestEffort` `SchedContext` can map directly. Tickless kernel keeps refill timers cheap. Cooperative + preemptive split gives a natural "executor cooperative, network IRQ preemptive" arrangement.

### 5.4 FreeRTOS

**Policies available.** Preemptive fixed-priority (default `configUSE_PREEMPTION=1`). Cooperative if `configUSE_PREEMPTION=0`. Time slicing via `configUSE_TIME_SLICING=1` (default) — round-robin among same-prio ready tasks at each tick. Idle hook (`configUSE_IDLE_HOOK=1` + `vApplicationIdleHook`).

**Priority space.** 0..`configMAX_PRIORITIES`-1, **higher numeric = higher pri**. Idle = 0. Typical 5–32 levels.

**No EDF / deadline / sporadic in mainline.** Research forks exist (AhmedAlaa2024, MariusRock/EDF-VD; Carraro 2016 LLREF) — none upstream.

**Refill / wake primitives.**
- **Software timers** — `xTimerCreate(name, period_ticks, autoreload, id, callback)`. **All callbacks run in the timer service (daemon) task** at `configTIMER_TASK_PRIORITY` (recommended: highest). Commands queued via `xTimerQueue` (depth `configTIMER_QUEUE_LENGTH`).
- `xTimerPendFunctionCall(fn, p1, p2)` — defer arbitrary call to daemon task. ISR-safe variant `xTimerPendFunctionCallFromISR`.
- `xQueueReceive(q, &item, xTicksToWait)` — block on queue w/ timeout. Wake via `xQueueSendFromISR` from ISR.
- `xSemaphoreCreateBinary` / `xSemaphoreCreateCounting` + `_Take` / `_Give` (+ `FromISR` variants).
- `vTaskDelayUntil(&prevWake, xPeriod)` — absolute periodic sleep, jitter-resistant.

**Important pattern for nano-ros.** Timer service callbacks run on the **daemon task**, not on the executor task. Best practice: timer callback only pushes `JobActivation` into the executor's `ReadySet` and signals an `xQueue` / `xSemaphore` to wake the executor task. **User callbacks run on the executor task** for deterministic priority.

**Sporadic / budget enforcement.** None. Emulate via `xTimerCreate` + `vTaskPrioritySet(NULL, lower_pri)` on exhaust + replenishment timer.

**Mutex.** `xSemaphoreCreateMutex` — priority inheritance built-in. `xSemaphoreCreateRecursiveMutex` — same w/ recursion. **No priority ceiling.** ISRs MUST NOT use mutexes (no `_FromISR` for mutex take).

**Critical sections.** `taskENTER_CRITICAL()` / `_FROM_ISR()`. Cannot call FreeRTOS APIs inside (use `_FromISR` variants explicitly).

**Yield / sleep.** `taskYIELD()` macro. `vTaskDelay(xTicks)` (relative), `vTaskDelayUntil` (absolute). `pdMS_TO_TICKS(ms)` macro for ms conversion.

**Tickless idle.** `configUSE_TICKLESS_IDLE=1` (basic) or `2` (custom port hook). Kernel calls `portSUPPRESS_TICKS_AND_SLEEP(xExpectedIdleTime)` when idle for ≥ `configEXPECTED_IDLE_TIME_BEFORE_SLEEP` ticks.

**SMP.** FreeRTOS V11+ mainline. `configNUMBER_OF_CORES > 1`. Affinity via `vTaskCoreAffinitySet(task, mask)` or `xTaskCreateAffinitySet`.

**Mixed-criticality.** **FreeRTOS-MPU** for ARMv7-M / ARMv8-M MPU-protected tasks (privileged vs unprivileged via `xTaskCreateRestricted`). **SafeRTOS** (commercial, IEC 61508 SIL 3 / ISO 26262 ASIL D, API-compatible subset).

**Fit check.**

| Component                           | FreeRTOS mapping                                                                                       |
|-------------------------------------|--------------------------------------------------------------------------------------------------------|
| `SchedClass::Fifo`                  | One FreeRTOS task, mid-prio. ✅ native (current behaviour).                                            |
| `SchedClass::Edf` (user-space)      | `EdfReadySet` heap on executor task. ✅.                                                               |
| `SchedClass::Edf` (native OS)       | ❌ No kernel EDF.                                                                                      |
| `SchedClass::Sporadic` (user-space) | Per-SC budget atomic + `xTimerCreate` for refill posted via `xTimerPendFunctionCall`. ✅.              |
| `SchedClass::Sporadic` (native OS)  | ❌ Emulate.                                                                                            |
| Multi-executor                      | One FreeRTOS task per Executor; affinity via `xTaskCreateAffinitySet` on SMP V11+. ✅.                 |
| Best-effort tier                    | Dedicated low-prio task + `xQueue` of work descriptors (no first-class work queue type). ✅ via idiom. |
| Budget refill timer                 | `xTimerCreate` (callback on daemon task; routes to executor via `xTimerPendFunctionCall`). ✅.         |
| Deadline timer source               | Tick-granular by default; sub-tick needs hardware timer or DWT cycle counter on Cortex-M.              |

**Verdict.** Good fit. The big caveat: **all timer callbacks run on the daemon task**, so user callbacks must be routed back to the executor task via `xTimerPendFunctionCall` or queue-post (already the pattern in `nros-platform-freertos`). `configMAX_PRIORITIES` defaults to 5–32 — PiCAS-style per-callback priority would exhaust quickly here. User-space EDF / bucketed criticality keeps OS pri usage to 1–3 slots regardless of callback count.

### 5.5 ThreadX (Eclipse / Azure RTOS)

**Policies available.** Preemptive FPP only. **No EDF, no sporadic, no MLFQ.** `tx_thread_create(...)` single API.

**Priority space.** 0..31 default (`TX_MAX_PRIORITIES` configurable up to 1024). **Direction-flipped: 0 = highest** (same as Cortex-M NVIC, opposite of POSIX/FreeRTOS/NuttX). Ready queue = 32-entry array of FIFO buckets, one per priority.

**Preemption threshold.** `tx_thread_preemption_threshold_change(thread, threshold, ...)` — thread w/ priority `P` and threshold `T ≤ P` can be preempted only by threads w/ priority `< T`. If `T == P` (default), normal preemption. If `T < P`, a *band* of higher-priority threads is suppressed → **non-preemptive groups** (Saksena-Wang scheduling). Useful for "all Normal-tier threads non-preemptive among themselves but preemptable by Critical".

**Time slicing.** `tx_thread_time_slice_change(thread, slice, ...)` per-thread, in ticks. `0` = disabled (default).

**Refill / wake primitives.**
- `tx_timer_create(...)` — runs in ThreadX timer thread (one global, `TX_TIMER_THREAD_PRIORITY`); ISR-safe via API restrictions. `tx_timer_change`, `tx_timer_activate / _deactivate`.
- `tx_event_flags_create / _set / _get` — 32-bit flag group. `set` is ISR-safe. **Cleanest activation source** — Activator polls via `tx_event_flags_get(... TX_OR, wait=ms)`.
- `tx_semaphore_*`, `tx_queue_*` — counting / message queues, ISR-safe `put`.

**Mutex.** `tx_mutex_create(... TX_INHERIT)` — priority inheritance only. No ceiling. Recursion supported. nano-ros default `TX_INHERIT=1` (per CLAUDE.md).

**SMP.** Optional `ThreadX SMP` variant. `tx_thread_smp_core_exclude(thread, exclude_map)` for per-thread CPU affinity.

**NetX Duo BSD gotcha (Phase reminder).** `SO_RCVTIMEO` takes `struct nx_bsd_timeval *`, NOT `int ms`. `fcntl(F_SETFL, O_NONBLOCK)` works. Activator must use non-blocking sockets + event-flag-driven wait, NOT `SO_RCVTIMEO=0`.

**Memory protection.** ThreadX Module Manager (optional, off by default in nano-ros). MMU/MPU-isolated modules.

**Fit check.**

| Component                           | ThreadX mapping                                                                                                                                                                               |
|-------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `SchedClass::Fifo`                  | One ThreadX thread, mid-prio (e.g. 8). ✅ native.                                                                                                                                             |
| `SchedClass::Edf` (user-space)      | `EdfReadySet` heap on one thread; activator wakes via event flag. ✅.                                                                                                                         |
| `SchedClass::Edf` (native OS)       | ❌ No kernel EDF.                                                                                                                                                                             |
| `SchedClass::Sporadic` (user-space) | `tx_timer_create` for refill + atomic budget counter + event flag to wake. ✅.                                                                                                                |
| `SchedClass::Sporadic` (native OS)  | ❌ Emulate.                                                                                                                                                                                   |
| Multi-executor                      | One ThreadX thread per Executor; priorities 4 / 8 / 16 for Critical / Normal / BestEffort. **Use preemption threshold = own priority within bucket** to suppress intra-bucket preemption. ✅. |
| Budget refill timer                 | `tx_timer_create`. ✅.                                                                                                                                                                        |
| Deadline timer source               | `tx_time_get()` (tick-granular). For sub-tick: combine w/ DWT cycle counter on Cortex-M targets.                                                                                              |

**Verdict.** Excellent fit, w/ a bonus knob: **preemption threshold** lets us implement HSE-style "Critical bucket non-preemptive among themselves but preempts Normal" without re-shuffling priorities. Direction-flipped priority needs careful handling in `PlatformScheduler` (already documented in `0015-rtos-orchestration.md` § 10.3).

### 5.6 Bare-metal (Cortex-M, RISC-V)

**Hardware scheduling.**
- **Cortex-M NVIC** — priorities 0..255 register-byte width but **only top N bits implemented** per silicon: M0/M0+ = 2 bits (4 levels), M3/M4/M7 = 3-4 bits (8-16 levels), M33 ARMv8-M = 3-8 bits. **Direction-flipped: 0 = highest.** Subpriority via `SCB_AIRCR.PRIGROUP` breaks ties but doesn't preempt.
- **RISC-V CLIC** — per-IRQ priority + level + preemption-threshold register. SiFive E2/E3, custom ESP32-C3 variant.
- **RISC-V PLIC** — per-IRQ priority, per-hart threshold register. No nesting (unless CLIC also present).
- Idle/main loop runs at "interrupt priority ∞" — preempted by any IRQ.

**RTIC SRP model.** Compile-time Stack Resource Policy. Each shared resource has a *ceiling* = max NVIC priority of any task that accesses it. Access raises `BASEPRI` to ceiling. Deadlock-free, single-stack, bounded blocking. nano-ros could (long-term) layer SchedContexts as RTIC tasks at distinct NVIC priorities.

**Idle / wake.**
- `cortex_m::asm::wfi()` — Wait-For-Interrupt. CPU halts until any pending IRQ. Already used by `nros-board-mps2-an385`.
- `wfe()` — Wait-For-Event (DMA, multi-core SEV). Used in spin-locks.
- RISC-V `wfi` — near-identical semantics.

**Timing primitives.**
- **SysTick** — 24-bit countdown, fires `SysTick_Handler`. Reload sets period.
- **DWT->CYCCNT** (Cortex-M3+) — 32-bit free-running cycle counter, sub-µs precision. Wraps at ~25 s @ 168 MHz.
- **Hardware timers (TIMx)** — 16/32-bit, multiple per chip. Use for periodic activation when SysTick busy.
- **RISC-V `mtime` / `mtimecmp`** — 64-bit memory-mapped, `mcycle` CSR cycle counter.

**Soft-IRQ.** **PendSV** (Cortex-M, lowest pri by convention) — trigger via `SCB->ICSR = SCB_ICSR_PENDSVSET_Msk`. Runs *after* all other ISRs. Could be used as scheduler tail: ISR enqueues + sets PendSV-pending → returns → PendSV ISR drains ReadySet. Removes need for main-loop spin.

**Atomics.**
- Cortex-M3+: `LDREX`/`STREX`, `core::sync::atomic` works.
- Cortex-M0/M0+: no LL/SC. `core::sync::atomic` only loads/stores; RMW needs `critical-section` crate (PRIMASK disable).
- RISC-V: `LR.W`/`SC.W` (RV32A). ESP32-C3 single-hart: interrupt disable suffices.

**BASEPRI / PRIMASK.**
- Cortex-M3+: `BASEPRI` masks IRQs at priority ≥ stored value. `BASEPRI_MAX(value)` only sets if more restrictive.
- `PRIMASK` (1 bit): masks all maskable IRQs. Used by `cortex_m::interrupt::free`.
- Cortex-M0/M0+: only PRIMASK.

**MPU / TrustZone-M.** Cortex-M MPU (8-16 regions, R/W/X), ARMv8-M TrustZone-M (secure / non-secure worlds). RISC-V PMP (≤16 regions). Out-of-scope for nano-ros currently.

**Fit check.**

| Component                           | Bare-metal mapping                                                                                                                                                |
|-------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `SchedClass::Fifo`                  | Single cooperative main loop. ✅ native (current behaviour).                                                                                                      |
| `SchedClass::Edf` (user-space)      | `EdfReadySet` heap on main loop; SysTick advances tick counter; min-deadline pop. ✅.                                                                             |
| `SchedClass::Edf` (native OS)       | ❌ No OS.                                                                                                                                                         |
| `SchedClass::Sporadic` (user-space) | Atomic budget counter (`AtomicI32::fetch_sub`); SysTick or dedicated TIMx ISR refills period. ✅.                                                                 |
| `SchedClass::Sporadic` (native OS)  | ❌ Emulate.                                                                                                                                                       |
| Multi-executor (NVIC tiers)         | RTIC-style: each SchedContext = task at distinct NVIC priority. Critical / Normal / BestEffort = 3 NVIC slots regardless of callback count. ✅ optional (future). |
| Budget refill timer                 | SysTick, TIMx, RISC-V `mtimecmp`. ✅.                                                                                                                             |
| Deadline timer source               | `DWT->CYCCNT` (sub-µs), `mcycle` CSR. ✅ excellent precision.                                                                                                     |
| Idle                                | `wfi`. ✅ already wired.                                                                                                                                          |
| Activator (transport)               | DMA + half/full-complete ISR feeds Activator. ✅ excellent for zero-copy.                                                                                         |

**Verdict.** Best fit for the proposed model overall — cooperative single-thread + priority-bucketed ReadySet maps directly onto current main-loop pattern. **Critical observation:** Cortex-M0/M0+ has only **4 NVIC levels** — PiCAS-style per-callback OS priority would cap at 4 callbacks total. Our user-space-EDF default avoids this entirely; bucketed-by-criticality uses 3 slots regardless of callback count. NVIC tier promotion (RTIC-style) is a long-term opt-in for users who want true cross-tier preemption.

## 6. Fit-Check Matrix

Cross-platform consolidation. ✅ = fits natively / cleanly. ⚠️ = fits w/ caveat (see § 5.x). ❌ = not available; emulate in user space. **n/a** = concept doesn't apply.

| Component                            | POSIX                             | NuttX                                          | Zephyr                         | FreeRTOS                                     | ThreadX                              | Bare-metal                       |
|--------------------------------------|-----------------------------------|------------------------------------------------|--------------------------------|----------------------------------------------|--------------------------------------|----------------------------------|
| `SchedClass::Fifo`                   | ✅ `SCHED_FIFO`                   | ✅ `SCHED_FIFO`                                | ✅ preempt thread              | ✅ default task                              | ✅ thread                            | ✅ main loop                     |
| `SchedClass::Edf` (user-space queue) | ✅                                | ✅                                             | ✅                             | ✅                                           | ✅                                   | ✅                               |
| `SchedClass::Edf` (native OS)        | ✅ `SCHED_DEADLINE` opt-in        | ❌                                             | ⚠️ tiebreaker only              | ❌                                           | ❌                                   | ❌                               |
| `SchedClass::Sporadic` (user-space)  | ✅                                | ✅                                             | ✅                             | ✅                                           | ✅                                   | ✅                               |
| `SchedClass::Sporadic` (native OS)   | ❌ (POSIX has spec, Linux unimpl) | ✅ **`SCHED_SPORADIC`**                        | ❌                             | ❌                                           | ❌                                   | ❌                               |
| Multi-executor (per-thread OS pri)   | ✅ pthread × FIFO prio            | ✅ pthread × FIFO prio                         | ✅ Zephyr threads              | ✅ FreeRTOS tasks                            | ✅ TX threads                        | ⚠️ RTIC-style NVIC tiers (future) |
| Budget refill timer                  | ✅ `timerfd`                      | ✅ `timer_create`                              | ✅ `k_timer` → `k_work_submit` | ✅ `xTimerCreate` + `xTimerPendFunctionCall` | ✅ `tx_timer_create`                 | ✅ SysTick / TIMx / `mtimecmp`   |
| Deadline timer (high-res)            | ✅ ns hrtimer                     | ⚠️ tick (1 ms); fine w/ `CONFIG_SCHED_TICKLESS` | ✅ `k_cycle_get_32`            | ⚠️ tick; sub-tick needs HW timer              | ⚠️ tick; sub-tick via DWT             | ✅ DWT / mcycle (sub-µs)         |
| ISR-safe activation source           | `eventfd` / signal                | POSIX timer / signal                           | `k_sem_give` / `k_work_submit` | `xQueueSendFromISR`                          | `tx_event_flags_set`                 | `critical_section` enqueue       |
| Idle / wake                          | `clock_nanosleep(ABSTIME)`        | same                                           | `k_sem_take` w/ timeout        | `xQueueReceive` w/ timeout                   | `tx_event_flags_get` w/ wait         | `wfi`                            |
| PI mutex                             | ✅ `PTHREAD_PRIO_INHERIT` opt-in  | ✅ default w/ `CONFIG_PRIORITY_INHERITANCE`    | ✅ `k_mutex` automatic         | ✅ `xSemaphoreCreateMutex`                   | ✅ `tx_mutex_create(... TX_INHERIT)` | n/a (single thread)              |
| Best-effort tier                     | ⚠️ separate process / SCHED_IDLE   | ⚠️ low-prio thread                              | ✅ **`k_work_q` first-class**  | ⚠️ low-prio task + `xQueue` idiom             | ⚠️ low-prio thread                    | ⚠️ tail of priority bitmap        |
| OS pri slots used (default model)    | **1**                             | **1**                                          | **1**                          | **1**                                        | **1**                                | **1**                            |
| OS pri slots used (multi-executor)   | 2-3 (RT/normal/BE)                | 2-3                                            | 2-3                            | 2-3                                          | 2-3                                  | 2-3 NVIC tiers                   |
| OS pri slots if PiCAS-native         | up to 99                          | up to 255                                      | up to 31                       | up to `configMAX_PRIORITIES`                 | up to 32                             | up to NVIC bits (4 on M0+!)      |

### Key takeaways

1. **User-space EDF / sporadic queue is the only universally-portable option.** All six platforms support `SchedClass::Edf` and `SchedClass::Sporadic` via user-space queue + cycle counter / tick. Native kernel EDF/sporadic is rare — only Linux `SCHED_DEADLINE` and NuttX `SCHED_SPORADIC` qualify, and both are *opt-in extras on top of* the user-space implementation, not replacements.
2. **Default model uses 1 OS priority slot per Executor.** Bucketed multi-executor at most 3 slots. PiCAS-style per-callback OS priority would saturate **4 NVIC levels on Cortex-M0+** instantly — disqualifying as a foundation.
3. **NuttX is the most RT-feature-complete**: native `SCHED_SPORADIC`, native PI mutex, POSIX timers, configurable tickless. Phase 110.E (`SchedClass::Sporadic` native) should land first on NuttX.
4. **Linux `SCHED_DEADLINE`** is the only kernel-side bandwidth enforcer for the executor as a whole. Worth a `ExecutorConfig::os_policy = OsPolicy::Deadline { runtime, deadline, period }` knob.
5. **Zephyr `k_work_q`** is the cleanest `BestEffort` tier sink. Recommend it as the default `BestEffort` mapping on Zephyr.
6. **ThreadX preemption threshold** lets us implement HSE-style "Critical bucket non-preemptive among themselves but preempts Normal" without re-shuffling priorities.
7. **Bare-metal (Cortex-M, RISC-V)** has the *fewest* OS pri slots but the *best* timing precision (cycle counter sub-µs). Default cooperative single-thread + `wfi` idle fits naturally — the current `nros-board-mps2-an385` pattern generalizes.
8. **All platforms support PI mutex** in some form — `nros-platform`'s mutex abstraction (already planned in `0015-rtos-orchestration.md` § 10.6) covers cross-executor sync.
9. **Direction-flipped priority** (ThreadX, Zephyr-preempt, Cortex-M NVIC: low = high) needs careful handling in the `PlatformScheduler` API. Lift this to a `Priority::abstract` enum (`Critical | Normal | BestEffort`) at the user surface; platform crate translates.

### Gaps requiring resolution

1. **High-resolution timer on Zephyr/FreeRTOS at default tick (~1 ms)** — sub-millisecond deadlines need `CONFIG_SCHED_TICKLESS=y` (Zephyr) or board-specific HW timer hookup (FreeRTOS). Document as a recommended Kconfig in `nros-platform-zephyr` / FreeRTOS board crates.
2. **Cortex-M0/M0+ atomic budget tracking** — no LDREX/STREX. Must use `critical_section` crate. Acceptable cost (µs) for budget bookkeeping but document.
3. **FreeRTOS daemon-task hop for timer callbacks** — already required by current platform layer. Refactor must preserve this; add Activator hop from daemon → executor task via `xTimerPendFunctionCall`.
4. **NuttX `CONFIG_SCHED_SPORADIC_MAXREPL` default 8** — caps replenishment FIFO depth per thread. May limit sporadic concurrency; bump in nano-ros NuttX defconfigs.

## 7. C / C++ / Rust API Parity

### 7.1 Rust

```rust
let sc_critical = exec.create_sched_context(SchedConfig {
    class: SchedClass::Edf,
    period_us: 50_000,
    deadline_us: 50_000,
    budget_us: 30_000,
    os_thread: None,
});

exec.add_subscription_in::<Int32, _>(sc_critical, "/lidar", |m| { ... })?;

// Existing API unchanged — binds to default Fifo SchedContext
exec.add_subscription::<Int32, _>("/topic", |m| { ... })?;
```

### 7.2 C (cbindgen-generated)

```c
typedef enum { NROS_SCHED_FIFO=0, NROS_SCHED_EDF, NROS_SCHED_SPORADIC, NROS_SCHED_BEST_EFFORT } nros_sched_class_e;

typedef struct {
    nros_sched_class_e class_;
    uint32_t period_us;
    uint32_t budget_us;
    uint32_t deadline_us;
} nros_sched_context_config_t;

nros_sched_context_id_t
nros_executor_create_sched_context(nros_executor_t* exec, const nros_sched_context_config_t* cfg);

nros_status_t
nros_executor_add_subscription_in(nros_executor_t* exec, nros_sched_context_id_t sc,
                                  const char* topic, ...);
```

User-supplied `ReadySet` via vtable + caller-owned storage (same shape used by built-ins):

```c
typedef struct {
    void  (*insert)(void* self_, const nros_active_job_t* job);
    bool  (*pop_next)(void* self_, nros_active_job_t* out);
    bool  (*is_empty)(const void* self_);
    size_t (*len)(const void* self_);
} nros_ready_set_vtable_t;

typedef struct {
    void* self_;
    const nros_ready_set_vtable_t* vt;
} nros_ready_set_t;
```

### 7.3 C++

Two flavours, user picks:

- **Header-only template** (zero-overhead, std-only): `nros::Executor exec{nros::EdfReadySet{}};`
- **Polymorphic** (freestanding-friendly): `class IReadySet { virtual void insert(...) = 0; ... };` — under the hood routes through the C vtable.

## 8. Open Issues

1. **Default `SchedContext` for unbound callbacks** — recommend a single `default_fifo` context auto-created at executor startup; `add_subscription` (no `_in`) binds to it. Zero behavioural change for existing examples.
2. **Budget enforcement on bare-metal** — needs SysTick or hardware-timer interrupt to refill. Cost = one ISR per budget period. Recommend opt-in (`SchedClass::Sporadic` requires the platform crate to provide a refill timer source).
3. **High-rate-publisher activation pile-up** — addressed by idempotent `ReadySet::insert` (§ 4.3). One ready bit per callback regardless of message count. Callback drains per-handle queue per ROS 2 QoS.
4. **`SchedContextId` lifetime** — created at startup, never destroyed in current model. Const-generic `MAX_SC` (default 8). Future: deregister on dynamic shutdown.
5. **Compile-time scheduler selection vs runtime** — OSEK lesson: pick at config/build time so MCU builds drop unused scheduler code. Recommend `feature = "scheduler-edf"`, `feature = "scheduler-sporadic"` Cargo gates; runtime selection only when multiple are compiled in.
6. **Const-generic `MAX_HANDLES` ↔ `ReadySet` capacity** — `Executor<const N: usize>` already const-generic over handle count; ReadySet size matches. Wire through.
7. **Trigger composition w/ `Predicate` closure on no_std** — current `Trigger::Predicate(fn(&Snapshot) -> bool)` is a function pointer (no closure capture). `RawPredicate` is C function pointer. Both stay no_std-compatible. Document.
8. **`os_thread: Option<ThreadHandle>` ABI** — `ThreadHandle` is platform-specific (POSIX `pthread_t`, FreeRTOS `TaskHandle_t`, etc.). Use platform-specific opaque type w/ `NROS_THREAD_HANDLE_NONE` sentinel for ABI compat. Concrete shape determined per-platform crate.

## 9. Phasing (proposed)

| Phase                 | Item                                                                                                                           | LOC est. | Use cases unlocked |
|-----------------------|--------------------------------------------------------------------------------------------------------------------------------|----------|--------------------|
| **110.A**             | Refactor `spin_once` → `Activator + ReadySet + Dispatcher` + ISR SPSC ring. Default `R = FifoReadySet`. Behavioural no-op.     | ~400     | Refactor only |
| **110.B**             | `SchedContext` API + `OptUs` newtype + `EdfReadySet`. Builder methods (`.deadline()`, `.period()`).                            | ~400     | Soft-RT EDF on single executor |
| **110.C**             | `BucketedFifoSet<N>` + `BucketedEdfSet<N>` for HSE-style criticality split (still single-thread).                              | ~250     | Soft-RT criticality split |
| **110.D**             | `Executor::open_threaded` + `PlatformScheduler` trait per RTOS. **Mandatory for hard-RT** (see § 4.6).                          | ~600     | **Hard-RT (drone, watchdog, kHz control)** |
| **110.E**             | `SchedClass::Sporadic` + budget refill timer integration. NuttX native `SCHED_SPORADIC`; user-space sporadic server elsewhere. | ~500     | Bandwidth-isolated chains |
| **110.F** _(stretch)_ | `OsPrioritySet` (PiCAS-style) for users who want native OS-level callback priorities.                                          | ~300     | PiCAS interop |
| **110.G** _(stretch)_ | Time-triggered class (`SchedClass::TimeTriggered`) for ARINC-653-style cyclic executive on safety platforms.                   | ~400     | Safety-cert paths |

Each sub-phase is independently shippable. **110.A** is the prerequisite refactor; **110.B** unlocks user-space EDF; **110.D** unlocks hard-RT. Note: 110.D is the *mandatory* phase for hard deadline guarantees — see § 4.6 single-thread limitations.

## 10. References (lit grounding)

- Choi, Xiang, Kim. **PiCAS** — RTAS 2021. Priority-driven chain-aware scheduling (callback-level OS priorities). Inspires `OsPrioritySet`.
- Choi, Karimi, Kim. **CIL-EDF** — ICCD 2020. Chain-instance EDF — inspires `EdfReadySet`.
- Wu, Hu, Yang, Zhang. **HSE + CATMS** — ISPA 2024. Hybrid critical/ordinary scheduler — inspires `BucketedEdfSet`.
- Staschulat, Lange, Dasari. **Budget-based real-time Executor for Micro-ROS** — RTSS 2021 (NuttX `SCHED_SPORADIC`). Inspires `SchedClass::Sporadic`.
- Casini, Blass, Lütkebohle, Brandenburg. **Response-Time Analysis of ROS 2 Processing Chains under Reservation-Based Scheduling** — ECRTS 2019. Linux `SCHED_DEADLINE` foundation.
- seL4 MCS — _Mixed-Criticality Scheduling_, capability-based `SchedContext`. Inspires the SchedContext-as-first-class abstraction.
- Embassy `InterruptExecutor` — multiple async executors at different interrupt priorities w/ unified `Spawner` API. Inspires multi-executor binding.
- ERIKA Enterprise / OSEK / AUTOSAR Classic — scheduler chosen at config time, task code unchanged. Inspires API-stability requirement.
- ARINC 653 / PikeOS — partition-level outer schedule + per-partition inner. Inspires future TT class.

## 11. Scenario Catalogue

Worked examples stress-testing the model. Each names the minimum phase required.

### S1 — Drone flight controller (NuttX, Pixhawk-class) [HARD-RT]

- Timer 1 kHz attitude (Critical, 0.8 ms WCET, 1 ms deadline)
- Sub `/imu` @ 200 Hz (Critical, 0.3 ms WCET, 5 ms deadline)
- Service `/set_mode` (Normal, 2 ms WCET)
- Sub `/telemetry` @ 10 Hz (BestEffort, 5 ms WCET)

**Single-thread `BucketedEdfSet<3>` FAILS.** If `BestEffort` telemetry is mid-callback (5 ms WCET), the 1 ms timer deadline is missed.

**Solution: 110.D multi-executor.** Critical exec @ NuttX prio 200 (`SCHED_FIFO`); BE exec @ prio 50. OS preempts BE thread when Critical wakes. NuttX's native `SCHED_SPORADIC` (110.E) optionally caps Critical exec budget.

### S2 — Mobile robot sense → plan → act (Zephyr) [SOFT-RT]

- LIDAR sub @ 10 Hz, 20 ms WCET → planner sub (chained) @ 30 ms → motor pub @ 10 ms
- Total chain WCET 60 ms < period 100 ms ⇒ no overlap

**Single-thread `EdfReadySet` (110.B) WORKS.** Activator emits jobs in data-flow order (sense data triggers sense; sense's output triggers plan; etc.). Only one callback ready at a time. EDF order irrelevant w/ |ready|=1. ✅

If period drops to 30 Hz (33 ms < 60 ms WCET) → instances pile up → EdfReadySet picks oldest (smallest abs_deadline), graceful FIFO degradation. Idempotent insert (§ 4.3) prevents duplicate ready bits per callback.

### S3 — Watchdog liveness [HARD-RT]

- Watchdog timer @ 100 ms, 1 ms deadline (Critical)
- Heavy user code w/ 200 ms callbacks possible

**Single-thread FAILS** — non-preemptive 200 ms callback blocks watchdog → reset.

**Solution: 110.D multi-executor.** Watchdog exec on dedicated high-priority OS thread.

### S4 — Autoware perception (POSIX, Linux) [HARD-RT]

- Control 1 kHz (0.5 ms WCET, 1 ms deadline)
- DNN inference 10 Hz (50–100 ms WCET, 100 ms deadline)
- Planner 5 Hz (200 ms WCET, 200 ms deadline)
- Logging variable rate (2 ms WCET, BestEffort)

**Solution: 110.D multi-executor.**
- Control exec @ `SCHED_FIFO` 90 + `SchedClass::Edf`
- Perception exec @ `SCHED_FIFO` 70 + `SchedClass::Edf`
- Planner exec @ `SCHED_FIFO` 50 + `SchedClass::Fifo`
- Logging exec @ `SCHED_OTHER` (CFS) + `SchedClass::BestEffort`

Optionally wrap control exec in `SCHED_DEADLINE` (1 ms / 1 ms / 1 ms) for kernel-side bandwidth cap (110.E.linux).

### S5 — Cortex-M0+ minimal sensor node [SOFT-RT]

- 4 NVIC priority levels, 32 KB RAM, 4 KB stack
- 2 callbacks: UART RX sensor @ 100 Hz (1 ms WCET) + control timer @ 100 Hz (5 ms WCET)

**Single-thread cooperative + `FifoReadySet<8>` (110.A only) WORKS.** Heapless capacity 8 sufficient. Atomic budget tracking via `critical_section` (~10 cycles).

PiCAS-native would need 2 NVIC priorities → fits, but 4 callbacks total would exhaust 4 NVIC slots. Avoid.

### S6 — Multi-rate sensor fusion w/ `Trigger::AllOf` (Zephyr) [SOFT-RT]

- IMU @ 1 kHz, GPS @ 10 Hz, Mag @ 100 Hz
- Fusion callback fires only when ALL three have new data

`Trigger::AllOf({imu, gps, mag})` evaluated in `Activator::scan` (§ 4.4). Activator emits *one* `ActiveJob` for the fusion callback when trigger passes. ✅ Maps to current `Trigger` enum unchanged.

### S7 — High-rate publisher overflow (any platform)

- `/imu` @ 1 kHz, callback WCET 1.5 ms
- Cycle wakes once per 10 ms ⇒ ~10 messages pending per cycle

Idempotent insert (§ 4.3): exactly **1** ActiveJob in ReadySet for `/imu`-callback. Callback drains rmw queue (e.g. KEEP_LAST(10)) on each invocation. Default ROS 2 semantics preserved. ✅

If callback can't keep up (1.5 ms × 1 kHz = 150% CPU): rmw drops messages per QoS history policy. Executor stays correct.

### S8 — Long-running action goal

- `/follow_path` goal, 10 s execution

User error if synchronous: blocks executor 10 s. ROS 2 contract: action goals must be non-blocking (state machine + feedback timer). Document. nano-ros's `ActionServer` uses cooperative state machine (Phase 77 work) — already correct.

### S9 — Service handler triggers downstream callback (drain mode)

- Service `/get_state` handler reads param, publishes notification
- Subscriber to notification on same executor

**Latched drain (default, § 4.4):** notification's ActiveJob queued for *next* cycle. Predictable cycle bound. ✅

**Greedy drain (opt-in):** same-cycle dispatch, lower latency, risk of recursive activation. Not recommended.

### S10 — Cross-SC priority inversion

- Critical SC callback waits on mutex held by BestEffort SC callback
- Shared state (e.g. parameter cache)

**Single-thread:** non-preemptive ⇒ BE callback runs to completion before Critical sees mutex. Blocking time = remainder of BE callback's WCET. Same as default ROS 2.

**Multi-thread (110.D):** PI mutex elevates BE thread to Critical priority until release. Standard approach.

### S11 — Combined-binary multi-node sharing one executor [SOFT-RT]

- 3 logical nodes (sensor, planner, controller) compiled into one binary
- All callbacks on shared executor (current nano-ros pattern)
- User decides per-callback SchedContext binding

Works w/ 110.B single executor + multiple SchedContexts. Each node's `add_subscription_in(sc, …)` binds to user-chosen SC. No node-vs-node isolation in single-thread mode (use 110.D for isolation).

This is the **default deployment target** for MCU builds — single binary, multi-node, soft-RT, single executor.

### S12 — Cycle wall-time overrun

- Many callbacks fire simultaneously
- Total cycle would exceed 10 ms but timer scheduled at 10 ms intervals

`ExecutorConfig::cycle_budget_us = Some(8000)` caps cycle at 8 ms. Surplus rolls to next cycle. Useful for cooperative MCU keeping a steady tick rate (e.g. 100 Hz control loop). Opt-in.
