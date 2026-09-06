# Phase 162 — RT Scheduling Harness + Hardware Acceptance for Phase 110

**Goal.** Stand up the privileged-scheduling test harness and
hardware-in-the-loop pathway needed to close the remaining
Phase 110 acceptance items. Phase 110 archived all CI-reachable
work; this phase owns the host-side / kernel-side / board-side
setup that turns the parked acceptance criteria into runnable
gates.

**Status.** Not Started — re-audited against the tree 2026-09-06.

None of this phase's six new deliverables exist (`rt-harness`,
`nuttx-sched-sporadic`, `just/rt.just`, the RT chapter, the procurement doc),
so "Not Started" is accurate for the WORK. Four of its PREMISES had drifted,
and are corrected in place below rather than left to mislead the next reader:

| premise as written | tree, 2026-09-06 |
| --- | --- |
| 162.B.1 adds `CONFIG_SCHED_SPORADIC=y` to the arm defconfig | **already `=y`, in all THREE** — `arm`, `arm-smp`, `riscv` |
| 162.C.4 lands a number in `wake-latency-cortex-m3/README.md` | that README **does not exist**; the crate has none |
| 162.C.4 patches `wake_alloc.rs::nros_rmw_runtime_wake_cb` | `wake_alloc.rs` is **gone**; the symbol is `executor/spin.rs:706` |
| the bench "already produces a P99 histogram" | true, but it had ROTTED into no build lane (issue 0317) and was resurrected by phase-313; it now has a `fixtures.toml` row and builds in the `freertos` group |

The blockers are unchanged and are the reason nothing has moved: a
`CAP_SYS_NICE`/RT-kernel host, real Cortex-M3 silicon, and a NuttX kernel built
with sporadic scheduling. Stock CI provides none of them.

**Priority.** P2 — the runtime is shipping today on every
CI-tested target with the documented "soft-RT only" caveat
(`docs/design/0002-rt-execution-model.md` § 4.6, single-thread
non-preemption note). Closing 162 promotes the existing
deadline / sporadic / multi-executor surfaces from "compiles +
unit tested" to "verified against real-time guarantees" — gate
for safety-cert pipelines, not a blocker for the soft-RT cases
already in production use.

**Depends on.** Phase 110 (archived — runtime + API surface
already in tree).

---

## Why this phase exists

Phase 110 split into "runtime + API" (closed) and "real-time
verification" (parked). The parked items can't run in stock CI
because they need one or more of:

1. **`CAP_SYS_NICE` / RT-kernel host.** SCHED_DEADLINE +
   SCHED_FIFO priority changes require either `CAP_SYS_NICE` on
   the running process or a PREEMPT_RT kernel with `chrt`
   permissions configured. GitHub Actions Linux runners give
   neither by default.
2. **Bare-metal Cortex-M3 hardware.** P99 wake-latency ≤ 100 µs
   needs an MPS2-AN385 dev board or STM32F4 Discovery; QEMU's
   DWT CYCCNT emulation is best-effort and the Phase 141 host
   runner `[SKIPPED]`s cleanly when CYCCNT returns 0.
3. **NuttX kernel with `CONFIG_SCHED_SPORADIC=y`** to exercise
   the SCHED_SPORADIC syscall path the runtime emits.

162 documents the setup once and wires the runnable harness so
future regression sweeps don't re-discover the same kernel /
toolchain prerequisites.

---

## Architecture

### A — Linux privileged-scheduling harness

A single host-side harness binary (`nros-rt-harness`) plus a
`just rt-test` recipe that:

1. Detects whether the running process has `CAP_SYS_NICE` (or
   equivalent under `chrt`).
2. Drops a structured `[SKIPPED]` if not — same shape as
   `nros_tests::skip!`.
3. Otherwise runs the parametrised scenario suite (drone S1,
   watchdog S3, sporadic budget enforcement) against the
   Phase 110.D `Executor::open_threaded` path with real
   `SchedPolicy::Deadline` / `Sporadic` policies.

Reused infra:
- `PlatformScheduler` trait from Phase 110.D (POSIX impl already
  emits the `sched_setattr` / `sched_setscheduler` syscalls).
- `nros-tests` skip mechanism + JUnit XML at
  `target/nextest/default/junit.xml`.

### B — NuttX SCHED_SPORADIC verification

NuttX exposes SCHED_SPORADIC via `sched_setscheduler` +
augmented `sched_param`. Verification harness:

1. NuttX QEMU build with `CONFIG_SCHED_SPORADIC=y` in the
   board defconfig (`packages/boards/nros-board-nuttx-qemu/nuttx-config/arm/`).
2. App pins a callback to a `Sporadic`-class SC with a known
   budget / period.
3. Long-running BE callback in another priority slot.
4. Harness asserts the Sporadic SC's elapsed CPU stays within
   budget over N major periods.

### C — Bare-metal Cortex-M3 wake-latency on real silicon

Phase 141.D bench crate
(`packages/testing/nros-bench/wake-latency-cortex-m3/`)
already produces a P99 histogram via DWT CYCCNT — but note its history before
relying on that sentence: it had rotted into NO build lane and did not compile
(issue 0317: retired `_start` entry, phase-230 feature drift, a libc-stub
duplicate symbol), and was resurrected by phase-313. It now carries a
`fixtures.toml` row and builds in the `freertos` group, so the QEMU flow this
item calls "the CI fallback" is real again. Phase 162:

1. Procurement-tracking checklist for MPS2-AN385 or
   STM32F4-Discovery dev kit.
2. Programming guide for the bench binary on hardware (the
   QEMU flow stays the CI fallback).
3. Host-side capture path (UART or RTT) that pipes the CSV
   into the existing host runner.
4. 10× baseline measurement (Phase 141 acceptance item) —
   patch `wake_alloc.rs::nros_rmw_runtime_wake_cb` to no-op,
   capture pre-141 P99 on the same hardware, restore.

### D — Documentation pass

The "book chapter on RT execution model + scenario catalogue"
acceptance item from Phase 110 v1 (line 516) lives here so
it tracks against the verification runs rather than the runtime
landing.

Target chapter: `book/src/internals/0002-rt-execution-model.md`.
Contents:
- Non-preemptive single-thread bound
- SchedClass selection guide (when Fifo / Edf / Sporadic /
  TimeTriggered each apply)
- Scenario catalogue S1–S12 from the design doc, each with
  measured acceptance status (hardware available vs CI-only)
- Pointer to the harness recipes from sections A–C

---

## Work items

### 162.A — Linux privileged-scheduling harness

- [ ] **162.A.1** `tools/rt-harness/` binary or
      `packages/testing/nros-rt-harness/` crate that wraps the
      Phase 110.D Executor + `SchedPolicy::Deadline /
      Sporadic` configuration. Reads scenario manifest from
      argv.
- [ ] **162.A.2** `just rt-test` recipe + a documented setup
      path for `setcap cap_sys_nice+eip` or PREEMPT_RT install.
- [ ] **162.A.3** Drone S1 scenario:
      `nros-rt-harness --scenario=drone-s1` enforces 1 ms
      deadline under sustained 5 ms BE-load on Linux. Asserts
      P99 deadline-miss = 0 over 60 s. Closes Phase 110 v1
      acceptance line 510.
- [ ] **162.A.4** Watchdog S3 scenario: multi-executor
      preemption verified — high-priority executor pre-empts
      low-priority within N µs. Closes Phase 110 v1 acceptance
      line 511.
- [ ] **162.A.5** Sporadic budget enforcement scenario:
      Linux SCHED_DEADLINE path emits the expected
      throttle-on-budget-exhaust behaviour. Closes Phase 110
      post-v1 acceptance line 520 (Linux side).

### 162.B — NuttX SCHED_SPORADIC verification

- [ ] **162.B.1** Add `CONFIG_SCHED_SPORADIC=y` to the
      `nros-board-nuttx-qemu` arm defconfig. **NOTHING TO DO — already true in
      the tree as of 2026-09-06, and wider than this item asked**: set in
      `arm`, `arm-smp` AND `riscv` defconfigs. Verified, not assumed.

      Left UNTICKED deliberately. The box tracks work this phase performs, and
      this phase has performed none; the setting arrived from elsewhere. A tick
      here would also contradict the header (`check-roadmap-claims` R1 catches
      exactly that, and did). So B is smaller than it reads — the Kconfig half
      needs nobody, and only the bench (B.2) and the budget assertion (B.3)
      remain.
- [ ] **162.B.2** Bench binary in
      `packages/testing/nros-bench/nuttx-sched-sporadic/` that
      runs the same scenario as 162.A.5 against NuttX QEMU.
- [ ] **162.B.3** Host runner asserts the Sporadic SC's
      cumulative CPU stays within budget. Closes Phase 110
      post-v1 acceptance line 520 (NuttX side).

### 162.C — Cortex-M3 P99 wake-latency on real silicon

- [ ] **162.C.1** Hardware procurement checklist
      (`docs/reference/hardware-procurement.md` new section):
      MPS2-AN385, STM32F4-Discovery; flashing tools (OpenOCD /
      ST-Link / J-Link); cable + USB-UART adapter for sample
      capture.
- [ ] **162.C.2** Programming guide for
      `wake-latency-cortex-m3` bench binary on real hardware.
      The bench already produces CSV over UART; document the
      capture command + host-side parser invocation.
- [ ] **162.C.3** Run the bench on real hardware; assert P99
      ≤ 100 µs across all three Phase 141.D scenarios. Closes
      Phase 141 spec-acceptance gate (parked in 141 archive).
- [ ] **162.C.4** Capture the 10× baseline: patch
      `nros-node/src/executor/spin.rs:706::nros_rmw_runtime_wake_cb` to no-op,
      measure pre-141 P99 on the same hardware, restore. Land the baseline
      number as a static reference in a **new**
      `wake-latency-cortex-m3/README.md`. Closes Phase 141 10× baseline
      acceptance.

      *Corrected 2026-09-06:* this item named `wake_alloc.rs`, which no longer
      exists — the symbol moved to `executor/spin.rs` (and there is a second,
      `nros_rmw_runtime_wake_cb_from_isr` at :916, which a no-op patch must
      also cover or the ISR path keeps waking). It also said "land the number
      IN `README.md`" as though editing one; the crate has no README, so C.4
      creates it.

### 162.D — PiCAS evaluation (optional / deferred)

The Phase 110.F `OsPrioritySet<N>` stub is in place but the
real dispatch model is reframed as "future node-orchestration
phase." 162.D is a placeholder: once an orchestration phase
proposes a callback-to-priority mapping, the PiCAS interop
acceptance items (Phase 110 line 521-522) land here.

- [ ] **162.D.1** (deferred) Wire 110.F `OsPrioritySet`
      dispatch through the orchestration layer.
- [ ] **162.D.2** (deferred) PiCAS RTAS '21 baseline
      reproduction on Linux (Xavier-class platform).
- [ ] **162.D.3** (deferred) `packages/testing/nros-tests/tests/bridge_picas_priority.rs`
      — Phase 110.F.bridge.

### 162.E — Documentation

- [ ] **162.E.1** `book/src/internals/0002-rt-execution-model.md`:
      non-preemption bound, SchedClass selection guide,
      scenario catalogue S1–S12 with measured status,
      pointer to harness recipes.
- [ ] **162.E.2** Cross-link archive note in
      `docs/roadmap/archived/phase-110-0002-rt-execution-model.md`
      to the chapter once it lands.

---

## Files

### New

- `tools/rt-harness/` or `packages/testing/nros-rt-harness/`:
  Linux privileged-scheduling harness binary.
- `packages/testing/nros-bench/nuttx-sched-sporadic/`:
  NuttX SCHED_SPORADIC bench.
- `book/src/internals/0002-rt-execution-model.md`: chapter.
- `docs/reference/hardware-procurement.md`: dev-board
  procurement + flashing checklist. (162.C.1 calls this a "new section"; the
  FILE does not exist, so it is a new file.)
- `packages/testing/nros-bench/wake-latency-cortex-m3/README.md`: hardware
  capture path + the 10× baseline number. Listed under *Modified* until
  2026-09-06; the crate has never had a README.
- `just/rt.just`: `just rt-test` recipe + capability detection.

### Modified

- ~~`packages/boards/nros-board-nuttx-qemu/nuttx-config/arm/`: defconfig adds
  `CONFIG_SCHED_SPORADIC=y`.~~ **Done already** (all three defconfigs).
- `packages/core/nros-node/src/executor/spin.rs`: the 10× baseline needs
  `nros_rmw_runtime_wake_cb` (:706) AND `..._from_isr` (:916) no-op'd for the
  measurement, then restored. (This list previously named `wake_alloc.rs`,
  which no longer exists.)

`wake-latency-cortex-m3/README.md` moved to the NEW list above: the crate has
no README to modify.

---

## Acceptance criteria

- [ ] `just rt-test` runs locally on a host with
      `cap_sys_nice` set; `[SKIPPED]` cleanly on stock CI hosts.
- [ ] Phase 110 v1 line 510 (drone S1) closed.
- [ ] Phase 110 v1 line 511 (watchdog S3) closed.
- [ ] Phase 110 post-v1 line 520 (Sporadic enforcement on
      NuttX + Linux + user-space) closed.
- [ ] Phase 141 spec acceptance (P99 ≤ 100 µs on Cortex-M3
      hardware) closed against at least one of MPS2-AN385 /
      STM32F4-Discovery.
- [ ] Phase 141 10× baseline captured as static reference.
- [ ] `book/src/internals/0002-rt-execution-model.md` published in
      the rendered book; archive note in Phase 110 points to it.
- [ ] 162.D PiCAS items stay deferred until orchestration
      phase ships — explicit `[ ] (deferred)` marker, not a
      blocker.

---

## Notes

- The harness binary is intentionally *out of* `just test-all` —
  privileged scheduling is opt-in. CI runs see `[SKIPPED]` and
  pass; local / RT-kernel runs flip to PASS / FAIL.
- Hardware procurement isn't a code task; the checklist exists
  so the next person picking up 162.C has a one-stop reference.
  MPS2-AN385 (Arm) and STM32F4-Discovery (ST) are both
  inexpensive ($50-150 range) and well-documented.
- The book chapter is a writing task, not a coding task. Land
  the harness first (162.A–C) so the chapter can cite measured
  numbers instead of theoretical ones.
