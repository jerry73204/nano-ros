# Phase 357 — WCET as declared data: making derived scheduling mean something

**Status (2026-09-04). W3 DONE; W1 DONE — RFC-0078 landed AND its type and
validator are in the tree; W2 is the only open strand, and it is blocked on
model INPUTS rather than on W1.** The 2026-08-18 line said the type and
validator were "still to come" and W2 was "blocked behind that"; both halves
were overtaken. `packages/core/nros-orchestration-ir/src/wcet.rs` carries
`WcetProfile`, its `clock_hz` field and the validator (including the
`clock_hz is 0` rejection and the cycles-to-milliseconds conversion), so W2 is
not waiting on W1 for anything.

W2's two remaining derivations and their real blockers are stated below and are
unchanged: `placement` needs an interference model on top of `SchedCaps.n_cores`
(which now EXISTS), and `non_preempt` needs per-callback priorities within a
tier, which is a MODEL change scoped in issue 0259. Note the inputs those would
consume are still SYNTHETIC — RFC-0078's worked example says so, because QEMU
cannot measure cycles and there is no hardware lane.

**Previously (2026-08-18):** [phase-356](archived/phase-356-test-evidence-and-measurement-trust.md)
W2 landed the artifact (#403 resolved), which was the only thing holding W1.
Three orchestration issues that are one dependency chain, not three tasks.

* **W1 (#404, the WCET schema)** — was **BLOCKED on #403**, which this phase
  failed to record when it was opened. #404's Direction was explicit: "Nothing
  here should be designed before 0403 produces an artifact … Doing it in the
  other order would produce a schema shaped around a hypothetical measurement,
  which is how the keying question gets answered by guess."
  **Unblocked 2026-08-16**: #403 emits `nros.wcet.measurements/1`. Read the
  caveat with it — no run has produced one yet (QEMU cannot measure, and there
  is no hardware lane), so W1 designs against a real FORMAT, not against real
  numbers. The format answers the keying question concretely in one respect
  already: the artifact carries no `clock_hz` and says `convertible_to_time:
  false`, so a declaration in `ms` cannot be derived from it as it stands.
* **W1 update 2026-08-18** — the schema is
  [RFC-0078](../design/0078-wcet-is-declared-per-profile.md): keyed on a named
  measurement PROFILE, declaring cycles plus `clock_hz` with the conversion to
  rlm's millisecond slot done inside this repo, per BOUNDARY at rlm's own
  `node/path` identity. Scope was RFC-only, so the type and validator remain.
  The acceptance's "worked example carrying one real measured callback" is NOT
  met and cannot be here: the RFC's example is marked SYNTHETIC because no run
  has ever produced an artifact (QEMU cannot measure, no hardware lane).
* **W2 (#259, quantitative scheduling)** — partly done 2026-08-19. The blocking
  term `B_i` is DERIVED (`RealizedNode.blocking_us`: the longest
  mutually-exclusive sibling, i.e. the second-largest WCET on the node, `None`
  when fewer than two callbacks carry one). The preemption THRESHOLD is
  deliberately not derived: over a node's own callbacks the ceiling equals the
  node's priority by construction, so it would be a tautology — what
  `non_preempt` actually needs is per-callback priorities within a tier, a
  MODEL change scoped in 0259. `placement` untouched. `B_i` now has a CONSUMER: a
  necessary-condition feasibility check (`C_i + B_i > D_i`) reported as a
  `Degradation { dim: "feasibility" }` carrying its inputs, on the channel
  codegen and the macro already print. The verdicts now reach the ARTIFACT:
  `nros-plan.json` carries an additive `sched_warnings` array and `nros explain`
  renders it above the SchedContext table, so the derivation's inputs are
  visible to a reader rather than only in the bake's scrollback. Acceptance for
  the `budget`/feasibility strand is met. System UTILISATION is derived too
  (`U = ΣC_i/T_i`, reported only when it exceeds one processor, naming the nodes
  that contributed nothing). `placement` and `non_preempt` remain underived, and
  BOTH are blocked on missing model INPUTS rather than on realizer logic:
  `SchedCaps.n_cores` now EXISTS
  (`[deploy.<board>] cores = <n>`, `None` = unknown, non-positive ignored),
  which also fixed the utilisation check's original gate — it tested
  `!affinity`, true for every real target, so it never fired. placement still
  needs finding 2's interference model on top of the count; non_preempt needs
  per-callback priorities within a tier.
* **W3 (#519, sub-millisecond timer period)** — DONE. The render was already
  correct; what was missing was a test pinning it, now added and proven by
  sabotage. The issue's SchedContext half is unowned and folded into W1 below.

**Owns:** [issue 0259](../issues/0259-realizer-placement-nonpreempt-not-derived.md),
[issue 0404](../issues/archived/0404-wcet-declaration-schema.md),
[issue 0519](../issues/archived/0519-plan-timer-period-truncates-sub-millisecond.md)
(**RESOLVED** — see the issue for what closed it).

**Related:** [phase-296](phase-296-system-model-consumption.md) (IN PROGRESS,
names #259 and #260), [phase-162](phase-162-rt-scheduling-harness.md) (the
harness that can measure), [phase-356](archived/phase-356-test-evidence-and-measurement-trust.md)
W2 (the bench that produces the numbers), RFC-0031 / the `system.toml` schema.

## The chain, in order

1. **#404** — there is no schema for declaring a measured WCET: where it lives,
   what it is keyed on, what makes it trustworthy.
2. **#259** — because there is no WCET in the model, derived scheduling is
   **quantitatively inert**: blocking is unmodelled, and `budget`, `placement`
   and `non_preempt` cannot be derived.
3. **#519** — separately, the plan's timer period is still milliseconds, so
   `nros explain` renders a sub-millisecond timer as `0ms`.

#259 cannot be fixed before #404, because the thing it lacks is the data #404
defines. Attempting #259 first produces a second, undeclared place for WCET to
live — the "second spelling" failure CLAUDE.md names repeatedly.

#519 is independent and small; it is here because it is the same subsystem and
because a model that gains WCET while still truncating periods to `0ms` would be
newly misleading.

---

## W1 — A schema for a declared WCET (#404)

The question is not "add a number to `system.toml`". It is the three the issue
asks:

* **Where it lives.** `system.toml` is authored; SystemModels are BUILD
  ARTIFACTS and never committed (phase-330 W4.a/W7, gate
  `check-no-tracked-models`). A measured WCET is neither purely authored nor
  purely derived, which is the actual design problem.
* **What it is keyed on.** A WCET is only meaningful for a (callback, platform,
  board, toolchain, optimisation level) tuple. Under-keying it is how a number
  measured on a host gets applied to a Cortex-M.
* **What makes it trustworthy.** How was it measured, when, on what, and what
  invalidates it. Phase-356 W2 is the measurement side: a bench that reports
  zeros from a dead counter must not be able to feed this.

Design decision ⇒ this wants an **RFC**, not just a phase work item. Write it
before implementing.

**Acceptance.** An RFC in `docs/design/` that answers all three questions, and a
worked example carrying one real measured callback end-to-end.

**UNBLOCKED 2026-08-16 — and one half of the acceptance still cannot be met
here.** #404 said not to design this before #403 emits an artifact, because a
schema written against a hypothetical measurement answers the keying question by
guess, and keying decides whether a number measured on an STM32F407 at 168 MHz
may be applied to a Cortex-M3 in QEMU. #403 now emits
`nros.wcet.measurements/1`, carrying per-measurement min/max/mean/iterations
plus `counter_valid`, `cpu`, `profile` and `commit` — the conditions the keying
question is about. Design against that.

Two facts to design WITH rather than around:

* **The artifact declares its own non-convertibility.** The bench cannot read
  the part's clock, so `clock_hz` is null and `convertible_to_time` is false. The
  mapper wants `ms`. A schema must either carry the clock rate from somewhere
  else or accept cycles as a first-class unit; it must not let a consumer pick a
  rate, which is the manufactured-WCET failure #404 exists to prevent.
* **No run has produced an artifact.** QEMU refuses (no DWT), and this tree has
  no hardware lane. So "a worked example carrying one REAL measured callback"
  remains unmeetable here — the schema can be designed and validated against the
  format, but the worked example needs hardware. Say which of the two a piece of
  work delivered; they are not the same claim.

## W2 — Make derived scheduling quantitative (#259)

Blocked on W1. Once WCET is declarable and keyed, the derivations the issue
names — blocking analysis, and deriving `budget` / `placement` / `non_preempt`
— become expressible.

The issue's word is "inert", not "wrong": today the scheduling model produces
*something*, and that something is not informed by timing. Any work here should
say plainly what changes in `nros explain` output, because that is where a user
sees whether the model has become quantitative.

**Acceptance.** For a system with declared WCETs, at least one of `budget`,
`placement` or `non_preempt` is DERIVED rather than authored, and `nros explain`
shows the derivation's inputs. Do not close #259 on a schema alone.

## W3 — The plan's timer period is milliseconds (#519)

`nros explain` shows a sub-millisecond timer as `0ms`. Independent of W1/W2 and
fixable now.

Note phase-352 (COMPLETE) already moved the platform clock ABI to nanoseconds
with an expressible resolution — "one nanosecond symbol, plus the resolution
nobody could ask for". So the runtime side of this unit question is settled and
the plan should follow it rather than invent a third unit.

**Acceptance.** A sub-millisecond timer renders with its actual period.
Whatever unit is chosen matches phase-352's, and the choice is stated.

**DONE 2026-08-16.** The render already preferred `period_us` and fell back to a
widened `period_ms`; a 500 µs timer prints `timer 500us`. Nothing pinned it, so
three tests were added in `cmd/explain.rs` and verified by sabotage — restoring
the truncation fails the 0519 case and only that one.

**Folded into W1:** #519 also flagged `SchedContext.period_ms` / `budget_ms` /
`deadline_ms`, deferring the unit decision to #505. #505 has since resolved
WITHOUT moving them, so that question is unowned rather than deferred. It is a
plan-SCHEMA change and belongs where the unit for declared timing is settled
once, which is W1 — not three more per-field migrations.

---

## Deliberately not doing

* **Not measuring WCET on hardware in this phase.** That is phase-162's harness
  and phase-356 W2's bench. This phase defines what a measurement must look like
  to be usable, and consumes it.
* **Not deriving anything before the schema exists.** Explicitly the failure
  mode this ordering avoids.
