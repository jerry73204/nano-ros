# Phase 416 — The tier-2 lane runs end to end, and one fact has one spelling

**Status (2026-09-04). Opened from a campaign that was already half-landed.**
Seven issues resolved and merged, four open, two of those filed on the last day
of the work. The phase exists because the campaign had no home: every issue in
it was unowned except 0968, and an issue with no phase is an issue nobody is
accountable for.

## Why these are one phase

The campaign started as "make `just build-test-fixtures lane=tier2` finish" and
turned into one recurring defect with two faces:

* **One fact, two authored spellings, drifting apart.** The generated header
  mirror (#0978), the config-header writer (#0990), the message-size bound
  (#0981), the ten CMake entry templates against `emit_cpp.rs` (#1003), and
  preflight's targets list read from the wrong root (#0999). In each, both
  copies were individually defensible and nothing compared them.
* **A check that cannot reach what it claims to check.** Four gates written
  DURING this campaign were vacuous on first draft and only caught by testing
  them against the pre-fix code. #0999's own test searched ancestors for the
  file it was meant to prove was findable, so it could never fail. #1003 shipped
  with no gate at all (#1017).

The two faces are the same thing: a second copy survives precisely because
nothing is in a position to notice it.

**Distinct from the neighbours.** [phase-410](archived/phase-410-ci-breadth-depth-restructure.md)
(complete) restructured which CI lane runs what; it did not make tier 2's
fixtures build. [phase-356](archived/phase-356-test-evidence-and-measurement-trust.md)
owns "a number you cannot distinguish from a non-measurement" and already owns
#0968; this phase owns the lane and the duplication that breaks it.
[phase-391](phase-391-allocation-unification-and-tier-model.md) takes #1010,
which surfaced here but is an arena-sizing defect and belongs with the funnel.

## Landed before this doc existed

Merged to main: **#0980** (runner Zephyr SDK path), **#0981** (message-size bound
parity), **#0979**, **#0984** (gate lane visibility), **#0985**, **#0987**,
**#0990** (config-header single writer).

**#0998 and #0999 were fixed CONCURRENTLY by other sessions and are on main**
(`bed98e8ef` deletes the duplicated sertype copy rather than replacing it;
`c003ae608` resolves the targets list from the checkout). I had independently
written both and dropped mine — theirs landed first and stands. Recorded here
because it is this phase's own subject happening to this phase: two spellings of
one fix, noticed only because the branch would not merge.

Carried by the branch that opened this doc: **#1003** — ten entry templates
dropped the session name, so every image built through `nano_ros_add_node`
registered as `"node"` and XRCE client keys collided. Still unfixed on main at
the time of writing (`run_components(&__nros_entry_setup)`, no name), which is
what makes it the one piece of that branch worth carrying.

`just build native` went from failing at the first Rust fixture to EXIT=0;
tier-2 from three broken modules to six of seven green.

## Work items

* **W1 — [issue 1011](../issues/archived/1011-cyclonedds-zephyr-publisher-cxx11-aggregate.md),
  six `rust-*-cyclonedds` zephyr leaves do not compile.** `-std=c++11` on the
  Zephyr lane against a struct with default member initializers, which is not an
  aggregate before C++14. Unmasked by #0998's fix, not caused by it. This is the
  last module standing between the lane and green, so it is first.
* **W2 — [issue 1016](../issues/1016-tier2-lane-omits-zephyr-west-leaves.md),
  `lane=tier2` does not build the zephyr rust/c west leaves.** Their cells then
  report as failures that are really skips, with a summary line identical to a
  real result. This is the tier-2 half of #0828's class and it already caused one
  wrong reading in #0968. Two fixes are wanted, not one: make the lane build what
  it runs, AND make an unbuilt cell distinguishable from a failed one at the
  summary level — the second is what survives a regression of the first.
* **W3 — [issue 1017](../issues/archived/1017-entry-template-session-name-ungated.md), the
  #1003 class has no gate.** The defect lived two months with a correct sibling
  producer beside it. Note the trap recorded in the issue: the arity carrying the
  session name differs per board, so the obvious textual check would become a
  THIRD copy of the overload sets — the very failure being guarded against.
* **W4 — the nuttx module of `lane=tier2` has never completed on this host.**
  #0999's root fix should unblock it and that is UNVERIFIED. Run it; if it fails,
  it fails for a new reason worth its own issue.

## Acceptance

* `just build-test-fixtures lane=tier2` completes with every module OK.
* A tier-2 run cannot report a count that conflates never-built with failed.
* W3 lands a gate, or records a decision not to with the reason — an open
  acceptance box that is deliberately open is fine; one nobody looked at is not.

## What this phase deliberately does NOT do

It does not chase the runtime failures the lane then reveals. Those go to
[phase-414](phase-414-rtos-runtime-correctness.md) (RTOS runtime correctness) and
[phase-391](phase-391-allocation-unification-and-tier-model.md) (#1010). The
distinction is worth keeping: this phase is about whether the lane can produce a
verdict at all, not about whether the verdict is green.
