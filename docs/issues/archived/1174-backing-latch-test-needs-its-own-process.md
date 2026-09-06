---
id: 1174
title: "`the_static_is_handed_out_once_and_only_once` shares a process with ~367 tests that open executors, and the latch it asserts about is single-shot"
status: wontfix
type: bug
area: testing, core
severity: medium
found: 2026-09-06
resolved: 2026-09-06
related: [0417, 1036, 1168, 1183, 1186]
---

> **DUPLICATE of issues 1183/1186 — retracted 2026-09-07.** I diagnosed and
> fixed this independently while `605d667eb fix(#1183, #1186): the backing latch
> test needs a virgin process` was landing on main. Same defect, same
> conclusion (two cargo invocations are two processes), reached from the same
> `boot_report` precedent. 1183/1186 have the numbers.
>
> **Their fix is the better one and is what ships.** Mine `--skip`ped the test
> from the big run and re-ran it `--exact`; theirs marks it `#[ignore]`, which
> documents the requirement at the test instead of in the recipe and cannot be
> undone by someone adding a lane that forgets the skip. Theirs also asserts the
> dedicated invocation actually RAN the test — a "0 tests" run is the
> pass-but-skip shape, and my version would have reported green on it.
>
> Everything below is kept because the diagnosis stands, including the one point
> their commit does not make: a `Mutex` guard — the fix `SimTimeGuard` and
> `TypedProbeGuard` use for this crate's other process-global statics — cannot
> work here, because ordering access to a single-shot latch does not un-consume
> it.

# A single-shot latch cannot be shared, and a lock cannot fix it

`check-node-std-tests` red inside `just ci gate`:

```
executor::backing::tests::the_static_is_handed_out_once_and_only_once --- FAILED
panicked at packages/core/nros-node/src/executor/backing.rs:213:
an over-large request must not claim the latch
```

The message accuses `take`. `take` is fine. `executor::backing::TAKEN` is
process-scoped and **single-shot**, and anything that opens an executor claims
it — `Executor::open` -> `default_backing` -> `backing::take`. The test runs in
the same binary as ~367 others, libtest runs them concurrently, and the loser is
whoever gets there second.

| | |
| --- | --- |
| `-- --exact <the test>` | passes |
| the full binary | **fails 3 of 3** |

## Why the crate's existing fix does not apply

`nros-node` already has this class twice, and both are fixed with a `Mutex`
guard: `SimTimeGuard`, and `TypedProbeGuard` from issue 0417's
*"the two typed-probe tests share four process-global statics and race"*.

A lock is the right tool there and the wrong one here. Those statics are
**re-settable** — a guard orders access and each test re-initialises what it
needs. `TAKEN` is not: once consumed it stays consumed for the life of the
process, so serialising access does not un-consume it. Whichever test holds the
lock second still finds the latch gone.

The answer this needs is already in the same recipe, three lines down.
`boot_report`'s record is *"a process-global static keeping only the FIRST
allocation failure"*, and the fix there was a second cargo invocation, with the
comment **"Two cargo invocations are two processes."** Same fact, same answer.

## Not introduced here

`packages/core/nros-node` is byte-identical between this branch and
`origin/main` — the test and its lane both come from the rebase span, so this
fails on main:

* `b60cb8460 feat(phase-392 W6): the executor backing was on the HEAP, not the
  stack` landed the latch and this test.
* a later commit added tests that open an executor, and the ordering went from
  lucky to lost.

The test's own doc comment shows the gap. It reasoned carefully about *a second
copy of this test* observing a consumed latch — and stopped there, never
considering that every other executor-opening test in the binary is the same
competitor.

## Fixed

**The lane gives it a process.** `node-std-tests` now `--skip`s it from the
367-test run and runs it `--exact` in its own cargo invocation.

**The test says which thing went wrong.** A `!is_taken()` assertion runs FIRST,
before anything touches `take`, and its message names the scheduling rather than
the module:

> the backing latch was ALREADY consumed before this test ran, so nothing below
> can be concluded ... This test needs its OWN process — see `node-std-tests` in
> just/check/lanes.just

That matters more than the lane change. Without it the next person who copies
this test, or adds a lane that runs it un-split, gets "an over-large request
must not claim the latch" and goes reading `take`.

Verified both directions: the lane passes, and the full binary still fails —
with the new message instead of the misleading one.
