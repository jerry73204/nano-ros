---
id: 1158
title: "The tier-2 lane has produced no runtime verdict in six consecutive scheduled runs, and no pre-merge event can produce one"
status: open
type: bug
area: ci, testing
severity: high
related: [0968, 1016, 1029, 1043, 1075, 1098, 1104, 1114, 1127, RFC-0061]
found: 2026-09-06
---

# The lane whose job is the runtime verdict has not delivered one since 2026-09-01

## Measured

`run-matrix.yml` is the **only** automated path to tier 2's runtime cells.
Its last eight runs:

| when | event | outcome | died at |
| --- | --- | --- | --- |
| 09-06 06:25 | schedule | `pending` | never started |
| 09-06 02:44 | dispatch | `queued` | never started (5 h+) |
| 09-06 00:11 | dispatch | failure | `just build tier2` |
| 09-05 06:24 | schedule | failure | `just setup tier2` |
| 09-04 06:28 | schedule | failure | `just setup tier2` |
| 09-03 06:27 | schedule | failure | `Verify this runner's labels are true` |
| 09-02 06:33 | schedule | failure | `Verify this runner's labels are true` |
| 09-01 06:29 | schedule | failure | `Verify this runner's labels are true` |

**Six consecutive failures, four distinct causes, and not one of them is a test
result.** Every failure is upstream of the cells: runner labels, provisioning,
the build. Two runs are now stacked unstarted.

## Why no other lane covers it

| lane | event | depth |
| --- | --- | --- |
| `run-matrix.yml` | `schedule 0 6 * * *`, dispatch | `just ci matrix` — build **+ tests** |
| `queue.yml` | `merge_group` | `just ci matrix build` — **no tests** |
| `build-wide.yml` | dispatch only | `just ci matrix build` — **no tests** |
| `nightly.yml` | schedule, dispatch | `ci-matrix-nightly` (a different, wider lane) |

**No `pull_request` event runs any matrix lane**, and the merge queue runs only
the BUILD depth. So a runtime regression cannot be caught before merge by
design, and the one lane that would catch it after merge has been failing for
six days.

`post-submit.yml` and `gate.yml` mention `ci-matrix` in COMMENTS only — neither
invokes it. A grep that counts hits says otherwise; read the `run:` lines.

## The structural half — it is not just bad luck

One self-hosted runner exists (`newslab-175-26-server-nros-qemu-nros-sdk`), and
`run-matrix.yml` declares `concurrency: group: run-matrix,
cancel-in-progress: false`. The merge queue's `gate` jobs run on the SAME
runner. Observed while writing this: `busy=true`, occupied by `2x gate`, with
both matrix runs queued behind it.

So the lane is starved by the very PRs it exists to validate: every merge
lengthens the wait, and a busy day can starve it indefinitely. `cancel-in-progress:
false` then stacks the scheduled runs rather than superseding them, which is
correct for a verdict lane and makes the backlog visible rather than hidden.

## Why this matters more than one red lane

CLAUDE.md states the rule this violates:

> A red CI lane answers one of two questions and they look identical — the lane
> RAN and the code is broken (a verdict), or it never ran (no verdict). A
> uniformly-red lane has NO signal capacity.

Issue 0968 recorded ~12 tier-2 runtime failures as unreproduced and named the
cause: `post-submit`'s tier-2 job has never run (interlocked on
`vars.NROS_SELF_HOSTED_READY`, unset) and `host-tests` was red for 20
consecutive runs. That diagnosis is now six days older and unchanged.

**Four real regressions rode in behind this during 2026-09-05/06 alone** — 1075
(a link failure), 1098 (a compile poison with no migrated consumer), 1104 (a red
unit test), 1114 (a gate demanding an artifact no lane builds). Each was found
by hand, each hid the next, and each would have been a tier-2 verdict on any day
the lane had produced one.

## What would close this

Not "fix today's failure" — that has been done four times and produced no
verdict. The lane needs to reach the cells ONCE and then keep reaching them:

1. **A verdict, once.** Get one green-or-red RUN of `just ci matrix` on `main`,
   so 0968's ~12 failures become reproduced-or-retracted rather than folklore.
2. **Contention.** The verdict lane and the merge queue share one runner. Either
   give the lane its own runner/label, or schedule it where the queue is idle,
   or accept and DOCUMENT that it yields — the current state is neither chosen
   nor written down.
3. **Distinguish "did not run" from "ran and failed"** at the summary level, the
   way issue 1043 did for `check-submodule-pins`. Six runs that all say
   "failure" hide four different stories; a lane that never reached its cells
   should not report the same word as one that did.

## Not covered

* Whether the runner-label failures (09-01..09-03) share a cause with the
  provisioning failures (09-04/05). Not investigated — they are simply
  different steps.
* `vars.NROS_SELF_HOSTED_READY`, still unset, which is what keeps
  `post-submit`'s tier-2 job from ever running (0968). Setting it is a decision
  about whether that runner is trusted for merge-gating, not a fix to make here.
* Issue 1127 (17 declared interop cells, no sweep runs any) is the same class one
  lane over and is filed separately.
