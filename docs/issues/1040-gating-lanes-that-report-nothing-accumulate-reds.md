---
id: 1040
title: "`check-api-parity` runs in NO workflow and `check-build` only on
  schedule/dispatch, so reds there accumulate until someone happens to run the
  full tier locally — five landed on main in one day"
status: open
type: task
area: [ci, process]
related: [1035, 1021, 0952, 1030, 1071, 1072]
---

## The measurement

**2026-09-04. Five reds were sitting on `main`, found in one day, all by the
same mechanism: a person ran `just ci gate` locally.**

| red | landed with | lane that sees it |
| --- | --- | --- |
| `error: this function has too many arguments (8/7)` (`nros-cli-core`) | phase-397 depend ladder + phase-413 W2 insertion | `check::build` |
| `check-api-parity`: `cpp:declared_depth` unclassified | phase-403 step 2 (`3d852da49`) | `check::api-parity` |
| `error: items after a test module` (`board_facts.rs`) | `e453399de` (phase-400 W1) | `check::build` |
| `check-api-parity`: 3 release-jitter items unclassified | `55324cc33`, landed 02:37 the same day | `check::api-parity` |
| zenoh-pico 1.8.0 does not build on NuttX ([#1035](archived/1035-zenoh-pico-1-8-0-true-in-preprocessor-breaks-nuttx.md)) | PR #299 | **no lane at all** |

Plus a sixth of a different kind, in the same window: `other.json` carried
`cpp:declared_depth` **twice** — two sessions classified one symbol
independently, git merged both (different regions, no textual conflict), and
JSON parsers keep the last silently. Now gated in `api-parity.py`.

## The two facts, verified in `.github/workflows/`

1. **`check-api-parity` runs in NO workflow.** `grep -rl api-parity
   .github/workflows/` returns nothing. Not on `pull_request`, not on
   `merge_group`, not nightly, not on dispatch. It exists only as a local
   recipe.
   *(2026-09-05: `2019640f7` changed the answer to that grep and did not change
   the fact — see "Why the 2026-09-04 fix did not fix it" below.)*
2. **`check-build` runs on `schedule` / `workflow_dispatch` only**
   (`gate.yml:740`). That is deliberate and correct — CLAUDE.md records why:
   it needs generated bindings and prebuilt `.compile-ok` that no CI job
   builds, so it was red for every pull request for a day when it was
   required.

So the *required* `CI` context is `check-fast` + `test-unit` + `check-cli-tests`,
and everything the compile tier and the parity ledger see is invisible to it.

## Why this is not "just run the tier"

`just ci gate` being stronger than the merge gate is the DESIGN, and it is a good
one: the queue stays cheap and always-satisfiable, and the person making a change
catches compile-tier breakage before the queue does. CLAUDE.md says so directly.

The gap is narrower than "CI is too weak": **nothing REPORTS on these lanes
between local runs.** A red there is not blocked, not announced, and not visible
in any dashboard — so its cost is paid by whoever next runs the full tier, who
then finds five unrelated failures with five unrelated owners and has to fix or
route all of them before their own work can be gated. That is exactly what
happened here, and it is a poor trade: the person who broke it pays nothing, the
person who runs the tier pays everything, and the delay means the author has
moved on.

A red lane also loses signal capacity, which CLAUDE.md already records for the
nightly (issue 0878): once a lane is habitually red, a NEW regression in it looks
exactly like yesterday's.

## What would fix it, in rough order of cost

1. **Report, do not gate.** Add `check-api-parity` (and `check-build`, which
   already has a nightly step) to a scheduled run whose only job is to
   ANNOUNCE — `just nightly-triage` already exists for classifying nightly
   failures, and `queue-triage` for merge-queue ejections. Neither covers this.
   Cheapest, and it does not risk deadlocking the queue the way a required
   check with unbuildable prerequisites did.
2. **Make `check-api-parity` affordable enough to gate.** It re-extracts our
   surface with clang + nightly rustdoc, which is why it is not on the fast
   line. Worth measuring whether the LEDGER half alone (the part that catches
   an unclassified row) can run buildless — that is the half that caught four
   of the five.
3. **A NuttX compile on some lane.** #1035 is the one red above that no lane
   would have caught at any frequency, and it is the most expensive kind: a
   platform that does not build at all, merged and unnoticed. It need not be a
   full fixture build — `cargo build -p zpico-sys --target armv7a-nuttx-eabihf`
   would have caught both of its breaks.

## Not proposed

Making these required checks. That is what put `check-build` on the merge group
and made it red for every PR for a day, because it resolves artifacts no CI job
builds — the failure mode `check-lane-contracts` now gates against. The problem
here is reporting, not enforcement.

---

## The survey (2026-09-05) — gate → workflow event

Measured, not reasoned: `python3 scripts/check-default-gates-run-somewhere.py
--survey` prints the table, derived from `.github/workflows/` and from
`check-gate-lists.py --list`, which is where lane membership lives since issue
1072 deleted the authored `fast-serial:` registry.

| bucket | count | who |
| --- | --- | --- |
| merge-gating (`pull_request` and/or `merge_group`) | 222 | the derived fast lane (218) + `cli-tests`, `compile-smoke`, `decoupling`, `submodule-commits-reachable` |
| post-submit (`push`) but not gating | 22 | the `build-serial` tier (20 of 21) + `api-parity` + `dep-chain` — reached by `just ci tier1` in `host-tests.yml` and by `post-submit.yml` |
| `schedule`/`workflow_dispatch` only | 1 | `no-std` |
| **NO event at all** | **0** | — |

So the headline claim above is now stale in one direction and was never the
whole story in the other. `check-api-parity` IS named in a workflow (gate.yml,
`schedule`/`workflow_dispatch`, added by `2019640f7`) — and had still never
produced a verdict. See below.

Two further facts the survey turned up, both outside the table because they are
about lane MEMBERSHIP rather than events:

* The gates that reach no event at all are the ones in NEITHER lane, and issue
  1071/1072 already enumerated them in `.config/gate-lane-exempt.txt` (ten
  entries marked `in no lane and no caller found`). Nothing is duplicated here.
* `just check sdk-index` reaches no event by NAME, but gate.yml's `sdk-index`
  job runs the same three scripts directly, so the checks do run.

## Why the 2026-09-04 fix did not fix it

`2019640f7` appended `just check api-parity` to gate.yml's existing
`just check build + no_std` step. A GitHub `run:` block is `bash -e`, so the
three shared a fate — and `just check build` has been red on that lane since
2026-09-01 (`workspace-all`, `workspace-features`; run 33937933817 and the four
before it). On every scheduled run since api-parity was wired there it
**never executed**. The lane reported one red for three gates, two of which had
no verdict at all.

That is issue 0952's withdrawal class one level down: `ci::gate` prints what a
red step withdrew precisely because a count of reds is not a coverage report,
and in YAML there is no recipe to print it. It is also the worse kind of
not-fixed, because `grep -rl api-parity .github/workflows/` now answers yes.

## What landed

1. **gate.yml: one gate per step.** `check build` and `check no-std` are now two
   steps, each with the same `schedule`/`workflow_dispatch` + `!cancelled()`
   guard — `!cancelled()` is what lets a step run after an earlier failure.
2. **post-submit.yml: `api-parity` as its own job**, on `push` to main +
   `workflow_dispatch`.
   * **Cost, measured:** `just check api-parity` is **28.6 s** wall on a 24-core
     host, and there is no warm/cold split — `extract_rust.rustdoc_json` builds
     into a FRESH temp target dir every run. It needs no fixture, no SDK, no
     QEMU, no cross toolchain and no ROS install: the ROS 2 side is the
     committed surface under `docs/reference/api-surface/`, and the C/C++ side
     parses the committed `nros_cpp_config_generated_nuttx.h`.
   * **One prerequisite the wiring had missed:** `clang` is NOT in
     `ci/docker/ci-base/Dockerfile` (it installs gcc cross toolchains and a
     pinned clang-format, never clang), and `extract_cxx` shells out to
     `clang`/`clang++` for `-Xclang -ast-dump=json`. The job installs it from
     the index SSoT (`scripts/sdk/prereq-packages.py --manager apt clang`), the
     same way nightly.yml does for its bindgen cells. Until this, the step could
     not have run even had it been reached.
   * **Why not `pull_request`/`merge_group`:** cost says it could — it is
     cheaper than `dep-chain` (158 s) already on this lane and than `test-unit`
     (~3.5 min) which the merge group affords. STATE says not yet: the gate is
     RED on main today (`rust:Executor::set_min_stack_headroom_bytes` has no
     ledger row), and a lane cannot start gating pull requests while it is
     already red. Promotion is a follow-up once that row lands.
   * **Its own job, not a step beside `dep-chain`** — for the reason this issue
     is about.
3. **`check-default-gates-run-somewhere` now carries the class**, in two rules:
   R1 every gate in `just check`'s lanes must reach some workflow event
   (widened from the four names on `default:`), and R2 no placement may be
   sequenced behind another gate in the same `run:` block. Verified against the
   UNFIXED `gate.yml`: two findings (`api-parity`, `no-std`), no false
   positives; green against the fixed one. Both allowlists
   (`NO_PLACEMENT_NEEDED`, `SHADOWED_PLACEMENT_OK`) are EMPTY by measurement.
4. **`check-lane-contracts` no longer looks away from report-only lanes** — see
   issue 1030's follow-up.

## Still open

* ~~`rust:Executor::set_min_stack_headroom_bytes` is unledgered, so
  `api-parity` is red on main.~~ CLEARED 2026-09-05: PR #474 landed the row (in
  `exec.json` — the ledger is sharded by TOPIC across all three languages;
  there is no `rust.json`) and `just check api-parity` is green.

  **Promotion to `merge_group` is now unblocked and deliberately NOT taken.**
  Cost allows it — 28.6 s, cheaper than `dep-chain` at 158 s which already
  gates. What is missing is a track record: one green local run is not evidence
  that a gate is stable enough to freeze the repo on. Promote it after it has
  been green on post-submit for a cycle, and remember that making it gating
  means adding it to the `CI` aggregator's `needs:`, never to the required-check
  set directly.

  The ROW ITSELF is still wrong about one thing, and that correction is owed:
  it claims the item was "newly EXTRACTABLE" after phase-425 moved an `alloc`
  gate off `nros_platform_api::task`. Measured 2026-09-05 on a tree WITHOUT
  that change and the extractor reports the item anyway, so the causal story is
  unverified. The row is right that it needed classifying; it is wrong about
  why it appeared.
* `check build` is red on the scheduled lane (`workspace-all`,
  `workspace-features`). Now that the steps are split, `no-std` reports
  independently of it.
* #1035 (a platform that does not build at all) still has no lane. Unchanged by
  this work.
