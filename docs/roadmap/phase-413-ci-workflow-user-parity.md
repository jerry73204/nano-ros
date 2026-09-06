# Phase 413 — make CI run the process a user runs

**Status (2026-09-06). W1, W4, W5 and W6 done and verified; W2 substantially
advanced; W3 done; W7 open.** Every claim here was re-derived from the tree on
2026-09-06 — three items were already complete and said otherwise, which is the
same stale-status class this phase's own audit is about.

**Status (2026-09-03). Opened from the workflow audit in issue 0996.** The
workflows are meant to read as a transcript of a user session: `just setup
<scope>`, then one command a developer can type (phase-411 W4). Six of eleven
still hand-roll the middle. The audit measured what that costs, and the cost is
not tidiness — it is a lane that passes in the merge queue and fails after the
merge, on the same commit, running the same recipe.

## The finding that orders this phase

`just ci matrix build` dispatches to `_matrix-build`, whose body is `l3`. So
`queue.yml` (`just ci l3`, on `merge_group`) and `build-wide.yml` (`just ci
matrix build`, on `push` to main) run the **same recipe on the same runner
labels**. Their verdicts on the same four commits, minutes apart:

| commit | queue.yml | build-wide.yml |
| --- | --- | --- |
| `36c306405` | success 16:25:49 | **failure** 16:26:23 |
| `da0344af1` | success 17:02:18 | **failure** 16:43:26 |
| `ad5a8ecf9` | success 17:06:55 | **failure** 17:22:19 |
| `d2a8955c5` | success 01:23:54 | **failure** 17:27:51 |

The code is identical. The difference is that `queue.yml` carries four
hand-rolled steps — `setup-cli`, `setup-launch-resolve`, `nros sync` on the rust
workspace, and a loop syncing the three leaves L3 builds — which `build-wide.yml`
does not, because it trusts `just setup tier2` and the build verb.

**The boilerplate is load-bearing.** It is not clutter around a working system;
it is what hides issue 0992 from the queue while the post-merge lane burns. That
is the general shape this phase is about: every hand-rolled step in a workflow is
a claim that the toolchain cannot do the thing itself, and each one that is true
is a defect nobody is looking at.

## The state to fix from

| workflow | last 5 conclusions | shape |
| --- | --- | --- |
| `gate.yml` | green (the required `CI`) | — |
| `post-submit.yml` | success ×4, cancelled | — |
| `build-wide.yml` | failure ×4 | `just setup tier2` + one command |
| `run-matrix.yml` | failure ×2 | `just setup tier2` + one command |
| `nightly.yml` | failure ×4 | mixed |
| `host-tests.yml` | failure ×4, cancelled | 5 provisioning steps hand-rolled |

Four of six verification lanes carry no signal. CLAUDE.md names the class: *"A
uniformly-red lane has NO signal capacity: a regression landing in it looks
exactly like yesterday's failure."* Until a lane is green once, nothing it says
counts, so **W2 gates most of the rest of this phase** — there is no way to prove
a conversion did not break something in a lane that was already red.

---

## W1 — collapse the duplicated `l3` lane — DONE (verified 2026-09-06)

**Landed.** Issue 0992 is resolved, and the lane now runs ONCE per change:
`build-wide.yml` is `workflow_dispatch:` only — the "reduce it to the
scheduled/dispatch entry point it also serves" option below — while `queue.yml`
runs `just ci matrix build` on `merge_group`, which is where a gate belongs. The
four hand-rolled sync steps are gone (retired by W1b).

*The stated acceptance is wrong and was never the test.* `grep -c 'nros sync'
.github/workflows/queue.yml` returns **2**, and both hits are COMMENTS describing
the steps that were retired. A grep that counts prose cannot answer "does this
lane still sync by hand"; the steps themselves are what to look at, and they are
gone.

Blocked on issue 0992 (PR #208) landing; that is what lets `build-wide` provision
through the verb instead of by hand.

Then: delete `queue.yml`'s four sync steps, and **decide whether this lane runs
in the queue or after it — once, not both.** Today the same self-hosted lane is
paid for twice per change, and a gate that already passed in the queue cannot
fail differently after the merge unless the two spellings have diverged. Which is
exactly what happened.

Recommendation: keep it on `merge_group` (a gate belongs before the merge, not
after) and retire `build-wide.yml`, or reduce it to the scheduled/dispatch entry
point it also serves. Whichever way, one spelling.

Acceptance: `grep -c 'nros sync' .github/workflows/queue.yml` is 0; exactly one
workflow runs the l3 lane; a full change cycle shows the lane running once.

## W2 — get each red lane green, in dependency order

Not a conversion task — a debugging one, and it comes first because no
conversion can be verified in a lane that is already red.

### Progress (2026-09-06), each item measured against the lane rather than the doc

* **`nuttx` — DONE.** `9de62f253` (09-04) made `just nuttx setup` provision the
  RISC-V board its `build-all` builds. nuttx is the only module with two boards
  under one setup/build-all pair, so the class has no other instance.
* **`freertos` / `threadx_linux` — DONE, by a route this doc did not predict.**
  Both cells now report `Real failures: 1 / 1` with **8 tests actually running**
  and nothing skipped for an unmet precondition — recovered from "0/0, all 9
  skipped". The fix was not the two image changes prescribed below: it was
  `575dc4372` (09-05, W3's own work) adding `nros setup --system --sudo` to
  nightly's `platform` job, which installs the declared closure at run time.
  A PR that baked the package into the CI image was closed as redundant — a
  second mechanism installing one package, where the runtime path is better on
  this campaign's own terms because it takes the name from the index instead of
  freezing a copy into an image. What remains in each cell is ONE genuine
  failure (`test_rtos_pubsub_e2e … Freertos/Rust`), which is signal restored.
* **`threadx_riscv64` — half of it was still open.** The reported manifest-parse
  failure was fixed, but only the central-table half: a leaf that also patches a
  message crate fails the same way one path deeper, because only `nros sync`
  materialises `generated/`. This lane's six rust roles build through
  Corrosion/CMake, the one seam that never reaches `fixtures-build.sh`'s
  per-row presync. Fixed with `nros_ensure_leaf_synced`.
* **`run-matrix` — diagnosed, and the top failure was self-inflicted.**
  `nros codegen resolve-deps` was refused by phase-431 W1's workspace guard: the
  zephyr lane builds into a west workspace that on the runner sits under a
  DIFFERENT nano-ros tree, so a correct, freshly built binary was rejected. The
  build-tool verbs skip that check now. **This does not make the lane green** —
  `run-matrix` has failed every scheduled run since at least 09-01 and the guard
  landed 09-05 22:00, so an older cause is now visible underneath.
* **`host-tests` — its named step is green.** *Build workspace fixtures* passes;
  the two breaks that stopped it (issues 1162, 1136) are fixed. The lane is red
  one step later at `just ci tier1`, on ten `E0599`s in `nros-c`'s no-alloc
  build — filed as issue 1169, with the measurement that this build has **never**
  compiled, so restoring it is an addition rather than a repair.
* **`build-wide`** — now `workflow_dispatch:` only (see W1), so it is no longer
  a per-change lane at all.

1. **`host-tests`** — dies in *Build workspace fixtures*. This is the honest
   failure: the step `exit 1`s instead of laundering into a skip, which is the
   policy working. Fix the fixture build.
2. **`run-matrix`** — tier-2 run depth, one step, exit 1 with no further
   detail in the log summary. Needs a real diagnosis before it can be trusted.
3. **`nightly`** — failure ×4. `just nightly-triage` classifies by which STEP
   failed and flags cells red across a window; use it rather than reading runs.

   **Triaged 2026-09-04 against run `33831829557`** (dispatched deliberately —
   the scheduled run predated #245 and its `nuttx` cell was reproducing a fixed
   bug, which is the issue-0859 trap). Six failing jobs, **zero product
   regressions**; details in issue 1038. Three are one class — the job builds a
   lane whose prerequisites its own `setup` never installs:

   | cell | first thing it reached | owner |
   | --- | --- | --- |
   | `nuttx` | `riscv-none-elf-gcc not found (run: nros setup qemu-riscv-nuttx)` | this wave |
   | `threadx_riscv64` | cargo manifest-parse: missing `include` of the generated `nros-patch.toml` (issue 0463) | this wave |
   | `esp32` | RED is issue 1025 (#303); the missing `libglib2-dev`/`libpixman-dev`/`libgcrypt-dev` are a separate COVERAGE hole | **W3** |

   `esp32` needed a correction after the fact: its RED is issue 1025 (the flash
   packer looking in a directory the build stopped using, PR #303), NOT the
   packages — `just esp32 setup` tolerates that failure by design, so it costs
   the e2e SKIP rather than the build. The package half is still W3's
   conversion unapplied rather than a missing mechanism: the
   index declares all three as `[prereq.*]` AND in `[tool.esp32-qemu]`'s own
   `system = [...]`, so `nros setup --system` already resolves them. Checked
   against the gate as IMPLEMENTED (`check-workflow-indexed-apt.py`), not as
   specified: it matches `apt(-get)? install` lines naming a `[prereq.*].apt`
   package, which is duplication and is the right rule — but this site installs
   nothing at all, so the gate is silent here by construction. Whether W3's
   acceptance should also cover OMISSION is a decision for this wave's owner.

   The other three cells are not conversions: `freertos` and `threadx_linux`
   report `Real failures: 0 / 0` with all 9 e2e cases skipped on one shared
   precondition (`_check-skip-budget` correctly refusing to call that a pass),
   and `tier 2 nightly` is runner state (three fixture families missing
   `.inputsig`).

   The `freertos`/`threadx_linux` cause is **CONFIRMED** (2026-09-04): the CI
   image carries NO zenoh packages — `FROM ros:humble-ros-base` plus an apt list
   whose only ROS entry is `example-interfaces`; a built image has no
   `rmw_zenoh_cpp` prefix, no `rmw_zenohd`, no `libzenohc*`, and
   `rmw-fastrtps-cpp` as its sole RMW. No router, so
   `zenohd_unavailable_reason()` fails `require_e2e()` and all 9 cases skip.
   **Fixing it takes TWO changes**: install `ros-humble-rmw-zenoh-cpp`, AND
   re-capture the image's static `AMENT_PREFIX_PATH`/`LD_LIBRARY_PATH` `ENV`
   block, which will not name `<prefix>/opt/zenoh_cpp_vendor/lib` — a router
   paired with the wrong `libzenohc.so` SEGVs mid-startup (issue 0774). The
   Dockerfile's own comment already says to re-run its capture command after
   changing the ROS package set. Detail in issue 1038.
4. **`build-wide`** — expected to go green with W1/0992; if it does not, its
   remaining failure is now visible rather than buried under the include
   preflight.

Acceptance: each lane green at least once, and the date recorded here. A lane
that cannot be made green is a finding, not a skip — file it.

## W3 — a gate: no workflow installs what the index already declares

`nros setup --system` resolves the `[prereq.*]` closure for the detected package
manager (RFC-0062 / phase-327). Measured on a dev host: 38 present, 5 missing, 3
unprobed. The mechanism works. Yet:

| site | installs | already declared |
| --- | --- | --- |
| `docs.yml` | `doxygen`, `graphviz` | **both** |
| `nightly.yml` | `clang`, `libclang-dev` | **both** |
| `gate.yml` colcon-parity | ROS apt repo, `ros-humble-ros-base`, `colcon`, curl-installs `just` | `colcon` yes; ROS no |

`[prereq.doxygen]`'s own `why` field reads *"found undeclared by
check-sysdep-remedies"* — so a gate already exists for the index-side gap. **The
reverse is unchecked**: nothing stops a workflow apt-installing what the index
knows. That asymmetry is why these three survived.

Two halves:

* **The gate.** A workflow `run:` block may not `apt-get install` / `pip install`
  a package name the index declares. Self-testing, on the fast line. It must skip
  comment lines and heredoc bodies, for the reason `check-workflow-repo-env`
  documents: a gate that cannot tell a command from a sentence about a command is
  worse than no gate.
* **The conversions.** `docs.yml` and `nightly.yml`'s clang step become
  `nros setup --system --sudo` (or a narrower `--tool`). colcon-parity becomes
  `container: ghcr.io/newslabntu/nano-ros-ci:humble` — `images.yml` builds that
  image and describes it as *"ROS 2 Humble + host tools, the base `container:`
  for the check / …"*, and **only `nightly.yml` uses it today**. A ROS desktop
  stack is installed by apt on every colcon-parity run for want of one line.

Acceptance: the gate reproduces each of the three sites before the fix and passes
after; no workflow apt-installs an indexed package.

## W4 — decide what the required check promises on a pull request — DONE (verified 2026-09-06)

**Decided: the DOC moved, not the trigger list.** `test-unit`'s `if:` still
names `merge_group`, `schedule` and `workflow_dispatch`, and CLAUDE.md now says
so outright — *"`test-unit` is NOT in it"* — with the reasoning beside it: it
costs a measured ~3.5 min per BATCH there instead of per PR push, nothing broken
lands because the queue runs it before merging, and an ejection comment is how
you hear about it. A contributor can predict what green means, which is what the
item was for.



`test-unit` does not run on `pull_request` — only on `merge_group`, `schedule`
and `workflow_dispatch`. So a PR shows a green required `CI` having never run a
unit test; they run first in the merge queue.

Nothing broken lands, because the queue is the real gate and `queue-notify.yml`
exists to comment on ejections. But CLAUDE.md states the required context *is*
`check-fast` + `test-unit` + `check-cli-tests`, which is true only in the queue.
Either the trigger list gains `pull_request` (cost: unit tests on every push to
every PR) or the doc stops claiming it. **Pick one and write down why** — the
value of this line is that a contributor can predict what green means.

Acceptance: the doc and the trigger list agree, and the reasoning is recorded
next to whichever moved.

## W5 — one scope vocabulary — DONE (verified 2026-09-06)

**All six lanes converted**, and the two loose ends with them:

| workflow | provisioning today |
| --- | --- |
| `build-wide`, `run-matrix` | `just setup tier2` |
| `nightly` (matrix) | `just setup tier2-nightly` |
| `nightly` (platform) | `just setup ${{ matrix.plat }}` — the dispatcher form |
| `queue` | `just setup tier2` (the four hand-rolled steps are gone) |
| `host-tests` | `just setup native` |

The *"22-line Build nros CLI block repeated verbatim in four jobs"* is now the
composite action `./.github/actions/setup-nros-cli`, used by all six sites — the
step NAME survives, which is what makes a grep still find "four copies".

`just ci provision-zenohd` is absorbed: `just native setup` calls it, so it is
reachable from `just setup native` rather than being a verb only a workflow knew.

phase-411 W4: a CI job is `just setup <scope>` then one command a developer can
type. Three lanes do this; three do not.

| workflow | provisioning | work |
| --- | --- | --- |
| `build-wide` | `just setup tier2` | `just ci matrix build` |
| `run-matrix` | `just setup tier2` | `just ci matrix` |
| `nightly` (matrix) | `just setup tier2-nightly` | `just ci matrix-nightly` |
| `host-tests` | `nros setup --source` ×6, `--tool` ×3, `provision-zenohd`, `setup-launch-resolve` | `just native build-fixture-rust-core`, `just native build-workspace-fixtures`, `just ci tier1` |
| `queue` | 4 hand-rolled steps (W1) | `just ci l3` |
| `nightly` (platform) | `just <plat> setup` + 6 more steps | `just <plat> test` |

`nightly.yml` also repeats a 22-line *"Build nros CLI from packages/cli/"* block
verbatim in four jobs.

The conversion is only safe once the target verbs actually provision what the
lane needs — which is what 0992's `_setup-common` starts and what W3 finishes for
OS packages. Two provisioning verbs are still unreachable from `just setup`:
`just ci provision-zenohd` and (before 0992) `setup-launch-resolve`. Absorb the
first.

Acceptance: every job body is `just setup <scope>` plus one command; the repeated
CLI-build block exists once; `just ci provision-zenohd` is reachable from
`just setup`.

## W6 — ANSWERED: `package.xml` as the dependency SSoT (rosdep parity)

**This one is not an implementation task and must not be started as one.** It
changes what a `package.xml` means in this repo, so it needs an RFC first.

The intent is the rosdep experience: dependencies are written in `package.xml`,
the tool scans the packages, and installs accordingly. The two halves do not
meet today:

* **406 tracked `package.xml` files.** Their entire dependency vocabulary is ROS
  message and build-tool keys — `std_msgs` (177), `example_interfaces` (106),
  `ament_cmake` (19), `rosidl_default_*` (27), plus workspace-local node
  packages. **Not one names a system dependency.**
* Those declarations are consumed **only by codegen** — which msg packages to
  generate. No provisioning path reads them.
* Provisioning reads `nros-sdk-index.toml`: `[prereq.*]`, `[rust.*]`,
  `[python.*]`, `[source.*]`, `[tool.*]`. Hand-maintained and repo-global.

So a `package.xml` cannot express "this package needs libssl-dev", and there is
no `nros setup <workspace>` that walks a workspace's manifests and resolves their
closure. The nearest thing, `nros setup --build-sources`, provisions a
repo-global union from the index — not a per-workspace scan.

Questions the RFC has to answer, none of which have obvious answers:

1. **What is the key namespace?** rosdep keys are a curated global database
   (`libssl-dev` → per-distro package names). Do we adopt rosdep keys, our own
   `[prereq.*]` names, or accept both? A `package.xml` that only our tool can
   read is a divergence from ROS, which is the thing the analogy exists to avoid.
2. **Who owns the mapping?** Today `[prereq.*]` carries `apt`/`dnf`/`pacman`/
   `brew` plus a `check` probe. That is a rosdep rules file in TOML. Does the
   index stay the mapping and `package.xml` become the *declaration*, or do we
   consume the real rosdep database where present?
3. **What about the 406 existing files?** They declare message deps that must
   keep meaning what they mean to codegen. A second meaning for the same tag is
   how a file ends up with two readers that disagree.
4. **Scope resolution.** `nros setup <workspace>` implies a workspace scope for
   provisioning, which the phase-411 scope vocabulary does not currently have —
   its scopes are platforms and lanes.
5. **Does this subsume `--build-sources`?** If a workspace's manifests can state
   their own needs, the repo-global union is a fallback, not the SSoT.

Deliverable for W6 is **an RFC in `docs/design/`**, not code. Only once it is
`Draft` and reviewed does an implementation work item get opened.

### ANSWERED (2026-09-06) — [RFC-0062 amendment 4](../design/0062-unified-dependency-ssot.md), implemented by phase-435

The RFC this item required exists, and the five questions above have answers.
Recorded here because a work item that reads DESIGN REQUIRED after its design
landed is how a settled question gets reopened.

**The premise needed correcting first.** This item opened on *"406 package.xml
files … not one names a system dependency"* and read it as a gap. Measurement
says it mostly is not one: the bridge already exists — `run_workspace_scan`
resolves every `<depend>` against `[prereq.*]` and classifies by role — and
these packages have nothing to declare, being message and node packages whose
content depends on no system library. What was missing was not a bridge but the
other three axes.

1. **Key namespace** — `[prereq.*]` names, not rosdep keys. A ROS *package*
   dependency is the one case that resolves by derivation (`ros_package` →
   `ros-<distro>-<pkg>`), so the ROS-facing vocabulary is honoured where it
   actually appears.
2. **Who owns the mapping** — the index stays the mapping; `package.xml` is the
   declaration. The real rosdep database is not consulted: we have no oracle to
   validate a name against, so an unknown name must fail a gate rather than an
   `apt` invocation on a user's machine.
3. **The 406 existing files** — untouched. The new axes read `<build_type>` and
   the `<nano_ros>` export, both of which already existed; `<depend>` keeps
   exactly the meaning codegen gives it.
4. **Scope resolution** — unchanged. The board and rmw axes resolve from the
   manifest's own export tag, so a workspace does not need a new scope.
5. **`--build-sources`** — not subsumed. It is the repo's own build-stage union,
   which is a different question from what a consumer's workspace declares.

The answer in one line: **four axes, and `package.xml` is the only input** —
`<build_type>`, `<depend>`, and the board/rmw selections in the `<nano_ros>`
export. Never a `CMakeLists.txt` or a `Cargo.toml` entry: those are build-system
facts in a different vocabulary, and a resolver reading them inherits every
shape they take.

phase-435 implemented it — the `buildtool` role, the `[build_type.*]` axis,
`ros_package` derivation (retiring issue 1128's placeholder), `[board.zephyr]`,
and a gate that a non-family board alias names one board.

## W7 — DESIGN: scope the arena-budget walk (issue 1001)

Found while doing W1, and it is the fast line's own promise at stake.

`check fast` is BUILDLESS and SOURCE-FREE, ~23 s, and the `pre-push` hook runs
it alone. `check-action-client-arena-budget` walks the ENTIRE repository — its
call site passes `roots = [ROOT]`, and `PRUNE` covers neither cargo `target*/`
nor `zephyr-workspace/` nor `tmp/`.

The cost is **I/O, not CPU**, which is why it went unnoticed: the same
traversal is 0.6 s warm and over 60 s cold, and the whole gate is 3.4 s warm but
did not finish in 600 s on a host whose page cache another build had evicted.
Cold is exactly the case a contributor hits, because `check fast` is often the
first thing in a session to touch these trees.

**Two fixes were tried and are wrong; issue 1001 records both so they are not
repeated.** Moving it to the build tier is refused by `check-gate-visibility`
for a good reason (a gate no pull request runs rots — issue 0981), and it skips
cleanly with no artifacts, which is that gate's own test for belonging on the
fast line. Pruning non-`out` siblings under `build/<pkg>-<hash>/` looked
provably free and silently dropped 159 of 410 findings, because the predicate
also matches the top-level `build/` shared cargo group dirs from phase-340.

So this is **design work, not a patch**: scoping `roots` to discovered cargo
target dirs without losing an image, and deciding whether a gate that reads
untracked artifacts can honour a "buildless and source-free" claim at all —
`check-lane-contracts` asserts that claim today and it is already false for this
gate. Issue 1001 carries the four open questions.

**Whoever takes it: capture `--verbose` before and after and diff.** The gate
prints the same shape while examining a smaller set, so a smoke test cannot see
a regression — that is how attempt 2 passed inspection.

## Acceptance for the phase

* One spelling of the l3 lane, running once per change (W1).
* Every verification lane green at least once, with the date recorded (W2).
* A gate that fails when a workflow installs an indexed package, plus the three
  conversions (W3).
* The required check's promise and its trigger list agree (W4).
* Every job body is `just setup <scope>` + one command (W5).
* An RFC for `package.xml`-driven provisioning, reviewed, with an implementation
  work item opened against it (W6).
* The arena-budget walk scoped, or a recorded decision that its cold cost is
  accepted and the fast line's claim amended to match (W7).

## What the audit found working, and this phase must not undo

Recorded because a cleanup pass is exactly where these get deleted by accident:

* `host-tests`' fixture steps `exit 1` on failure. The comment above them records
  the ten runs where a green step sat over a failed build.
* `gate.yml`'s `ci-ok` refuses a vacuous pass: a skipped `check` is accepted only
  for a `pull_request` that `changes` proved touched no code; any other skip is a
  hard failure.
* `report-interlock-coverage.sh` turns "this self-hosted lane did not run" into a
  stated outcome instead of silence.
* `check-ci-no-verb-fallback` forbids one `just` verb falling back to another
  with `||` — the shell spelling of skip-and-report-success.


## Adopted issues (2026-09-04) — gates that exist and cannot run

Three open issues had no phase and belong to this one's premise from the other
direction: this phase asks whether CI runs the process a user runs, and these
are three places where a check CANNOT run at all, so its verdict is neither
true nor false.

* **[#1040](../issues/1040-gating-lanes-that-report-nothing-accumulate-reds.md)**
  — `check-api-parity` runs in NO workflow and `check-build` only on dispatch.
  A lane that reports nothing accumulates reds nobody sees.
* **[#1043](../issues/archived/1043-pin-gate-cannot-verify-uninitialised-submodule.md)**
  (RESOLVED 2026-09-06) — `check-submodule-pins` failed CLOSED on any submodule
  CI does not initialise, so a whole class of pin bump could never pass the
  required lane. Both halves are fixed now: the workflow initialises the pins
  that moved, and the gate distinguishes NOT VERIFIED (a skip, reported through
  the `nros_check_skip` ledger) from a measured non-fast-forward, with both
  directions mutation-tested on its normal path.
* **[#0930](../issues/0930-built-qemu-can-be-stale-against-its-pin.md)** — the
  built QEMU can be older than the commit `third-party/qemu/qemu` pins, and
  nothing says so. Provisioning that silently disagrees with the pin is the same
  failure one layer down.

They are one class: **a gate whose result is unavailable is not a gate**, and
this repo has now hit it four separate times (the `declared-qos-header` recipe
lost in a rebase, `check-test-scripts-have-callers`'s own subject, and these).
