# nano-ros

Lightweight ROS 2 client for embedded RTOS (Zephyr, FreeRTOS, NuttX, ThreadX). `no_std`.

This file is a **router + agent practices + pitfall index**, kept short because it is loaded
every session. Design rationale lives in RFCs, operational detail in `AGENTS.md` and `docs/`.

**Docs convention — three numbered series, do not mix them:**
- **Design decision** → an RFC in [`docs/design/`](docs/design/README.md) (`NNNN-slug.md`,
  living docs; `Draft`→`Stable`→`Superseded`). Whole-system view = `ARCHITECTURE.md`.
- **Planned / in-flight work** → a phase doc in [`docs/roadmap/`](docs/roadmap/) (work items +
  acceptance; names the RFC it implements; completed → `archived/`).
- **Known bug / limitation / tech-debt** → an issue in [`docs/issues/`](docs/issues/README.md)
  (`NNNN-slug.md` + frontmatter; `status: open`→`resolved`/`wontfix`; resolved → `archived/`).
  Issues cross-link the RFCs/phases that inform or close them.

**When you learn something durable, file it in the right series above and add only a one-line
pointer here — never grow CLAUDE.md with design/impl detail.**

## Where things live

| You need… | Go to |
| --- | --- |
| Finalized whole-system design | [docs/design/ARCHITECTURE.md](docs/design/ARCHITECTURE.md) |
| A specific design decision (stable vs evolving) | [docs/design/](docs/design/README.md) — numbered RFCs |
| A known bug / limitation / tech-debt (troubleshooting) | [docs/issues/](docs/issues/README.md) — numbered issues (open) + `archived/`. Query with `just issues [--area X] [--id N] [text]` (~20 ms, offline); the generated list is `open.md`, a GITIGNORED build artifact so concurrent filings never conflict — never `git add` it (issues 0883/0884) |
| Build / test / SDK tiers / jobserver / zephyr versions | [AGENTS.md](AGENTS.md) + [docs/development/](docs/development/) + `just/*.just` |
| Dev utilities (towncrier, clang-format) | `just dev-tools [--install]` — checks the interpreter you chose and installs the repo's OWN tools into it; never creates a venv, never touches build groups (issue 0885) |
| Long-form practices + pitfalls (cmake, tests, multi-session) | AGENTS.md “Practices & Pitfalls” (this file keeps the one-liners) |
| `nros setup` / provisioning / `nros-sdk-index.toml` | RFC-0014 + AGENTS.md “Toolchain & SDK Provisioning” |
| ROS 2 on a host with no apt ROS (Arch, Fedora, NixOS) | [docs/development/ros2-on-non-ubuntu.md](docs/development/ros2-on-non-ubuntu.md) — Ubuntu distrobox; `scripts/dev/ros2-{distrobox-setup,box-env}.sh`. **Box in play ⇒ EVERY job in the box, on its OWN tree** (`ros2-box-sync.sh`): different compiler + libc, shared artifacts, nothing checks they agree — refused since issue 0759 |
| Feature axes (RMW × platform × ROS edition) | ARCHITECTURE §2 + RFC-0005, RFC-0006 |
| Platform/RMW impl notes + deep pitfalls | [docs/reference/platform-implementation-notes.md](docs/reference/platform-implementation-notes.md) |
| C/C++ integration shape | AGENTS.md “C/C++ Integration” + RFC-0018/0019 + [docs/reference/c-api-cmake.md](docs/reference/c-api-cmake.md) |
| User-facing workflow | [book/src/](book/src/) (`just book`) |
| Phase history / current work items | [docs/roadmap/](docs/roadmap/) (active) + `archived/` |
| What do I type? | `just` with no args — the ~8 verbs, grouped by when you need them (phase-399). The 207 gates live in `just/check.just` behind `mod check`, so the verb is `just check <name>` and the recipe carries no `check-` prefix: `just check fast`, `just check leaf-lockfiles`. The old flat spelling is gone, with no forwarders. `just --list` is still everything |
| Periodic tech-debt / antipattern / UX audit | [docs/development/codebase-audit-checklist.md](docs/development/codebase-audit-checklist.md) |
| How our RMW C API compares to upstream `rmw` | `just check rmw-api-parity` (is every contract symbol CLASSIFIED?) + `just check rmw-abi-shape` (does the vtable MIRROR it — name, args, return?), both phase-376, both on the fast line. Contract is the 88 symbols EVERY `librmw_*_cpp.so` defines, not the 177 headers declare. The parity MAP is AUTHORED, so it drifts when slots move: it read `("gap", "no vtable slot")` for 28 slots W4 had landed and named 17 pre-W3.b spellings, while the shape tool found one real gap — two green tools disagreeing by 25 symbols. Cross-checked now (`check_against_vtable`, both directions). A gap that is real but open goes in `gap` with a TRACKED ISSUE ID in the reason; that is what `--check` tolerates, and nothing else |
| Profile a build's time (passive, read-only) | `just profile <dir>` → `nros-build-profile` (phase-251); [book](book/src/user-guide/build-profiling.md) |
| Measure a built image's static RAM (by symbol, crate, declared pool) | `just mem-report <elf>` → `scripts/nros-mem-report.py` (phase-392 W1); `--json` + `--baseline` for a before/after delta |
| Verify the book's setup flow on a pristine host | `just probe bootstrap` — runs the `probe=NN`-tagged book blocks in a clean container (`scripts/probe/`, issue 0204) |

## Naming
- **`native` / `posix` / `linux` answer TWO different questions — never synonyms.**
  - **native** — ROLE: this is the HOST build, not a cross build.
  - **posix** — REACH: works on any POSIX-compliant system.
  - **linux** — REACH: works only on Linux.

  `native` sits beside either reach; the two reaches exclude each other. They
  had collapsed — the host board read `names = ["linux", "native", "posix"]`,
  asserting both reaches at once. **Which one is false is MEASURED, and the
  first answer was wrong:** `nros-platform-posix` IS POSIX-clean (`sched_yield`,
  `sched_get_priority_*`, `pthread_setschedparam`, `SCHED_FIFO`; its one
  `__linux__` picks `MSG_NOSIGNAL` behind a portable `#else`) — but the BOARD
  crate is not. `nros-board-linux::apply_tier_affinity` calls
  `sched_setaffinity` with `cpu_set_t`/`CPU_SET`, **ungated by
  `cfg(target_os)`**, and libc defines those for linux/android/freebsd/
  dragonfly/fuchsia/cygwin and **not apple** — so the crate cannot build on
  macOS and `posix` was the false claim. The board is `["native", "linux"]`,
  the platform stays `posix`, and the two layers legitimately differ: the
  platform names software-stack facts, the board names what we support.
  Gates: `check-host-platform-vocabulary` (no board claims both reaches) and
  `check-posix-platform-purity` (the crate named for the standard holds to one —
  Linux-only code belongs in `nros-board-linux`). (Strictly the reach is "Linux and
  some BSDs, not macOS"; `linux` is the closest of the three words. Making
  `posix` true would mean cfg-gating that affinity call to a loud no-op.)
- **nano-ros** — project name (prose, docs)
- **nros** — code shorthand (crates, Rust/C idents, `CONFIG_NROS_*`)
- **nano_ros** — C header dir, CMake targets (`NanoRos::NanoRos`), CMake fn (`nros_generate_interfaces()`)

Workspace: `packages/{core,zpico,xrce,dds,boards,drivers,interfaces,testing,verification,reference,codegen,cli}/`,
`examples/`, `third-party/` (gitignored SDKs), `zephyr/` module. Run `ls packages/` for the current
crate list. Layer map → RFC-0001. `packages/drivers/` is split by what a crate talks
to — `net/` `serial/` `ipc/` `sys/` — documented in `packages/drivers/README.md`
(phase-321 W2.f). RFC-0012 is board/BSP integration and defines no such split.

## Practices
- **Run the TIER your change earns, after every task** (RFC-0061 / phase-318).
  **Never `sudo`** — tell the user.
  - `just ci gate` (was `just ci l1`, still forwards) — **compile + unit, NO
    FIXTURES** (phase-395 W2; renamed phase-410 W4 because it visits no
    coordinates, so it was never a rung on the breadth ladder). Measured 230 s.
    Depth is a positional arg on the tiers: `just ci matrix` runs, `just ci
    matrix build` builds+links only (27 s) — `name=value` does NOT parse for a
    module recipe.
    Run this before every push. It is `check` + `test-unit`, and it needs no
    fixture build, no SDK, no QEMU and no cross toolchain — only `test`/
    `test-all` depend on fixtures, and 74 of 163 test files never needed them.
    **That claim is now GATED (`check-lane-contracts`) because it was FALSE**:
    `check-source-gates` reached `platform_header_compile`, which resolves a
    fixture stamp nothing in the lane produced, so the lane silently required
    `build-test-fixtures`. Nobody noticed for as long as the claim had existed —
    the push lane runs `check-fast` alone — until this lane became a REQUIRED
    status check and every CI run went red on `BuildFailed("Test fixture binary
    not prebuilt")`. A gate in an affordability tier may resolve a COMPILE-stage
    stamp only if the lane BUILDS it (~13 s); a runtime fixture is banned
    outright.
    **CI runs a SUBSET of it, deliberately, and the subset DIFFERS BY EVENT**
    (phase-395 "PR cheap, batch thorough"; phase-396/399). On `pull_request` the
    required `CI` context is `check-fast` + `check-submodule-commits-reachable` +
    `check-compile-smoke` + `check-cli-tests`. **`test-unit` is NOT in it** — it
    runs on `merge_group` (and schedule/dispatch), where it costs a measured
    ~3.5 min per batch instead of per PR push. So a green `CI` on a pull request
    means "it compiles and the source gates hold", NOT "the unit tests pass";
    nothing broken lands, because the queue runs them before the merge, but the
    feedback arrives at the queue and an ejection is how you hear about it
    (`queue-notify.yml` comments on the PR). Run `just ci gate` locally if you
    want that answer sooner — it is the same lane. (`check-cli-tests` was added
    2026-09-02 — it lived only in `check-build`, which no merge-gating event
    runs, so issue 0896's two reds landed and stayed; no fixture/SDK/ROS, ~2 min
    warm / ~8 min cold, including the `nros-launch-resolve` it must build
    first.) `ci-l1` also runs `check-build` + `check-api-parity`. Your local tier is STRONGER than the gate, so you catch
    compile-tier breakage before the queue does and the queue stays cheap and
    always-satisfiable. `check-build` is now `schedule`/`workflow_dispatch` only —
    it was on the merge group and could never pass there (it needs generated
    bindings and prebuilt `.compile-ok` that no CI job builds), so the required
    check was red for EVERY pull request for a day. **A gate in an affordability
    tier may only resolve artifacts the JOB ITSELF builds** — `check-lane-contracts`
    enforces this for every merge-gating lane, not just `ci-l1`.
    It caught two reds on main in its first two runs (three clippy `-D warnings`
    errors and an unregenerated pool inventory), both in the compile tier that
    the `pre-push` `check-fast` hook deliberately excludes.
  - `just ci` — **tier 1**, host only. Gates and runs only native fixtures, so a
    stale ThreadX fixture cannot block it. Costs a fixture build; run it when
    you need fixture-backed coverage, not as the reflex before every push.
  - `just ci matrix` — **tier 2**, when the diff touches `packages/core`, codegen,
    or `cmake/`. 1-wise over platform/lang/rmw/kind: every value once, ~28 % of
    the coordinates. It sees each platform and each language, but NOT their
    pairing. `just build-test-fixtures lane=tier2` is the build it needs
    (phase-340 W3): the RUN narrows to the same coordinates at fixture
    RESOLUTION time, so an out-of-lane fixture SKIPS rather than failing. Between
    #482 and W3 this lane required `lane=all` — the ~26 % was the FRESHNESS gate,
    not the build. **Skipping needs ATTRIBUTION** — a row whose
    artifact root is shared fails CLOSED, so it is in the RUN set at every lane
    whatever its coordinate. Issue 0828 (FIXED): the build omitted exactly those
    rows, so after a core-crate edit `lane=tier2` left them stale, `_lane-gate`
    PASSED, and `test-all` reported ~190 stale-fixture failures — green only on
    a machine carrying older `lane=all` residue. `--coords-from` now omits a row
    only if the run could have skipped it too, which moved tier 2 from 114 to
    142 of 258 rows. Bound by
    `lane_build_covers_run::every_unskippable_row_is_in_its_lane_build`, which
    checks MEMBERSHIP: its first version compared counts (114 >= 28) and passed
    the mutation it existed to catch.
  - `just ci matrix-nightly` — the pairwise cover (~70 %). Where the
    platform×language and rmw×language classes actually surface (0268/0245 sizes
    headers, 0332 freestanding headers, 0331 vtable ABI). Tier 2 costs a day of
    latency on those, which is the price of a middle tier anyone can afford.
  - `just ci full` — **tier 3**, the whole matrix. Pre-release, on demand.
  Green tier 1 means "the logic and the seams are sound", never "it builds on the
  targets". Say which tier you ran; do not report a tier-1 green as if it were a
  sweep. The old single `just ci` WAS tier 3 — an instruction nobody could afford
  per task, so it got followed selectively, which is worse than a smaller
  instruction followed honestly.
- **Fix the CLASS, not the reported site — then prove the sweep.** Every bug here that
  recurred did so because a fix landed only where the symptom was seen: the sizes-header
  mirror (0088→0114→0122→0123→0245→0268), the Zephyr unset-variable guard (#282 fixed 1
  of 6 sites — and added a *second* idiom instead of a shared helper → #326), the fixture
  freshness probe (#222 fixed 4 RTOS resolvers, left ~30 in `binaries/mod.rs` → #328).
  So: grep for every sibling of the pattern, fix them together, add ONE shared helper
  rather than a second spelling, and put the sweep command in the commit message so the
  next person can re-run it. If a gate exists for the class, check the gate actually
  covers the new site (issue-0196 rule) — audit 2026-07-28 found four gates whose
  coverage was narrower than the rule they enforce.
- **Green CI locally BEFORE pushing — don't iterate on remote CI.** Run `just format`
  then the tier your change earns (above) locally and fix every failure first, so the
  push passes remote CI on the first try. `just ci full` = `check` (fast + build, incl. embedded
  clippy + every per-feature/per-example clippy, and the per-component lanes `check-c` /
  `check-cpp` / `check-rmw-cyclonedds` / `check-cli-tests`) + `rust-rtos-link-check` +
  `test-all`. A backend's own test suite belongs in a `check-*` lane, never as a named step
  on the `ci` line — the Cyclone suite had one, and a red sat on main for two days because
  `just check` never ran it (issue 0319). Note: `check` runs clippy with `-D warnings`, so a toolchain bump can
  surface NEW pre-existing lints (e.g. rust-1.96 `unnecessary_cast` / `drop_non_drop` /
  `not_unsafe_ptr_arg_deref`); fix them locally rather than discovering them remotely. CI
  stops at the first failing step, so one fix can unmask the next — re-run until fully green.
- **`just format` before broad changes** (Rust + C/C++ + Python).
- **Always nightly for `rustfmt` / `cargo fmt`** — `rustfmt.toml` enables nightly-only options;
  stable produces different output. Run `cargo +nightly fmt`.
- **C/C++ style:** `.clang-format` LLVM-based, 4-space indent, 100-col.
- **Linear history:** `git pull --rebase` or `git fetch` + `git rebase`. Never merge unless asked.
  **Now ENFORCED SERVER-SIDE** — the `main-rules` ruleset on `refs/heads/main` blocks
  force-pushes, deletions and MERGE COMMITS, with no bypass actors. So the old failure
  mode is gone: an accidental `git pull` merge used to LAND silently (three did, on
  2026-05-15) and now the push is REJECTED. Recovery is `git rebase origin/main`, never
  `--force`. **DIRECT PUSH TO `main` NO LONGER WORKS** (live 2026-08-28): the
  ruleset also requires a pull request and the merge queue, with no bypass. Work
  goes `just claim` -> branch -> `just ci l1` -> PR -> `gh pr merge --auto
  --rebase`. ONE required check, the aggregator `CI` — never add a job name to
  the required set, and never path-filter a required workflow: a check that
  produces no verdict blocks forever, which deadlocked two PRs on 2026-08-28.
  Read AGENTS.md "Branch policy" — it has the flow and the break-glass.
- **Never `git add -A` / `git add .`** — stage the paths you actually changed
  (`git add <path>…`, or `git add -u <dir>` for tracked-only edits). A blanket add
  scoops up build output, leftover dirs and stray artifacts. Twice in one session it
  re-added a submodule dir that upstream had MOVED, as an embedded git repo — a
  gitlink with no `.gitmodules` entry, which clones as an empty directory nobody can
  populate. git prints a warning; a blanket add buries it in noise. Read `git status`
  before staging, and when a warning does appear, stop rather than push.
- **Submodule rebase on superproject pull:** if a pull advances a submodule pointer AND local work
  exists in the submodule → enter it, fetch, rebase local onto upstream, check out the
  superproject’s expected commit, record the result in the parent. Never leave a submodule at an
  older local commit when the remote pointer advanced.
- **A submodule pin moves FORWARD only** — every submodule keeps linear history on
  its branch, so a bump is a fast-forward to a descendant. A rewind silently unships
  whatever the skipped commits fixed, and `-Subproject commit <hex>` is two hex
  strings no reviewer can order by eye: on 2026-08-15 a 24-file commit about
  issue-ID renumbering moved zenoh-pico BACK over a Zephyr `socklen_t` build fix and
  nothing noticed for seven hours. Gated by `check-submodule-pins` (fast line) AND
  the `pre-push` hook — the hook because a rewind is usually INHERITED from a rebase
  rather than authored, which is exactly the window a pre-commit check misses.
  Deliberate rollback: `NROS_ALLOW_SUBMODULE_REWIND=1`, and say why.
  **The gate has THREE outcomes, not two** (issue 1043): FAIL (ancestry MEASURED,
  not a fast-forward), NOT VERIFIED (no object store here — a reported skip, via the
  `nros_check_skip` ledger, because no lane checks out all 20 submodules) and OK. It
  reads `.git/modules/<name>` when the worktree is empty, which is EVERY agent
  worktree — before that, every pin move evaluated from one failed `CANNOT VERIFY`.
  `NROS_SUBMODULE_PINS_STRICT=1` restores fail-closed for a lane that really provides
  every submodule; setting it on a lane that provides a subset re-creates 1043.
  **`just setup-hooks` also sets the three git BUILTINS that make a pin move
  legible** — git has no setting that refuses a rewind, but it does know how to
  describe one, and by default it does not: `diff.submodule=log` prints
  `(rewind)` and the dropped commits with `<`; `status.submoduleSummary=true`
  shows the same BEFORE you commit; `push.recurseSubmodules=check` refuses a push
  whose pins are on no remote. Visibility from git, enforcement from the gate.
- **Vendored-fork branch workflow (cyclonedds, netxduo, zenoh-pico, …):** land fixes with linear
  history (commit in submodule → `git fetch origin` + `git remote prune origin` →
  `git rebase origin/<branch>` → push). **Push the fork branch FIRST, then bump the superproject
  pointer** to the pushed commit.
  **By default the agent does NOT push fork remotes** (they sit outside the trusted repo →
  exfiltration guard): the agent commits + rebases locally and leaves the branch ready; the
  maintainer pushes. The agent may push only when a scoped `Bash(git -C <submodule-path> push:*)`
  allow-rule exists — never a blanket `git push:*`.
  **The patch branch is a PATCHSET ON A STABLE VERSION, and it is not squashed.** Its commits are
  the readable history of what we changed and why, so they stay individual — a squash to "one
  delta commit" trades the only record of intent for a tidier graph, which is the wrong trade.
  New work REBASES onto the branch tip and pushes; that is what keeps it linear.
  **Upstream contribution is a SEPARATE line with no 1:1 correspondence, deliberately.** A
  contributor may also PR the same fix to the project's own `main`, and the two versions will
  differ in shape, because upstream `main` and the stable version we pin have diverged. Do NOT
  try to keep the commits identical, and do not hold our branch back waiting for upstream to
  merge: the contributor CHERRY-PICKS onto our patch branch and pushes there. Ours is the line
  that ships; upstream's is a courtesy that lands on its own schedule.
  **Reading a fork's remote: use `git ls-remote`, not `git branch -r`.** A submodule clone is
  often single-branch (`+refs/heads/main:refs/remotes/origin/main`), so remote-tracking refs show
  ONE branch however many the remote has, and every check built on them answers "what has my
  clone fetched", not "what exists". `push.recurseSubmodules=check` reads those same local refs,
  so on such a clone it refuses a pin that IS published, and the refusal reads exactly like an
  unpushed commit. On 2026-08-31 that cost a false data-loss alarm — zenoh-pico's 28 remote
  branches looked like one, and a pin sitting on the tracked branch looked like 31 orphan commits.
  Widen the refspec (`git config --add remote.origin.fetch '+refs/heads/*:refs/remotes/origin/*'`)
  and re-fetch before believing anything about a fork's remote.
- **A submodule has exactly two legitimate shapes, and patching decides which.** Either it
  **pins a stable version on an untouched third-party repo** (a tag or release commit — the
  usual case, and the pin is a decision, see below), or it **tracks a patch-line branch on OUR
  fork**. There is no third shape where we patch someone else's repo in place.
  The distinction that matters is *fork* vs *ours*: a fork of an upstream project keeps its
  patches on a named branch and its `main` clean, while a repo WE AUTHOR (`play_launch`,
  `px4-rs`) has no upstream to mirror, so its `main` IS the line and `branch = main` is correct
  there. Both still record the branch.
  **Before writing the first patch line, make the fork and the branch exist** — fork the repo,
  create the patch-line branch (`nano-ros` by convention; `nros-lp64-ulong`, `nuttx-0.2`,
  `nano-ros-v11.0.0-patches` where a narrower name reads better), and point the submodule at
  it. Doing this after the fact is how a patch line ends up somewhere nobody can find it.
  **The fork's `main` mirrors upstream.** It is the thing you diff against and the thing you
  rebase onto; the moment patches land on it, "what have we changed?" stops having an answer.
  On 2026-08-23 zenoh-pico's fork had it backwards — `main` was a four-commit orphan carrying
  the patches (root `07de44f`, no upstream history beneath it) while the real `nano-ros` branch
  sat four months stale without the IVC or CUSTOM links. Reconstructing which patches were live
  took a tree-level audit, because commit history could not answer it.
  **Record the branch in `.gitmodules`** (`git submodule set-branch --branch <b> <path>`), for
  every fork, always. Be precise about what that buys: the gitlink is still a COMMIT and still
  what a clone and CI check out — `branch =` only tells `git submodule update --remote` which
  line to follow, and tells a reader where the patches live without an archaeology session
  against the remote. Absence of `branch =` should mean "unmodified upstream", so it is only
  informative if every fork has one: as of 2026-08-23, 11 of 20 submodules are forks and all 11
  declare a branch; the other 9 are untouched upstream.
  **Resolve the branch by containment, not by guessing.** A pin is often BEHIND its branch tip,
  so matching the pin against branch heads finds nothing and matching it against `main` is a
  coin flip — `ros2_rust_examples` pins a commit on `add-justfile`, not `main`. Fetch the
  candidate branches and ask `git merge-base --is-ancestor <pin> <branch>`.
  **Before force-pushing a patch line, prove the replacement is a superset.** Not "the commits
  look similar" — compare the trees: `comm -23 <(git ls-tree -r --name-only OLD|sort)
  <(git ls-tree -r --name-only NEW|sort)` must be empty, and spot-check each old patch's content
  in the new tree. And recover the old ref first (`git reflog show refs/remotes/origin/<branch>`,
  anchor it to a branch, push it) so the check is reversible rather than merely careful.
- **Two kinds of submodule, and the difference decides whether a lag is a BUG.** A `git status`
  showing a submodule dirty is not uniformly "stale, update it":
  - **Vendored third-party (`third-party/dds/cyclonedds`, `third-party/threadx/kernel`,
    netxduo, zenoh-pico, nuttx, …) = a chosen upstream release PLUS our commits.** The pin is a
    DECISION, not a lag — cyclonedds tracks the version ROS ships, so bumping it off that is how
    you stop interoperating (the class issue 0609 measured for the zenoh router). Never
    "update to latest" on these; move them only with a reason, and re-read `nros-sdk-index.toml`
    /the RMW pin notes first.
  - **Ours (`packages/cli/third-party/play_launch`) = we author it, so it TRACKS ITS LATEST
    `main`.** A lag here is just a lag. Advance it with
    `git -C packages/cli/third-party/play_launch fetch origin && … checkout origin/main`, then
    `just setup-launch-resolve` (issue 0409 — a resolver from a different layer-2 checkout writes
    models that are MISSING DATA rather than failing) and `just check submodule-pinned-locks`.
    Still forward-only, and still non-recursive (layer-3 `src/vendor/*`, container, msgs are never
    built here, so `src/play_launch_container` showing modified is expected, not dirt to fix).

  What both share: when `check-tier-preconditions` says a submodule is BEHIND the recorded pointer,
  that is the superproject's pin disagreeing with your checkout, and the fix is
  `git submodule update <path>` regardless of kind. Three of them sat behind for a whole session
  because a dirty-looking `git status` line was read as someone else's mess.
- **Codegen + orchestration CLI lives in-tree at `packages/cli/`** (a sub-workspace, own
  `Cargo.toml`/`Cargo.lock`). Edits to codegen / `colcon_nano_ros` / orchestration land there; build
  via `just setup-cli`. The retired `packages/codegen` submodule is fully gone (no stray leftover).
  `packages/cli/` nests `third-party/play_launch` + `testing_workspaces/ros2_rust_examples`.
- **Launch toolchain (RFC-0060, amended to TWO repositories — phase-332 W1/W2 landed):** nano-ros
  pins the **`play_launch`** repo at `packages/cli/third-party/play_launch`; layer 2 (the resolver,
  launch tree → SystemModel, needs CPython NOT ROS/colcon) is REGULAR FILES at
  `src/ros-launch-resolve`. Init NON-recursively (`git submodule update --init
  packages/cli/third-party/play_launch`) — layer-3 runtime submodules (`src/vendor/*`, container,
  msgs) are never built by nano-ros. `ros-launch-manifest` (spec) is a git-TAG cargo dep — **`v0.1.8`
  across every nano-ros crate**, no longer nested (the 0285 double-vendoring is gone), and the old
  `--recursive` landmine is retired. The one other tag in the tree is play_launch's own
  in `play_launch/tests/`, which is its pin, not ours. Read the pin from the manifests, not
  from here: this line said `v0.1.0` for long enough to mislead (2026-08-18).
  The `nros-launch-resolve` helper is built by `just setup-launch-resolve` and
  invoked by ABSOLUTE PATH, never `$PATH` (issue 0285).
- **Don’t modify vendored/generated:** `third-party/`, `packages/interfaces/*/generated/`, build
  output — unless the task explicitly requires regeneration. Preserve worktree changes.
- **Examples are standalone copy-out projects** (`examples/<plat>/<lang>/<example>/`); no workspace
  walk-up. Non-example bins live under `packages/testing/{nros-tests/bins,nros-bench,nros-smoke}/`.
  Detail → RFC-0026 + `examples/README.md` coverage matrix.
- **Workspace examples follow RFC-0066 (phase-331, landed): a FEATURE is a node package,
  a CONFIGURATION is a fixture axis — never a new directory.** Feature demos (params/lifecycle/
  qos/custom-msg/remap) live as node pkgs in the native-only `workspaces/features/`; RMW ×
  feature-set variants are `fixtures.toml` rows over the four large workspaces
  (`workspaces/{rust,c,cpp,mixed}`). The themed `ws-*` dirs are GONE (W3, verified zero remain) —
  don't reintroduce one. Naming rules (no language prefixes in single-language workspaces,
  role-not-payload pkg names, one platform vocabulary for entries) →
  `examples/workspaces/README-layout.md`. West-built zephyr entry leaves need BOTH the nested
  workspace `exclude` AND a repo-root `Cargo.toml` exclude, and their dep keys must match the
  generated `<entry>_nros_selection` package name (phase-331 fallout class, 2026-08-03).
- **SystemModels are BUILD ARTIFACTS — never committed, never referenced by entries**
  (phase-330 W4.a/W7, landed 2026-08-03). Dims/params/capabilities are authored in
  `system.toml` (+ launch files); `nros sync` resolves into `<ws>/build/nros/models/
  <bringup>/`; entries name their INPUT (`nros::main!(launch = "bringup[:file]")`,
  `nano_ros_entry(BRINGUP … LAUNCH …)`); consumers locate the artifact via
  `nros_orchestration_ir::model_location` (never a hand-derived path). `model =`/`MODEL`
  are deprecated expert overrides. Gate: `check-no-tracked-models` (issue 0380 was four
  hand-edit deletions; the ban is the structural fix). Inspect with `nros ws model-dims`.
- **Message deps are PATH deps pinned `0.0.0` (RFC-0067 / phase-333)** — never registry-name a
  message crate (`std_msgs = "*"`) in a leaf manifest; #378 showed a bare name resolving against
  the PUBLIC crates.io.
- **`generated/` in examples/fixtures/tests is USER-side — never commit it, and therefore never
  commit their `Cargo.lock`.** Those trees are codegen'd from the USER's own msg packages, so they
  don't exist in a fresh clone; a lock committed beside one names crates nobody has and every cargo
  command in that leaf fails (this made `build-test-fixtures` unrunnable on a fresh host,
  2026-08-03 — ten such locks deleted). Tell users to run `nros sync`. When a lock and a missing
  `generated/` collide, DELETE THE LOCK — never commit a `generated/` tree to keep one.
  **Exception: the core pre-generated msg packages** (`packages/interfaces/*`), committed under
  `nros-`prefixed names because core crates need those messages BEFORE any codegen runs and the
  prefix keeps them from colliding with a user package of the same ROS name. They resolve from a
  bare clone, so their LOCKS are tracked — but their crate version is still the constant `0.0.0`
  like every generated crate, and consumers must path-dep them with NO version (pinning either
  spelling broke root-workspace resolution twice, #394).
  Invariant, enforced by `check-leaf-lockfiles`: **tracked lock ⟺ (no message deps) ∨ (committed
  `generated/`)** — boards/drivers/smoke qualify via the first arm. (The old "track all of them"
  rule predates the shim keying on TRACKED rather than ignored, #386.)
- **`std` is being DELETED; write `core::`/`alloc::`, never `std::`** (ARCHITECTURE §2, phase-359).
  Terminal state of the core crates is `core` and `core+alloc` — `std` there is a second
  implementation of the platform layer, not a convenience over it. Whose requirement it is decides
  the spelling: a capability the CONSUMER picks REQUIRES `std` (`env` = `std::env` —
  `compile_error!`, consumer names `std`); a purely INTERNAL one GRANTS it (`metadata-mode`'s own
  code uses `Mutex`/`format!`, so making callers name `std` would turn an implementation detail
  into a consumer-facing flavour). Corollary: **a capability that grants what it needs can never
  reach its own guard** — `metadata-mode` carried an unreachable `compile_error!` for two phases.
  And where a consumer GETS `std` is not uniform (`examples/native/rust/*` inherit it from
  `nros-board-linux`; `nros-tests/bins/*` must name it), so a manifest-READING sweep looks complete
  while half the tree is red — build the candidates. → issues 0632, 0640.
- **Messages are generated** (`nros generate-rust` from `package.xml`) — never hand-write. Detail
  → RFC-0023 + [docs/guides/message-generation.md](docs/guides/message-generation.md).
- Unused vars: `_name` + comment, or `#[allow(dead_code)]` for test struct fields.
- Reusable tests → `packages/testing/nros-tests/tests/` (Rust) or `tests/` (sh). Temp tests → Bash
  then promote. Temp files in `$project/tmp/` (gitignored), not `/tmp`; use Write/Edit not heredoc.
- **Tests must fail on unmet preconditions** (`assert!`/`bail!`/`nros_tests::skip!`). Bare
  `eprintln!`+`return` reports PASS — never. Same for runtime: panic, not silent early-return.
  Gate: `check-no-vacuous-tests` — a test body whose only effects are PRINTS. 17 of these
  existed (10 files, 2 literal cross-file duplicates), each reading `is_*_available()` probes
  and printing them, so each passed on the very host it was meant to warn about; one said so
  itself ("These are informational - don't fail if Zephyr isn't set up"). The same probes were
  already load-bearing three lines away as the `skip!` guards, so the call read as coverage in
  one place and as a precondition in the other. The gate keys on "prints only", NOT on "has no
  `assert!`" — ~40 correct tests delegate to an asserting helper, and a bare call statement is
  exactly the shape whose assertion is one frame down.
  **A target behind `required-features` that no recipe enables is the same lie one level up** —
  cargo skips it SILENTLY (not reported as filtered, simply never built) so it reads as coverage.
  Gate: `check-required-features-reachable`; lane: `just check required-features-tests`. When the
  laned targets finally ran, four were broken and one capability was entirely non-functional
  (issues 0652/0612/0667). Assert LOWER bounds on timing too: an upper-bound-only wake test
  passes a `spin_once` that never waited.
- **No compilation inside tests** — never `cargo`/`cmake`/`idf.py`/`west build` at run time. Compile in
  the build stage (`build-test-fixtures` + `examples/fixtures.toml`); the test consumes the prebuilt
  fixture. "Does it compile?" intent → make it a build-step fixture and assert the artifact. → AGENTS.md Testing.
- **Fixture builds are LANE-SCOPED (#393):** `just build-test-fixtures lane=<all|native|tier1|tier2|tier2-nightly>`
  narrows both the platform-family fan-out and the manifest rows; the `.fixtures-built` stamp
  records `lane=` + per-coordinate rows, and `_require-fixtures` checks COVERAGE against the run's
  lane. Build the lane you'll test — tier 1 doesn't need all 314 rows.
  **A lane answers TWO questions and they have different answers (#482):** which fixtures must be
  FRESH (its cell cover) vs which must EXIST (a property of the RUN). `nros_lane_build_lane` maps
  lane→required build and `CiLane::run_scope` declares it. Tier 1 narrows its run by NAME
  (`NROS_TEST_SCOPE`) so it needs the broader `native` build; **tier 2 / nightly narrow by
  COORDINATE in the fixture RESOLVER** (`NROS_TEST_COORDS` → `nros_tests::fixtures::lane`,
  phase-340 W3), so each is its own build lane. Name filtering cannot express tier 2 — it is
  1-wise over platform, so every platform is in it (#357/#482); the resolver attributes an
  artifact back to its manifest row via `row_artifact_root()`, the sibling of `row_coord()`, so
  build-set and run-set are ONE predicate on one coordinate file. The skip is keyed on the
  COORDINATE, never on "artifact absent": an in-lane fixture that is missing or stale still fails
  hard, and an unattributable path (zephyr west leaves, compile-check — built module-level) is
  never skipped. A row's coordinate still has exactly one computation, `row_coord()` in
  `fixtures-manifest.py` (`rmw` defaults to zenoh THERE) — `matrix_fixture_coverage.rs` consumes
  its `coords` subcommand rather than re-deriving. The second derivation left 67 of 240 rows in
  no lane at all. New runtime tests join a
  matrix: cells in `matrix::CELLS` / `interop::CELLS` (RFC-0051; phase-331 W4 put workspace RMW
  cells there too), not new hand-coordinated files — the consolidation plan is phase-329.
- **Fixture mtime treadmill:** any pull/rebase — and any `git stash push`/`pop`, which rewrites
  tracked files just the same — refreshes source mtimes → EVERY prebuilt fixture
  reads STALE. **`just format` is one of these and does not look like one:** `native::format`
  runs `cargo fmt` per example leaf, which formats that leaf's `generated/` members too, so
  every native fixture re-stales at one timestamp. Format BEFORE building fixtures, never
  after or during — a `format` run overlapping a fixture build costs the whole build. **A refresh re-arms the in-tree CLI's source stamp too, not just fixtures
  (issue 0466)** — and the order is load-bearing: rebuild the CLI FIRST (`just setup-cli`),
  THEN fixtures, because fixtures key on that stamp; doing it the other way re-stales
  everything you just built. `just check tier-preconditions` reports every unmet
  precondition at once (CLI, leaf `nros sync`, build sources, fixtures for the lane)
  instead of one per attempt; it runs at the head of `just ci`. Rebase once → rebuild affected fixtures → test WITHOUT pulling again. Core-crate
  or repr(C)-struct changes ⇒ wipe workspace build dirs (incremental mixes pre/post-append
  objects → garbage-pointer SEGVs). Long-unrebuilt families "pass" on museum binaries — trust
  only a fresh full sweep, and re-measure any perf number on cleanly rebuilt fixtures before
  filing an issue from it (→ archived issues 0148/0164). A `nros` CLI rebuild also stales every
  WORKSPACE fixture (the codegen tool is in the input signature + CONFIGURE_DEPENDS since #182 —
  rebuild the family, don't debug the "runtime bug").
- **A STALE verdict is ABSORBING — read the `probe:` and NOT RUN lines before believing it**
  (issue 0445). The fixture never launches, so whatever it would have done at runtime is
  replaced by a message that explains itself; issue 0444 hid behind 0442 for exactly that
  long, and my first explanation of the symptom was plausible and wrong. Verdicts now print
  what the probe examined and exempted, and count consecutive non-running resolutions —
  `x2+` means suspect the probe, not just the fixture. `just fixture-staleness` lists every
  coordinate producing no runtime result. Exemptions have ONE spelling
  (`nros_tests::fixtures::staleness::exempt_probe_input`, gated by
  `check-staleness-probe-exemptions`) because per-arm subsets ARE 0442.
- **A test result is only about the tree its FIXTURES were built from — check the mtimes
  before filing (issues 0859/0860/0861/0862, all four retracted).** A sweep, then a rebase,
  then reading the sweep's output is four ghost issues: the artifacts were built at 03:19
  against a tree rebased past 04:48, so each red reproduced a bug that had ALREADY BEEN
  FIXED, in convincing detail. Two of the four also got a confident wrong ROOT CAUSE written
  into them (an allocator that was fine; a "no compilation inside tests" violation in a test
  that answers in 82 ms), which is worse than the bogus filing — it aims the next person at
  a dead end. Before filing from a failing fixture: `stat -c '%y' <artifact>` against
  `git log -1 --format=%ci` of the code being blamed, and re-run the single test SOLO — a
  60 s timeout under a 32-way build was 0.082 s alone. `Real failures: N` from the junit
  rewrite counts what the RUN saw, not what is true of HEAD.
- **Test greps use `nros_tests::output::*` constants, never literal strings** — example
  banners/markers get slimmed (phase-277 broke ~10 tests grepping `"Result:"`/`"[OK]"`/old
  banners while delivery worked). If a test times out, FIRST diff the grep pattern against what
  the fixture actually prints. → archived issues 0157/0164.
- **Test names describe behavior, not phase numbers** (`zephyr_xrce_service_e2e`, not `phase212_n9_…`).
  Phases go stale; cross-ref a phase in a doc-comment, never the identifier. → AGENTS.md Testing.
- **Two test-intent lists (RFC-0051 / phase-324):** `matrix::CELLS` = baked/self-contained;
  interop & bridge cells live in `interop::CELLS` (nano `Cell` + peer + dir + build + test) —
  they have NO `fixtures.toml` row (ephemeral peer, west-leaves/native nano side). Gated by
  `matrix_fixture_coverage.rs` G1–G4; each interop test carries an `interop::assert_test_bound`
  coordinate tripwire. Don't add an interop test that hand-picks a fixture without a matching
  `interop::CELLS` row. `ros_editions_e2e` is the docker edition axis (#0327), not a cell.
- **Deleting a test target strands its `.config/nextest.toml` override, and a stale `binary()`
  is a PARSE ERROR, not a no-op** (issue 0743) — "operator didn't match any binary names" kills
  EVERY nextest run in the repo, not just that lane, and `just check` never notices because it
  runs no nextest. Gate: `check-nextest-binary-filters`. A stale `test()` degrades the other
  way — silently inert, dropping its timeout/retries/test-group (five did, phase-329 W1) — and
  is NOT statically checkable: those are rstest-generated case names (`Platform__Nuttx`) that
  appear nowhere in the sources. Delete the override with the target, or keep the note in a
  COMMENT.
- **Bare `cargo nextest` counts `nros_tests::skip!` panics as FAILURES** — only `just test-all`'s
  junit rewrite makes them skips. Read the panic text before filing a bare-run red as a regression.
  And full-sweep QEMU lanes flake under load (287-W7: six nuttx lanes failed 3/3 in-sweep, passed
  solo) — retest a QEMU red SOLO before filing. A "solo red" can ALSO be a stale-build artifact,
  not code (issue 0268: the sizes-header mirror race made incremental trees red and clean trees
  green — a bisect whose steps rebuilt clean "converged" on a docs-only commit). When a bisect's
  first-bad is implausible for the symptom, the test tracked a confounder (build state, load,
  ports) — rerun one rev N times before trusting any boundary. → AGENTS.md Test Pitfalls.
- **A missing per-build sizes header can be ABSORBING, not a race (issue 0834).** The
  0088-family fixes were all ordering; this one is not. When a mirror dir holds
  `nros_cpp_config_generated.h.stamp` and NOT the header, cargo is up to date, so the
  build script does not re-emit the byproduct, the POST_BUILD copy has nothing to copy,
  and ninja records its custom command as successful. Deleting the stamp does not help;
  building the header target directly does not help; only `rm -rf` on the west build dir
  does. Survey with: for each `zephyr-workspace/build-*/nros-rust/nros-{cpp,c}-generated/nros`,
  a `.stamp` with no `.h` beside it. Two such leaves stopped a whole `lane=all` sweep.
- **`rm -rf` + rebuild is an ANTIPATTERN — it destroys the evidence and fixes nothing.**
  The build system supports incremental builds; if an incremental build produces a wrong
  artifact, that is a MISSING DEPENDENCY EDGE and the edge is the bug. Wiping proves only
  that a full build works, which was never in doubt, and it costs the one reproduction you
  needed to find the edge. It also teaches the next person that the tree is untrustworthy,
  so they wipe too, and the real defect never gets filed.
  Before reaching for `rm -rf`, in this order: `ninja -C <dir> -t query <target>` (does the
  artifact actually depend on what changed?), `touch` the input and confirm a relink, and
  read the `.d`/`build.ninja` edge. Issue 0475 is the canonical case — a lib inside a raw
  `-Wl,` flag gets NO edge, and the fix was `LINK_DEPENDS`, not a wipe.
  The rule has exactly two exemptions, both documented and both about a build that CANNOT
  converge: the sizes-header mirror reaching a state no re-run repairs (issue 0834), and a
  core-crate/`repr(C)` change mixing pre/post-append objects. Outside those, if you wiped
  and it worked, you have a lead — not a diagnosis; say so, and go find the edge.
  **When a GENERATED build file is itself the problem, re-configure — do not wipe.**
  `cmake <build-dir>` re-runs configure from the cached settings and regenerates
  `build.ninja` in place. This is the escape hatch for the one state ninja cannot recover
  from on its own: `multiple rules generate <x>` is raised at LOAD, before any rule runs,
  so ninja can never re-run cmake to fix itself (issue 0882 hit exactly this after a
  half-applied fix). `just reconfigure-stale` does it across the tree — it probes every
  `build.ninja` with a load-only `ninja -t targets`, re-runs `cmake` on the ones that
  fail, and REPORTS rather than deletes anything it cannot repair (a wedged dir is the
  only reproduction of whatever wedged it). `just reconfigure-stale check` reports
  without repairing; gate `check-reconfigure-stale` is its negative control, since
  "N build dir(s) load" is also what a probe that can never fail would print.
- **Build-side stale probes must watch the same inputs as test-side gates** — a probe that misses
  `generated/**` lets a museum binary pass every sweep while tests fail STALE (issue 0196).
- **Sweep contract:** every `just <plat>` invocation needs `source ./activate.sh` first (PATH wires
  `nros`, `play_launch_parser`). `just doctor` enforces it. The pre-218
  `export PATH="$HOME/.nros/bin:$PATH"` is insufficient.

## Pitfall index

One-liners; detail in the linked doc. (Many also captured in agent memory.)

- **After clone, run ONE of** `direnv allow` / `source ./activate.sh` / `source ./activate.fish`
  else `zpico-sys/build.rs` panics `"FREERTOS_PORT not set"`. Activate files are the env/PATH SSoT.
- **The router is ROS's `rmw_zenohd`; we ship none** (RFC-0075 / phase-362). It links the same
  `libzenohc.so` as the `rmw_zenoh_cpp` a ROS node uses, so it cannot drift from it — which a
  pinned vendored router did: issue 0609 measured `ros-humble-rmw-zenoh-cpp` 0.1.1→0.1.9 moving
  its zenoh 1.2.0→1.8.0 with our pin taking no part. **`third-party/zenoh/zenoh/` and
  `build/zenohd/` are GONE** — a doc naming either is stale. Resolve it with `nros_zenohd_bin`,
  start it with `nros_router_exec` / `just zenohd`, and TELL a user with `nros_router_hint`
  (prints the path-independent `ros2 run rmw_zenoh_cpp rmw_zenohd`); never spell the install path
  — gated by `check-zenohd-flag-invocations` (issues 0653/0654). Resolution order is
  `NROS_RMW_ZENOHD` → `AMENT_PREFIX_PATH` → `$ROS_DISTRO` under `/opt/ros`; PATH and a
  `/opt/ros/*` glob are deliberately NOT searched, because both return a router nobody chose.
  zenoh-pico stays pinned 1.7.2 at `packages/rmw/zenoh/zpico-sys/zenoh-pico/` — its wire is
  proto-stable across 1.x, so it interops with a newer ROS zenoh (issue 0291).
- **A router that RESOLVES is not a router that RUNS — `rmw_zenohd` loads whatever
  `libzenohc.so` the LOADER finds** (issue 0774). It links by SONAME, and
  `<prefix>/opt/zenoh_cpp_vendor/lib` is on `LD_LIBRARY_PATH` only when `setup.bash` (or
  `activate.sh`) was sourced; otherwise another `libzenohc.so` on the default loader
  path wins — on this host the `libzenohc` package (upstream zenoh-c, installed on
  purpose, not debris) — and a zenoh the router was not built against SEGVs mid-startup
  rather than failing to load. Two correctly installed zenoh-c builds coexisting is a
  SUPPORTED state, so pairing is the caller's job, not the host's. That took
  13 of 20 `check-required-features-tests` red on a host that HAS ROS, reporting only
  `signal: 11`. RFC-0075's drift through the loader instead of a pin. The fixture now
  pins the pairing itself (`paired_zenoh_library_dir`), so this no longer depends on the
  caller's env — but a bare `rmw_zenohd` you start BY HAND still does. And when asking
  who owns a `/lib/...` file, query the RESOLVED path: `/lib` is a symlink to `usr/lib`
  on merged-`/usr`, so `dpkg -S /lib/<f>` says "no path found" for a perfectly
  well-owned file — which is how this entry first called that library a stray.
- **Zephyr merges Kconfig fragments LAST-WINS, so a leaf value a later fragment
  also sets is DEAD** (issue 0876). `CONFIG_HEAP_MEM_POOL_SIZE=0` in the c/talker
  conf was measured on mps2/an385 — where `cmake/zephyr/mps2-an385.conf` merges
  after it and sets `131072`, so it changed nothing there — and its only live
  effect was making every native_sim talker image fail to COMPILE
  (`nsos_sockets.c` asserts `> 0`; Zephyr's own offloaded-socket driver
  `k_malloc`s per socket, so "nothing allocates from the kernel heap" is a claim
  about the WHOLE image, not about nano-ros). Gate: `check-kconfig-overridden-values`
  (a ratchet — shared board fragments are legitimately an override layer).
  **And the per-leaf `examples/zephyr/**/boards/*.conf` are NEVER merged** — 18
  dead files whose content is duplicated in `cmake/zephyr/native-sim-line-*.conf`,
  which is what actually applies. Read the build's own `Merged configuration`
  lines before editing a conf; a fix in the wrong file changes nothing and looks
  like it should.
- **A column-0 line inside a `just` recipe or a YAML `run:` block is parsed as
  SYNTAX, not as text** — so a heredoc is unusable there (its terminator must be
  at column 0), and `just`'s own body-dedent then fights any `sed` strip added to
  compensate. Use `printf` with one argument per line, indentation inside the
  quotes. Cost four separate failures in one day, each with an error naming
  something else (`unknown start of token '—'`, `expected ':'`, YAML
  `could not find expected ':'`).
- **A generated file that is COMMITTED and touched by every PR serialises the
  merge queue** (issues 0883/0884) — `docs/issues/README.md` was the only
  conflicting path in three of five open branches, so one merge ejected the rest.
  A custom merge driver cannot fix it: GitHub rebases queue entries SERVER-SIDE,
  where `.gitattributes` drivers do not run.
- **A script a git hook reaches must clear the inherited git environment first**
  (issues 0986/0988). `GIT_DIR` & co. override BOTH a path argument and
  `git -C`, so `git init "$tmp/x"` in a selftest builds nothing and instead
  writes `core.bare=true` into the CALLER's config and stages the selftest's
  invalid gitlink into its index — from `pre-push`, the hook whose job is
  refusing bad submodule pins. One spelling:
  `scripts/lib/git-hook-env.sh`'s `nros_clear_inherited_git_env` (list from
  `git rev-parse --local-env-vars`, so it cannot drift; four hand-written copies
  already had). **It does not reproduce by hand and it is self-masking** — an
  interactive shell has no `GIT_DIR`, and from run 2 the hook dies at
  `rev-parse --show-toplevel` before reaching the cause, so it reads as a broken
  repo rather than a bad script. Measured: an ordinary `git push` sets only
  `GIT_PREFIX`; a push **from a linked worktree** sets `GIT_DIR` — which is how
  parallel agent sessions work here, and that gitdir's `index` is live state.
  Gate: `check-hook-repo-side-effects` (runs the hook and every `git init`-ing
  script under both environment shapes against a victim repo compared byte for
  byte, mtimes included — a hook that rewrites a tracked file re-stales every
  fixture).
- **A red CI lane answers one of two questions and they look identical** — the
  lane RAN and the code is broken (a verdict), or it never ran (no verdict). A
  uniformly-red lane has NO signal capacity: a regression landing in it looks
  exactly like yesterday's failure, which is how issue 0876 rode in while the
  nightly's own `c/talker` cell reported `failure` before and after. `just
  nightly-triage` classifies by which STEP failed and flags cells red across a
  whole window; `just queue-triage` does the same for merge-queue ejections
  (INFRA vs MINE).
- **Rust edition 2024:** `unsafe extern "C" {}`, `#[unsafe(no_mangle)]`, explicit `unsafe {}` in
  `unsafe fn`. `nros-c` keeps `#![allow(unsafe_op_in_unsafe_fn)]`.
- **No POSIX-style Rust ctor sections on Zephyr/native_sim/RTOS** — backend registration is an
  explicit call: C/C++ via `nros_cpp_init` → strong `nros_app_register_backends`; pure-Rust via
  `zephyr_component_main!` (calls the hook + cfg-gated direct `register()`). A pure-Rust image
  needs the REAL backend dep (`rmw-zenoh = ["dep:nros-rmw-zenoh"]`) — and a direct reference,
  or rustc's staticlib DCE drops the dep's `#[no_mangle]` export (symbol in the rlib, absent
  from the `.a`; nros-c's FORCE_LINK class). → issues 0155/0163 (archived).
- **Rust `std::println!`/`eprintln!` KILLS a Zephyr native_sim image — use `nros_log`** (issue 0589).
  `zvfs_write(1, …)` dispatches to `stdinout_write_vmeth`, which calls `zvfs_write(1, …)` again;
  `k_mutex` is recursive, so it exhausts the stack instead of deadlocking and the image dies with
  no message. Armed in EVERY native_sim image (identical Kconfig in cells that pass), fires only
  when a Rust std stdio call is finally reached — so a diagnostic makes an error path LESS
  informative. C `printf` is safe (picolibc console hook, not the POSIX fdtable), which is why it
  stayed latent. Write `nros_log::nros_error!(nros_log::get_logger("<crate>"), …)`: it lands on
  `LOG_ERR`/`printk`, never fatal, and reaches `no_std` targets that every `cfg(feature = "std")`
  arm silently skipped. Do NOT gate the CALL SITE on `not(platform-zephyr)` — that was the
  workaround, and it discards the information exactly where a return code is all you get. Gated by
  `check-no-std-stdio` (`just check`), which forbids `std::`-qualified stdio in a `#![no_std]`
  crate's `src/`; a bare `println!` there is a board's own console macro, not this hazard.
- **nros-cpp headers: gate `<string>`/std includes on `NROS_CPP_STD`, not `__STDC_HOSTED__`** — a
  hosted compiler can still run `-nostdinc++` against Zephyr's minimal libcpp (no `<string>`).
  → issue 0112 (archived).
- **Domain ID:** compile-time on embedded (Kconfig / per-example `config.toml`), runtime env on
  native via `nros_tests::unique_ros_domain_id()`. `CONFIG_NROS_CYCLONE_DOMAIN_ID` defaults to
  `NROS_DOMAIN_ID` — never pin it to a literal in confs (the phase-180 split-brain silently ran
  every cyclone image on domain 0). Cyclone fixture pairs bake distinct domains (50–58) for
  parallel SPDP. → issue 0161 (archived), platform-implementation-notes.md.
- **`zpico_spin_once` on multi-threaded platforms uses `z_sleep_ms()`, not `select()`** (else
  `Promise::wait()` burns its budget in ~39 ms). → platform-implementation-notes.md.
- **FreeRTOS:** `APP_TASK_STACK` 64 KB (inline executor arena on stack) → "Invalid mbox" otherwise;
  IP-seeded `srand()`; poll-task priority ≥ 4; manual action server needs
  `try_handle_get_result()`. → platform-implementation-notes.md.
- **Tier and transport priorities land in ONE scheduler — they now share ONE vocabulary,
  RAW FreeRTOS (issue 0623, FIXED).** `FreertosScheduling`'s `zenoh_read_priority` /
  `zenoh_lease_priority` / `poll_priority` are raw `0..configMAX_PRIORITIES-1`, the same
  units `[tiers.*.freertos] priority` is written in. Shipped defaults: app **3**,
  transport band **4/4/4** — transport above the app, no collision. The normalised 0–31
  scale WAS the defect (a tier written as 5 against a transport reading "16" was above it,
  not below); `to_freertos_priority` survives as THE single conversion for anyone still
  supplying normalised values, and nothing in the default path calls it. Do NOT "just
  lower the read task below the tiers": that is the config that starved the RX drain and
  froze `rt-eval`'s island 1–3 s on lwIP retransmission. Both orderings are legitimate;
  choosing by accident is not. Boot prints `report_tiers_above_transport` when a tier meets
  the band floor (`min(read, lease, poll)`). Same defect phase-364 W5 fixed one layer down
  in the platform ABI.
- **Zephyr POSIX:** raise `CONFIG_MAX_PTHREAD_MUTEX_COUNT` (zenoh-pico needs ~8+; default 5 fails
  with -80). → platform-implementation-notes.md.
- **Zephyr's pthread mutex/cond pools are per-OBJECT, so a mutex-per-entity library makes them a
  cap on WORKLOAD size** — cyclone (3 per writer + 1 per addrset) exhausted 16384 slots joining a
  40-participant Autoware graph and died as an anonymous `abort()` 20 s later (issues 0371/0496).
  Cyclone on Zephyr now uses a NATIVE ddsrt sync backend (embedded `k_mutex`/`k_condvar`, zero
  pool slots): `DDSRT_WITH_ZEPHYR` picks the types, `nros_rmw_cyclonedds.cmake` swaps the TU —
  **both halves move together or the layouts disagree.** NOTE `k_mutex` is recursive where a
  pthread NORMAL mutex deadlocks, so a self-relock bug hangs natively and passes on Zephyr.
  → platform-implementation-notes.md.
- **Zephyr zsock serializes send/recv per-fd:** `Z_CONFIG_SOCKET_TIMEOUT` must stay 100 ms (5 s
  starves tx → lease death, silent session drop); intra-image pub→sub needs
  `Z_FEATURE_LOCAL_SUBSCRIBER=1`. → platform-implementation-notes.md (issues 0129/0139).
- **NuttX spin uses `sem_timedwait`** (pthread condvar hangs). → platform-implementation-notes.md.
- **NetX Duo BSD `SO_RCVTIMEO` takes `nx_bsd_timeval*`, not `INT` ms** (deadlock otherwise).
- **A task's `stack_bytes` is a FLOOR the PORT raises, never a size the caller can get right**
  (issue 0667). Every port's minimum differs — `PTHREAD_STACK_MIN`, `configMINIMAL_STACK_SIZE`,
  `TX_MINIMUM_STACK` — and on POSIX it differs by ARCHITECTURE (16384 on glibc/x86_64, 131072 on
  glibc/aarch64), so no portable caller can name a legal number. Refusing a below-floor request
  killed `Executor::signal_fd()` on every Linux host for two phases (the worker asks 8192);
  FreeRTOS/ESP-IDF/ThreadX had the quieter form, forwarding it to a task that overflows later.
  Same "ask, do not assume" as the storage probes (0570). Zephyr honours none of it and says so.
  → platform-implementation-notes.md.
- **smoltcp multicast:** join the GROUP addr, not `0.0.0.0`; LAN9118 needs promiscuous in QEMU.
  → platform-implementation-notes.md.
- **QEMU:** `-icount shift=auto`; use `nros_tests::qemu::qemu_system_arm_cmd()`. →
  [docs/reference/qemu-icount.md](docs/reference/qemu-icount.md).
- **Embedded Cyclone:** transient samples use `ddsrt_{malloc,calloc,free}`, never libc — RTOS heap
  is separate. → [docs/reference/cyclonedds-known-limitations.md](docs/reference/cyclonedds-known-limitations.md).
- **The cyclonedds fork is pinned to the cyclonedds ROS SHIPS (0.10.5), not upstream** — an image
  must speak the same Cyclone as the host's `rmw_cyclonedds_cpp`, so upstream `master` (11.x) is not
  a rebase target and an upstream PR cannot retire the delta before a distro migration (issue 0507,
  same drift class as 0609). The 15 carried commits are enumerated in
  [docs/reference/cyclonedds-fork-delta.md](docs/reference/cyclonedds-fork-delta.md) — add a row
  there when the fork gains one. Before believing ANY claim about what the fork carries, run
  `git remote prune origin && git fetch --unshallow origin`: stale remote-tracking refs and the
  shallow default checkout each make the pin read as unfetchable, which cost time three times.
- **XRCE:** flush `uxr_buffer_request_data` immediately; reliable `STREAM_HISTORY ≥ 2`.
  → platform-implementation-notes.md.
- **Zephyr Rust allocator is picolibc `malloc`** — size `CONFIG_COMMON_LIBC_MALLOC_ARENA_SIZE`
  (default 16 KB; executor backing alone needs ~75 KB), NOT `CONFIG_HEAP_MEM_POOL_SIZE`.
  → issue 0163 (archived).
- **The SDK store ACCUMULATES, so a stale Corrosion can shadow the pin you just installed**
  (issue 0500) — prefixes are enumerated newest-version-first (`COMPARE NATURAL ORDER
  DESCENDING` / `sort -Vr`) precisely because `find_package` takes the FIRST that resolves,
  and both provisioning paths print success either way. Corrosion `< 0.6.0` shares one
  `cargo/build` across workspace roots ⇒ duplicate `#[no_mangle]` ⇒ `mixed` cannot link.
  **Read the configure's `nano-ros: Corrosion <ver> via <origin>` line — never infer the
  version from having run the installer** — the ordering gate that used to back this
  was retired with the globbed prefix (phase-365 W3a), so the printed line is the only
  evidence. And READ IT: the resolver's PATH fallback could not fire for two phases
  (`find_program` no-ops on an already-defined var), so every configure that did not
  pass `-D_NANO_ROS_CODEGEN_TOOL` — which is the book's documented user flow — cloned
  Corrosion from git and FAILED OFFLINE while the store held it. Gate:
  `check-cmake-find-program-shadowed` (issue 0726).
- **A cargo `--target-dir` serves exactly ONE workspace root (issue 0616)** — `-C metadata`
  includes the path SPELLING a crate was reached by (a member is recorded relative to its
  root, an external path dep absolutely), so two roots sharing a directory get two units of
  every shared crate, identical in features/deps/profile and differing only in the `path`
  fingerprint field. `nros-platform` holds the tree's one `#[global_allocator]` (0594), so
  both copies define it, and a TRANSITIVE lookup (no `--extern`, only `-L dependency=`) may
  bind either — the failure is intermittent while the cause is permanent. `cargo tree` cannot
  see this: it reports ONE workspace. Sharing never bought anything — units keyed by that
  spelling can't be reused across roots. Derive the dir from `cargo locate-project
  --workspace`, never a path-prefix test (`packages/cli` is a separate workspace INSIDE the
  repo); a second root claiming a claimed dir is a configure FATAL_ERROR. Same class as 0500
  one lane over (Corrosion sharing `cargo/build` ⇒ duplicate `#[no_mangle]`); `mixed` is the
  entry that catches it both times.
- **A Kconfig knob reaches the Zephyr C lane and NOT the RUST one** — `nros_cargo_build.cmake`
  publishes knobs with `set(ENV{…})`, which only touches the configure-time process; the C lane
  re-bakes them into its command (`cmake -E env`), zephyr-lang-rust's `rust_cargo_application`
  builds its own and inherits nothing. So every Zephyr Rust image compiled crate DEFAULTS
  whatever Kconfig said — and when the two halves disagree it is also an 0135 ABI split
  (`MAX_QUERYABLES` 16 in the cmake TU, 8 in the cargo one). Build scripts resolve knobs with
  `nros_zephyr_build::knob_usize(env, CONFIG_key, default)` (reads `$DOTCONFIG`); gate:
  `check-kconfig-knob-forwarding`. → issue 0460.
- **A pool's FLOOR belongs to the consumer that names the knob, never to the shared
  derivation** (issues 1015 + 1033, both measured). A derived count is the image's
  DEMAND and zero is a legitimate demand; whether zero is a legal SIZE is a property
  of the storage: `ZPICO_MAX_QUERYABLES=0` left a board transmitting nothing for 15 s
  with no diagnostic (fixed C array), while `XRCE_MAX_SUBSCRIBERS=0` is worth 33,296
  bytes of heap a slot and is the answer. One `EntityInventory::derive` feeds BOTH, so
  1015's floor — landed in the derivation a day before 1033's fix — silently defeated
  it, and every knob gate stayed green because the number was derived correctly and
  delivered faithfully. Floor at the pool (`c_array_pool_floor` /
  `_nros_c_array_pool_floor`), keep `#if X < 1 / #error` beside the array as the
  backstop that binds a producer neither reaches. Gate: `check-c-array-pool-floors`
  (also refuses an unruled new one; the 15 still unruled are issue 1131).
- **A service server IS a zenoh queryable** — `[param_services]` (6) + `[lifecycle]` (5) claim
  eleven slots before the app declares anything, against `ZPICO_MAX_QUERYABLES` = 8 embedded.
  Raise `CONFIG_NROS_MAX_QUERYABLES`; the table is a static array, so the default stays small.
  → issue 0460.
- **The interop DDS bus is pinned to LOOPBACK by a profile FILE the harness writes, not
  by a variable you export** (issue 1009) — `nros_tests::dds_isolation` generates the
  Fast-DDS / Cyclone profiles into a per-PROCESS dir and hands them out as
  `FASTRTPS_DEFAULT_PROFILES_FILE` / `CYCLONEDDS_URI`, through
  `env_exports_for_rmw()` in every `ros2_env_setup_*` and `apply_fastdds_profile()` on
  the XRCE Agent's own `Command`. `ROS_LOCALHOST_ONLY=1` is NOT this and does not work:
  it reaches ROS processes only, so it isolates one side of a pair and measures **0/15**
  with EMPTY output — the Agent is a bare Fast-DDS app and ignores it. **Pin both sides
  or neither; half is no discovery**, and a whitelist profile on both is 15/15. Never
  combine the two: a custom `userTransports` profile DEFEATS `ROS_LOCALHOST_ONLY`, which
  is what confounded the issue's batch E. `useBuiltinTransports` must stay FALSE —
  `userTransports` ADDS to the builtin set, so leaving it true keeps the unrestricted
  transport beside the whitelisted one and isolates nothing while reading as if it does.
  Without the pin a foreign `add_two_ints_server` on ANOTHER HOST failed our XRCE service
  cell 4 of 15 and cost issue 0741 five wrong diagnoses. Opt out: `NROS_DDS_ALLOW_LAN=1`.
- **Manual native_sim pair repros need distinct `--seed`** — unseeded processes share the test
  entropy source → identical GUIDs/ports → discovery sees the peer as itself → false-negative
  "no delivery". The test harness seeds automatically; hand-run repros must too. → issue 0157
  (archived).
- **Never clang-format `cmake/templates/*`** — reflow splits `@VAR@` configure_file tokens
  (`@SYM @_create`) → generated TU fails "stray '@'". `.clang-format-ignore` guards; format
  recipes already exclude them. → issue 0159 (archived).
- **RMW + platform C ABI: the C headers ARE the SSoT (RFC-0054)** — Rust consumes
  COMMITTED bindgen output (`packages/core/{nros-rmw-cffi,nros-platform-cffi}/src/generated.rs`).
  Header edit ⇒ run `scripts/gen-abi-bindings.sh` (pinned bindgen-cli 0.72.1) + commit both;
  `check-abi-bindings` gates staleness. Never hand-edit `generated.rs`; vtable slots are
  `Option<fn>` (C nullability); no layout tests in generated code (host-64-bit literals
  break 32-bit targets).
- **Hand-mirrored FFI structs drift on append** (QoS `tx_express`, `callback_group` — 3×):
  mirror-only TU passes a SHORTER struct by value → tail field garbage. Gated:
  `check-ffi-struct-mirrors` (push lane) + cross-include TU in `check-c`. Include order is
  one-way: `nros_cpp_ffi.h` BEFORE `component.h`. → issue 0160 (archived).
- **zpico shim + zenoh-pico library MUST share the generated zenoh config** — flag-gated struct
  fields (`Z_FEATURE_LOCAL_QUERYABLE`…) make mismatched TUs a silent ABI break (queries went
  session-local-only). `build_c_shim` injects `ZENOH_GENERIC` + the OUT_DIR config. → issue 0135
  (archived). Local fixture binaries embed the shim — rebuild fixtures after zpico config changes.
- **One FORMULA is not enough — the INPUTS are derived twice too (issue 1025).** The group key
  is a function of (platform, cargo args, env); `just esp32 build-qemu` called the shared helper
  with the platform and `"" ""`, the producer passed the ROW's, and supplying two of the three
  constants a different answer. It agreed until the esp32 rows gained
  `env = { ZPICO_MAX_QUERYABLES = "2" }`, after which NO esp32 flash image could be packed. Not
  silently: the nightly esp32 lane went red and printed this exact path every night. It survived
  because the PUSH gates are green either way — the packer runs in `build-all`, which only the
  schedule reaches — and because a lane already red reports its FIRST failure, so a second fault
  behind it (issue 1070) stayed invisible until this one was fixed. A consumer that did not itself run the build names the ROW:
  `nros_fixture_row_artifact_dir_by_id <row-id>`. Gate: `check-fixture-artifact-dir-inputs`
  (a packer of a lane-built artifact must derive its row's args and env from the manifest, not
  supply them); acceptance is still a BUILD, never a gate.

- **A CONFIGURE-time emitter has no `DEPENDS` to carry its tool — put the tool in
  `CMAKE_CONFIGURE_DEPENDS` (issue 1018).** `execute_process()` has already run by
  the time ninja decides anything, so the freshness of anything it emits reduces to
  *does a configure happen*. Four sites emit at configure time and #182 registered
  the CLI at exactly one of them, inline; the Zephyr interfaces generator's own
  `IS_NEWER_THAN` predicate on the tool was therefore unreachable on an incremental
  build (measured: `build-rust-talker-zenoh`'s `RERUN_CMAKE` edge, 3592 inputs, none
  under `packages/cli`), so a single-example Zephyr image kept museum generated code
  after a `nros` rebuild. One spelling now: `nros_codegen_tool_reconfigure()`, gated
  by `check-codegen-tool-reconfigure`. The BUILD-time lane is fine — its
  `add_custom_command` names the tool in `DEPENDS`, and `restat = 1` plus codegen's
  write-if-changed keep an emitter-identical rebuild from cascading.
- **A lib reached through a raw `-Wl,...` link FLAG gets no rebuild edge (issue 0475)** — CMake cannot
  see a file inside a flag string, and `add_dependencies()` only adds build ORDER, which ninja renders
  `||` (order-only): "must exist before linking", never "relink when it changes". The RMW backends are
  whole-archived exactly that way, so a backend edit rebuilt the archive and left every C/C++ example
  binary holding the OLD code — museum binaries by construction, clearable only by `rm -rf` on the build
  dir (~687 s per Cyclone leaf). Fix is `LINK_DEPENDS` on the consuming target (`nano_ros_link_rmw`),
  which adds the file edge without touching the link LINE — do NOT also `target_link_libraries()` the
  archive: that reorders ld's single pass and breaks the whole-archive group (`undefined reference to
  ddsrt_*`). Verify with `ninja -C <build-dir> -t query <exe>`: the `.a` must appear under `|`, and a
  `touch` of a backend source must relink.
- **Every cargo command cmake emits passes `--target`, HOST INCLUDED (phase-340 W3)** —
  `--target <host-triple>` and no `--target` are different `-C metadata` identities that
  share nothing, not even sccache entries (measured 0 hits / 62 misses). Corrosion hardcodes
  the flag and is upstream, so it is the fixed point; resolve the triple with
  `_nros_resolve_rust_target()` (never `Rust_CARGO_TARGET` directly — it is a normal var that
  does not cross `add_subdirectory()`, which is phase-155's wrong-arch link). Gate:
  `check-cargo-target-spelling`.
- **A build with no `[[fixture]]` row has no COORDINATE, so it gets no shared cargo group
  (phase-340 P2)** — that is how a bare `cd <leaf> && cargo build` in `build-examples` kept
  re-creating `examples/**/target/` two minutes after the group dir was written, on a platform
  already migrated. Give such a build a row (preferred) or derive its dir from
  `nros_fixture_target_dir_flag` + `nros_fixture_row_artifact_dir` — **never a literal, and move
  the test-side locator in the SAME commit** (#393). `examples/**/target-*/` is ignored per-LEAF (`/target*/` or a named entry in the leaf's own `.gitignore`), NOT globally — a new sibling dir needs its leaf's rule added;
  a plain `target/` is not, so it is gated: `check-example-leaf-target-dirs`. A PLATFORM's fixture
  profile is `nros_cargo_platform_profile` — the staleness probe must use it too, or it rebuilds
  into a second profile dir and reports permanent false-STALE. Residue → issue 0488.
- **Never `cargo:rerun-if-env-changed` on a PATH variable — watch the CONTENT (issue 0491)** —
  cargo compares an env value as TEXT, and one directory has three spellings here: a leaf
  `.cargo/config.toml`'s `relative = true` (one per leaf), `just/sdk-env.just`'s absolute
  export, and unset. Per-leaf `target/` dirs hid it; a shared phase-340 group put them in one
  fingerprint namespace, so every sibling — and every build-vs-probe pair — rebuilt the board +
  zpico chain forever (6 units per probe on freertos AND threadx). Resolve through
  `nros_build_paths::{env_or_repo_path,env_path,watch_path}`; a `rerun-if-changed` path is
  safe per-leaf (cargo reads it back from the STORED output, never re-resolved). The rule has
  TWO producers — Rust literals AND the `rerun_if_env_changed` lists in
  `config/*/nros-platform.toml` — and fixing one leaves the other running. Gate:
  `check-path-env-fingerprints` (both producers, self-testing).
- **cmake `include()` inside a FUNCTION drops the file's normal vars when the frame pops** —
  capture module dirs `CACHE INTERNAL` (the `_NROS_ENTRY_DIR` pattern); a plain
  `set(_X_DIR ${CMAKE_CURRENT_LIST_DIR})` broke every freertos ws member's `configure_file`
  (287-W6; posix hid it). And `find_program` HINTS beat PATH — a stale `~/.nros/bin` binary
  shadows the activate.sh CLI; use `PATHS` for fallbacks. → AGENTS.md CMake Pitfalls.
- **Case-normalize enum-ish cmake args** (`string(TOUPPER)`) — the ament verbs pass lowercase
  `cpp`; a case-sensitive `STREQUAL "CPP"` silently takes the C branch. → AGENTS.md CMake Pitfalls.
- **Lockfiles change ONLY when a dev means it** (issues 0359/0378). `Cargo.lock` is a promise
  that someone else's build resolves what yours did, so `just lock-update [crate] [version] [dir]`
  is the only sanctioned way to move one — never bare `cargo generate-lockfile`, which re-resolves
  EVERY package (26 leaf locks once moved 5388 lines as a "cleanup"). `--locked` is injected
  PROJECT-WIDE by the `scripts/bin/cargo` PATH shim (`NROS_CARGO_FLAGS`, wired in `activate.sh`),
  so a mismatch FAILS instead of silently rewriting the file. Cargo has no config/env knob for it
  (`[build] locked` is an unused key), and per-site flags would miss cmake/corrosion, which invoke
  `cargo` by NAME. Escape hatch: `NROS_CARGO_FLAGS= just <recipe>`. **Generated msg crates are the
  exception**: they are produced per host by `nros sync` from the consumer's ament install and
  never shipped, so codegen emits a CONSTANT `version = "0.0.0"` (the ament version moves to
  `[package.metadata.nros] ament_version`) — otherwise a committed lock asserts which ROS install
  built it and every other host reads as drift.
- **Rust leaf `.cargo/config.toml` is `nros sync`-managed (RFC-0048 W9)**: one
  `include = ["…/nros-patch.toml"]` (central, gitignored, absolute paths) + leaf-local
  `generated/*`/platform patches. Never hand-edit; moved checkout → re-run `nros sync`. Central
  membership = only crates registry-named in EVERY graph (else cargo "unused patch" warnings).
  **Sync's `[patch.crates-io]` rows split by ORIGIN, not by "sync wrote it" (issues 0457/0463):**
  IN-REPO rows (`nros-log`, board crates, `mps2-an385-pac` — relative paths, identical in every
  checkout) stay INLINE in the tracked `config.toml`, tagged `# nros-managed`; only `generated/`
  rows go to the GITIGNORED sidecar `.cargo/nros-managed-patch.toml`, whose `include` is written
  only when that file is. So a leaf with no message dep has no sidecar, no include, and resolves in
  a fresh clone with NO sync — only ament-derived content sits behind sync. (0457 moved the WHOLE
  set to the sidecar; that stranded every leaf on `no matching package named 'mps2-an385-pac'`,
  an in-repo patch a clone needs.) The authored half (`[build] target`, a QEMU `runner`, link
  rustflags, a user `libc` patch) stays tracked because a clone cannot regenerate it. Corollary,
  gated by `check-cargo-config-tracked`: **a tracked config must never patch an uncommitted
  `generated/` tree** (`packages/interfaces/*` are exempt — they commit theirs). An out-of-tree
  consumer keeps everything INLINE: no `include` outside this checkout (#272).
  **After a sync the tracked config legitimately gains the sidecar `include` on disk — NEVER commit
  that line** (`git add -u` scoops it up; it did twice). The invariant is about the COMMITTED blob,
  so the gate reads `git show HEAD:<path>`, not the worktree. **A missing `include` target is a HARD cargo error during MANIFEST PARSE — not the
  silent drop #272 and #457 both assumed (issue 0463).** Both generated targets are gitignored, so
  before `nros sync` these leaves cannot even be READ (`cargo metadata` fails too, four frames deep,
  never naming sync). Guarded by `_require-leaf-includes`; `check-cargo-config-tracked` also rejects
  an include naming a target no generator writes. → AGENTS.md Rust Consumption.
- **The CLI freshness closure is GENERATED, not walked** (issue 0627). `packages/cli/
  cli-source-dirs.txt` names the in-repo dirs outside `packages/cli` that the `nros` binary
  compiles; `source_stamp.rs` hashes those plus all of `packages/cli`, and a stale CLI re-stales
  everything keyed on it. Change a CLI dep ⇒ `python3 scripts/gen-cli-source-dirs.py` + commit;
  `check-cli-source-dirs` gates drift and names the direction. Never hand-edit it and never go
  back to a textual `path =` walk — that one was wrong BOTH ways at once (23 dirs where cargo
  resolves 8): blind to `workspace = true` (so `nros-core`/`nros-rmw` edits left `setup-cli`
  skipping the rebuild while reporting success) and blind to `optional = true` (so 17 crates the
  CLI never compiles — every platform port, `nros-node`, `nros-log` — re-staled it). A MISSING
  list makes `source_stamp` return `None` (⇒ rebuild), never a stamp over a smaller closure.
- **Parallel agent sessions push to `main`** — **reserve issue ids with `just issue-new <slug>`,
  never by reading the highest number.** Reading-then-writing is a race that has collided seven
  times (0367→0372→0377 collided TWICE, the second time while renumbering the first). The tool
  claims `refs/issue-ids/NNNN` on origin, which git rejects if it already exists; the `pre-push`
  hook (`just setup-hooks`) refuses to push a duplicate even if the tool was skipped. Expect
  ledger conflicts NO LONGER (issue 0884 — the generated list is `docs/issues/open.md`,
  which is GITIGNORED: `merge=union` fixed it locally and not on GitHub, whose
  server-side merge/rebase does not run `.gitattributes` drivers, so an artifact
  nobody commits is the fix that reaches the queue. Regenerate with
  `python3 scripts/gen-issue-index.py`); write full background logs to files (`| tail` hides
  the real error). **Same race, same fix, third series: `just phase-new <slug>`** for work needing
  its OWN phase number (two sessions opened `phase-350` for unrelated work on 2026-08-13; the later became 352). NOT for
  a doc joining an existing effort — a phase number is deliberately not unique per file (26 of 342
  carry several), so there is no phase uniqueness gate and adding a doc to an existing phase reuses
  its number. → AGENTS.md Multi-Session Pitfalls.

## Verification
Kani (bounded harnesses, `just verify-kani`) + Verus (unbounded proofs, `just verify-verus`).
Patterns + the `verify = true` footgun → [docs/guides/verus-verification.md](docs/guides/verus-verification.md).
