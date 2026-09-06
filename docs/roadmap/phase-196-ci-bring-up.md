# Phase 196 — CI bring-up + hardening for nano-ros

**Goal.** Give nano-ros a reliable, **live-validated** CI surface. Today CI is
thin and partly broken: the root `ci.yml` only fans out one crate on one target,
`zephyr-dual-line.yml` had never run live (7 stacked bring-up bugs), and several
workflows assume host state (submodules, ROS, SDKs) the GitHub runners don't
have. This phase finishes the Zephyr dual-line bring-up, audits the other
workflows for the same class of gaps, and codifies the provisioning conventions
so a new workflow works on its first push.

**Status (2026-09-04). Every work item is DONE (196.1-196.9, 33 of 34 boxes
checked); the phase is held open by ONE acceptance criterion that names a
workflow deleted fifteen months of commits ago.** The 2026-05-28 line below said
"in progress ... one product skew (codegen CLI) remains" and none of that is
true now: the skew was resolved at the tool level (see the handoff note), the
196.7 [P2] item's three boxes are all checked, and 196.3's audit is complete.

**The CI topology this phase was written against no longer exists.** phase-253
(`682706dbf ci: merge 13 workflows into 6 tier files`) consolidated the lot, and
ALL TWELVE workflows named in the acceptance criteria are gone: `ci.yml`,
`zephyr-dual-line.yml`, `platform-ci.yml`, `lint.yml`, `host-unit.yml`,
`codegen-convention.yml`, `sdk-index-gate.yml`, `nros-acceptance.yml`,
`dep-chain.yml`, `scaffold-journey.yml`, `embedded-feature-unification.yml`,
`colcon-parity.yml`. Read every workflow filename below as historical record;
the live set is `gate.yml`, `queue.yml`, `nightly.yml`, `post-submit.yml`,
`run-matrix.yml` and friends.

**The one open criterion is still a real question, and it now has no answer.**
"`zephyr-dual-line` is green end-to-end on both lines" survived the
consolidation as the zephyr matrix inside `nightly.yml`, on its own `0 5` cron —
but every zephyr job in the last EIGHT consecutive scheduled runs is `skipped`,
so the lane reports neither green nor red. Filed as
[issue 1029](../issues/archived/1029-zephyr-nightly-never-produces-a-verdict.md),
now RESOLVED: the cron gate was open all along and the `needs: changes` edge —
whose own `lane` dependency is skipped on the 05:00 cron — was skipping the
jobs. The three zephyr jobs no longer carry that edge.
This criterion cannot be met or refuted until the first 05:00 run of the fixed
workflow reports, and it should be tracked THERE rather than holding a phase
open whose own work is finished.

> **Post-Phase-218**: The CLI-skew install path discussed below
> (`scripts/install-nros.sh` → `~/.nros/bin/nros`) was retired by
> Phase 218 (`19d1d29ba`). The canonical install is now in-tree:
> `git submodule update --init packages/cli && just setup-cli &&
> source ./activate.sh` (binary at `packages/cli/target/release/nros`).
> References below are historical record.

**Handoff note (2026-05-29).** Findings from another line, relevant to the open
items — not worked here (196 is owned elsewhere):
- **`nros codegen` CLI skew (the "one product skew" above) — resolved at the
  tool level.** It is *not* a missing feature: `nros 0.3.1` (tag `nros-v0.3.1`,
  nros-cli) ships the `nros codegen` subcommand (`cmd::codegen`). The breakage
  seen locally was a **stale install** — `~/.cargo/bin/nros 0.2.0` (pre-`codegen`)
  shadowing PATH, which also made `scripts/install-nros.sh` skip. Fix that bit:
  remove the stale cargo binary, `scripts/install-nros.sh` (installs 0.3.1 →
  `~/.nros/bin`); `.envrc` now puts `~/.nros/bin` on PATH (committed this
  session). **CI implication:** lanes that run `nros codegen` must install
  0.3.1+ via `install-nros.sh`, not rely on a cargo-installed `nros`. Verified:
  fresh-configured `examples/native/cpp/listener` builds end-to-end + the C/C++
  `native_api` talker/listener e2e passes once 0.3.1 is on PATH.
- **`packages/codegen` (the nros-cli clone) is gone from the worktree.** 196.7's
  `nros new` scaffold edit lives in **nros-cli** (separate `NEWSLabNTU/nros-cli`
  repo), not this tree — re-clone / edit there.
- **196.7 README install line:** the suggested `cargo install --git
  …/nano-ros nros-cli` is wrong — `nros` ships from the standalone nros-cli repo
  as a **prebuilt** binary; the source-release install is `scripts/install-nros.sh`
  (or `cargo install --git …/nros-cli`). Document that, not a nano-ros cargo-install.

**Priority.** P2 — no product capability depends on it, but green CI is the gate
for trusting every other phase's "verified" claim, and broken-by-default
workflows train contributors to ignore CI.

**Depends on.** Phase 180 (the dual-line workflow + recipes), Phase 195 (the
in-flight `nros codegen` CLI migration — the one open item rides on it).

---

## Background — the dual-line bring-up (what this phase already did)

`zephyr-dual-line.yml` (Phase 180.A Task 10) shipped CI-only, explicitly **never
run live**. The first live runs failed 100%, in stacked layers (each CI pass
≈8–16 min revealed the next). All landed on `feature/phase-172`:

1. **Workspace created inside the SDK dir.** `WORKSPACE_DIR` was relative;
   `install_sdk` `cd`s into the SDK and never returns, so the later relative
   `cd "$WORKSPACE_DIR"` landed in `scripts/zephyr/sdk/…`. Only triggers on a
   fresh install (SDK build runs) — why local cached-SDK runs passed.
   Fix: normalize `WORKSPACE_DIR` to absolute while cwd is the repo root
   (`scripts/zephyr/setup.sh`).
2. **`cortex-a9-rust-patch.sh` hard-fails on 4.4.** Zephyr 4.4 relocated the
   Zynq-7000 SoC (`soc/xlnx/zynq7000/xc7zxxxs/soc.c`). Fix: gate the patch to the
   3.7 manifest in `setup.sh`; make the patch version-tolerant (missing SoC →
   warn+skip, not `exit 1`).
3. **Stray committed `zephyr-workspace` symlink** (→ `../nano-ros-workspace`,
   broken on CI) — `mkdir -p` errors on a non-dir. Untracked it (it was already
   gitignored); guarded the mkdir with `[ -d ] ||`.
4. **Zephyr 4.4 needs Python ≥3.12.** `provision-py312-venv.sh` requires `uv`
   (no fallback); runner lacked it. Fix: `astral-sh/setup-uv@v5` step.
5. **Submodules not initialized + a needless XRCE-agent build.**
   `actions/checkout` inits no submodules; the 3.7 cyclone patches need
   `third-party/dds/cyclonedds`, the zenoh build needs
   `packages/rmw/zenoh/zpico-sys/zenoh-pico`, and the setup recipe's common tail ran
   `just xrce setup` (a Fast-DDS superbuild) on every job though the workflow
   only builds zenoh. Fix: init just the needed submodules; gate the agent build
   behind `NROS_ZEPHYR_SKIP_XRCE_AGENT` (set workflow-wide).
6. **`packages/codegen` submodule not initialized** — `build-one` builds the
   host codegen tool from it. Added to the init step.
7. **No ROS 2 on the runner.** The interface codegen resolves `std_msgs`'s
   `msg/*.msg` via `AMENT_PREFIX_PATH` from a sourced ROS 2. Fix: jammy runner
   (Humble baseline) + `ros-tooling/setup-ros@v0.7` + `source
   /opt/ros/humble/setup.bash` before each build.

Result: setup passes on **both lines**; builds reach the interface codegen.

---

## Work Items

### 196.1 — [DONE] Finish the dual-line build: codegen CLI skew
The build now fails at the codegen call:
`nros-codegen failed for std_msgs (exit 2): error: unexpected argument '--args-file'`.
The codegen CLI moved to a `nros codegen` subcommand (Phase 195 `27e9be2`/
`07e3339`, "make `nros codegen` canonical") and 195.D switched in-tree
consumers — but **`zephyr/cmake/nros_generate_interfaces.cmake` still invokes
`nros --args-file …` without the `codegen` subcommand** (lines 305/308). Two
coupled facts:
- the cmake consumer was missed by the 195.D consumer-switch;
- the `packages/codegen` submodule pointer is **drifting** (superproject records
  `624e5bc6`, a local working tree sat at `860f301`).

- [x] **DONE.** `zephyr/cmake/nros_generate_interfaces.cmake` now invokes
      `nros codegen --args-file …` / `nros codegen --language cpp --args-file …`
      (the canonical `nros codegen` subcommand). Verified `nros codegen
      --args-file` parses. It was the only superproject `--args-file` consumer
      (the canonical cmake module lives in `packages/codegen`).
- [x] **Submodule pointer reconciled + the 195.D codegen-workspace bug fixed:**
      195.D deleted the `nros-codegen-c` crate dir but left it in
      `packages/Cargo.toml` `[workspace].members` → the whole codegen workspace
      failed to load (broke every `nros` CLI build). Dropped the stale member
      (codegen `00a1b2d`, pushed); bumped the superproject pointer. nros builds
      again.

### 196.2 — [DONE] `nros codegen` consumer-coverage check
195.D switched consumers to `nros codegen` but missed the Zephyr cmake (196.1).
A guard now makes that exact regression un-mergeable.

- [x] **DONE.** `scripts/ci/codegen-invocation-check.sh` — a static lint
      (`git ls-files` + `grep`, no toolchain/ROS) over superproject build glue
      (`*.cmake`/`CMakeLists.txt`/`*.just`/`justfile`/`*.sh`/`*.rs`, excluding
      `third-party/` + the `packages/codegen/` submodule). The signature is
      precise: any line driving the codegen tool with `--args-file` MUST carry
      the `codegen` subcommand token first; the legacy top-level `--args-file`
      (the 196.1 bug) fails. User-facing verbs (`nros generate-rust`,
      `nros generate cpp`) don't use `--args-file`, so they're untouched.
      Wired as `.github/workflows/codegen-convention.yml` (push/PR on
      cmake/just/scripts paths). Verified: clean tree OK, canonical form OK,
      injected legacy form exits 1 with the offending line.

**Files**: `scripts/ci/codegen-invocation-check.sh`,
`.github/workflows/codegen-convention.yml`.

### 196.3 — [mostly DONE] Audit/split the workflows (core-libs + per-platform)
The dual-line bugs (host assumes: submodules, ROS, SDK, Python, runner OS) are
generic. Audit each workflow with a fresh-runner lens; each must be live-run
once before being trusted:
- [x] `ci.yml` → **core-libraries lane** (DONE). **Scope decision** (maintainer,
      2026-05-29): *split CI into several parts — a core-libraries lane + one lane
      per platform, each pulling only its own tools/submodules so per-workflow
      minutes stay low.* `ci.yml` is now the **core-libs** lane: the portable
      `no_std` core crates cross-checked on bare embedded targets, fresh-runner-
      safe (no SDK/submodule deps, so no provisioning beyond a rustup target).
      Split by target (one job per target, parallel + isolated), each running a
      single `cargo check` over the compatible crates. Two targets:
      `thumbv7m-none-eabi` (atomic CAS — full set incl. `nros-rmw-cffi`) and
      `riscv32imc-unknown-none-elf` (no CAS — drops `nros-rmw-cffi`; that exact
      capability split is what the lane guards). Crates: nros-core, nros-log,
      nros-serdes, nros-params, nros-platform-api, nros-platform-cffi,
      nros-platform-critical-section, nros-rmw (+ nros-rmw-cffi on CAS targets).
      Verified both combined checks pass locally. Triggers broadened to
      `packages/core/**`.
- [x] **Per-platform lanes** (architecture recorded; buildout = follow-up). The
      split is realized structurally today by `zephyr-dual-line.yml` (the first
      per-platform build lane: pulls only Zephyr's SDK + cyclonedds/zenoh-pico
      submodules) and `dep-chain.yml` (the cheap cross-platform *resolution*
      cut — `nros setup` per board pulls only that board's tools, one ROS install
      shared). Adding a dedicated **build** lane per remaining platform
      (freertos, nuttx, threadx, esp32, bare-metal, stm32f4) on the dual-line
      pattern is follow-up work — each its own workflow scoped to its tools.
- [x] **Host unit-test lane** (DONE, 2026-05-29). Added `host-unit-tests.yml`:
      `just test-unit` (workspace unit tests — `cargo nextest --workspace
      --exclude nros-tests`) on a fresh runner. Closes the gap where **no** lane
      executed the host test suite (core-libs only `cargo check`s the no_std
      crates, dep-chain only resolves, the platform lanes build/boot fixtures) —
      a unit-level regression could land green. Fresh-runner-safe: prebuilt nros +
      `nros setup --source` for the workspace build sources the `*-sys` crates
      compile (px4-rs, zenoh-pico, mbedtls, micro-cdr, micro-xrce-dds-client) — no
      QEMU/zenohd/ROS/agent. Verified green on the runner (422/422 unit tests).
- [x] **Submodule provisioning is nros-driven, not hand `git submodule`**
      (DONE, 2026-05-29 — maintainer directive: "use nros tools to pull
      toolchains and submodules to simulate user workflow; if missing, fix nros
      and config"). Every lane now pulls sources the way a user does — build the
      `nros` CLI, then `nros setup <board>` / `nros setup --source <name>` —
      with the *only* hand-init being `packages/codegen` (the CLI bootstrap;
      chicken/egg). Index gaps fixed as "missing config": added
      `[source.px4-rs]` (core-libs workspace-load dep) and
      `[source.cyclonedds-src]` (the in-tree cyclonedds fork the Zephyr setup
      patches; distinct from `[tool.cyclonedds]`). `ci.yml` provisions px4-rs;
      `zephyr-dual-line.yml` (all 3 jobs) provisions zenoh-pico + cyclonedds-src;
      `dep-chain.yml` was already board-driven. Verified each `--source` resolves
      + the structure gate passes. Rule codified in `ci-conventions.md`.
- [x] `deploy-book.yml` — confirmed building (GREEN on `88488d296`, 2026-05-29);
      `cancel-in-progress: false` concurrency added (196.5) so a Pages deploy
      isn't interrupted mid-flight.
- [x] `sdk-index-gate.yml` — DONE (2026-05-29). `verify-index.py` gained an
      **offline structure pass** (always runs, alongside the network `dist`
      hash check; `--structure-only` for local/CI-without-network): (1) every
      `[board.*]`/`[rmw.*]` (Phase 191.6) package ref resolves to a defined
      `[tool]`/`[source]`/`[gated]` entry (static mirror of `SdkIndex::validate`,
      no `nros` build); (2) `[source.*]` (195.B) coherence — submodule mode needs
      `dest`, clone mode needs `ref`+`dest`; (3) **index↔`.gitmodules` drift
      guard** — each `[source.*].submodule` path must be a real submodule and a
      declared `git` URL must match `.gitmodules`. Gate now also triggers on
      `.gitmodules` changes. Verified: passes on the real index; catches
      undefined-ref, clone-missing-ref, missing-submodule-path, and URL-drift.
- [x] `zephyr-dual-line.yml` — 196.1 fixed. **SDK caching reverted** (added then
      backed out, 2026-05-29): caching `scripts/zephyr/sdk` restores the SDK
      *files* but not its CMake-package registration (`~/.cmake/packages/Zephyr-sdk`,
      written by the SDK's `setup.sh -c`), and `setup.sh` *skips* re-registration
      when the SDK dir is already present — so a cache **hit** left the SDK
      unregistered and `find_package(Zephyr-sdk)` failed at configure (broke
      every example, C included). A correct cache must also restore/redo the
      registration (cache `~/.cmake/packages/Zephyr-sdk`, or export
      `ZEPHYR_SDK_INSTALL_DIR`, or unconditionally re-run `setup.sh -c`) — deferred
      to a proper 196.3 follow-up. West-workspace caching likewise deferred.
- [x] **Prebuilt CI image — both lines' C/C++ green in-container (DONE,
      2026-05-29).** Superseded the file-only SDK cache: `ci/docker/zephyr-ros/`
      bakes ROS Humble + the Zephyr SDK 0.17.4 + host tools (cmake/ninja/dtc/
      gperf, uv, just, rustup+targets); `build-zephyr-ci-image.yml` publishes
      `ghcr.io/newslabntu/nano-ros-zephyr-ci:humble-sdk0.17.4`. All 3 dual-line
      jobs run `container:` against it. Result (run `26637668875`): **3.7 + 4.4
      c/cpp talker+listener all 8 green**, ~2× faster (91s image-pull replaces
      ~6–8 min of setup-ros/SDK/uv/just/rustup). Key fixes: bake **0.17.4** (4.4
      requires ≥ that; 3.7's FindZephyr-sdk is min-only so it accepts it) and do
      **not** force `ZEPHYR_SDK_INSTALL_DIR` (env forces exclusive /opt → 4.4's
      `find_package(Zephyr-sdk 1.0)` rejects the 0.17.4 release Config; left as a
      registry *candidate*, 3.7 accepts it, 4.4 falls through to its west SDK).
      Rust cells still fail on the Phase 200 rust-zephyr link/feature gap (not the
      image). The `rust-cargo-extra-args-patch` was made version-tolerant
      (Phase 202.5) so a divergent lang-rust shape no longer blocks the shared
      setup. **Files:** `ci/docker/zephyr-ros/Dockerfile`,
      `.github/workflows/{build-zephyr-ci-image,zephyr-dual-line}.yml`.

### 196.8 — [fixed, CI-confirming] Zephyr dual-line: rust/cpp example build gaps

**Two distinct root causes, both fixed (`fdcd1bb56`, `c791e4dac`):**
- **rust/\*** — `failed to load source for builtin_interfaces`: `just zephyr
  build-one` never ran `nros generate-rust` for rust examples, so the generated
  interface path-crates (std_msgs + transitive builtin_interfaces) the
  `.cargo/config.toml` patch points at didn't exist before the cargo-driven west
  build (the C/C++ paths codegen via `nros_generate_interfaces()` in cmake; rust
  had no equivalent). Fix: `build-one` runs `"$codegen_tool" generate-rust
  --force` in the example dir for `rust/*` (needs ROS sourced — AMENT). Reproduced
  the wipe→regen locally.
- **cpp/\*** — `node.hpp: fatal error: type_traits: No such file`: Phase
  189.M3.3.e added `#include <type_traits>` to nros-cpp `node.hpp`; Zephyr builds
  C++ `-ffreestanding`/picolibc (no C++ stdlib). Same class as the
  threadx-qemu-riscv64 shim (`c8a4bec2f`). Fix: add `zephyr/cxx-compat/type_traits`
  (enable_if + SFINAE is_convertible + integral_constant), which is already on the
  `CONFIG_NROS_CPP_API` include path next to the existing `<cstdint>`/`<utility>`
  shims.

Both surfaced only now because the build first had to clear provisioning + the
px4-rs workspace-load. CI run on `c791e4dac` confirming.

Original triage notes:
First full dual-line build that gets *past provisioning* (after the px4-rs
workspace-load fix, `64ca9f69a`): **C examples build green on both lines**
(`c/talker`, `c/listener` — 3.7 + 4.4), but **every `rust/*` and `cpp/*` example
fails** at the cargo build with `failed to load source for dependency
builtin_interfaces` (`run 26616922003`). Pre-existing / newly-surfaced, **not** a
provisioning regression — before the px4-rs fix every job died earlier at the
`px4-sitl-tests` workspace-load, so the rust/cpp lines had never reached their own
codegen step. The generated `builtin_interfaces` path-crate the rust/cpp examples
depend on isn't produced before cargo runs in the `just zephyr build-one`
rust/cpp flow (the C path codegens fine via `nros codegen` / cmake — 196.1). Needs
its own debugging pass on the dual-line rust/cpp interface-codegen wiring (likely
a missing `nros generate-rust` / `NROS_BUILTIN_INTERFACES_DIR` step for the
cargo-driven examples). The C line + all other lanes (core-libs, dep-chain,
codegen-convention, deploy-book, sdk-index-gate) are green.

### 196.4 — [DONE] Codify the CI provisioning conventions
- [x] **DONE.** `docs/development/ci-conventions.md` written: the "runner is a
      fresh clone" model + eight conventions with copy-paste step snippets —
      minimal submodule init (never recursive-all; *and* the don't-hand-init-what-
      `nros setup`-provisions exception), ROS provisioning (jammy + `setup-ros` +
      `source`), runner-OS-follows-distro (Humble ⇒ jammy), Python 3.12 via `uv`,
      build-the-`nros`-CLI-from-source (published bin is stale), canonical
      `nros codegen` invocation (196.2), `cancel-in-progress` concurrency (+ the
      "cancelled = dedup, not failure" note), and path-filter triggers. Plus a
      cost-discipline section (dep-chain vs full builds, SDK caching), the
      fail-loud precondition rule, and a worked-examples table mapping each
      convention to a live workflow.

### 196.5 — [DONE] Workflow trigger hygiene
`zephyr-dual-line` (and others) trigger on `packages/**` — nearly every push.
Combined with a broken workflow, that's constant red. Keep broad triggers (core
changes do affect Zephyr), but every workflow now dedups in-flight runs.

- [x] **DONE.** Audited all six workflows for `concurrency: cancel-in-progress`.
      Three were missing it: `ci.yml` + `sdk-index-gate.yml` now cancel in-flight
      per `${{ github.ref }}`; `deploy-book.yml` uses `group: deploy-book` with
      `cancel-in-progress: false` (a Pages deploy must not be interrupted
      mid-flight — serialize, don't cancel). `dep-chain`, `codegen-convention`,
      `zephyr-dual-line` already had it. All keep `workflow_dispatch` + scoped
      `paths:`; broad `packages/**` on the platform lanes is intentional and the
      concurrency dedup keeps it cheap.

### 196.6 — [DONE] Per-platform **dependency-chain** validation (light, not full builds)

**Distribution model (confirmed):** nano-ros ships as a **source release +
prebuilt host toolchains** (`nros setup` fetches the toolchains; the crates are
consumed from the source tree, **not** crates.io). So the dep chain per platform
is: `example → nros (path) + <backend> (path) + generated std_msgs (path, made
by codegen) + the prebuilt host toolchain (nros setup)`.

The goal of this CI is to **prove that dep chain resolves for every
`(platform, rmw)`**, *not* to run the heavy full-build matrix (that stays in the
sparse `just build-all` / `zephyr-dual-line` lanes). Per `(board, example, rmw)`:

- [x] **Toolchain side:** `nros setup <board> --rmw <rmw> --dry-run` resolves the
      right prebuilt host tools (validates the `[board.*]`/`[rmw.*]` index wiring,
      Phase 191.6) — instant, no fetch.
- [x] **Codegen step:** run `nros generate-rust` (or the build's codegen
      pre-step) so the example's `generated/std_msgs` path-crate exists.
      **Gotcha:** the examples declare `std_msgs = "*"`, a *generated* crate; with
      no codegen run, even `cargo tree` fails (`failed to select std_msgs … crates.io`).
      Codegen needs ROS-sourced `.msg` defs (`AMENT_PREFIX_PATH`), so this lane
      installs ROS like the dual-line.
- [x] **Crate/feature side:** `cargo tree --target <triple> --no-default-features
      --features <combo> -e features` (resolution only — **no compile**) proves
      the feature graph pulls the right backend/platform crates, unifies, and the
      target-cfg deps line up. This is the cheap "dep chain correct" check.
- [x] Matrix over the **board × rmw** cells from `examples/README.md` (the
      authoritative triple list). Each cell is seconds, so all platforms fit one
      cheap workflow; the full compile stays opt-in.

Distinct from the heavy lanes: this catches a broken feature/crate/toolchain
wiring (a missing optional dep, a feature that doesn't resolve on a target, a
board→toolchain typo) in seconds, without compiling every platform.

**Greened 2026-06-11.** The lane (`dep-chain.yml` → `scripts/ci/dep-chain-check.sh`)
was red: the `qemu-arm-baremetal` codegen step hit the `nros-core` ABI guard
(CLI 0.5.0 vs the example's committed `Cargo.lock` pinning 0.1.0 — known-issue
#12, the 218.J bump that never reached standalone locks). For a resolution-only
lane that's a false positive, so the check now runs `nros generate-rust` with
`NROS_SKIP_VERSION_CHECK=1`. Validated locally: `qemu-arm-baremetal` codegen
emits `generated/` and `cargo tree` resolves `nros-core 0.5.0` from the path
deps. The underlying stale-lockfile cleanup stays tracked in known-issue #12.

### 196.7 — [P2] Fix the dep convention for the source-release model
**Design home:** [RFC-0040](../design/0040-distribution-and-scaffolding-deps.md)
(no-crates.io distribution model + the `nros new` dependency convention). The
out-of-tree dep shape is RFC-0040 open question D-Q1 — resolve it there before
implementing the scaffold change below.
**Policy confirmed 2026-06-11 (maintainer):** nano-ros crates are **never
published to crates.io** — the distribution is source-release + prebuilt host
toolchains (`nros setup`). So the items below are the right direction (drop the
`version = "0.1"` crates.io shape for path/git source deps); they are not
"maybe", they are required. (The dep-chain lane 196.6 already validates the
source-release path resolution.)
The three conventions (191.6 review) must collapse to the source-release reality:
- [x] `nros new` scaffolds `nros = { version = "0.1" }` (crates.io — **the crates
      are not published**). Change it to the source-release dep: a path/relative
      reference into the installed source tree (or a documented `git =
      "https://github.com/NEWSLabNTU/nano-ros"` for out-of-tree projects) — match
      whatever the source-release layout actually places the crates at.
- [x] README install line: wrong org + contradictory `--git`＋`--path`. Correct to
      `cargo install --git https://github.com/NEWSLabNTU/nano-ros nros-cli` (or the
      source-release's documented CLI install).
- [x] The user-journey CI lane (196 background) builds a scaffolded project end to
      end, so this convention is exercised, not assumed.

### 196.9 — [DONE] Per-platform setup/build/test CI matrix
**Goal.** Consolidate CI around the user procedure *per platform* — the gap where
only Zephyr had a build lane (core-libs only `cargo check`s, dep-chain only
resolves, host-unit only runs workspace units). Each platform gets one lane:
`install prebuilt nros → just <plat> setup (nros-provisioned) → build → test`.

**Design (`.github/workflows/platform-ci.yml`).** One template, a **dynamic
matrix** over platforms:
- A `changes` pre-job (dorny/paths-filter) emits a JSON platform list → the
  `platform` job's `matrix.plat` (`fromJSON`). On a PR only the platforms whose
  code or the shared core (`packages/core`, `zpico`, `just`, `cmake`, the index)
  changed run; on `main`/dispatch, all. (Job-level `if` can't read `matrix`, so
  selection happens in the pre-job — the standard GHA pattern.)
- **Build** runs on push/PR (filtered); the heavier **QEMU test/e2e** is
  `workflow_dispatch`-only (`run_e2e`) — keeps runtime flakiness off every push ×
  N platforms (a nightly `schedule:` is the natural place to turn e2e on later).
- **Additive**, not a replacement: the cheap distinct gates stay — host-unit-tests
  (workspace units), core-libs (no_std cross-target), dep-chain (all-board
  *resolution*, incl. stm32f4/orin/fvp which get no full lane), and the bespoke
  zephyr-dual-line. Once these per-platform lanes are trusted, **dep-chain slims**
  (its resolution is subsumed for covered platforms) — the one real "replace".

**First live run (2026-05-29, run 26630807819, build-only).** The matrix structure
works: `changes` resolved the dynamic matrix, all cells ran (fail-fast off),
**nuttx built green**. The 6 others failed on wiring, now diagnosed:

- [x] **Matrix `plat` ↔ `just` module-name mismatch.** The matrix used hyphenated
      ids but the root `justfile` `mod` names differ: `qemu-baremetal` → **`qemu`**,
      `threadx-linux` → **`threadx_linux`**, `threadx-riscv64` → **`threadx_riscv64`**
      (underscores). `freertos`/`nuttx`/`esp32` already match (nuttx is the proof).
      Fix: matrix + path-filter keys use the exact `mod` names.
- [x] **`native` has no `setup` recipe** (`just native --list` → none) — it's the
      host platform (no SDK to provision). Decide: add a `native setup` (workspace
      sources only) or drop native from the matrix (host coverage = host-unit-tests
      + a native-examples build). Dropped from the matrix for now.
- [x] **Per-cell setup/build greening (fresh runner).** All 6 cells now build green
      (run 26633900068, build-only, 2026-05-29). `nros setup <board>` covers the
      build sources per 197.4-A; esp32's `setup` provisions sources + best-effort
      `[tool.esp32-qemu]` (the e2e emulator, not the build). qemu needed three more
      recipe/image fixes (below).
- [x] **Wire e2e on a nightly `schedule:`** — `platform-ci.yml` has
      `schedule: '0 7 * * *'`; the per-platform e2e step fires on `schedule` +
      `workflow_dispatch -f run_e2e=true`. Push/PR stay build-only. The
      concurrency group is keyed on `(ref, event_name)` so a build-only push run
      doesn't cancel an in-flight dispatch/nightly. Per-cell skip-tolerance is
      enforced via `_count-real-failures` (`[SKIPPED]` panics don't fail the
      recipe).

**Review findings + maintainer directives (2026-05-29).**
- [x] **ROS-for-codegen gap (likely part of the 6 failing cells, beyond naming).**
      platform-ci installs **no ROS**, but the build step runs `nros generate-rust`
      for the rust examples (`just/freertos.just:75,148`, etc.), which resolves a
      package's `msg/*.msg` via `AMENT_PREFIX_PATH` from a **sourced ROS**.
      `./setup.bash` only PATHs the nano-ros tools (nros/qemu/zenohd/xrce-agent +
      SDK store) — it does **not** source ROS / set AMENT. So any cell building a
      rust example fails at codegen. (nuttx green is consistent — its cell either
      doesn't gen rust interfaces or its C path uses `NROS_*_DIR`, not AMENT.) The
      dep-chain + zephyr-dual-line lanes install ROS for exactly this reason.
- [x] **Directive: provision ROS via a prebuilt image, not per-cell `setup-ros`.**
      Done — `ci/docker/ci-base/Dockerfile` builds `ghcr.io/newslabntu/nano-ros-ci:humble`
      (`FROM ros:humble-ros-base` + cmake/ninja/dtc/gperf, uv, just, rustup + cross
      targets, gcc-arm-none-eabi, cargo-nextest, `safe.directory '*'`; published by
      `.github/workflows/build-ci-base-image.yml`). platform-ci's `platform` job runs
      `container:` against it (`shell: bash`, `packages: read` + GITHUB_TOKEN creds);
      all per-cell setup-ros/apt/rustup installs deleted. Image extras the cells
      surfaced: `libslirp0`+`libpixman` (prebuilt patched qemu links libslirp.so.0)
      and `ros-humble-example-interfaces` (qemu service/action codegen — ros-base
      ships only common_interfaces/std_msgs). The zephyr image `FROM nano-ros-ci`
      refactor is a follow-up (own item).
- [x] **Directive: cover C/C++ tests too, not just rust/build.** Verified — the
      recipes are *not* rust-only. For all four hosted-RTOS platforms (freertos,
      nuttx, threadx_linux, threadx_riscv64) `build-fixtures` builds the c **and**
      cpp examples (`scripts/build/fixtures-build.sh <plat> {c,cpp} zenoh`; e.g.
      `just/freertos.just:166-167`, `just/nuttx.just:127-128`,
      `just/threadx-linux.just:116-117`, `just/threadx-riscv64.just:215-216`), and
      `packages/testing/nros-tests/tests/rtos_e2e.rs` runs the pubsub/service/action
      e2e matrix parametrized over rust **+ c + cpp** per platform. C e2e runs on all
      four; C++ e2e runs on freertos (full), threadx_riscv64 (full), threadx_linux
      (pubsub+service). Remaining C++ gaps are intentional/pre-tracked, not a recipe
      gap: NuttX C++ fixtures build but e2e is skipped (upstream libc limitation,
      `rtos_e2e.rs:369`); threadx_linux C++ **action** is skipped → Phase 177.2;
      threadx_riscv64 Cyclone c/cpp fixtures are experimental-gated. The bare-metal
      cells (qemu, esp32) have no c/cpp by design (examples/README.md matrix). So the
      per-cell `test` step already exercises c/cpp wherever it exists — no recipe
      change needed; closing the skips is owned by their respective phases.

**Container refactor + 6/6 build-green (2026-05-29, run 26633900068).** All six
platform cells (qemu, freertos, nuttx, threadx_linux, threadx_riscv64, esp32) build
green in the `nano-ros-ci:humble` container. freertos + threadx cells went green on
the baked ROS/AMENT alone; the two outliers needed recipe completion of the 197.4
`nros setup` rewrite + image deps:
- **esp32 `setup`** (`just/esp32.just`) provisions `nros setup esp32` (zenoh-pico/
  mbedtls); the Espressif QEMU fork is best-effort (`|| warn`) — build doesn't need
  the emulator, the e2e step gates on it.
- **qemu** (`just/qemu-baremetal.just`) took four fixes: (1) `setup` sources
  `setup.bash` so `build-zenoh-pico` uses the nros-provisioned arm-gcc 13.2 (newlib)
  not apt's headerless gcc-arm-none-eabi; (2) `libslirp0`+`libpixman` in the image
  for the prebuilt patched qemu; (3) `build` runs `nros generate-rust` for examples
  with a package.xml; (4) same codegen for bins with a package.xml (cdr-roundtrip-qemu).
- A non-fatal `cargo metadata`/cbindgen warning recurs (`px4-sitl-tests` workspace
  member unprovisioned — px4 is extended-tier); harmless to the bare-metal builds.

**6/6 build + e2e green (2026-05-30, run 26658300402).** Every platform cell
(qemu, freertos, nuttx, threadx_linux, threadx_riscv64, esp32) passes **both**
build and QEMU e2e on the dispatch path; the nightly `schedule: '0 7 * * *'`
exercises the same matrix daily. Reaching this required finishing the container
findings recorded in **Phase 203** (`platform-ci` section): the parallel cbindgen-
header race (committed `nros_generated.h` + atomic build.rs write), the rustup
`-Z build-std` concurrent-install race (serial nuttx c/cpp builds), the FFI
staticlib `+nightly` toolchain pin (use `Rust_TOOLCHAIN`, the dated nightly is
what's installed), the workflow `NROS_VERSION` bump tracking the index schema
(0.3.1 → 0.3.7), plus the per-platform fixes already listed (eh_frame, profile
dirs, picolibc, etc.). Residual runtime items are the Group-B set (timing-
sensitive, cross-ref archived Phase 200 / 177.2), surfaced by the nightly
without blocking pushes.

**Files.** `.github/workflows/platform-ci.yml`; the per-platform `just/<plat>.just`
setup/ci recipes; `nros-sdk-index.toml` (per-board package coverage);
`ci/docker/ci-base/Dockerfile` + `.github/workflows/build-ci-base-image.yml`
(the `nano-ros-ci` base image).

---

## Acceptance criteria

> CI status checked 2026-06-12 (`gh run list`). 5/7 met; the 2 open ones are
> real failing lanes (zephyr-dual-line regressed; colcon-parity never green).

- [ ] `zephyr-dual-line` is green end-to-end on both lines (196.1). **OPEN —
      regressed:** the lane has been green historically but the latest completed
      runs fail (2026-06). Needs a re-green (196.8 build gaps / 196.1 codegen skew
      class). The active 196 hardening blocker.
- [x] A codegen-consumer check fails on a stray `nros --args-file` / legacy form
      (196.2). `codegen-convention.yml` ships it; CI green.
- [x] Every `.github/workflows/*.yml` has had ≥1 successful live run.
      **colcon-parity fixed + CI-green 2026-06-12 (run 27364170073, first-ever pass):** `colcon-parity.yml` had never
      passed — `colcon build` discovered the top-level nano-ros umbrella CMakeLists
      (pure-CMake, needs `nros`) alongside `src/`; restricted it to `--base-paths
      src` (validated locally: 3 pkgs + consumer binary build clean). The other
      workflows have ≥1 green run. All other workflows have ≥1 green
      (platform-ci / lint / zephyr-dual-line green historically; ci / host-unit /
      codegen-convention / sdk-index-gate / nros-acceptance / dep-chain /
      scaffold-journey / embedded-feature-unification / string-conventions green).
- [x] `ci.yml` runs a meaningful workspace check, not a single-crate stub. It
      cross-`check`s the package set over a target matrix (`core no_std`); CI green.
- [x] A light **per-platform dep-chain** lane (196.6) green over the board × rmw
      matrix: `nros setup` + codegen + `cargo tree` (resolution, no full build)
      for every cell. `dep-chain.yml` CI green (10/10 cells).
- [x] `nros new` scaffolds a dep that resolves under the source-release model
      (196.7); a scaffolded project resolves in the user-journey lane.
      `scaffold-journey.yml` CI green.
- [x] `docs/development/ci-conventions.md` exists and the dual-line + ci
      workflows follow it.

## Notes
- **Lesson (the expensive one):** a workflow that "has NOT been validated by a
  live GitHub Actions run" (its own header) is effectively broken. Validate via
  `gh workflow run --ref <branch>` before merging — the 7 dual-line layers were
  all first-run-only failures invisible to local runs (cached SDK, pre-init'd
  submodules, sourced ROS, dev symlinks).
- **Push/dispatch race seen during bring-up:** a `git push` that was rejected
  (branch behind) followed by `gh workflow run` dispatched the *stale* tip, so a
  "fixed" run silently re-ran the old code. Always confirm the push landed
  (`git fetch` + check the ref/headSha) before dispatching.
- The dual-line fixes live in `scripts/zephyr/setup.sh`,
  `scripts/zephyr/cortex-a9-rust-patch.sh`, `just/zephyr.just`,
  `.github/workflows/zephyr-dual-line.yml`, and the untracked `zephyr-workspace`
  symlink — all on `feature/phase-172`.
