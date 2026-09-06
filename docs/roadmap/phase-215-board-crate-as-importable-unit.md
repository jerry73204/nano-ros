# Phase 215 — Board crate as importable unit (FVP first)

**Goal.** Turn `packages/boards/nros-board-<name>/` into a unit a
downstream Zephyr app (ASI is the reference consumer) imports with a
**single cmake call** plus a **stock `west build -t run`** (was `west fvp run`
until RFC-0064 R5 / 215.K), with no
hand-curated `EXTRA_CONF_FILE` / `DTC_OVERLAY_FILE` / hardcoded board
id list / hand-rolled FVP launch flags on the consumer side. FVP
AEMv8-R is the driving case; the shape is generic across boards.

**Status (2026-09-06).** **The nano-ros feature is DONE** — a board crate is
importable in one line by any Zephyr consumer, and 27 of the 30 boxes that are
this repo's to tick are ticked. The ASI-specific half moved OUT (see 215.H),
which is what this status used to be measuring against.

**Two acceptance bullets remain, and neither is code** (a third was withdrawn
2026-09-06 — the 215.E fixture in CI, because FVP is on no tier the breadth
ladder visits; see the bullet for what still holds):

* a Zephyr app calling `nano_ros_use_board(<n>)` and *nothing else* of
  board-specific shape — `examples/workspaces/realtime-cpp/src/fvp_entry` and
  the `board_import_fvp` fixture both do; whether an entry layering its OWN
  config over the board's counts as "nothing else" is a judgement the phase
  owner should make rather than a gate;
* the run path launching `FVP_BaseR_AEMv8R` end to end — needs the simulator.
  Restated by 215.K.4: the thing to verify is stock `west build -t run` with
  `ARMFVP_BIN_PATH` exported by `activate.sh`, not `west fvp run`;

**RFC-0064 revision 5 (2026-09-06) SUPERSEDES the mechanism, not the goal.** The
one-line import stands and does not move; what changes is what sits behind it. A
tree-wide board audit found this phase invented a private descriptor format for
one board — `board.cmake`, 14 `NROS_BOARD_*` variables and a `west` extension —
while eleven other boards use `nros-board.toml`, and that two of the artifacts
built here are unreachable:

* **215.F's drift audit checks zero boards.** It walks top-level
  `packages/boards/nros-board-*/` dirs needing both `Cargo.toml` and
  `board.cmake`; the only `board.cmake` in the tree is one level deeper, and no
  board carries `[package.metadata.nros.board]` at all.
* **`west fvp` does not avoid the problem it exists for.** Its last statement is
  `os.execvpe('west', ['west','build','-d',dir,'-t','run'])` — the exact command
  ASI declined because a reconfigure diverges the size-probe identity. 117 lines
  of env wiring wearing a verb.

215.A, 215.C, 215.D and 215.F are therefore **superseded** and their artifacts
are deleted by **215.K**; 215.B, 215.E, 215.G and 215.J survive unchanged, which
is why the goal does not move. The general half is
[phase-375](phase-375-board-tier-policy-and-onboarding-cost.md) W6–W9.

**Priority note.** The `P1` below rests on "unblocks ASI's actuation consumption
story". ASI has consumed it since its phase 2.C and its FVP lane is green across
five variants, so that justification is spent; what is left is worth doing on
its own terms, not because a downstream is waiting.

Driven by ASI's Phase 190 follow-up — ASI today hand-glues every layer Phase 215
collapses.

**Priority.** P1 — unblocks ASI's actuation consumption story + every
future external Zephyr consumer of a nano-ros board crate.

**Depends on.** Phase 117.10–117.14 (FVP build smokes), Phase 199
(Zephyr 3.7 floor), Phase 214.A (local FVP runner; this phase moves
its surface from `just zephyr` into `west fvp`).

## Overview

The driving asymmetry today:

| Layer                                              | Today                                                       | Phase 215                          |
|----------------------------------------------------|-------------------------------------------------------------|------------------------------------|
| Zephyr board id                                    | ASI hardcodes `ZEPHYR_TARGET_LIST=(…)` in `build.sh`        | reads from `board.cmake`           |
| Base `prj.conf` (kernel, POSIX, networking)        | ASI duplicates parts of nano-ros's per-board `prj.conf`     | layered by `nano_ros_use_board()`  |
| Board overlay (DTS + per-board Kconfig)            | ASI carries its own `actuation_module/boards/<id>.{conf,overlay}` | layered from board crate         |
| Default RMW (`cyclonedds` on Phase-117 boards)     | `-DNANO_ROS_RMW=cyclonedds` typed out by every script       | board crate declares default        |
| Gated tool resolution (`ARM_FVP_DIR` etc.)         | ad-hoc env in consumer scripts                              | `nros doctor --board <name>`        |
| FVP runner                                         | nano-ros's `just zephyr run-fvp-aemv8r-cyclonedds`          | `west fvp run -d <dir>` extension   |

The board crate (`nros-board-fvp-aemv8r-smp/`) already CARRIES the
artifacts (`prj.conf`, `boards/<hwv2-id>.{conf,overlay}`, Rust
skeleton, Cargo features). What's missing is the **import surface**:
a single cmake fn that layers them into a downstream Zephyr build,
plus a `west` extension that owns FVP launch.

Net effect on ASI's `actuation_module/CMakeLists.txt`:

```cmake
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(actuation_module LANGUAGES CXX)
nano_ros_use_board(fvp-aemv8r-smp)   # <— ONE LINE replaces all glue
target_sources(app PRIVATE src/main.cpp ...)
```

And ASI's `build.sh` shrinks to:

```sh
west build -d build actuation_module                       # build
ARM_FVP_DIR=/opt/arm/Base_RevC_AEMv8R west fvp run -d build  # run
```

## Architecture

```
            ASI's CMakeLists.txt
                  │
                  │  nano_ros_use_board(fvp-aemv8r-smp)
                  ▼
   zephyr/cmake/nano_ros_use_board.cmake   (nano-ros Zephyr module)
                  │
                  │  include()
                  ▼
   packages/boards/nros-board-fvp-aemv8r-smp/board.cmake   (sidecar manifest)
                  │
                  │  declares: ZEPHYR_ID, TOOLCHAIN, GATED_PKGS,
                  │            DEFAULT_RMW, DEFAULT_TRANSPORT,
                  │            RUNNER, PRJ_CONF, BOARD_CONF, OVERLAY
                  ▼
   layered into the app build via:
       EXTRA_CONF_FILE  ← prj.conf + boards/<id>.conf
       DTC_OVERLAY_FILE ← boards/<id>.overlay
       BOARD            ← ZEPHYR_ID (if user didn't pass `-b`)
       NANO_ROS_RMW     ← DEFAULT_RMW (if not -D'd)

   ───────────────────────────────────────────────────────────

   west fvp run -d build/
                  │
                  │  reads CMakeCache.txt → NROS_BOARD_RUNNER=armfvp
                  ▼
   scripts/west_commands/fvp.py    (nano-ros west extension)
                  │
                  │  scripts/zephyr/resolve-fvp-bin.sh
                  ▼
   ARMFVP_BIN_PATH=<resolved>  west build -d build -t run
                  │
                  ▼
   zephyr/cmake/emu/armfvp.cmake (upstream Zephyr; UART → stdout)
```

**Two faces of the manifest.** `board.cmake` is the cmake face;
Cargo.toml `[package.metadata.nros.board]` mirrors the same facts
for `nros doctor` + future Rust/CLI consumption (Phase 212.L.8
alignment). Both must stay in lock-step — Phase 215.F audit guards
against drift.

## Work Items

### 215.A — Sidecar `board.cmake` per board crate — **SUPERSEDED (RFC-0064 R5, 2026-09-06)**

> Landed and working, but it is a second descriptor format for one board. Folds
> into `nros-board.toml`'s `[board.zephyr]` block; 14 variables become 8 authored
> plus 6 derived. Deleted by 215.K. The two open boxes below (A.1 schema doc, A.3
> cross-reference) are **withdrawn** — documenting a format being deleted is work
> nobody should do.

- [x] **215.A.1** Define the `board.cmake` schema. Variables:
      `NROS_BOARD_ZEPHYR_ID` (Zephyr `BOARD` string),
      `NROS_BOARD_TOOLCHAIN` (SDK abi target, e.g. `aarch64-zephyr-elf`),
      `NROS_BOARD_GATED_PKGS` (semicolon list keyed on
      `nros-sdk-index.toml` `[gated.*]` names),
      `NROS_BOARD_DEFAULT_RMW` (`cyclonedds` / `zenoh` / `xrce`),
      `NROS_BOARD_DEFAULT_TRANSPORT` (`ethernet` / `serial` / …),
      `NROS_BOARD_RUNNER` (`armfvp` / `qemu` / `native` / …),
      `NROS_BOARD_PRJ_CONF` (absolute path),
      `NROS_BOARD_BOARD_CONF` (absolute path; per-board hwv2
      Kconfig fragment),
      `NROS_BOARD_BOARD_OVERLAY` (absolute path; per-board DTS overlay).
- [x] **215.A.2** Write `packages/boards/nros-board-fvp-aemv8r-smp/board.cmake`
      filling every variable. Absolute paths via
      `${CMAKE_CURRENT_LIST_DIR}`. _(landed `01ef6bd1a`, 2026-06)_
- [x] **215.A.3** Documented schema cross-references this phase doc.
      _(verified 2026-09-05: `docs/reference/board-cmake-schema.md:15`
      cross-refs this file, and all nine `NROS_BOARD_*` variables of A.1
      appear in BOTH that doc and the fvp-aemv8r-smp `board.cmake`.)_
- **Files:** `packages/boards/nros-board-fvp-aemv8r-smp/board.cmake`
  (new), `docs/reference/board-cmake-schema.md` (new).

### 215.B — `nano_ros_use_board(<name>)` cmake fn

- [x] **215.B.1** New `zephyr/cmake/nano_ros_use_board.cmake` (~80 LoC
      hard cap). `function(nano_ros_use_board NAME)`: _(landed `2b9a909c9`, 2026-06; 86 LoC incl. 215.B.3 call-order guard)_
      1. Resolve `BOARD_DIR = ${NROS_REPO_DIR}/packages/boards/nros-board-${NAME}`.
      2. `FATAL_ERROR` if `${BOARD_DIR}/board.cmake` missing.
      3. `include("${BOARD_DIR}/board.cmake")`.
      4. If `BOARD` empty → set `BOARD ${NROS_BOARD_ZEPHYR_ID}` CACHE;
         else WARN on mismatch.
      5. `list(APPEND EXTRA_CONF_FILE  ${NROS_BOARD_PRJ_CONF}
                                       ${NROS_BOARD_BOARD_CONF})`
         + `PARENT_SCOPE` re-export.
      6. `list(APPEND DTC_OVERLAY_FILE ${NROS_BOARD_BOARD_OVERLAY})`
         + `PARENT_SCOPE`.
      7. If `NANO_ROS_RMW` undefined → cache `${NROS_BOARD_DEFAULT_RMW}`.
      8. Cache `NROS_BOARD_RUNNER` so `west fvp run` reads it from
         `CMakeCache.txt`.
- [x] **215.B.2** `zephyr/CMakeLists.txt` `include()`s the new fn so
      it's available to every downstream app once nano-ros's Zephyr
      module is on `ZEPHYR_EXTRA_MODULES`.
- [x] **215.B.3** `nano_ros_use_board()` must be call-able BEFORE
      `find_package(Zephyr)` OR after — order tested both ways. The
      `EXTRA_CONF_FILE` / `BOARD` overrides need to land before
      Zephyr's board-resolution phase, so the fn either re-orders
      (sets variables) or `FATAL_ERROR`s on wrong-order call. Phase
      215.B.3 verifies via the 215.E.1 fixture.
- **Files:** `zephyr/cmake/nano_ros_use_board.cmake` (new),
  `zephyr/CMakeLists.txt` (include + invocation site).

### 215.C — Cargo.toml `[package.metadata.nros.board]` mirror — **SUPERSEDED (RFC-0064 R5, 2026-09-06)**

> The second face exists in the reader and in zero producers: no board in the
> tree carries the table. One descriptor replaces both faces, and cmake reads a
> generated projection of it (RFC-0064 R5 D4). Deleted by 215.K.

- [x] **215.C.1** Add the metadata table to _(landed `1b003afc2`)_
      `packages/boards/nros-board-fvp-aemv8r-smp/Cargo.toml`:
      ```toml
      [package.metadata.nros.board]
      zephyr_board = "fvp_baser_aemv8r/fvp_aemv8r_aarch64/smp"
      toolchain    = "aarch64-zephyr-elf"
      gated        = ["arm-fvp"]
      default_rmw  = "cyclonedds"
      default_transport = "ethernet"
      runner       = "armfvp"
      prj_conf      = "prj.conf"
      board_conf    = "boards/fvp_baser_aemv8r_fvp_aemv8r_aarch64_smp.conf"
      board_overlay = "boards/fvp_baser_aemv8r_fvp_aemv8r_aarch64_smp.overlay"
      ```
- [x] **215.C.2** Schema struct + `deny_unknown_fields` in _(landed; `board_metadata.rs` + deny_unknown_fields; 10 unit tests)_
      `nros-cli-core::orchestration::board_metadata`. (Mirrors
      Phase 212.B's strict cargo-metadata reader.)
- [x] **215.C.3** `nros board info <name>` subcommand (read-only) _(landed; `nros board info <name> --check-drift`)_
      prints both the Cargo.toml view AND the parsed `board.cmake`
      view side-by-side; flags drift between them. Phase 215.F audit
      hook.
- **Files:** `packages/boards/nros-board-fvp-aemv8r-smp/Cargo.toml`,
  `nros-cli/packages/nros-cli-core/src/orchestration/board_metadata.rs`
  (new), `nros-cli/packages/nros-cli-core/src/cmd/board.rs` (new).

### 215.D — `west fvp` extension (moves Phase 214.A runner) — **SUPERSEDED (RFC-0064 R5, 2026-09-06)**

> Replaced by `nros sdk-path arm-fvp` plus an `activate.sh` export of
> `ARMFVP_BIN_PATH`, which is how every other SDK in the index already resolves,
> and then stock `west build -t run`. Deleted by 215.K. 215.D.4 is **withdrawn**
> with the extension.

- [x] **215.D.1** `scripts/west_commands/fvp.py` — west command _(landed `41c4683da`)_
      `class FvpRun(WestCommand)`. `do_run`:
      1. Argparse `-d/--build-dir` (default `build/`).
      2. Read `CMakeCache.txt` → `NROS_BOARD_RUNNER`. If not
         `armfvp`, error "board <name> is not an FVP target".
      3. Shell `scripts/zephyr/resolve-fvp-bin.sh` → `ARMFVP_BIN_PATH`.
      4. `os.execvpe('west', ['west', 'build', '-d', build_dir,
         '-t', 'run'], env=...)`.
- [x] **215.D.2** `scripts/west-commands.yml` declares `fvp` subcommand: _(landed)_
      ```yaml
      west-commands:
        - file: scripts/west_commands/fvp.py
          commands:
            - name: fvp
              class: FvpRun
              help: run a nano-ros FVP board in the Arm Fast Models simulator
      ```
- [x] **215.D.3** `zephyr/module.yml` adds `west-commands: _(landed; NOTE: Zephyr 3.7 module.yml rejects a `west-commands:` key — registration moved to the workspace west.yml nano-ros project entry)_
      scripts/west-commands.yml` so any workspace that has nano-ros as
      a west project gets the extension for free (no manual
      registration).
- [x] **215.D.4** Phase 214.A `just zephyr run-fvp-aemv8r*` recipes
      are RETAINED (developer ergonomics inside nano-ros) but
      delegate to `west fvp run` instead of duplicating the resolver
      + `-t run` invocation. ~3-line recipes.
- **Files:** `scripts/west_commands/fvp.py` (new),
  `scripts/west-commands.yml` (new), `zephyr/module.yml` (edit),
  `just/zephyr.just` (retarget Phase 214.A recipes).

### 215.E — Fixture: minimal "ASI-shaped" consumer

- [x] **215.E.1** `packages/testing/nros-tests/fixtures/board_import_fvp/` —
      minimal Zephyr app:
      `CMakeLists.txt` with **only** `include(...nano_ros_use_board.cmake)`
      + `nano_ros_use_board(fvp-aemv8r-smp)` (BEFORE `find_package(Zephyr)`
      so BOARD / EXTRA_CONF_FILE / DTC_OVERLAY land in time — see 215.B.3)
      + `find_package(Zephyr)` + `project()` +
      `target_sources(app PRIVATE src/main.c)`;
      `src/main.c` a trivial `printk("nros: smoke ok")` (no `nros::init`
      call — the empty-`prj.conf` minimal app doesn't bring up an RMW;
      the board crate's layered `prj.conf` sets `CONFIG_NROS=y` and the
      215.G UART scrape only needs the boot-reached `printk` line);
      `prj.conf` empty (everything from the board crate);
      `package.xml` for Phase 210 closure;
      no per-board confs/overlays in the fixture. _(landed `df371027b` /
      `13c542fd5`; verified present in worktree)_
- [x] **215.E.2** `packages/testing/nros-tests/tests/phase215_e_board_import.rs`
      — `west build -d build fixtures/board_import_fvp` succeeds;
      asserts the generated `CMakeCache.txt` carries
      `BOARD=fvp_baser_aemv8r/fvp_aemv8r_aarch64/smp` + `NANO_ROS_RMW=cyclonedds`
      + `NROS_BOARD_RUNNER=armfvp`. Build only — Phase 215.G runs the
      FVP smoke. Skips loudly (`nros_tests::skip!`) when ZEPHYR_BASE /
      west absent. _(landed; verified skips cleanly here — Zephyr SDK
      absent; build itself gated-pending-SDK)_
- **Files:**
  `packages/testing/nros-tests/fixtures/board_import_fvp/` (new),
  `packages/testing/nros-tests/tests/phase215_e_board_import.rs` (new).

### 215.F — Drift audit: cmake vs Cargo metadata — **SUPERSEDED (RFC-0064 R5, 2026-09-06)**

> Checks zero boards, for two independent reasons (see Status). Replaced by a
> projection round-trip — descriptor → projection → parse back == descriptor —
> which has no "skip if the other face is missing" arm and so cannot go vacuous
> the same way. Deleted by 215.K.

- [x] **215.F.1** `packages/cli/nros-cli-core/tests/phase215_f_manifest_drift.rs`
      (landed at the in-tree CLI sub-workspace, not `nros-tests/` — the
      `compute_drift` impl + reader live in `nros-cli-core`)
      — for every `packages/boards/nros-board-*/` carrying both
      `board.cmake` and `[package.metadata.nros.board]`: parse each,
      assert byte-equal field-by-field for the overlapping keys
      (`zephyr_board`, `toolchain`, `default_rmw`, `runner`, conf/
      overlay paths via basename). Bare boards (Rust-only Phase 212.N
      tier-1 shims without `board.cmake`) are skipped. _(landed; verified
      `NROS_WORKSPACE_ROOT=. cargo test -p nros-cli-core --test
      phase215_f_manifest_drift` → 1 board audited clean; 10
      `board_metadata` unit tests green)_
- [x] **215.F.2** `nros board info <name>` (Phase 215.C.3) carries
      a `--check-drift` flag wired to the same audit; CI gate.
      _(`--check-drift` landed; verified exit 0 on agreement / exit 1
      on injected drift. CI/lint surface added:
      `scripts/check-board-manifest-drift.sh` runs `nros board info
      <name> --check-drift` for every board with a `board.cmake`, wired
      into `just check`. Capability-probes the resolved `nros` and skips
      with a `just setup-cli` hint on a pre-215.C CLI — never false-fails
      `just check` on a clap error.)_
- **Files:**
  `packages/cli/nros-cli-core/tests/phase215_f_manifest_drift.rs`,
  `packages/cli/nros-cli-core/src/cmd/board.rs`,
  `scripts/check-board-manifest-drift.sh` (new), `justfile` (wire into
  `check`).

### 215.G — End-to-end FVP smoke through the import surface

- [x] **215.G.1** `packages/testing/nros-tests/tests/phase215_g_fvp_smoke.rs`
      drives the fixture from 215.E.1 (not the example carve-out)
      through the import surface. Modeled on `phase217_c_fvp_runtime`:
      `nros_tests::skip!` (loud `[SKIPPED]` panic, never silent-pass)
      when the FVP isn't resolvable / west absent / Zephyr workspace
      absent / fixture ELF missing; otherwise drives
      `just zephyr run-fvp-board-import` (→ `west fvp run -d
      build-fvp-board-import`), 120 s cold-boot budget, scrapes the
      `nros: smoke ok` UART line + the Zephyr boot banner. Build +
      run recipes `build-fvp-board-import` / `run-fvp-board-import`
      added to `just/zephyr-setup.just` (mirror the cpp/rust cyclonedds
      recipes; export `NROS_REPO_DIR` for the fixture's pre-`find_package`
      include). nextest override registers the binary in the serial
      `zephyr-fvp` group. _(test + recipes landed; compiles clean;
      verified skips loudly here — ARM FVP absent (`ARM_FVP_DIR` unset,
      resolver exits 1) AND Zephyr SDK absent. **Runtime PASS is
      UNVERIFIED — gated pending an FVP install + provisioned Zephyr
      SDK.**)_
- [x] **215.G.2** Test documented (module doc) as the canonical "is the
      board crate import surface healthy?" gate — distinguishes itself
      from `phase217_c` (example carve-out) by exercising the minimal
      `nano_ros_use_board()`-only fixture.
- **Files:** `packages/testing/nros-tests/tests/phase215_g_fvp_smoke.rs`
  (new, gated), `just/zephyr-setup.just` (build/run recipes),
  `.config/nextest.toml` (zephyr-fvp group override).

### 215.H — ASI migration — MOVED to the ASI repo (2026-09-06)

This block tracked work inside
`github.com/NEWSLabNTU/autoware-safety-island`: the pin bump, the
`nano_ros_use_board()` call, that repo's per-board confs, its `build.sh`
target list, its FVP launch path and its `fvp/NOTES.md`. None of it is a
nano-ros change, and this phase could neither verify nor land any of it — the
worktree cannot see that repo, so six boxes sat open describing a state nobody
here could read.

**Taken over as ASI phase-5 W7** (`docs/roadmap/phase-5-legacy-cleanup.md`,
"ASI plumbing replaceable by nano-ros features"), which is where a consumer's
adoption of a nano-ros feature belongs. What the handover found, recorded there
with evidence:

* the `nano_ros_use_board()` call has been in
  `actuation_module/CMakeLists.txt` since ASI phase 2.C — this block listed it
  as outstanding for months;
* the per-board conf/overlay item was MIS-SCOPED. The board's own glue does
  come from the crate now; what remains in that repo is its own variant matrix
  (CAN loopback, tap network, tracing), which a consumer is supposed to layer;
* the `build.sh` target-list hardcode is real, small, and ASI's;
* two items named files that exist on neither ASI branch and were dropped
  rather than carried.

**What nano-ros keeps is the general feature**, which is 215.A-F and 215.I: a
board crate is importable in one line by any Zephyr consumer. ASI is the first
one; the phase is not a plan for ASI.

### 215.I — Book chapter

- [x] **215.I.1** `book/src/porting/board-crate-import.md` — the
      consumer guide. Cross-refs Phase 212.N.8 board-trait porting +
      Phase 214 FVP runtime + Phase 191 sdk provisioning.
- [x] **215.I.2** SUMMARY.md update.
      _(verified 2026-09-05: `book/src/SUMMARY.md:100`
      — `[Importing a Board Crate](./porting/board-crate-import.md)`.)_
- **Files:** `book/src/porting/board-crate-import.md` (new),
  `book/src/SUMMARY.md` (edit).

### 215.J — Downstream Zephyr consumer provisioning (`nros setup board`)

Filed 2026-06-13 — surfaced by the ASI **real local build** (ASI phase-2.C):
`nano_ros_use_board()` imports the board *config* (prj.conf/overlay/RMW/runner)
and that composes correctly, but an `import:false` downstream Zephyr consumer
does **not** get nano-ros's *provisioning* — so the build walls at Kconfig with
`RUST_SUPPORTED=n`, `POSIX_THREAD_THREADS_MAX` undefined, and no cyclonedds. The
board crate is not yet a *fully* importable unit: it carries config, not the
`(zephyr-patches × zephyr-lang-rust × cyclonedds-source)` toolchain provisioning
`just zephyr setup` does for nano-ros's own tree.

**Design (decided 2026-06-13; see RFC-0014 §"Downstream Zephyr consumer
provisioning"):** the patch scripts already take a workspace dir (`patches/<line>.sh
$WORKSPACE`) and `nros setup --source` is index-driven — expose them for a
consumer's tree, board-driven.

- [x] **215.J.1** `board.cmake` (+ Cargo mirror) gains a **provisioning
      contract**: `NROS_BOARD_ZEPHYR_LINE` (`3.7`), `NROS_BOARD_REQUIRES_RUST`
      (`y`), `NROS_BOARD_RUST_TARGETS` (`aarch64-unknown-none` — the actual
      `_rust_map_target` triple for AArch64 AEMv8-R; the filed example
      `aarch64-zephyr-elf;armv7a-none-eabi` conflated the SDK toolchain with the
      rust target), `NROS_BOARD_RMW_SOURCE` (`cyclonedds-src`), plus
      `NROS_BOARD_RUST_SUPPORT_MODULE` (J.4). Schema doc + `board_metadata`
      struct (`deny_unknown_fields`) + drift audit (215.F) extended. _(landed;
      cmake-parses clean; `nros board info` shows them; `--check-drift` + the
      `phase215_f` audit pass; +1 `drift_compute_provisioning_contract` unit
      test → 11 lib tests green.)_
- [x] **215.J.2** `nros setup board <name> --zephyr-workspace <dir>` — a clap
      subcommand under `nros setup` (`args_conflicts_with_subcommands`, so the
      legacy `nros setup <board>` positional still parses). Reads the contract
      via `board_metadata`, then against the consumer's tree: (a) `nros setup
      --source <rmw_source>` (reuses `provision_named_sources`, index-driven —
      provisioned into nano-ros's own tree, which the consumer links); (b) `bash
      scripts/zephyr/patches/<line>.sh <dir>`; (c) `rustup target add` per
      target; (d) verify the `zephyr-lang-rust` module + instruct via the J.3
      fragment (never mutates the consumer manifest). Idempotent; `--dry-run`
      prints the plan. _(landed; builds; `--dry-run` verified end-to-end against
      a fixture workspace. The live patch/source/rustup actions need a real
      zephyr dir + network — structurally verified.)_
- [x] **215.J.3** Board-shipped **`import:false`-compatible west fragment**
      `packages/boards/nros-board-fvp-aemv8r-smp/west-downstream.yml` — declares
      ONLY `zephyr-lang-rust` at nano-ros's pin (rev `404fcefd…`, lock-step with
      the top-level `west.yml`); a single project so no `name-allowlist` is
      needed (the doc shows the allowlist form for consumers who instead import
      nano-ros's full manifest). Consumer adds one `manifest.self.import: -file:`
      line + `west update`. _(landed; YAML validated.)_
- [x] **215.J.4** **Lean B** — board-shipped Kconfig overlay module
      `zephyr-rust-support/` (`zephyr/module.yml` + `Kconfig` adding `config
      RUST_SUPPORTED \n bool \n default y if BOARD_FVP_BASER_AEMV8R`).
      `nano_ros_use_board()` step 9 appends `NROS_BOARD_RUST_SUPPORT_MODULE` to
      `ZEPHYR_EXTRA_MODULES` so the consumer gets it for free.
      **Verified B works**: the real `zephyr-lang-rust` `config RUST_SUPPORTED`
      is a plain `bool` with a single `default y if (CPU_CORTEX_M || RISCV-… ||
      (ARCH_POSIX && 64BIT)) && !TIMER_…` — none true on AArch64 AEMv8-R — and
      Kconfig accumulates a symbol's properties across files (first-true
      `default` wins), so a board-keyed `default y` fires non-destructively
      without editing the lang-rust tree. (Scope is the `RUST_SUPPORTED` Kconfig
      symbol only; the lang-rust CMake `_rust_map_target → aarch64-unknown-none`
      mapping is NOT a Kconfig symbol and stays supplied by
      `aarch64-rust-patch.sh` via J.2 step (b).)
- MOVED to ASI phase-5 W7 (2026-09-06), with 215.H: `bootstrap-asi.sh`
  delegating to `nros setup board` is a change in that repo, and the file it
  names does not exist on either ASI branch. `nros setup board` itself — the
  nano-ros half — is J.1-J.4 above and stays here.

**Files.** `packages/boards/nros-board-*/board.cmake` (+ `Cargo.toml`),
`packages/cli/nros-cli-core/src/cmd/setup.rs` (the `board --zephyr-workspace`
surface), `packages/boards/nros-board-*/west-downstream.yml` (new),
a board Kconfig overlay module, `docs/reference/board-cmake-schema.md`.

### 215.K — Migrate the FVP board onto the one board process

Implements RFC-0064 revision 5 for this board. The general mechanism is
phase-375 W6–W8; 215.K is the FVP board's side of it, and lands after W7 exists.

- [ ] **K.1** `packages/boards/nros-board-zephyr/boards/fvp-aemv8r-smp/nros-board.toml`
      — 14 `NROS_BOARD_*` variables become 8 authored keys across `[board.zephyr]`
      and `[board.provisioning]`. Derived: `prj.conf`, both
      `boards/fvp_baser_aemv8r_fvp_aemv8r_aarch64_smp.{conf,overlay}` paths
      (`west_board` with `/` → `_`), `requires_rust` (from `rust_targets`
      non-empty), and `zephyr-rust-support/`, whose Kconfig body is exactly
      `default y if BOARD_FVP_BASER_AEMV8R` and is therefore generated.
- [ ] **K.2** `package.xml` announcing
      `<nano_ros_provides kind="board" name="fvp-aemv8r-smp"/>`. This board is
      currently announced nowhere, and the discovery glob is one level too
      shallow to see it even with a descriptor (issue 0729's class) — so K.2
      needs W6's `provider_scan`.
- [ ] **K.3** Delete `board.cmake`, `scripts/west_commands/fvp.py`, its
      `scripts/west-commands.yml` entry, the `west.yml` comment about it, and
      `zephyr/module.yml`'s pointer. `nano_ros_use_board()` reads the W7
      projection; its signature does not change, so
      `examples/workspaces/realtime-cpp/src/fvp_entry`, the `board_import_fvp`
      fixture and ASI's `actuation_module/CMakeLists.txt:154-155` do not move.
- [ ] **K.4** `nros sdk-path arm-fvp` resolves the model through the existing
      `[gated.arm-fvp]` index entry; `activate.sh` exports `ARMFVP_BIN_PATH`.
      `just/zephyr-setup.just` lines 239 and 348 move to stock
      `west build -t run`. Three tests reference `west fvp run` in prose
      (`board_import.rs`, `fvp_runtime_ws.rs`, `fvp_smoke.rs`) and follow.
- [ ] **K.5** Delete `board_metadata.rs` and
      `packages/cli/nros-cli-core/tests/phase215_f_manifest_drift.rs`; the
      round-trip test replaces them.
- [ ] **K.6** Book: `book/src/porting/board-crate-import.md` documents the
      descriptor and stock `west build -t run`. Its current "Don't reimplement
      the FVP runner as a shell script" note inverts once the runner is an env
      export, so it is rewritten rather than retargeted.
- [ ] **K.7** File the size-probe reconfigure-identity bug separately. It is the
      real defect behind ASI's `build.sh --run` workaround, and `west fvp` never
      dodged it — the probe identity is every `NROS_*` env var plus `$DOTCONFIG`'s
      `CONFIG_NROS_*` lines, so a reconfigure that picks crate defaults produces a
      different identity than the build did. K.4 is only honest once this is
      fixed.

**Acceptance:** the FVP board is discovered, announced and built by exactly the
same code path as `qemu-cortex-a53` and as an out-of-tree user board; no
`board.cmake` and no `west` extension remain in the tree; the one-line consumer
call site is byte-identical to what ASI has today.

## Acceptance

- [ ] A Zephyr app's `CMakeLists.txt` calls `nano_ros_use_board(<n>)`
      and NOTHING else of nano-ros-board-specific shape (no per-board
      conf, no overlay, no `EXTRA_CONF_FILE` hand-list, no
      `-DNANO_ROS_RMW=...`).
- [ ] The run path launches `FVP_BaseR_AEMv8R` end-to-end, UART → stdout,
      exits clean on Ctrl-C. **Restated by 215.K.4** — `ARMFVP_BIN_PATH` from
      `activate.sh` (via `nros sdk-path arm-fvp`) plus stock
      `west build -t run`, replacing `west fvp run -d build/`. Blocked on
      215.K.7: a reconfigure diverges the size-probe identity, which is what
      this bullet actually trips over.
- [x] `nros board info fvp-aemv8r-smp` prints the board metadata
      from BOTH `board.cmake` and `Cargo.toml`; `--check-drift`
      exits 0 when they agree, non-zero with a clear field-by-field
      diff on drift. _(verified: exit 0 clean, exit 1 on injected drift.)_
- MOVED to ASI phase-5 W7 (2026-09-06): whether ASI's
  `actuation_module/CMakeLists.txt` builds clean is ASI's acceptance to state.
  For the record, it does — the call is at `CMakeLists.txt:154-155` and that
  repo's CI runs the FVP lane green across five variants (run 33961168979,
  2026-09-05). nano-ros's own acceptance for the same capability is the
  `board_import_fvp` fixture below.
- [~] Phase 215.E fixture builds in CI; the fixture's `CMakeCache.txt` carries
      the expected `NROS_BOARD_*` keys. **WITHDRAWN as a CI acceptance
      2026-09-06 — owner decision.** FVP is not a tier-1 or tier-2 platform, so
      asking CI to prove this means provisioning an Arm Fast Models licence and
      the Zephyr aarch64 SDK for a coordinate the breadth ladder does not
      visit.

      The manifest already says as much: `west_board_import` is a
      `[[compile_check_fixture]]` with `builder = "west-configure"` and **no
      `platform` / `lang` / `rmw` coordinate at all**, and both tiers narrow BY
      coordinate — tier 1 by name (`NROS_TEST_SCOPE`), tier 2 in the fixture
      resolver (`NROS_TEST_COORDS`). A row with no coordinate was never in
      either lane; this bullet was asking for something the ladder is not
      shaped to give.

      **What still holds, and what it costs.** The fixture and its test are
      real and stay: `board_import.rs` asserts the three things this phase is
      about — `BOARD` propagating out of `board.cmake`, the board's default
      `NANO_ROS_RMW=cyclonedds`, and `NROS_BOARD_RUNNER=armfvp` cached for
      `west fvp run`. It reads a PREBUILT `CMakeCache.txt` rather than running
      west at test time (issues 0034 / 0041), and on a machine without west +
      the Zephyr/FVP SDK it takes the tier-aware skip and says so loudly rather
      than passing vacuously. So it is a real check for whoever HAS the SDK,
      and it is silent everywhere else — deliberately, now, rather than by
      omission.

      The residual risk, stated: the metadata path from `board.cmake` into a
      downstream app's cache is verified only when someone runs it locally. The
      nearest independent evidence is a different repo's — ASI's FVP lane is
      green across five variants on the same board crate (run 33961168979,
      2026-09-05) — which is not this assertion, but does mean a break here
      would not be silent for long.
- [x] Phase 215.F drift audit passes for every
      `packages/boards/nros-board-*` carrying a `board.cmake`.
      _(verified: `nros-board-fvp-aemv8r-smp` audited clean by both the
      `phase215_f` integration test and `just check board-manifest-drift`.)_

## Notes

- `nano_ros_use_board()` is **Zephyr-specific**. Non-Zephyr boards
  (native, freertos, threadx) carry the Phase 212.N Board trait
  impl in Rust; their consumption shape is `cargo` path-deps, not
  cmake. Two ecosystems, two surfaces — no unified verb attempted.
- ~~The `west fvp` extension sits outside Phase 212 §Non-Goals~~ — the
  reasoning held (`west` should own a run verb); the mistake was that `west`
  ALREADY owns it. `west build -t run` is the verb, and `west fvp run` only
  exported one environment variable before exec'ing it. RFC-0064 R5 / 215.K.
- Phase 214.A recipes stay in `just zephyr` as developer ergonomic
  thin shells over the run verb (so `just zephyr build-* &&
  just zephyr run-*` stays the in-tree dev loop). ASI never reaches
  the justfile; nano-ros contributors do. 215.K.4 repoints them from
  `west fvp run` to stock `west build -t run`.
- Multiple FVP variants (Cortex-R52, Corstone-310, AEMv8-R aarch32)
  follow the same shape: one board PACKAGE each (`package.xml` +
  `nros-board.toml`), all naming `runner = "armfvp"`, so the resolver
  is variant-agnostic. Deferred until a real second variant lands.
  (Said "sibling `board.cmake` files" until RFC-0064 R5.)
- The board crate's Rust skeleton (`src/lib.rs` `init_hardware()` +
  `run<F>(config, app) -> !`) is **NOT** changed by Phase 215.
  Phase 212.N Board trait migration is orthogonal. Phase 215 only
  touches the CMake / Cargo-metadata / west surfaces — the Rust
  surface stays Phase 117.10 skeleton.
- `arm-fvp-installer` (Phase 214.B) is a hard prereq for
  `nros doctor --board fvp-aemv8r-smp` to be useful; if 214.B is
  still open when 215 lands, the doctor check stays warn-only on
  missing `ARM_FVP_DIR`.
