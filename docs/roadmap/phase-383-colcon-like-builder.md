# phase-383 — `nros build`, the colcon-like builder (implement RFC-0065)

**Implements:** [RFC-0065](../design/0065-colcon-like-workspace-builder.md) (D1–D14)
**Amends in passing:** [RFC-0024](../design/0024-multi-node-workspace-layout.md) §2.4/§9,
[RFC-0026](../design/0026-example-directory-layout.md) (copied-out *workspaces*),
[RFC-0070](../design/0070-build-cache-layout.md) R1 (`install/` → `dist/`)
**Closes:** issue 0798 (the (entry × board) pairing becomes derived)
**Touches:** [RFC-0063](../design/0063-system-model-is-a-build-artifact.md)/phase-330
(who owns `build/`), [RFC-0077](../design/0077-image-runtime-is-the-images-choice.md)
(the `panic` policy this forwards)

**Status (2026-08-31). W1–W9 and W10.a / W10.c / W10.d COMPLETE — all bringups
declare `[image.*]` (123 images tree-wide, every one resolving), and `panic` /
`args` are carried (see the W9.a note below). W10.b alone remains, and is NOT
DUE: it is a 0.6.0 action.** The W10 chain is PROVEN on one
real workspace: `examples/workspaces/rust` built with its hand-written root and
entry deleted and the generated pair compiling. That proof found three more
defects (all fixed) and was then reverted — the tree is unchanged.

**W9.c update (2026-08-27).** Two blockers found and cleared, neither of them
the migration:

* **Issue 0825 (FIXED, c7fa5cb3d).** Three `codegen_system` tests had been red
  all session. `model_search_paths` keyed its `$OUT_DIR` rung on the bringup's
  NAME, cargo sets `$OUT_DIR` when running the tests of a package with a build
  script, and that directory is shared by every test target — so this phase's
  own `tests/build_verb_pipeline.rs` wrote a model that three sibling tests
  read as theirs. Reader and writer now share one path-keyed helper.
* **Issue 0828 (FILED, open).** `lane=tier2` is NOT the build tier 2 needs. A
  row the resolver cannot attribute fails closed, so it is in the run set at
  every lane while no lane builds it; 47 rows share one `build_subdir`. After a
  core-crate edit the lane gate passes and the run reports ~190 stale-fixture
  failures. Tier 2 is green today only on residue from an older `lane=all`.
  W9.c therefore runs against a `lane=all` build until 0828 lands.

**W9.a's gap is CLOSED (re-measured 2026-08-31).** It was real when written:
two keys had never been carried over from the entries the images were derived
from, and a generated entry would have silently lost its panic policy or its
machine binding — a build that succeeds and is wrong.

Both are carried now. Measured against the tree rather than re-read:
**7 images declare `panic`** (`workspaces/rust`: esp32, freertos, nuttx;
`workspaces/realtime-rust`: freertos_realtime, nuttx, riscv_nuttx;
`workspaces/cpp`: native) and **9 declare `args`** (the robot1/robot2 pairs in
`c`, `cpp`, `mixed`, `rust`, plus `rust`'s `zephyr_robot1`). The count is higher
than the nine this note predicted because every multihost image needs its own
binding. On the entry side the keys are simply gone: across 67 entry packages,
**zero** declare `panic` or `args` — `[package.metadata.nros.entry]` now carries
only `deploy` / `name` / `class` / `default_namespace` / `board` / `rmw` /
`node_pkgs` / `domain_id`.

**What remains needs a validated build cycle, not more code:** W9.b (retarget
`examples/fixtures.toml`'s rows at `nros build`), W9.c (`just build-test-fixtures
lane=all` — see 0828 — then `just ci-matrix`), and W10.a–W10.c (delete nine roots
and ~100 entry packages, then add the gate that keeps them gone). Those are one
irreversible sweep gated on a multi-hour run, and starting it blind is how a tree
ends up half-migrated.

## Goal

A nano-ros workspace user authors three things — node code, the launch tree,
and per-board overlays — and types **one command**. The workspace root build
file and the entry packages both stop being hand-written, because both are
derived from `(launch, args, board)`.

Today's user types six commands and maintains nine root build files plus
fifteen entry packages in one workspace alone.

## The ordering constraint that matters

**The builder ships and proves itself BEFORE anything migrates, and the old
paths are retired only after the migration lands** (RFC-0065 D13).

> W1–W7 build it · W8 proves it on hostile trees · W9 migrates the nine ·
> W10 retires the old paths.

Migrating during development means re-migrating on every design change.
Retiring before migrating strands the tree. Never retiring is how a project
ends up with two ways to build — the thing this phase exists to remove.

## Global constraints

Every wave inherits these; they are not restated per task.

| constraint | value | source |
| --- | --- | --- |
| CMake floor | **3.22** — `$<LINK_LIBRARY:WHOLE_ARCHIVE,…>` (3.24+) is unavailable | measured on this host |
| Stage 5 | **`execvp`** — never a pipe, never `Command::output()` | RFC-0065 D1; the RFC-0024 §2.4 amendment depends on it |
| Flag vocabulary | **no `--target`** — the word already means four things here and users pass cargo's through | RFC-0065 D7 |
| `panic` values | `platform` \| `halt` \| `own` — the **existing** RFC-0077 enum, forwarded | RFC-0077 |
| Generated output | deterministic: no timestamps, no absolute paths, stable ordering | reproducibility (W3.c gates it) |
| Build trees | `build/<coord>/`, coordinate in RFC-0070 R2's vocabulary — **never a new suffix** | RFC-0070 R2 |
| Cargo invocation | one target dir per workspace, never per package | RFC-0070's 80.6 GB measurement |

## What the simulation already found

RFC-0065's *Migration simulation* walked the nine in-tree workspaces plus
`autoware-safety-island` (branch `nano-ros`) and `nano-ros-rt-eval`. Ten
findings, F1–F10, are requirements on the waves below rather than discoveries
to be made during them. Each wave names the ones it carries.

**These two downstream projects are the acceptance bar** (W8). Our own
workspaces are uniform by construction and will not surface F4–F6.

**One decision deliberately has no task.** RFC-0065 **D11** ("a custom board is
a board crate, not an overlay") is a boundary statement, not work: it says where
vendor-generated board *source* belongs and defers the authoring story to
RFC-0012. W7.e/W7.f give that crate its force-link and linker-fragment seams;
nothing else is owed.

---

## W1 — The image declaration (BLOCKS EVERYTHING)

Carries **F3, F7, F8**. Nothing else can start: every later wave reads this
schema.

- [x] **W1.a** Add a **new, nano-ros-owned** `[image.<id>]` table (RFC-0065 D6)
      to `SystemToml` in
      `packages/cli/nros-cli-core/src/orchestration/cargo_metadata_schema.rs`.
      **Not a rename**: `[deploy.*]` is
      `ros_launch_manifest_model::system_config::DeployBlock`, an upstream
      `deny_unknown_fields` type (git tag v0.1.11), and it keeps its placement
      meaning. Fields: `board`, `launch`, `args`, `panic`, `profile`, `variant`,
      `conf`, `rmw`, `edition`, `features` — where `variant`/`conf` are **F3/F8**
      (`nano-ros-rt-eval` builds one app on one board twice, and
      `demo_bringup/ablation/*.toml` are whole alternate system configs).
      `deny_unknown_fields` on our side too, matching upstream's reasoning that
      a mistyped key must error rather than silently drop.
- [x] **W1.b** Add the `[image]` base table (RFC-0065 D5.1) merged under each
      `[image.<id>]`, so an eight-image workspace states its RMW and edition
      once. Precedent: PlatformIO's `[env]` / `[env:NAME]`.
- [x] **W1.c** Add `[system] default_images`. **F7**: a workspace may declare
      several bringups (`nano-ros-rt-eval` has `demo_bringup` AND
      `load_bringup`), so resolve `default_images` per bringup and require
      `nros build <image>` to be unambiguous across all of them — a duplicate
      image id across two bringups is a hard error naming both files.
- [x] **W1.d** `panic` is the **existing** RFC-0077 policy enum
      (`platform`|`halt`|`own`), forwarded to `nros::main!`. Do not invent
      values; `"semihosting"` is a crate, not a policy.
- [x] **W1.e** Resolve `board` through `packages/boards/board-support.toml`
      (RFC-0065 D9): the user always writes a nano-ros board id, and the
      registry carries the framework's own board string for platforms that have
      one. `[image.*].board` today mixes seven nano-ros keys with one raw Zephyr
      string (`native_sim/native/64`) — the conflation phase-337 W2 removed from
      `PlatformId`, still live one layer up.

      **Landed smaller than planned.** No registry field was needed: a
      descriptor already carries the downstream ecosystem's id among its
      `names` — `packages/boards/zephyr/nros-board.toml` lists
      `["zephyr", "native_sim/native/64"]` — so both spellings already resolve
      to one descriptor. Resolution delegates to
      `BoardCatalog::resolve_deploy`, the single rule issue 0606 established
      after three consumers each grew a private opinion about what a board is
      called; a fourth would be that defect with a new name. The only addition
      is an error naming the IMAGE, which `resolve_deploy` cannot know.
- [x] **W1.f** Deprecate the **build fields inside `[deploy.*]`** — never the
      table, which stays upstream's placement declaration. A `[deploy.<id>]`
      carrying `board`/`rmw`/`profile`/`optimize`/`features` with no
      `[image.<id>]` beside it warns once per invocation naming the replacement,
      using phase-222's shipped pattern verbatim: `NROS_SUPPRESS_DEPRECATION=1`
      opts out, `nros doctor` flags it in config files, removal at the next
      minor version. **A version boundary, not a time period.** A
      `kind = "embedded"` block whose only content was build fields becomes
      deletable once its `[image.*]` lands — upstream already excludes it from
      placement.

**Acceptance.** `nros ws model-dims` over all nine in-tree workspaces returns
byte-identical output before and after the bringups are rewritten to
`[image.*]` — the schema change moves no resolved value.

---

## W2 — Discovery that survives a real workspace

Carries **F4, F9**. Delivers `nros build` for the cases needing **no**
generation (west, ESP-IDF, copy-out leaves), so the pipeline is proven before
any emitter exists.

- [x] **W2.a** Stage 1 discovery = the `package.xml` walk **UNION the cargo
      members already visible**. **F4**: `nano-ros-rt-eval/src/island_clock/`
      is `Cargo.toml` + `src/` with no `package.xml`; a members list derived
      from the walk alone drops it and the build dies on an unresolved path
      dependency.
- [x] **W2.b** Honour `.nros-ignore` / `COLCON_IGNORE` in the walk. **F9**:
      `nano-ros-rt-eval` vendors nano-ros as a submodule and `touch`es
      `.nros-ignore` so the walk does not descend into it. `nros-pkg-index`
      already knows both markers — assert it, do not re-implement it.
- [x] **W2.c** Wire stages 1→2→3→5 with **no stage 4**: resolve the image,
      preflight, then `exec` the framework tool. Zephyr and ESP-IDF need no
      generated root (RFC-0065 D3), so this is a complete, shippable
      `nros build` for them.
- [x] **W2.d** Stage 3 preflight (RFC-0065 D2): auto-fetch what the index can
      fetch **after prompting with the download size**; `--yes` skips the
      prompt; **a non-TTY behaves as verify-only** and never blocks.
      License-gated packages are never auto-fetched in any mode — they fail
      naming the package and the manual `nros setup` line. Reuse
      `nros setup --check`'s existing "verify, name what is missing, fetch
      nothing" path rather than a second implementation.
- [x] **W2.e** Stage 5 is `execvp`, not a pipe, not `Command::output()`. The
      test asserts that a deliberately broken source produces **byte-identical
      stderr** to the native tool invoked directly. This is the whole RFC-0024
      §2.4 reconciliation; if it is a pipe, the amendment is void.

**Acceptance.** `nros build zephyr` in `examples/workspaces/rust` produces the
same artifact as today's `west build -b native_sim/native/64 …` line, and a
syntax error in `talker_pkg` yields identical stderr under both.

---

## W3 — The cargo root emitter

Carries **F10**.

- [x] **W3.a** Emit `build/<coord>/Cargo.toml` with `[workspace] members` from
      W2.a's union, `exclude` for west/idf entries, and the workspace-level
      `[patch]` set `nros sync` already computes.
- [x] **W3.b** **Emit the Rust entry** (RFC-0065 D4 — the heart of this
      phase). From `(launch, args, board)` produce `build/<coord>/<image>_entry/`
      carrying `Cargo.toml` (board crate, node rlibs, the `*_nros_selection`
      facade `nros sync` generates) and a `main.rs` whose body is the one-line
      `nros::main!(launch = …, args = …)` plus the board's shell —
      `#![no_std]`/`#![no_main]`, the `panic` policy from W7.a, and board
      boilerplate such as `esp_app_desc!()`. The existing emitters in
      `packages/cli/nros-cli-core/src/codegen/entry/emit_rust.rs` already
      produce this shape for `nros codegen entry`; this wave gives them
      `(image, board)` as input instead of a hand-written package.

- [x] **W3.c** Output is **deterministic**: no timestamps, no absolute paths,
      stable ordering. Gate it the way issue 0320 gated model paths —
      `check-no-absolute-model-paths` is the pattern to copy. A production
      build's reproducibility depends on this and nothing else in the phase
      enforces it.
- [x] **W3.d** `nros build a b c` builds several images in one invocation.
      **F10**: `nano-ros-rt-eval`'s `just build` is
      `cargo build -p native_entry -p peer_entry`.
- [x] **W3.e** Bare `nros build` honours `default_images`; absent it, a
      multi-image workspace lists them and **fails**. `--all` opts in.
      `nano-ros-rt-eval`'s manifest documents why: a bare workspace build
      "would try [the cross-target member] for the host and fail".

**Acceptance.** `examples/workspaces/rust`'s tracked root `Cargo.toml` and the
generated one produce identical `cargo metadata` output modulo path prefixes.

---

## W4 — The cmake root emitter

Carries **F5**. This is the wave that must not fragment the corrosion cargo
tree — **one cmake configure per workspace** (RFC-0070's 80.6 GB measurement).

- [x] **W4.a** Emit `build/<coord>/CMakeLists.txt` calling
      `nano_ros_workspace(BACKEND … PLATFORM … SYSTEM … ORDER_FROM_DEPENDS
      SUBDIRS …)`, with the board→toolchain map resolved **before**
      `project()` and the `NUTTX_DIR` promotion the hand-written roots do by
      hand.
- [x] **W4.b** **Emit the C/C++ entry** — the `nano_ros_add_executable(BOARD …
      BRINGUP … LAUNCH … LANG … DEPLOY …)` call that the hand-written entry
      leaves carry today, with `BOARD`/`DEPLOY` taken from the resolved image
      rather than written as literals. This is what closes **issue 0798**: a
      generated entry cannot disagree with the board it was generated for.
      Embedded C/C++ entries already carry zero source, so for them this wave
      deletes the package and emits its one call.

- [x] **W4.c** **User preamble hook.** **F5**: `autoware-safety-island`'s root
      calls `find_package(Eigen3 REQUIRED)`. An optional
      `<bringup>/cmake/preamble.cmake` is `include()`d before `project()` if
      present; absent, nothing changes.
- [x] **W4.d** **A package that gates itself out is not an error.** **F5**:
      ASI adds `src/s32z2_board_glue` only when the NXP SDK is provisioned —
      "the pkg's own CMakeLists gates and reports". The emitter lists it; the
      package decides.
- [x] **W4.e** Assert one cargo target dir per workspace configure, not per
      package. Test: configure a mixed workspace, then assert exactly one
      `cargo/build` tree under `$NROS_BUILD_ROOT`.

**Acceptance.** `examples/workspaces/mixed` builds from a generated root with
no tracked root `CMakeLists.txt` present, and `ninja -t query` shows the RMW
archive under `|` (the issue-0475 link-edge check).

---

## W5 — Zephyr overlays and multi-image output

Carries **F1, F2**. F1 is a **correction already made in the RFC** — implement
the corrected form.

- [x] **W5.a** (RFC-0065 D10) Pass overlays as **`EXTRA_CONF_FILE`, never `CONF_FILE`**.
      Zephyr's `configuration_files.cmake` puts `boards/` and `socs/`
      auto-discovery inside `if(NOT DEFINED CONF_FILE)`, so setting `CONF_FILE`
      suppresses both. Our own zephyr entries suppress it today;
      `nano-ros-rt-eval`'s justfile carries the workaround note.
- [x] **W5.b** Point `APPLICATION_CONFIG_DIR` at
      `src/<bringup>/boards/<board>/`. Verified reachable for sysbuild too:
      `share/sysbuild/cmake/modules/sysbuild_kconfig.cmake` resolves
      `sysbuild.conf` through it and FORCEs it into the cache so images inherit
      it.
- [x] **W5.c** Map an image's `variant` onto Zephyr's own
      `prj_<buildtype>.conf` → `CONF_FILE_BUILD_TYPE` mechanism (**F2**),
      rather than inventing a parallel axis. `autoware-safety-island`'s
      `prj_actuation.conf` is already this shape.
- [x] **W5.d** Detect sysbuild by the **presence of `sysbuild.conf`** in the
      board's config dir and pass `--sysbuild`. Zephyr's own comment:
      "sysbuild.conf is an optional file, because sysbuild is an opt-in
      feature." Invent no key.

**Acceptance.** A `native_sim` image built through `nros build` has a `.config`
identical to the one today's explicit `west build` line produces, including the
`boards/native_sim_native_64.conf` values that `CONF_FILE` currently drops.

---

## W6 — `dist/` and the manifest

- [x] **W6.a** (RFC-0065 D8) `dist/<image>/` holds the artifact **set** plus `manifest.toml`
      naming which member is flashable and at what address. A host image is a
      set of one — the same shape, not a special case.
- [x] **W6.b** **Completeness gate**: every file in `dist/<image>/` is named by
      that image's manifest; an unnamed artifact fails the build. ESP-IDF's
      `flasher_args.json` supplies the cautionary tale — a filed bug reads
      "missing entry for `bootloader` when built with secure boot v2", the
      manifest silently falling behind its artifacts.
- [x] **W6.c** (RFC-0065 D14) `--offline`: stages 1–4 perform no network I/O, and stage 5 is
      invoked with the native tool's offline spelling (`cargo --frozen`).
      **The value is converting a silent fallback into a named failure** —
      `NanoRosCorrosion.cmake` falls through to
      `FetchContent_Declare(Corrosion GIT_REPOSITORY …)` when the SDK store has
      no matching prefix, and the file calls itself "offline-hostile".
- [x] **W6.d** Amend RFC-0070 R1 in the same commit as W6.a lands
      (`install/` → `dist/`), so the two documents never disagree. **Landed
      early**, during the 2026-08-25 framework-fit pass that introduced `dist/`
      — the two documents have never disagreed.

**Acceptance.** `nros build native --offline` in a tree with no Corrosion in
the SDK store fails naming the missing package, and `strace -f -e trace=connect`
records no outbound connection.

---

## W7 — The escapes

Carries **F6**.

- [x] **W7.a** Forward `panic` and `profile` from `[image.<id>]` into the
      generated entry (RFC-0065 D5). These are the only escapes the D4 survey
      found; the custom spin loop already has RFC-0024 §11.8.
- [x] **W7.b** `nros materialize <image>` writes `src/<image>_entry/`. Naming
      follows the **image**, which D6 made the named unit.

      **Landed once W3.b unblocked it.** Materialising COPIES a real generated
      entry rather than running a second emitter, so the two can never drift.
      `--force` is required to overwrite an already-materialised entry: a second
      run would otherwise silently discard the edits the first existed to
      enable.
- [x] **W7.c** **Shape stamp that WARNS, never errors.** **F6**:
      `autoware-safety-island` carries `freertos_main.cpp`, `board_init.c`,
      `cp15_arm.S` and four `.ld` fragments — it will hold a materialised entry
      **forever, by design**. An error would break a legitimate downstream
      permanently.
- [x] **W7.d** A test that a materialised entry still builds. A decorative
      escape silently deletes capability.
- [x] **W7.e** `nano_ros_support_library(<name> SRCS … INCLUDES … WHOLE_ARCHIVE)`
      (RFC-0065 D12). The keyword emits the flag **and** the `LINK_DEPENDS` —
      issue 0475 is the reason users must not hand-write it. Our CMake floor is
      3.22, so `$<LINK_LIBRARY:WHOLE_ARCHIVE,…>` (3.24+) is unavailable; owning
      the spelling means the floor can rise later with no user edit.
- [x] **W7.f** Add `LINKER_FRAGMENTS` to the same function. **F6**: D12 covered
      libraries but not `.ld` fragments.

      **Two corrections found while implementing (2026-08-26).** *The seam is
      not primarily Zephyr's:* this item said "`zephyr_linker_sources()` is the
      seam", but the motivating fragments belong to ASI's **s32z2 FreeRTOS**
      lane, where that command does not exist. The non-Zephyr path —
      `-L<fragment-dir>` plus `LINK_DEPENDS`, the idiom
      `nano-ros-board-mps2-an385-freertos.cmake` already uses — is the PRIMARY
      seam and Zephyr's is the alternate. Both are implemented; followed
      literally, the wording would have put the motivating case on the fallback.
      *And `zephyr_linker_sources()` takes a location argument* (`SECTIONS`,
      `ROM_START`, `NOCACHE_SECTION`, …) the drafted signature had no slot for —
      a no-cache buffer fragment is exactly one needing a non-default location.
      Hence `ZEPHYR_SECTION`, defaulting to `SECTIONS`.

---

## W8 — Hostile-workspace acceptance (GATES W9)

Our nine workspaces are uniform by construction. These two are not, and they
are where F4–F6 came from.

- [x] **W8.a** Dry-run migrate `nano-ros-rt-eval`: two bringups, ablation
      variants, a cargo member with no `package.xml`, a vendored nano-ros
      submodule, a cross-target member, two images built together. Assert the
      generated root produces the same `cargo metadata` as its tracked one.
- [x] **W8.b** Dry-run migrate `autoware-safety-island` (branch `nano-ros`):
      a hand-written `main`, four linker scripts, assembly, a self-gating
      NXP-licensed package, a root preamble, and concern-named conf files.
      Assert its FreeRTOS-POSIX lane still builds.
- [x] **W8.c** Neither project is modified in this phase. A dry run that needs
      an upstream edit is a **W1–W7 defect**, not a migration task.

**This wave gates W9.** If it does not pass, the builder is not ready for the
in-tree migration regardless of how green our own examples are.

### What it found — three W1–W7 defects, exactly as W8.c predicted

Both dry runs were done on COPIES; neither project was modified.

1. **Preflight probed one sync location of two.** `nros sync` writes generated
   msg crates to `<ws>/generated/` and models to `<ws>/build/nros/`; the check
   looked only at the second. `nano-ros-rt-eval` carries fourteen msg packages
   in `generated/` and was told it had never been synced.

2. **A framework entry's `CMakeLists.txt` was read as a language signal.**
   rt-eval is pure Rust and holds exactly one CMakeLists —
   `src/zephyr_entry/CMakeLists.txt`, which belongs to WEST. Counting it routed
   every native image through cmake, on a workspace with no C or C++ in it.

3. **Entries for other boards were listed.** ASI has three FreeRTOS entries
   (an536, posix, s32z2) and a `freertos-posix` build listed all three. This is
   one of the four jobs RFC-0065's Problem statement says a hand-written root
   does by hand — "which entries belong to the active platform" — and the
   emitter simply had not done it.

None needed an upstream edit, so W8.c holds: all three were defects in W1–W7,
found only because these trees are not uniform the way ours are.

---

## W9 — Migrate the nine (RFC-0065 D13, stage 2)

- [x] **W9.a** Migrate all nine workspace roots and their entry packages, one
      commit per workspace so a regression bisects to one.

      Images declared AND complete: 7 carry `panic`, 9 carry `args`, and no
      entry package declares either any more. See the re-measured status note
      at the top — this item was marked `[~]` for a gap that has since closed.
- [x] **W9.b** Update `examples/fixtures.toml` rows to invoke `nros build`.
      **Done for every row it can apply to. The "337 rows" figure conflated two
      tables that are not the same kind of thing** — measured 2026-08-29:

      | rows | state |
      | --- | --- |
      | 96 of 110 `[[workspace_fixture]]` | **invoke `nros build`** (`image =`) |
      | 14 of 110 `[[workspace_fixture]]` | blocked — see below |
      | 314 `[[fixture]]` | **out of scope by design** |

      **The 314 are not workspaces.** Zero of their 192 distinct dirs carry an
      `[image.*]` bringup, and `nros build` refuses them by construction:

          $ cd examples/native/rust/talker && nros build --all --dry-run
          Error: this workspace declares no `[image.*]`. An image is the
          buildable unit — see RFC-0065 D6.

      They are RFC-0026 standalone copy-out leaves — a single package, no
      workspace walk-up, deliberately buildable on their own. Giving them a
      bringup so `nros build` could drive them would contradict the contract
      that makes them copy-outable. The row already IS the invocation for them;
      there is nothing to retarget.

      **The 14 are all framework entries** — 13 zephyr/west, 1 esp32/idf — and
      they are blocked on **issue 0892**: `nros build`'s west driver points at
      the BRINGUP, which has no `CMakeLists.txt` and so is not a west
      application, and runs west from the nros workspace, which is not a west
      workspace (`west: unknown command "build"`). Nothing had ever built a
      zephyr image through `nros build`, because every zephyr row still goes
      through `west-fixtures.sh`. Until a west application is GENERATED the way
      the cargo and cmake roots are, those rows cannot move.

      **Update 2026-08-29 — the blocker is gone; a zephyr image now builds and
      RUNS through `nros build`.** No generated west application was needed:
      RFC-0085 D2 says west calls the workspace build rather than the reverse,
      so the driver points at the entry package the image already names. Three
      layers had to be fixed, each hidden behind the previous one:

      1. **the application, for non-Rust entries.** The resolver read
         `[package.metadata.nros.entry] deploy` from `Cargo.toml` only, so the
         five C/C++ workspaces — whose entries declare
         `nano_ros_add_executable(... DEPLOY zephyr)` — fell through to the
         bringup. It now reads both, and refuses when several packages match
         (six of fourteen images do; for `[image.zephyr_robot1]` first-match
         picked the WRONG entry). RFC-0085 D4.
      2. **the conf fragments.** Searched beside bringup and board dir, never
         beside the application — which is where a Zephyr app keeps its
         `prj-<rmw>.conf`, and these entries `FATAL_ERROR` without one. All 14
         images now declare their overlay. RFC-0085 D5.
      3. **the locator.** `zephyr/Kconfig` defaulted to `7456` where
         `just zenohd` and `rmw_zenoh_cpp` both use `7447`, so the built image
         could not reach the router the flow tells you to start.

      Verified: `nros build demo_bringup:zephyr` in `examples/workspaces/rust`
      → 1312/1312 → `zephyr.exe` → `talker publishing chatter`, and
      `ros2 topic echo /chatter --once` → `data: 6`. `examples/workspaces/c`
      builds the same way (1326/1326). All 14 images resolve to the right
      application under `--dry-run`.

      **Update 2026-08-30 — the harness fields are decided (RFC-0085 D16) and
      7 of the 14 rows are retargeted.** They stay ROW keys and reach west
      through `nros build … -- -d <build-dir> -p <mode> <-D defines>`;
      `route_native_arg` already splits that passthrough (west zone / cmake
      zone), so no CLI change was needed. `entry` comes OFF a retargeted row —
      D4 made the image name its application, and `validate_workspace_fixture`
      refuses a row naming both.

      Plumbing: `west-leaves` gained two columns (`ws_dir`, qualified
      `<bringup>:<image>`), EMPTY on an unmigrated leaf, and they join the leaf
      SIGNATURE only when set — an always-present column would have re-signed
      all ~70 zephyr leaves and re-run west on every one at ~20 min each.
      Verified: the other 69 signatures are byte-identical to the ones on disk.

      **Proven on `workspace-rust-zephyr`** (`demo_bringup:zephyr`), through the
      real lane (`zephyr-fixture-run-one.sh`, exit 0, 1305 ninja steps):

      * the Zephyr `.config` is byte-identical to the lane's own `west build`
        in the same build dir — the retarget adds only
        `-DEXTRA_CONF_FILE=<app>/prj-zenoh.conf`, and a control build with
        `-UEXTRA_CONF_FILE` diffed clean (the fragment is already 2nd of 3 in
        `CONF_FILE`, and shares no symbol with the NSOS tail);
      * `test_zephyr_workspace_entry_native_sim_e2e` PASSES on it — 19 messages
        delivered cross-process.

      Six more were retargeted on `--dry-run` equivalence plus the same
      conf-overlap check, NOT on a build: `zephyr_robot1`, and `zephyr` in
      `realtime-cpp`, `realtime-c`, `c`, `cpp`, `mixed`. Each dry-run resolves
      the same application dir and the same board the row already names.

      **Six are still blocked, and NOT on the harness fields — on their IMAGE
      declarations (W9.a):**

      | rows | why |
      | --- | --- |
      | rs-params, rs-lifecycle, rs-qos (features), rs-safety, rs-realtime | their images declare `board = "zephyr"` — a DEPLOY TOKEN, not a board id. `nros build` hands it to west verbatim (`west build -b zephyr`), which is not a Zephyr board. The rows all say `native_sim/native/64`. |
      | c-realtime-smp | `realtime-c` `smp_bringup:zephyr` declares `board = "native_sim/native/64"`, but the row builds `qemu_cortex_a53/qemu_cortex_a53/smp` with an a53 board conf. Board IS an image property, so this row needs its own image (e.g. `[image.zephyr_smp]`) — retargeting as-is would silently build the wrong board. |

      The esp32 row (`workspace-rust-esp32`) is untouched and was NOT
      investigated; it carries no `west_*` field and is a separate question.
- [x] **W9.c** Re-run the tier the diff earns per CLAUDE.md — this touches
      `cmake/` and codegen, so **`just ci-matrix`** at minimum.

      **GREEN** (2026-08-27): `just build-test-fixtures lane=all` all nine legs
      OK, then `just ci-matrix` EXIT=0, "CI passed (tier 2)". The raw nextest
      line reads "180 failed" — those are `skip!` panics before the junit
      rewrite, and `check-skip-budget: 180 skip(s) — capability=13 lane=167`
      accounts for every one.

      Getting there cost five defects that had nothing to do with the migration,
      each filed: 0825 ($OUT_DIR model shadow), 0828 (tier 2 runs rows no lane
      builds), 0834 (a sizes-header mirror state no re-run repairs), 0835 (the
      staleness probes reported build ACTIVITY as staleness — fixed, the gate
      reaches a fixed point for the first time), 0837 (a submodule bump strands
      fixtures the build side calls fresh), 0838 (four tests sharing one baked
      Cyclone domain, plus an uncapped `test-threads` above the domain
      partition's 25 slots — both fixed).

      The migration itself introduced ZERO test failures: two `ci-matrix` runs
      either side of the migration commit produced the identical 92-test failure
      set.

      **`lane=all`, not `lane=tier2`** (issue 0828): tier 2's run set contains
      rows no lane builds, so `lane=tier2` leaves them stale, the lane gate
      passes anyway, and the run fails ~190 tests on freshness. Cleared issue
      0825 on the way — three `codegen_system` tests that had been red all
      session, and this phase's own test target was the writer.

---

## W10 — Retire the old paths (RFC-0065 D13, stage 3)

- [x] **W10.a** Delete the nine tracked root `CMakeLists.txt` and the eight
      root `[workspace]` manifests, plus the entry packages W9 made derivable.

      **Done — all 15 example workspaces, zero tracked roots.** The count grew
      past "nine + eight" because the survey predated `bridge-*`, `sizing` and
      `managed`. West and FVP entries stay hand-written: they carry authored
      Kconfig no image declaration expresses (D5).

      The two BRIDGES were the last, and they needed a real derivation rather
      than a migration. A bridge's binary links TWO backends — it selects per
      domain at run time — which `[image.<id>].rmw` cannot say. It did not need
      new syntax either: the bringup already declares `[[domain]]` with an
      `rmw` each, which is the same set the `nros::main!` macro derives for its
      own `register()` emission. So the facade and the entry generator now read
      it from there, and the entry's backend CRATE names come from the board
      crate's own `rmw-<x> = ["dep:<crate>"]` features rather than a fourth
      copy of the macro's `rmw_crate_ident` table.
- [ ] **W10.b** Delete `[deploy.*]` parsing at the version boundary W1.f
      declared. **NOT DUE — and the reason is not the calendar.**

      W1.f promised "warn on every invocation while still working … removal at
      the next minor version". Two things were measured before touching this:

      1. **The version boundary has not been crossed.** The deprecation landed
         in `88d3758b8` at `0.5.0`; the workspace is still `0.5.0`. Removal is
         a `0.6.0` action.
      2. **The warning had never once been emitted.**
         `deprecated_deploy_build_field_warnings` shipped with four passing unit
         tests and NO production caller — all four call sites were its own
         `#[test]` functions — and `nros doctor` never grew the check either. So
         the deprecation period had not started, let alone elapsed.

      Deleting on that state would be the worst of both: support removed with a
      warning nobody ever saw. So W10.b's *preparation* landed instead — the
      lint is now reached from `collect_images`, which every `nros build`
      passes through, and a wiring test asserts it (mutation-checked by
      re-unwiring it, which reds).

      **The surface the eventual deletion touches, measured now** (excluding
      `third-party/`, build output and other agents' `.claude/worktrees/`):
      **37 deploy blocks across 14 files** — `board` ×29, `target` ×13,
      `rmw` ×2. That confirms the W1.f survey's "only `board` and `target` fire
      in practice", plus two `rmw`. Nothing reads these fields at build time
      today; `cmd/build.rs` already resolves everything from `[image.*]`.

      **Re-measured 2026-08-30, and the "37" was stale: 56 build fields across
      13 files** — `board` ×48, `target` ×12, `rmw` ×2. Splitting them by what
      the migration would actually have to do:

      | | count | what 0.6.0 must do |
      | --- | --- | --- |
      | image already carries the same value | 20 | nothing — pure duplication |
      | image lacks the key, or has no `[image.*]` at all | 42 | create the key/block |
      | image and deploy DISAGREE | **0** | — |

      Zero disagreements is the reassuring number: every deploy build field is
      either redundant or unopposed, so the migration cannot silently change
      what an image builds.

      **The 20 redundant ones are now GONE** (7 files). Removing a key the image
      already agrees with cannot change a resolution, and that was proven rather
      than assumed: `nros build <img> --dry-run` captured for five images of
      `examples/workspaces/rust` before and after is byte-identical.

      **The other 42 were deliberately left.** Each needs an `[image.*]` key or
      a whole new image block, and creating one makes a new BUILDABLE UNIT that
      `nros build --all` would then build — a behaviour change, not a
      migration. Several are placement-only deploys (`robot1`/`robot2` carrying
      a host `target`), where whether an image should exist at all is a design
      question, not a mechanical one.

      **Do at 0.6.0:** migrate the remaining 42 blocks, then delete the lint, the
      `DEPRECATED_DEPLOY_BUILD_FIELDS` list and the wiring added here.

      **Re-checked 2026-08-31, on request to "do W10.b". Still not due, and the
      blockers are now three rather than two — but the FIRST one has cleared.**

      1. **The deprecation period has STARTED.** The reason W10.b could not
         proceed before was that the warning had never been emitted. It is now:
         a plain `nros build --workspace examples/workspaces/rust --dry-run`
         prints one line per offending block, naming the field and the
         `[image.*]` it moves to. So the clock this promise depends on is
         finally running.
      2. **The version boundary still has not been crossed.** The workspace is
         `0.5.0`; W1.f promised removal "at the next minor version". Deleting
         now would break the promise a week after the warning first appeared,
         and bumping the minor is a release decision, not a migration step.
      3. **All 42 remaining fields are the NEW-BLOCK case — 0 are add-a-key.**
         Measured: every deploy still carrying a build field has no `[image.*]`
         of that name at all. So "migrate the remaining 42" means creating ~14
         image blocks, each a new BUILDABLE UNIT that `nros build --all` would
         then build. That is the behaviour change this work item already
         declined once, and the count says there is no mechanical subset hiding
         inside it.
      4. **`[deploy.*].rmw` IS still read — and as of 2026-08-31 it is
         OUTRANKED.** This item said the fields were inert, a same-day
         retraction agreed, and both were wrong about the 2 `rmw` ones; the
         compiler settled it.

         `SystemToml::resolved_rmw` has three production callers, not one.
         `bridged_rmws()` passes `target = None` — what a grep finds — but
         `codegen_system.rs` and `planner.rs` pass a real deploy target, and
         `nros plan --target` advertises it in its own `--help`. So `nros build`
         resolved RMW from `[image.*]` while `plan` and `codegen-system` read
         `[deploy.<t>].rmw`, and a workspace setting both BUILT one backend and
         BAKED another into `#define NROS_SYSTEM_RMW`. Issue 0938.

         **Fixed by adding an image rung above the deploy one**, so every verb
         now answers with the image where both are set. The deploy rung stays
         below it until the field retires — deleting it outright would change
         behaviour for workspaces still carrying it, before the deprecation
         boundary.

         **What this leaves W10.b:** all 42 fields are now safely deletable at
         0.6.0. `board` and `target` were always unread; `rmw` is now
         unreachable whenever an image declares one, and where no image does,
         deleting it falls through to `[system].rmw` — which is what those two
         fixtures already set it to.

      **So the deletion is mechanically safe for all 42**, and what remains is
      a shape decision rather than a correctness risk.

      **SURVEYED 2026-08-31, and OVERTAKEN the same day.** The survey below is
      kept because its method was right and its conclusion has an expiry date
      worth seeing. Everything it measured was true when measured; issue 0951
      landed hours later and removed the subject.

      What it found, and what is true now:

      | survey claim (2026-08-31, morning) | now |
      | --- | --- |
      | "77 deploy blocks across 26 files" | **0** in any `system.toml`. 123 `[image.*]`, 28 `[host.*]`. |
      | "a `[deploy.<n>.nros]` site layer — 30 `sdk` + 17 `netstack` — with no `[image.*]` counterpart" | 24 `[board_config.<board>]` blocks. The 30 held 3 distinct value-sets; keying on the board removed 25 duplicates. |
      | "`board_facts.rs` hard-ERRORS when `.board` is absent" | It selects candidates from `[image.*] ∪ [deploy.*]` and matches `--board` through the board catalog. |
      | "`check-deploy-board-resolves.py` errors if it finds zero deploy values" | It reads `[image.*]` and `[image_defaults]` too. Run today: `deploy boards resolve: OK (20 distinct value(s))`. |
      | "13 production reader sites … the build fields are NOT inert" | Every one migrated to `[image.*]`-first with a deploy fallback: `resolve_target` (now `resolve_image`, W10.e), `derive_target_rtos`, `schema_build_json`, the `codegen_system` launch fallback, `doctor`, `board_facts`, and `synthesise_self_bringup`. |
      | "`nros build --dry-run` cannot see any of this" | Still true, and it is why 0951 verified with regenerated SystemModels plus `nros codegen entry` before/after, not with dry-runs. |

      The one claim that stands is the first: `[deploy.*]` is upstream's type
      and rlm has no image concept. That did not block the retirement — it
      shaped it. Placement moved to `[host.<name>]`, which is ALSO upstream's
      (rlm v0.1.21), so the projection other repositories consume still exists;
      it is keyed by machine now instead of by a table that also described
      builds.

      **What W10.b therefore means at 0.6.0:** delete the PARSING of a table
      nothing in the tree authors — `SystemToml::deploy`, the deprecation
      lints, the fallback rungs in the readers above, and the `nros check`
      counter. Not a migration; a subtraction. The three surviving
      `[deploy.*]` blocks are in
      `packages/cli/testing_workspaces/orchestration_e2e/nros.toml`, the root
      overlay `cmd/check.rs` already reports as "retired; unsupported and
      ignored" — dead text in a file no reader loads.

      **What it still needs:** out-of-tree users may author `[deploy.*]`, and
      the deprecation warning is what moves them. That warning was never
      emitted until it was wired to `collect_images`, so the clock started
      then, not when the lint was written.

      **Superseded direction (2026-08-31, kept for the record): `[deploy.*]`
      retires ENTIRELY, not just its build fields.** This item was scoped to the build fields, on the standing
      claim that "`[deploy.*]` keeps PLACEMENT and is not being retired" — the
      deprecation message said exactly that, and it has been corrected. Placement
      moved to `[host.*]`, so the eventual state is one table describing a
      buildable unit and another describing where it runs, rather than one
      describing halves of both.

- [x] **W10.c** Add `check-no-tracked-workspace-roots` so the shape cannot
      return. Every gate in this repo exists because a class recurred; this one
      is cheap and the class is "someone re-adds a hand-written root".

      **Done.** The class recurred DURING W10.a: a `git add -A` re-added three
      generated roots (`features`, `realtime-rust`, `sizing`) to the index, and
      `.gitignore` cannot prevent that — it does not apply to a file git already
      tracks. The gate matches exactly one level below `examples/workspaces/`,
      so package manifests are untouched, and it runs its own negative control
      on every invocation (2 shapes that must flag, 5 that must not).
- [x] **W10.e** Retire `resolve_target` — the last identifier asserting the
      concept 0951 removed.

      **Done.** `[system].default_target` was already retired and `--target`
      already only an alias, but the resolver was still called
      `resolve_target` while returning an IMAGE id, and the field it flowed
      through was `PlanOptions.target`. That is the shape issue 0938 cost:
      `nros build` resolving RMW from `[image.*]` while `plan` read
      `[deploy.<t>]`, because "target" and "image" read as synonyms.

      `resolve_target` → `resolve_image`, `PlanOptions.target` → `.image`,
      `plan::Args.target` → `.image`, and the planner's `selected_target` /
      `cli_target` → `selected_image` / `cli_image`. The names converge on what
      `build.rs` and `image.rs` already spelled. **`target` now means the rustc
      triple, and nothing else** — `plan.build.target` still does, which is why
      the field rename mattered: the two sat one struct apart under one word.

      The rename was driven by the COMPILER, not by sed, and that was load-
      bearing: `target: None` appears in several unrelated option structs where
      it IS the triple, so a textual pass would have renamed the wrong ones.
      The compiler named all 21 sites across 9 files, in four rounds — and the
      lines it did NOT name are the proof, being `target: None` in structs
      where target is the triple.

      No behaviour change: `--image` and its `--target` alias parse as before,
      and the deprecated `[deploy.*]` sole-block rung stays (it retires with the
      rest of the table at 0.6.0, W10.b) — now with a comment saying it is
      unreachable for anything this tree authors and why.

- [x] **W10.d** Book sweep: `examples/workspaces/*/README.md` still print the
      six-command ritual this phase deletes.

      **Done — 10 READMEs, zero `nros codegen-system` rituals left.** Two things
      kept deliberately: `nros setup <board>` stays explicit (D2 has the builder
      PROMPT rather than fetch silently, so hiding provisioning would
      misrepresent the first run), and the trailing comments that explain the
      mechanism rather than the ritual.

---

## Measured blast radius (surveyed 2026-08-26, before W1 landed)

Two audits ran before any code was written. Both changed the plan.

**The authored data is far thinner than the schema.** 63 `[deploy.*]` blocks
across 20 `system.toml` files, and **not one** uses `rmw`, `domain_id`,
`locator`, `profile`, `optimize` or `features`. Only `kind`, `board`, `target`,
`launch`, `nodes` and the `.nros` sub-table appear. So W1.f's deprecation of
"build fields inside `[deploy.*]`" is nearly a no-op in practice — `board` and
`target` are the only contested keys.

| class | count | note |
| --- | --- | --- |
| BUILD ONLY (`kind = "embedded"`) | **40** | upstream excludes these from placement |
| BOTH (`self` + ≥1 entry) | 14 | the `native` id in each workspace |
| PLACEMENT ONLY (`self`, no entry) | 9 | `robot1`/`robot2` ×4, plus `smp_bringup:native` |

**Two findings that are latent defects today, not migration work:**

- **14 dangling `deploy = "<id>"` references** — entry packages naming an id no
  `[deploy.*]` declares. `realtime-rust` is the worst: six entries, and its
  bringup has no `[deploy.*]` table at all.
- **8 duplicate-by-alias embedded ids** (`freertos` vs `mps2-an385-freertos`,
  `nuttx` vs `nuttx-qemu-arm`) with identical `board` and identical `.nros`.
  W9 should collapse these rather than migrate both.
- `examples/workspaces/ws-*` (30 dirs) hold **only build artefacts** — no
  `system.toml`, no sources. Untracked residue of phase-331, not workspaces.

### The coupling the D6 argument missed

D6 argued from cardinality that image and deploy are independent concepts, and
that holds. But the **implementation** couples them through the resolved model,
and W2 must not discover this the hard way:

Upstream's `DeployBlock::apply_to_launch` reads BOTH halves — it takes `board`,
`rmw`, `domain_id`, `locator`, `target`, `framework`, `profile`, `optimize`,
`features` and stuffs them into `Deploy.extra` on the resolved model. Both entry
emitters and `nros-macros` then read `execution.deploy[fqn].target` and
`.extra["kind"]` from there.

So moving build fields OUT of `[deploy.*]` silently empties `Deploy.extra`, and
the macro path loses values it depends on. **The schema addition is safe** —
upstream's `SystemConfigToml` is NOT `deny_unknown_fields`, so a top-level
`[image.*]` is ignored by the resolver rather than fatal — but the *migration*
of `board`/`target` is not, and needs one of: passing the image set into
`apply_to_launch` as a second input (an upstream change), or having nano-ros
populate `Deploy.extra` from `[image.*]` after the resolver returns.

**RESOLVED 2026-08-26, and it was smaller than this first read.** Measured
rather than assumed: upstream writes `kind`, `target`, `framework`, `profile`,
`optimize`, `features`, `deploy_name` into `extra`, and nano-ros reads exactly
three keys anywhere in the tree — `kind` (Str), `edf` (Bool), `cores` (Int).

The intersection is **`kind` alone**, and `kind` is PLACEMENT: it stays in
`[deploy.*]`, which is upstream's table and is not being retired. So moving
build fields out empties `extra` of keys nano-ros reads NOWHERE, and no shim is
needed in either direction.

Two by-products worth keeping:

* `edf` and `cores` are read but **cannot be authored** — `DeployBlock` is
  `deny_unknown_fields` and declares neither. A dead knob, reachable only by
  hand-editing a resolved model.
* the proc-macro path does NOT go through `model_ingest::load_model`; it parses
  the YAML itself. Any future patch of the resolved model must target the
  artifact (the `stamp_resolver_pin` precedent), not the loader, or it reaches
  only half the consumers.

Second-highest risk site: `scripts/check-site-config.py --write` GENERATES
`[deploy.<n>.nros]` blocks, so it must be retargeted or the gate will fight the
migration.

## Risks

- **W4 is the corrosion-fragmentation risk.** One cmake configure per workspace
  is what preserves the single cargo tree; a per-package configure would
  re-create the 151.7 GiB duplication RFC-0070 measured. W4.e asserts it
  directly rather than trusting the emitter.
- **W8 may reject the design, not just the code.** F4–F6 came from these two
  projects on a *reading*. Building against them may surface an F11 that needs
  an RFC amendment. That is the wave working as intended, and W8.c says so.
- **W2.e is load-bearing for the RFC-0024 amendment.** If stage 5 ever becomes
  a pipe — for a progress bar, for log capture — the reconciliation that makes
  `nros build` admissible is void. The test is the guard.
- **W9 is nine migrations with one shared cause.** If they land as one commit,
  a regression bisects to a 9-workspace diff. W9.a splits them for that reason
  alone.
- **The phase is large.** W1–W7 is a builder; W8–W10 is a migration. If it must
  be cut, W1–W3 plus W8.a is a coherent shippable subset (Rust workspaces only,
  proven against a hostile tree); W4–W7 then follow per language.

## Issues homed here (survey 2026-09-03)
Every open issue was checked for a home phase; these had none, or were
mentioned here only in passing. A mention is not an owner — an issue with
no work item is an issue nobody is accountable for, which is the same shape
as a gate sitting in a lane no CI job runs. Each row is a work item: the issue
holds the evidence, the item is *close it*.

| issue | why it belongs here |
| --- | --- |
| [#0798](../issues/0798-c-freertos-entry-hardcodes-board-while-root-routes-s32z270.md) | `examples/workspaces/c`'s root routes `s32z270-freertos` to an entry that cannot serve it |
| [#0809](../issues/0809-two-walks-honour-different-nros-ignore-spellings.md) | `provider_scan` honours `NROS_IGNORE` while `nros-pkg-index` does not — two discovery paths, one vocabulary |
| [#0831](../issues/archived/0831-per-image-rmw-is-inert-on-the-cargo-driver.md) | RESOLVED — `[image.<id>].rmw` selects the backend on the cargo driver, and a missing facade is healed or refused rather than warned past |
| [#0849](../issues/0849-nros-sync-bakes-the-invocation-path-spelling.md) | `nros sync` bakes the invocation's path SPELLING into every leaf patch, so a moved checkout resolves wrong |
| [#0914](../issues/0914-resolver-shipped-pair-untested.md) | nothing exercises the SHIPPED resolver + `pyexec` pair, so a resolver that cannot load is indistinguishable from one that works |
| [#0939](../issues/0939-probe-links-node-name-not-component-lib.md) | the metadata probe links the node NAME, which is not a target |
| [#0953](../issues/0953-pyexec-panic-crosses-c-abi.md) | a panic in the Python half crosses `extern "C"` and ABORTS the resolver — the builder's own subprocess boundary |

