# Repository Guidelines

## Project Structure & Module Organization

nano-ros is a Rust workspace for a `no_std` ROS 2 client stack with C/C++ integration. Core crates live under `packages/core/`; RMW backends under `packages/rmw/zenoh/`, `packages/rmw/xrce/`, and `packages/rmw/cyclonedds/`; board/platform support under `packages/boards/` and `config/`; drivers under `packages/drivers/`; and reusable integration tests under `packages/testing/nros-tests/`. Shell and smoke fixtures live in `tests/`. Examples are standalone copy-out projects under `examples/`, with the canonical shape `examples/<platform>/<language>/<example>/`; the RMW is selected at build time.

**Supported hosts: Linux (primary) and *BSD (POSIX path). macOS is NOT supported** (dropped 2026-06-18, phase-260): no macOS CI runner means macOS-specific link/section paths ship un-run, so the project does not carry them. Do not add `APPLE`/`target_os = "macos"`/`env::consts::OS == "macos"`/`*-apple-darwin` branches to nano-ros source, CMake, or CI (issue 0916 — the `"macos"` string spelling survived phase-260's sweep because its acceptance named only `apple`/`darwin`/`APPLE`); embedded RTOS targets + the Linux host are the supported surface.

Reference and contributor docs live in `docs/`; user-facing mdBook docs live in `book/src/`; build orchestration lives in `justfile` and `just/*.just`.

## Design Documents (RFCs)

`docs/design/` is the design source of truth: numbered, living RFCs (`NNNN-slug.md`) with a
`status` of `Draft` / `Stable` / `Superseded`. [docs/design/ARCHITECTURE.md](docs/design/ARCHITECTURE.md)
is the finalized whole-system view; [docs/design/README.md](docs/design/README.md) is the index.
New RFC: copy `docs/design/0000-template.md`.

Two rules:

- **Design rationale goes in an RFC, never only in a phase doc.** Phase docs in `docs/roadmap/`
  are work breakdowns; they name the RFC they implement in an `Implements: RFC-NNNN` header.
- **Drift rule:** flipping an RFC to `status: Stable` requires updating the matching section of
  `ARCHITECTURE.md` in the same commit.

## Build, Test, and Development Commands

**Start with bare `just`** — phase-399 made it print the ~8 verbs you actually
type, grouped by when you need them, instead of the 215-line `--list` wall it
used to be.

The 207 gates live in `just/check.just`, brought in with **`mod check`**, so the
verb is `just check <name>` and the recipes are named for what they check —
`just check fast`, `just check leaf-lockfiles` — with no `check-` prefix on the
recipe itself. The old flat `check-<name>` spelling is GONE; there are no
forwarders, so a stale call site fails loudly rather than resolving to something
else.

Four entry points live in the module beside the gates: `default` (the full tier,
and it must stay FIRST — `just check` runs a module's first recipe, not the one
named `default`), `fast`, `fast-serial` and `build`.

**Adding a fast gate is writing its recipe. There is no list to append to**
(issue 1072). `fast-serial:` used to carry all 218 names as a sorted
dependency list, and that list was the single busiest merge target in the tree
— 15 of the 31 pull requests open on 2026-09-05 touched `just/check.just`, and
two authors adding ALPHABETICALLY ADJACENT gate names insert at the same base
line, which git cannot merge. No `.gitattributes` driver can fix it either,
because GitHub rebases queue entries server-side (issue 0884), so the shared
line was deleted rather than reformatted.

Membership is now DERIVED: a recipe in `just/check.just` is a fast gate unless
it is in `build-serial:`, named in `.config/gate-lane-exempt.txt` with a
reason, or declares parameters (a gate runs as `just check <name>`, with
nothing after the name). `scripts/check/check-gate-lists.py --list <lane>` is
the one place that derivation lives; `run-gates-parallel.sh` and
`check-gate-visibility.py` both ask it rather than parsing the justfile, and it
exits non-zero rather than emitting a short list.

Put a gate in `build-serial:` only when it CANNOT run without something built
— **not** as a default, because since phase-396 W1 that lane runs on
`schedule`/`workflow_dispatch` only and therefore gates nothing a merge passes
through.

- `just --list`: show every recipe (215 of them; the answer is usually bare `just`).
- `scripts/bootstrap.sh`: first-time entrypoint; installs/checks `just`, then prints setup choices.
- `scripts/bootstrap.sh base`: first-time native/ROS/zenoh quick-start setup.
- `scripts/bootstrap.sh all`: contributor/full-matrix setup; pulls and installs every supported SDK tier.
- `scripts/bootstrap.sh platform <platform>`: first-time focused setup for one platform.
**One verb, one scope, one position (phase-411 W3).** `setup`, `doctor`, `build`
and `test` all take the same word in their first argument: a PLATFORM (`native
zephyr freertos nuttx threadx_linux threadx_riscv64 esp32 esp_idf qemu px4 xrce
cyclonedds`) or a PRESET, which today is exactly the fixture lanes (`all native
tier1 tier2 tier2-nightly`). `just setup zephyr` was already this shape; the
other three now follow it. The vocabulary is `scripts/build/scope.sh`, one
namespace, gated by `check-scope-namespace` — a preset may share a platform's
name only while it denotes the same scope, which is why `native` is legal as
both. `NROS_SCOPE_EXPLAIN=1 just build tier2` prints the command a scope
resolves to without running it.

Naming a scope IS the specification of what you asked to be covered; a bare
`just test` is best effort over what is provisioned, and prints the probed set
rather than reading a recorded one. `just <platform> <verb>` still works and
still is the implementation — it prints a deprecation for one release.

- `just setup`: print setup choices; does not fetch/install.
- `just setup base`: install the base quick-start SDK/tooling tier.
- `just setup all` or `just setup tier=all`: install the full contributor/test-all tier.
- `just setup <scope>`: install a focused platform SDK/tooling tier.
- `just doctor`: host tooling, then a PROBE of every platform (the derived default scope).
- `just doctor <scope>` / `just doctor tier=all`: one scope's readiness, or the pre-407 tier walk.
- `just build <scope>`: that scope's test fixtures — a lane routes through
  `build-test-fixtures lane=<lane>` (which stamps coverage), a platform through
  `just <platform> build-fixtures` (which does not).
- `just test <scope>`: that scope's tests. Verbosity is a FLAG now (`verbose` /
  `-v` / `--verbose`, anywhere in the arguments) — `just test 1` no longer means
  verbose, which is the incompatibility phase-411 accepts.
- `just build`: build the workspace plus generated bindings and transport artifacts.
- `just build-examples`: build the workspace and example matrix.
- `just build-test-fixtures`: prebuild binaries required by the full test matrix.
- `just build-all`: run the broad build tier; it auto-routes through the GNU make jobserver path when the pinned make/ninja tools are available.
- `just <platform> build`, `just <platform> build-examples`, `just <platform> build-fixtures`, `just <platform> build-all`: run platform-scoped tiers first when a platform-specific failure appears.
- `just test-unit`: fast workspace unit tests.
- `just test`: standard dev tier; skips heavy platform/ROS 2 groups.
- `just test-all`: full matrix, doctests, Miri, and C codegen tests. Run `just build-test-fixtures` first.
- `just check`: formatting and clippy checks across Rust, C, C++, and Python surfaces.
- `just ci l1`: `check` plus `test-unit`. ~6 min, and it builds **no fixtures** —
  only `test`/`test-all` require them. This is the verb to run before a push
  (phase-395 W2); it catches the compile-tier reds that the `pre-push`
  `check-fast` hook deliberately excludes.
- `just ci`: `check` plus `test-all`. Requires a fixture build; run it when you
  need fixture-backed coverage.

Treat `<platform>` as target families such as `qemu`, `zephyr`, `freertos`, `nuttx`, `threadx_linux`, `threadx_riscv64`, `esp32`, or board groups. Support services such as `zenohd`, `cyclonedds`, and `xrce` are not platform scopes.

Codex sandbox notes:

- `just` may fail before running a recipe if the default runtime
  directory is read-only, with an error about creating a temporary
  directory under `/run/user/.../just`. In that environment, rerun with
  `XDG_RUNTIME_DIR=/tmp`.
- Cargo commands inside `just` may need to update the user's registry
  cache under `$HOME/.cargo`. If a recipe fails with read-only
  filesystem errors in `.cargo/registry`, rerun the same command with
  sandbox escalation rather than treating it as a project failure.
- A failed pre-nextest Cargo setup can leave an old
  `target/nextest/default/junit.xml` in place. If a recipe prints
  slow-test output after such a setup failure, verify whether nextest
  actually ran before trusting the timing report.

## Coding Style & Naming Conventions

Rust uses edition 2024 and `rustfmt.toml` with nightly-only formatting options. Use `cargo +nightly fmt` or `rustup run nightly cargo fmt`; stable rustfmt produces different output. C and C++ follow `.clang-format` based on LLVM, 4-space indentation, and a 100-column limit. **clang-format output drifts across major versions** (e.g. v17 vs v22 reformat `reinterpret_cast<T(*)[N]>` differently → spurious `just format`/`check-{c,cpp}-fmt` diffs), so the version is **pinned in `.clang-format-version`** and provisioned by `just setup-clang-format` as a PROJECT-LOCAL binary at `build/clang-format/bin/clang-format` (the standalone binary extracted from the exact-version PyPI `clang-format` wheel — no venv, no `pip install`, nothing user-wide; like `build/qemu`). Run as part of `just setup`. The `format-*`/`check-*-fmt` recipes resolve that pinned binary via `scripts/dev/clang-format.sh` (`nros_clang_format`), falling back to a PATH `clang-format` with a loud version-skew warning. `just doctor` reports the pin status. Keep crate names and package paths in the existing `nros-*`, `zpico-*`, backend-specific, and platform-specific patterns.

Project naming:

- `nano-ros`: prose and docs.
- `nros`: crates, Rust/C identifiers, and `CONFIG_NROS_*`.
- `nano_ros`: C header dir, CMake targets, and CMake helpers such as `nros_generate_interfaces()`.

## Testing Guidelines

Prefer the narrowest tier that covers the change. Reusable Rust integration tests belong in `packages/testing/nros-tests/tests/`; shell tests belong in `tests/`; temporary tests can start as Bash and should be promoted when reused. Tests must fail on unmet preconditions with `assert!`, `bail!`, or the project skip helper; do not silently `eprintln!` and return from a test.

**New runtime tests join a matrix, not a new file.** A (platform × language × RMW × workload) coordinate belongs as a cell in `matrix::CELLS` (self-contained/baked) or `interop::CELLS` (live ROS 2 peer + direction), consumed by the existing rstest consumers — phase-331 W4 put workspace RMW cells there too. Hand-coordinated per-cell test files are the pre-RFC-0051 shape the tree keeps having to re-consolidate (phase-295, phase-329); the taxonomy and the fold-in plan are `docs/roadmap/phase-329-test-taxonomy-completion.md`. Fixture builds are **lane-scoped** (#393): `just build-test-fixtures lane=<all|native|tier1|tier2|tier2-nightly>` builds exactly the lane's coordinates and stamps the scope; build the lane you will test rather than the whole matrix. **But a lane's coordinates are what it keeps FRESH, not what its run needs to EXIST** (#482) — the two are separate questions, mapped by `nros_lane_build_lane` (declared by `CiLane::run_scope`), and `_require-fixtures` refuses a mismatch instead of letting the run discover it 231 failures later. Each lane narrows its run in the way its cost lives: tier 1 by test/binary NAME (`NROS_TEST_SCOPE=native`), so it needs the broader `lane=native` build; tier 2 and the nightly lane by fixture COORDINATE (`NROS_TEST_COORDS`, phase-340 W3), so each is its own build lane — `just build-test-fixtures lane=tier2 && just ci matrix`. Name filtering cannot express tier 2: it is 1-wise over platform, so every platform is in the lane and a platform-token filter excludes nothing, while the saving is in lang x rmw *within* a platform (#357). The coordinate narrowing therefore happens in the fixture RESOLVER (`nros_tests::fixtures::lane`), which attributes a resolved artifact back to its `examples/fixtures.toml` row through `row_artifact_root()` — the sibling of `row_coord()`, so the BUILD's `--coords-from` filter and the RUN's skip are one predicate over one coordinate file. Out-of-lane fixtures report `[SKIPPED] out of lane`; an IN-lane fixture that is absent or stale still fails hard, and a path no manifest row claims (zephyr west leaves, the compile-check lane, the shared `build/cargo-fixtures` dirs — all built module-level) is never skipped.

**No compilation inside tests.** A test must not invoke `cargo build`, `cmake --build`, `idf.py build`, `west build`, `nros generate` + compile, or any other compiler/build at run time. Compilation belongs in the **build stage** — `just build-test-fixtures` and the per-platform `build-fixtures` lanes (driven by `examples/fixtures.toml`). A test consumes a **prebuilt fixture artifact** and exercises its behavior. Reasons: in-test builds make the test wall-clock dominated by compile time (so they blow the per-test timeout under any load and report as spurious `timed out` failures), serialize on the cargo/cmake build locks, and conflate "does it build" with "does it behave". If a test's *intent* is to verify that something compiles (a macro form, a codegen output, an API shape), make it a **fixture in the build step** — add a row to `examples/fixtures.toml` (or a build-lane target) so the artifact is built once during `build-test-fixtures`, and have the test assert the fixture exists / inspect the built artifact, the same way the native C/XRCE tests consume their prebuilt CMake fixtures. The build either succeeds (fixture present → test checks it) or fails loudly in the build stage where it belongs.

**Test names describe behavior, not phase numbers.** Name a test for what it verifies, e.g. `zephyr_xrce_service_request_reply_e2e`, `rust_talker_to_cpp_listener_delivers`, `main_macro_accepts_no_arg_form`. Do **not** encode roadmap phase numbers in test names or test-file names (`phase212_n9_main_macro_forms`, `phase217_c_fvp_runtime`); phases are planning artifacts that go stale, and a phase-numbered name tells a future reader nothing about what broke. Cross-reference a phase in a doc-comment if useful, not in the identifier.

**Fixture prerequisites are provisioned by `nros setup`.** The build-stage test fixtures (`build-test-fixtures` + `scripts/build/compile-check-fixtures.sh`) need the cross toolchains, `play_launch_parser`, `corrosion`, `cmake`, etc. These are installed by `nros setup` / `just setup all` (RFC-0014), NOT built ad-hoc. Before a fixture build or a `test-all` run, ensure they are present with `just doctor tier=all` (it lists every tier's prereqs as `[OK]` / `[MISSING]`); run `nros setup <board>` / `just setup all` to fill gaps. A fixture that can't build because a toolchain is absent is an environment gap to fix via setup — not a per-test workaround. (If you find a prereq a fixture needs that `nros setup` does not provision, add it to the SDK index / setup flow rather than hand-installing it.)

For platform failures, rerun the closest platform recipe first, for example `just zephyr build-all`, `just freertos build-fixtures`, or `just qemu build`, before spending time on root `just build-all`.

Native C/XRCE tests are runtime-only and consume prebuilt CMake fixtures under `examples/native/c/{talker,listener}/build-xrce/`. If `c_xrce_api` fails with `Test fixture binary not prebuilt`, run `just native build-fixtures` or `just build-test-fixtures` before debugging runtime behavior. A focused verification of phase 177.9.C passed with `just native test-c-xrce verbose` after fixture prebuild.

**Multi-edition ROS interop (RFC-0058 / phase-309).** A host installs exactly one ROS 2 edition (humble→22.04, jazzy→24.04 — apt trees don't coexist), so codegen + interop against OTHER editions run in per-edition docker images through the `nros_tests::ros_env::RosEnv` provider (test-only; never in the product/CLI/user build). `HostRosEnv` sources the host `/opt/ros/<distro>` (the DEFAULT edition, today's `ros2.rs` path, unchanged); `DockerRosEnv` runs commands in a locally-built `nano-ros-ros:<distro>` image (`--network host`), spawns peers + `domain_bridge` (RAII: killing `docker run` does NOT stop the container, so `RosPeer` `docker kill`s a named container on drop), and runs `nros generate-rust` in-container (host `nros` bind-mounted — glibc is backward-compatible). These lanes are **opt-in**: excluded from `just ci`; run them with `just ros_editions ci <distro>` (builds the image on demand, then the smoke/codegen/bridge tests). To add an edition, run the recipe with a new distro arg — W2–W5 are distro-parametric. Adding a live nano-ros publisher (built against an edition's generated code) to the bridge lane is the remaining product-interop step; the cyclone wire path is already edition-compatible (proven by issue 0267 against live jazzy). **Rich E2E (phase-310):** `ros_editions_e2e_{pubsub,service,action}.rs` cover pub/sub, service, and action in BOTH directions (nano-ros ↔ live edition peer, direct same-domain cyclone), driving the six native example nodes built per-edition by `just ros_editions build-e2e-fixtures <distro>` against a container-side `ros2` CLI + rclpy servers. Nano-ros nodes are wrapped `bash -c 'exec … 2>&1'` so `env_logger`'s STDERR markers reach the captured stdout. All lanes fold into `just ros_editions ci <distro>` (jazzy + iron). **RMW axis (phase-311):** the harness crosses the edition axis with `Rmw` {Cyclone, Zenoh, Xrce}. Fixtures build per-(edition, rmw) into `target-ros-edition-<distro>-<rmw>/` (`build-e2e-fixtures <distro> <rmw>`; cyclone lanes now use the same `-cyclone` suffix). Delivered matrix: **{jazzy, iron} × {cyclone, xrce}**, both directions, pub/sub + service + action, green. The **XRCE lane** (`ros_editions_xrce.rs`) runs the relocatable micro-XRCE Agent from the nros SDK store ON THE HOST (`spawn_xrce_agent`; the launcher resolves its bundled Fast-DDS libs) bridging a nano-ros `rmw-xrce` node ↔ a `rmw_fastrtps_cpp` container peer (`Middleware::FastRtps`, `--network host`, same domain). **Zenoh lane deferred on issue #0291** (zpico pins zenoh 1.7.2; a stock jazzy `rmw_zenoh_cpp` is 1.11.2 — needs a zpico bump). **`NROS_RMW` footgun:** `just ros_editions ci` exports `NROS_RMW=<token>` as a lane selector, and that env LEAKS into the spawned nano-ros node, where `Executor::open` resolves the backend by matching the value against the backend's REGISTERED name. Cyclone registers as **`cyclonedds`**, NOT the short harness token `cyclone` — an ambient `NROS_RMW=cyclone` mis-resolves → `Transport(ConnectionFailed)` (a silent, discovery-looking red). So `nano_node_cmd`/`nano_node_cmd_rmw` set `NROS_RMW` EXPLICITLY to `Rmw::nros_rmw_name()` (the canonical `cyclonedds`/`zenoh`/`xrce`), never inheriting the ambient value. **Zenoh lane (`ros_editions_zenoh.rs`, jazzy):** zpico node ↔ a host-net `rmw_zenohd` (`spawn_zenoh_router`, readiness-polled) ↔ a `rmw_zenoh_cpp` peer. Only jazzy ships `rmw_zenoh_cpp` (iron/humble skip loudly). 5/6 green — ROS→nano action SERVER is #0292 (`#[ignore]`). **The ROS edition is selected on the standalone examples like the RMW** — a `ros-humble`(default)/`ros-iron`/`ros-jazzy` passthrough feature forwarding to `nros` + `nros-rmw-zenoh?` (NOT a hardcoded `ros-humble` on the `nros` dep); `build-e2e-fixtures <d> <rmw>` regenerates msgs per edition (real RIHS01) + builds `--features "rmw-<x> ros-<d>"`. This is what puts the real RIHS01 type-hash tail in the zenoh keyexpr (compile-time cfg-gated on `ros-iron`/`ros-jazzy` in `nros-rmw-zenoh`). **#0291 lesson:** zenoh's wire is proto `0x09`-stable across 1.x, so zpico 1.7.2 interops with jazzy's 1.11.2 — a version bump was NEVER the fix; the keyexpr type-hash was. **zpico domain gotcha:** the native zpico zenoh node uses a COMPILE-TIME domain (0) — it ignores `ROS_DOMAIN_ID`/`NROS_DOMAIN_ID` at runtime (unlike cyclone), and the domain is the first keyexpr segment, so a zenoh peer must run on domain 0 to match.

## SDK Environment Defaults

Keep repo-local SDK defaults centralized in `just/sdk-env.just`. This includes paths such as `FREERTOS_DIR`, `NUTTX_DIR`, `THREADX_DIR`, `PX4_AUTOPILOT_DIR`, `NROS_ESP_IDF_WORKSPACE`, `NROS_ESP_IDF_ENV_SHIM`, and `IDF_PATH`. Local overrides belong in `.env` or the caller environment.

Do not duplicate those defaults in package code, tests, examples, CMake, or scripts. Packages and examples must remain position-independent: they may read explicit environment variables and should skip or fail with a clear setup hint when a required SDK variable is absent, but they must not assume the checkout lives at a particular repo-relative path.

Shells that need the same defaults should source `scripts/sdk-env.sh`, which evaluates `just/sdk-env.just` and exports only missing variables. `.envrc`, `setup.bash`, and `setup.fish` all use that adapter. When a direct `cargo test` or `cargo nextest` run needs these defaults, either source `scripts/sdk-env.sh` first or run it through a `just` recipe so `just/sdk-env.just` is imported and exported to the child process. Prefer adding a focused `just` test helper over adding repo-path fallbacks inside `packages/`.

## Build-Cache Root (RFC-0070)

`NROS_BUILD_ROOT` names the ONE directory every build cache lives under; it
defaults to `<repo>/build`, so an unset environment behaves exactly as before.
Set it to move the whole cache tree to a faster or larger volume — the
generalisation of what `NROS_ZEPHYR_BUILD_ROOT` already does for one family.
This matters more than it sounds: the native fixture lane alone holds ~115 GiB
of target dirs, and phase-340 measured the checkout's volume at 96 % full.

Scripts must derive cache paths through `scripts/build/build-root.sh`
(`nros_build_root`, `nros_build_dir <kind> [<coordinate>...]`) rather than
spelling a literal. A cache directory is `<root>/<kind>/<coordinate>` where the
coordinate uses the fixture-manifest vocabulary (platform, lang, rmw,
feature-sig); a new ad-hoc suffix is a bug, not a naming choice.

The build, the staleness gate and the test resolver must call the SAME
derivation — the #393 rule applied to paths. `check-build-root` (in
`check-fast`) asserts the derivation's output, and the migration is deliberately
ordered derivation → callers → paths → gate, because there are still 236
hardcoded cache-path literals across 17 files. Do not move a directory before
its readers agree on where to look. → RFC-0070, phase-334 W2.

## Toolchain & SDK Provisioning

Design rationale → RFC-0014 (`docs/design/0014-nros-setup-toolchain-management.md`). Operational
contract:

Host toolchains/tools (`qemu`, cross-GCC, `openocd`) are provisioned by `nros setup`,
not built ad-hoc. `nros-sdk-index.toml` (repo root) is the SSOT: each `[tool.*]` has a per-host
sha256-pinned prebuilt `dist` **and** a `[tool.*.source]` recipe; `[source.*]` build with the app;
`[gated.*]` (NVIDIA SPE, ARM FVP) are never fetched, only instructed. Prebuilt assets live on the
separate `NEWSLabNTU/nano-ros-sdk` repo's Releases (referenced by URL, not a submodule).

- `nros setup <board>` / `nros setup --tool <name> [--prefix <dir>]` installs to
  `$NROS_HOME/sdk/<tool>/<version>/` (`.nros-provenance` + `nros-sdk.lock` record it).
- **`nros` is the single SDK resolver:** platform builds lazy-install a board's index tools on
  first use (`setup::ensure_tools`; opt out via `NROS_NO_AUTO_SETUP`) and prepend the locked store
  `bin/` to the child PATH. (The former `nros build`/`deploy` verbs were retired in Phase 222 —
  `nros doctor` lints for leftovers.) **Non-`nros` scripts, the test harness, and
  CMake do NOT resolve SDK paths — they assume the SDK is given and only check + warn**
  (`nros doctor` / `just <plat> doctor`). Do not re-add store-path probing to test code.
- `just qemu setup-qemu` / `just zenohd setup` are **thin `nros setup --tool` callers** (install
  into `build/<tool>` where the harness looks). Do not reintroduce the in-tree configure/make or
  the `third-party/qemu` submodule build. `just <tool> build` still source-builds for devs.
- ESP32 = **ESP32-C3 (RISC-V)** (`riscv32imc-unknown-none-elf` via rustup + build-std, espflash,
  Espressif `qemu-system-riscv32` fork). Needs no index host-tool.
- CI gate: the `sdk-index` job in `.github/workflows/gate.yml` sha256-verifies any index `dist` change. Bump a
  tool's `-nros<N>` suffix when rebuilding the same upstream version with different config.

## C/C++ Integration Shape

C and C++ consumers use source-tree CMake integration, not an installed package. The expected pattern is:

```cmake
set(NANO_ROS_PLATFORM <platform>)
set(NANO_ROS_RMW <zenoh|xrce|cyclonedds>)
add_subdirectory(<repo-root> nano_ros)
target_link_libraries(<app> PRIVATE NanoRos::NanoRos)
nros_platform_link_app(<app>)
```

Use `NanoRos::NanoRosCpp` for the C++ API where needed. There is no supported `find_package(NanoRos)` path and no `just install-local` flow. Per-platform CMake glue lives under `cmake/platform/`; RTOS-native integration shells live under `integrations/<rtos>/`.

Never hard-code project-relative paths in example CMake, package CMake, build scripts, or in-tree tooling. The outer build driver should pass SDK paths and selection via cache variables or environment variables such as `NANO_ROS_PLATFORM`, `NANO_ROS_RMW`, `CMAKE_TOOLCHAIN_FILE`, `<SDK>_DIR`, or board-specific config paths.

## Examples and Generated Content

Each `examples/` directory is a standalone copy-out template. Do not rely on workspace walk-up behavior from an example. Non-example test, benchmark, and smoke binaries belong under `packages/testing/{nros-tests/bins,nros-bench,nros-smoke}/`, not under `examples/`.

**Workspace examples (RFC-0066 / phase-331, migration in flight).** The target is ~12 workspaces: the four large language workspaces (`workspaces/{rust,c,cpp,mixed}`, read side-by-side — keep their `src/` listings parallel), the native-only `features/` capability-demo workspace, `safety/`, `ws-managed-cpp`, and a reduced realtime set. The rules:

- **A feature is a node package, not a workspace.** New capability demos (params, lifecycle, QoS, custom messages, remapping) go into `workspaces/features/` as node pkgs registered in its `demo_bringup`. Do not create or extend a themed `ws-<feature>-<lang>` workspace — the remaining ones are scheduled for deletion (W3), and tests that name them will be re-pointed at the consolidated workspaces.
- **A configuration is a fixture axis, not a directory.** RMW and feature-set variants of a workspace are `[[workspace_fixture]]` rows (`(workspace) × (rmw) × (feature set)`) declared as cells in `matrix::CELLS` (phase-331 W4), never a copied directory.
- **Naming**: no language prefixes inside single-language workspaces; role-based pkg names (`service_server_pkg`, not `add_server_pkg`); one platform vocabulary for entries (`zephyr_entry`, `nuttx_entry`, …). Node names/executables are ROS wire identity and are NOT normalised. Full table + known coverage gaps: `examples/workspaces/README-layout.md`.
- **Capabilities are IMAGE-level**: `param_services`/`lifecycle` are alloc-gated; `features/` stays native-only so embedded entries in the language workspaces never inherit them implicitly. Declare capabilities in the SYSTEM (`system.toml` — `[param_services]`, `[system].features`), which the resolver projects into `execution.features`.
- **West-built Zephyr entry leaves** (staticlib apps, `west` not cargo) must be `exclude`d from their nested workspace AND listed in the repo-root `Cargo.toml` exclude block, carry NO own `[workspace]` table, and name their generated selection dep exactly `<entry>_nros_selection` (the generated package name). Missing any of these breaks `just zephyr build-fixtures` for every workspace entry after it.

**System models are resolver output (phase-330).** Scheduling/placement dims, capabilities, and deploy config are authored in `system.toml`; the committed `config/system_model.yaml` is regenerated from it and is transitioning to a pure build artifact (generated into the active build's output dir — the W3 seam). Never hand-edit dims into a model yaml: four separate regenerations have silently deleted hand-authored model content (issue 0380). If a dim can't be expressed in `system.toml`, that is a schema gap to fix in the resolver, not a reason to hand-edit the model.

**Message dependencies are path deps (RFC-0067 / phase-333).** A leaf declares `std_msgs = { path = "generated/std_msgs", version = "0.0.0" }`-style deps; never a bare registry name (issue 0378: a bare `std_msgs = "*"` resolved against the PUBLIC crates.io). Generated message crates are version `0.0.0` (ament version lives in metadata). All `packages/testing/{nros-bench,…}` leaves track their `Cargo.lock`; the `--locked` cargo shim refuses stale tracked locks — refresh the lock in the same change that bumps versions.

Do not modify vendored or generated content under `third-party/`, `packages/interfaces/*/generated/`, or build output directories unless the task explicitly requires regeneration. Generated message code should come from the nano-ros codegen tools, not hand-written edits.

## RMW and Platform Notes

Active RMW choices are `zenoh`, `xrce`, and `cyclonedds`; the legacy dust-DDS backend was retired. Platform choices include POSIX, Zephyr, bare-metal, FreeRTOS, NuttX, and ThreadX. RMW backend registration must be explicit on targets such as Zephyr/native_sim; do not assume POSIX-style Rust constructors or linker sections run there.

For Zephyr XRCE C++ service/action work, keep `nros_cpp_spin_once` routed through `executor.spin_once`. Do not reintroduce a `drive_io(0) + msleep` shortcut; that path starves reliable XRCE streams and skips arena dispatch.

CycloneDDS work is active. Native C++ action result/feedback paths have recent fixes, but stock ROS 2 interop and some embedded Cyclone paths remain ongoing work. Pure-Cargo Rust Cyclone examples are not the supported path; use the CMake/Corrosion route for Cyclone.

## Git and Worktree Rules

Preserve existing user changes in the worktree. Do not revert unrelated changes. Use linear history when integrating remote changes: `git pull --rebase` or `git fetch` plus `git rebase`; create merge commits only when explicitly requested.

When pulling or rebasing the superproject, inspect submodule changes. If a pull advances a submodule pointer and local work exists in that submodule, enter the submodule, fetch its remote, rebase local work onto the updated upstream commit, check out the commit expected by the superproject, and record the resulting submodule commit in the parent commit.

After rebasing over a remote submodule-pointer change, run `git submodule status --recursive <path>` and update the checkout to the commit recorded by `HEAD` before pushing. Recent pulls advanced `third-party/dds/cyclonedds`; leaving the worktree at the old detached commit made the superproject appear dirty even though the parent commit was correct.

## Branch policy for `main` (GitHub ruleset `main-rules`)

`main` is protected by a **ruleset**, not classic branch protection. The two are
separate systems that do not read each other, so "is the branch protected?" has
to be asked of the right one: `gh api repos/NEWSLabNTU/nano-ros/rulesets`, not
`.../branches/main/protection` (which returns `Branch not protected` and means
nothing here).

### In force now

| rule | effect |
| --- | --- |
| `required_linear_history` | a **merge commit cannot be pushed to `main`** |
| `non_fast_forward` | no force-push to `main` |
| `deletion` | `main` cannot be deleted |

Scope is `refs/heads/main` only, and **`bypass_actors` is empty** — nobody,
including admins, is exempt. A rule everyone can bypass is not a rule.

### DIRECT PUSH TO `main` NO LONGER WORKS (live since 2026-08-28)

`git push origin main` is REJECTED:

```
remote: - Changes must be made through a pull request.
remote: - Changes must be made through the merge queue
! [remote rejected] main -> main (push declined due to repository rule violations)
```

That is not a misconfiguration and there is no override — `bypass_actors` is
empty, admins included. **The flow is now:**

```
just claim issue-NNNN                  # so two agents do not do one job
git switch -c fix/NNNN-<slug>
# ... work ...
just ci l1                             # STRICTER than the gate (see below)
git push -u origin fix/NNNN-<slug>
gh pr create --base main --fill
gh pr merge --auto --rebase            # fire and forget; the queue lands it
just claim-release issue-NNNN
```

`gh pr merge` prints *"The merge strategy for main is set by the merge queue"* —
that is INFORMATIONAL and the PR is queued. A FORCE-PUSH cancels auto-merge, so
re-run `gh pr merge --auto --rebase` after any amend.

**Enable auto-merge when you OPEN the pull request, not when it looks ready**
(phase-396 W2). This is the line that makes the queue a queue. While each author
arms their own PR by hand, only one entry is ever in flight, `max_entries_to_build`
never engages, and the expensive tier is paid per pull request instead of per
batch — which is the whole reason the queue exists. Measured: every merge group
this repository ran before 2026-08-29 contained exactly ONE pull request.

**To take a PR back OUT of the queue, `--disable-auto` is not enough.** It
reports *"already queued to merge"* and leaves the entry in place. Use the
**Remove from queue** button on the PR, or:

```
nid=$(gh api graphql -f query='{repository(owner:"NEWSLabNTU",name:"nano-ros"){pullRequest(number:NN){id}}}' --jq '.data.repository.pullRequest.id')
gh api graphql -f query="mutation { dequeuePullRequest(input:{pullRequestId:\"$nid\"}) { mergeQueueEntry { position } } }"
```

You need this more often than it sounds. A queue entry that cannot pass holds
everything behind it (see below), and the entry is frequently one you already
superseded by stacking.

### The one required check is `CI`

One aggregator job (`ci-ok`, context **`CI`**) gates everything. It `needs:`
every job, runs `if: always()`, and inspects `needs.*.result`. Individual jobs
may be skipped, path-filtered or renamed freely — only `CI` is required.

**So do not add a job name to the required set, ever.** The reason is a defect
that froze this repo four ways in one day: a required check that produces no
VERDICT blocks forever, because GitHub cannot tell "not applicable" from "not
yet". Path-filter a required workflow and a pull request touching other paths
can NEVER merge — two PRs deadlocked on each other that way (#6 and #16).
`scripts/ci/enable-merge-queue.sh` refuses to make a path-filtered context
required, and will tell you so.

### You WILL be told if the queue ejects you

GitHub records an ejection only in the PR timeline — no comment, no
notification, nothing that reaches an agent who has moved on. `queue-notify`
closes that: when a merge-group run fails it comments on the ejected pull
request with the run log and the triage command.

It is a SEPARATE workflow on `workflow_run`, not a step inside the failing run,
because a failing run cannot reliably report on itself — the failure may be the
runner, the container, or a cancellation, and a notifier that shares a fate with
the thing it reports is not a notifier.

So: an `OPEN` PR with no ejection comment is still queued. One WITH a comment is
waiting on you, and the comment says what to do.

### If everything is blocked and the blocker is CI itself

That is a real state, not a mistake you made — the fix for CI can be blocked by
CI. It happened on 2026-08-28. The break-glass, in order:

1. **Confirm it is not just you**: `just queue-triage`. The same check failing
   for several DIFFERENT pull requests is infrastructure, not your change.
2. **Say so** on the issue or PR thread before anyone else burns an hour on it.
3. **Drop only the required-checks RULE**, never the ruleset and never
   enforcement — the other guardrails (no force-push, no merge commit, PR
   required) must stay on. Read the ruleset, remove that one rule, PUT it back:
   `gh api repos/<owner>/<repo>/rulesets/<id>` -> edit -> `-X PUT --input`.
4. Land the unblocking PR, then **restore immediately** with
   `scripts/ci/enable-merge-queue.sh --apply --with-queue`.

Keep the window minutes, not hours, and say when you open and close it. Note
that even with the check dropped you STILL cannot push to `main` — the PR and
queue rules are independent, which is the point.

### What the ruleset does NOT touch, verified

Branch rulesets target `refs/heads/**`, so the custom refs the coordination
primitives depend on are unaffected — **verified by pushing and deleting
`refs/issue-ids/9999` under the active ruleset**, both rc 0. So `just issue-new`
and `just claim` keep working, and a `deletion` rule on `main` does not stop a
claim being released.

Agent branches are also untouched: the ruleset targets `refs/heads/main`, NOT
`~ALL`. Force-pushing your own `fix/<id>` branch while rebasing is still fine,
which is the reason `~ALL` was rejected — it would have broken every agent and
every outside contributor on day one.

### Head-of-line blocking IS real, and it is not the same as a failing PR

GitHub's queue ejects a pull request whose own merge group fails and re-tests
the rest without it. That is the change-specific case, and it works. What it
does NOT do is skip past an entry that is simply *taking forever*:

```
pos=1  AWAITING_CHECKS  #21     <- grinding through a check that cannot pass
pos=2  MERGEABLE        #22     <- green, and waiting
```

A `MERGEABLE` entry behind an `AWAITING_CHECKS` one waits for it — up to
`check_response_timeout_minutes` (60). Observed 2026-08-29: #22 was green for 30
minutes and merged only once #21 was removed by hand.

So when your PR is green and not merging, **read the queue before assuming your
PR is the problem**:

```
gh api graphql -f query='{repository(owner:"NEWSLabNTU",name:"nano-ros"){mergeQueue(branch:"main"){entries(first:10){nodes{position state pullRequest{number}}}}}}'
```

If something ahead of you is stuck, dequeue it (above) rather than waiting out
the timeout — especially when it is a PR your own stack already contains, which
is the common case.

## Where each tier runs, and why your local run is load-bearing

The merge queue batches up to **five** pull requests so a tier is paid once per
batch instead of once per pull request. The one required status context does
DIFFERENT WORK depending on the event:

**The expensive tier is NOT on the merge group** (phase-396 W1). It was, and it
could never pass there: `check-build` ends with `native::check`, which requires
generated message bindings, and includes `check-source-gates`, which requires
prebuilt `.compile-ok` stamps — and that CI job builds neither. A required check
red for every input looks exactly like fourteen broken pull requests, and froze
merging for a day. The heavy tier runs post-merge in `post-submit.yml` instead,
which is the standard split: required = cheap and deterministic, heavy =
post-merge with a revert path. `check-lane-contracts` now enforces it — a CI job
may resolve an artifact only if that JOB builds it.

| stage | runs | measured | gates? |
| --- | --- | --- | --- |
| local, before push | `just ci l1` | ~6 min warm | you |
| pull request | `check-fast` only (133 source gates) | ~5 min | **required** |
| merge group | + `test-unit` | ~9 min | **required** |

**`just ci l1` is NOT what CI runs, and that is deliberate** (phase-399 W3).
CI's required context is `check-fast` + `test-unit`; `ci-l1` additionally runs
`check-build` and `check-api-parity`. So the local tier is a SUPERSET of the
gate — you catch compile-tier breakage before the queue does, and the queue
stays cheap and always-satisfiable. This line used to say "the SAME tier the
merge group runs", which stopped being true when phase-396 W1 took `check-build`
off the merge group (it could never pass there — it needs generated bindings and
prebuilt `.compile-ok` that no CI job builds).

**Exactly one CI job builds a fixture:** `post-submit`'s `build-test-fixtures
lane=tier2`, after merge. Everything pre-merge is fixture-free by construction,
which is why `ci-l1`'s "NO FIXTURES" claim has to hold and why
`check-lane-contracts` enforces it — a CI job may resolve an artifact only if
that job builds it.
| push to `main` | `host-tests` (L2) | ~15 min | no |
| schedule | `nightly` (L3/L4 matrix) | hours | no |

Measured on the hosted runner: of an 878 s gate, `check-fast` is 131 s and
`check-build` is 587 s. Moving that 587 s off the per-PR path is most of the
latency, and it costs nothing in coverage because the merge group still runs it
before anything lands.

**No stage in the merge path builds fixtures.** `check-lane-contracts` enforces
that: a lane in an affordability tier may resolve a compile-stage stamp only if
it builds it (~13 s), and may never reach a runtime fixture. Fixture-bearing
lanes are post-merge.

### Why `just ci l1` locally is not optional

The PR gate no longer compiles. That means **a compile error in your branch is
caught by the merge queue, not by your PR** — and there it ejects the other
pull requests batched with it. GitHub re-tests them in smaller groups, so the
damage is bounded, but it is real and it is paid by other agents.

`just ci l1` is exactly the tier the merge group runs. Running it before you
push is what makes the cheap PR gate affordable for everyone else. Skipping it
does not save you time; it moves your time onto three other agents.

## Stacking: when PR B builds on PR A

**Base B on `fix/a`, never on `main`.** The merge queue is FIFO by entry time and
understands nothing about branch relationships, so ordering is something you
arrange or do not get.

If B's base is **`main`** while its branch contains A's commits, B's diff is
`A + B`. That mostly works — and fails in the one case that matters: **if A is
rejected, its commits are still in B's branch, so they land anyway when B
merges.** The queue did its job and the change shipped regardless. Nothing in
the tooling catches this; the diff is legitimate and the queue has no way to
know those commits were someone else's rejected work.

If B's base is **`fix/a`**, ordering is structural. The ruleset targets
`refs/heads/main` only, so a pull request into `fix/a` has no required checks and
no queue — it merges into A freely, and A then carries both into the queue as one
unit. When A lands and `fix/a` is deleted, GitHub retargets B to `main`
automatically, and B queues with only its own changes.

**Simplest of all: do not stack.** Land A, rebase B onto the new `main`, then
open B. It costs one queue cycle and has no failure mode. Stack only when B is
genuinely blocked on A's code, which is rarer than it feels — `just claim` exists
so two agents do not need the same work in flight.

## When the merge queue rejects your pull request

A merge group tests **`main` + your PR**, which is a commit that exists nowhere
else — not on your branch, not on `main`. So "it passed on my PR" and "it failed
in the queue" are both true and not in conflict. That is the whole point of the
queue, and it is why the next step is never obvious from the PR page alone.

**Start with `just queue-triage`.** GitHub already does the part people expect to
do by hand: it tests speculative PREFIXES of the queue concurrently, ejects only
the pull request it can attribute the failure to, and re-tests the innocent ones
automatically. What it cannot tell you is whether the failure is yours.

### If the check is red for several different PRs — it is not yours

Rebasing will not fix it, and re-queuing burns a batch slot against a check that
cannot go green for anyone. **Stop and say so** on the issue or the PR thread,
then either fix the check or drop it from the required set until it is fixed.
Ten agents independently rediscovering one broken gate is the expensive failure
here.

### If it is yours — reproduce the merged state, not your branch

```
git fetch origin && git rebase origin/main
just ci l1        # STRICTER than the gate (see 'Where each tier runs')
```

Two things reproduce this way, and the second is the reason the queue exists:

* **Your PR is simply broken.** The PR gate is deliberately cheap (source gates
  only), so a compile error reaches the queue rather than your PR. Rebasing is
  incidental; `just ci l1` finds it.
* **A semantic conflict.** Your change and something that landed while you
  waited are each correct alone and wrong together — a caller you added against
  a signature someone else changed. No textual conflict, so git merges cleanly
  and only the merged state fails. Rebasing is what surfaces it, and nothing
  cheaper can: it does not exist in either branch.

### Never re-queue an unchanged commit

The queue re-runs the same tree and fails the same way. Re-queue only with a new
commit, or with evidence the failure was a flake — and if you believe it was a
flake, **capture the merge-group log first and file it**, or the next author
pays the same hour you just did.

If it stays green locally after a rebase, you are looking at a flake or a
host difference, not a defect you can fix by pushing again. That is a finding to
record, not a reason to retry.

## Submitting work: what CI enforces, and what it cannot

Three obligations. **One is a gate. Two are conventions, and they are labelled
that way on purpose** — a rule CI cannot check, presented as though it could, is
the same defect as a gate that cannot fail.

### 1. A gate must run its own selftest — ENFORCED (`check-gate-selftests`)

Every script backing a `check-*` recipe must exercise its own failure path, and
must do it on the NORMAL path, not only behind `--selftest`. `check-board-tiers.py`
states the reason in its own comment: *a negative control nobody runs decays
into a comment.* A selftest behind a flag is run once, by its author, on the day
it is written; after that it is prose. Running it every time turns "someone once
demonstrated a red" from a claim in a commit message into something CI
re-verifies on every push.

Do NOT satisfy this by adding a second `--selftest` line to the justfile. That
is twice the cost and still leaves every direct caller unprotected.

It is a **baseline ratchet** (`.config/gate-selftest-baseline.txt`), because a
blanket rule would red 129 of 166 scripts on day one and never land. The
baseline may only shrink: the gate fails if a script outside it is
non-compliant, if a baselined script GAINS a selftest (remove the line), or if a
baselined script no longer exists (the issue-0743 stale-entry class). Today:
**37 of 166 compliant.**

### 2. Separate what you MEASURED from what you REASONED — convention

State plainly which claims came from running something and which came from
reading it, and give the command for the first kind. This is not enforceable —
CI can check that a section exists, never that it is honest — but it is the
single most valuable thing in an agent report, and the evidence is direct: the
one-day sample that produced this section had four agents volunteer their
unverified claims, and each one mattered. Two of four cache-key input classes
declared UNCOVERED with reasons; a `gh` rejection string admitted to be
unconfirmed against the real remote; an entire live registration path flagged as
never executed.

**Declining to claim something is a reason to trust a report more, not less.**

### 3. RUN it; do not read it — convention

Bugs that survive reading do not survive execution. In the same sample, running
rather than reading found a `printf '%s'` that silently dropped the last element
of a label list (so one label was never checked at all), a probe reading a bare
shell instead of the `activate.sh` environment every lane uses, and integer
formatting that printed `0 GiB` for every sub-gigabyte value. All three read
fine.

The corollary for whoever integrates the work: **re-run the evidence rather than
reading the summary.** It costs seconds and it is the difference between trusted
and verified.

## Practices & Pitfalls

### Agent Practices

- **Run `just ci l1` before every push** (~6 min, no fixtures); run `just ci` when the change earns fixture-backed coverage. Never `sudo` — tell the user.
- **Green CI locally BEFORE pushing** — run `just format` then `just ci`. CI stops at the first failing step; re-run until fully green. A toolchain bump can surface new pre-existing lints (e.g. rust-1.96 `unnecessary_cast` / `drop_non_drop` / `not_unsafe_ptr_arg_deref`) — fix them locally rather than discovering them remotely.
- **Always nightly for `rustfmt`** — `rustfmt.toml` enables nightly-only options; stable produces different output. Run `cargo +nightly fmt`.
- **Never merge in git.** Use `git pull --rebase` or `git fetch` + `git rebase`. Never create merge commits unless asked.
- **Submodule rebase on superproject pull:** if a pull advances a submodule pointer AND local work exists → enter it, fetch, rebase local onto upstream, check out the expected commit, record in parent. Never leave a submodule at an older commit when remote advanced.
- **Vendored-fork branch workflow** (cyclonedds, netxduo): land fixes with linear history. **Push the fork branch FIRST, then bump the superproject pointer.** By default, the agent does NOT push fork remotes (exfiltration guard) — the agent commits + rebases locally and leaves the branch ready; the maintainer pushes.
- **Don't modify vendored/generated:** `third-party/`, `packages/interfaces/*/generated/`, build output — unless the task explicitly requires regeneration. Preserve worktree changes.
- **Examples are standalone copy-out projects** (`examples/<plat>/<lang>/<example>/`); no workspace walk-up. Non-example bins under `packages/testing/{nros-tests/bins,nros-bench,nros-smoke}/`.
- **Unused vars:** `_name` + comment, or `#[allow(dead_code)]` for test struct fields.

### Platform Pitfalls

- **After clone, run ONE of** `direnv allow` / `source ./activate.sh` / `source ./activate.fish` — else `zpico-sys/build.rs` panics `"FREERTOS_PORT not set"`.
- **The router is ROS's `rmw_zenohd`; nano-ros ships none** (RFC-0075 / phase-362) — `third-party/zenoh/zenoh/` and `build/zenohd/` are GONE, and a doc naming either is stale. Resolve with `nros_zenohd_bin`, start with `nros_router_exec` / `just zenohd`, print for a human with `nros_router_hint`. zenoh-pico stays pinned 1.7.2 at `packages/rmw/zenoh/zpico-sys/zenoh-pico/`; its wire is proto-stable across 1.x, so it interops with a newer ROS zenoh (issue 0291).
- **Rust edition 2024:** `unsafe extern "C" {}`, `#[unsafe(no_mangle)]`, explicit `unsafe {}` in `unsafe fn`. `nros-c` keeps `#![allow(unsafe_op_in_unsafe_fn)]`.
- **No POSIX-style Rust ctor sections on Zephyr/native_sim/RTOS** — backend registration is an explicit call. A pure-Rust image needs the REAL backend dep (`rmw-zenoh = ["dep:nros-rmw-zenoh"]`) — and a direct reference, or rustc's staticlib DCE drops the dep's `#[no_mangle]` export (symbol in the rlib, absent from the `.a`).
- **Domain ID:** compile-time on embedded (Kconfig / per-example `config.toml`), runtime env on native. `CONFIG_NROS_CYCLONE_DOMAIN_ID` defaults to `NROS_DOMAIN_ID` — never pin it to a literal in confs (the phase-180 split-brain silently ran every cyclone image on domain 0). Cyclone fixture pairs bake distinct domains (50–58) for parallel SPDP.
- **FreeRTOS:** IP-seeded `srand()`; poll-task priority ≥ 4; manual action server needs `try_handle_get_result()`. `APP_TASK_STACK` was deleted in phase-76 — the live knob is `app_stack_bytes` (default **128 KiB**, MEASURED in issue 1146 against a worst in-tree peak of 36 152 bytes; override `NROS_FREERTOS_APP_STACK_KB`), it is charged PER TASK, every image prints its own peak at boot, and the executor arena it was once sized for has not been on that stack since phase-271. → platform-implementation-notes.md "FreeRTOS pitfalls".
- **Zephyr POSIX:** raise `CONFIG_MAX_PTHREAD_MUTEX_COUNT` (zenoh-pico needs ~8+; default 5 fails with -80).
- **Zephyr zsock serializes send/recv per-fd:** `Z_CONFIG_SOCKET_TIMEOUT` must stay 100 ms (5 s starves tx → lease death); intra-image pub→sub needs `Z_FEATURE_LOCAL_SUBSCRIBER=1`.
- **NuttX spin uses `sem_timedwait`** (pthread condvar hangs).
- **NetX Duo BSD `SO_RCVTIMEO` takes `nx_bsd_timeval*`, not `INT` ms** (deadlock otherwise).
- **smoltcp multicast:** join the GROUP addr, not `0.0.0.0`; LAN9118 needs promiscuous in QEMU.
- **QEMU:** `-icount shift=auto`; use `nros_tests::qemu::qemu_system_arm_cmd()`.
- **Embedded Cyclone:** transient samples use `ddsrt_{malloc,calloc,free}`, never libc — RTOS heap is separate.
- **XRCE:** flush `uxr_buffer_request_data` immediately; reliable `STREAM_HISTORY ≥ 2`.
- **Zephyr Rust allocator is picolibc `malloc`** — size `CONFIG_COMMON_LIBC_MALLOC_ARENA_SIZE` (default 16 KB), NOT `CONFIG_HEAP_MEM_POOL_SIZE`.
- **Manual native_sim pair repros need distinct `--seed`** — unseeded processes share entropy → identical GUIDs/ports → discovery sees the peer as itself → false-negative "no delivery".
- **Never clang-format `cmake/templates/*`** — reflow splits `@VAR@` configure_file tokens → generated TU fails "stray '@'". `.clang-format-ignore` guards.
- **Hand-mirrored FFI structs drift on append** (QoS `tx_express`, `callback_group` — 3×): mirror-only TU passes a SHORTER struct by value → tail field garbage. Gated: `check-ffi-struct-mirrors` (push lane) + cross-include TU in `check-c`. Include order is one-way: `nros_cpp_ffi.h` BEFORE `component.h`.
- **zpico shim + zenoh-pico library MUST share the generated zenoh config** — flag-gated struct fields (`Z_FEATURE_LOCAL_QUERYABLE`…) make mismatched TUs a silent ABI break. `build_c_shim` injects `ZENOH_GENERIC` + the OUT_DIR config. Local fixture binaries embed the shim — rebuild fixtures after zpico config changes.

### CMake Pitfalls

- **A cmake `include()` inside a FUNCTION scope drops the included file's normal
  variables when the frame pops** — only the function/macro *definitions* survive.
  Any module that captures its own dir (`set(_X_DIR "${CMAKE_CURRENT_LIST_DIR}")`)
  for later `configure_file` template lookups must use `CACHE INTERNAL` (the
  `_NROS_ENTRY_DIR` pattern). 287-W6: `_NROS_NODE_REGISTER_DIR` was a normal var,
  the workspace path includes NodeRegister inside `_nros_import_once()` (a
  function), and every FreeRTOS workspace member failed `configure_file` on
  `"/templates/freertos_entry_main_c_typed.cpp.in"` — posix never touches the
  templates, which hid it for months.
- **`find_program` HINTS are searched BEFORE the environment PATH.** A resolver
  that lists `~/.nros/bin` as a HINT lets a stale provisioned binary shadow the
  activate.sh-wired in-tree CLI (a June-era `nros` baked the retired pre-258
  bake shape and turned every `nros_system_generate` west fixture red). Use
  `PATHS` (searched AFTER PATH) for fallback locations, and keep
  `~/.nros/bin/nros` fresh after CLI-shape changes.
- **Case-normalize enum-ish cmake args at the function top**
  (`string(TOUPPER …)`) — the RFC-0048 verbs pass inferred values in lowercase
  (`cpp`), and a case-sensitive `STREQUAL "CPP"` chain silently falls into the
  wrong branch (287-W6: the Zephyr interface generator emitted C bindings for a
  C++ leaf → "std_msgs.hpp: No such file"). The canonical generator already
  normalizes; keep siblings in lockstep.
- **`cmake_parse_arguments` swallows positional args after a multi-value
  keyword** — a call like `nano_ros_add_node(n CALLBACK_GROUPS g src/A.cpp)`
  loses the source. Verbs accept an explicit `SOURCES` keyword for this; when
  extending a verb, add new multi-value keywords BEFORE the positional-sources
  convention breaks someone.
- **Old-shape CMake surface is retired** (287-W8 + post-287): `nano_ros_bootstrap`
  / `nano_ros_link` are `_nros_*` config internals, `nano_ros_deploy` /
  `nano_ros_application` / `nano_ros_component_register` are gone. The ament
  shape (`find_package(nano_ros REQUIRED)` + `nano_ros_add_executable` /
  `nano_ros_add_node`, deploy tuple in package.xml) is the only leaf shape;
  `example_shape.rs`'s FORBIDDEN list gates regressions.

### Rust Consumption (RFC-0048 W9)

- **`nros sync` owns each Rust leaf's `.cargo/config.toml` managed surface**: one
  `include = ["…/nros-patch.toml"]` line (the central, gitignored, absolute-path
  patch file at the checkout root) + the leaf-local
  platform-specific `# nros-managed` patch lines. Never hand-edit them; a moved
  checkout needs one `nros sync` to re-point the central file.
- **The managed rows split by ORIGIN (issues 0457/0463), not by "sync wrote it":**
  - **in-repo** rows (`nros-log`, board crates, `mps2-an385-pac`) are relative
    paths identical in every checkout → INLINE in the tracked `config.toml`,
    tagged `# nros-managed`. A clone needs them; 183 of them across the tree.
  - **`generated/`** rows are built per host from the consumer's ament install →
    the gitignored sidecar `.cargo/nros-managed-patch.toml`, whose `include` is
    written ONLY when that file is. 88 of them.

  A leaf with no message dependency therefore has no sidecar, no include, and
  resolves in a fresh clone with no sync at all; only ament-derived content sits
  behind sync. 0457 moved the WHOLE set to the sidecar and stranded every leaf on
  `no matching package named 'mps2-an385-pac'` — an in-repo patch. Dropping the
  include alone does not fix that, it only changes the message.
- **A missing `include` target is a HARD cargo error during MANIFEST PARSE**, not
  a silent drop — the leaf becomes unreadable, so `cargo metadata` and every gate
  walking it fail four frames deep without ever naming sync. Both #272 and #457
  assumed the opposite, inherited from a comment written before `include`
  stabilised in 1.93; measured false on 1.96/1.97. `_require-leaf-includes` is the
  preflight that says "run `nros sync`" first, and `check-cargo-config-tracked`
  rejects an include naming a target no generator writes.
- **After a sync the tracked config gains the sidecar `include` on disk — never
  commit that line.** `git add -u` scoops it up (it did, twice). The invariant is
  about the COMMITTED blob, so the gate reads `git show HEAD:<path>` rather than
  the working copy.
- **Central-patch membership rule:** a crate may live in `nros-patch.toml` only
  if it is registry-named in EVERY consumer's dependency graph — cargo emits
  "patch `X` was not used in the crate graph" per unused entry, and the file is
  shared by all leaves. That limits it to the universal trio
  (`nros`/`nros-core`/`nros-serdes`); RMW/board/driver crates are NOT universal
  (verified: a freertos entry's slim graph lacks the cyclone/xrce crates).
- cargo's `config-include` is STABLE (verified on 1.96) — no nightly gate.

### Test Pitfalls

- **Tests must fail on unmet preconditions** (`assert!`/`bail!`/`nros_tests::skip!`). A bare `eprintln!`+`return` reports PASS. Same rule at runtime: panic, never silent early-return.
- **No compilation inside tests.** Compile in the build stage; test consumes prebuilt fixture.
- **Fixture mtime treadmill:** any pull/rebase — and any `git stash push`/`pop`, which rewrites tracked files just the same — refreshes source mtimes → EVERY prebuilt fixture reads STALE. Rebase once → rebuild affected fixtures → test WITHOUT pulling again. Core-crate or repr(C)-struct changes ⇒ wipe workspace build dirs (incremental mixes pre/post-append objects). Long-unrebuilt families "pass" on museum binaries — trust only a fresh full sweep.
- **Bare `cargo nextest` counts `nros_tests::skip!` panics as FAILURES.** Only `just test-all`'s junit rewrite converts the `[SKIPPED]` panic into a skip. When triaging a bare-run red, read the panic text first — "fixture not built"/`[SKIPPED]` reds are skips in CI semantics, not regressions.
- **Full-sweep QEMU lanes flake under load.** With the whole suite fanned out, concurrent QEMU boots miss readiness banners (287-W7: all six nuttx C/C++ rtos lanes failed 3/3 retries in-sweep, then passed solo). Retest a QEMU-lane red SOLO before filing an issue from it.
- **A bisect whose per-step protocol differs from the failing environment measures the protocol, not the code.** Issue 0268's lesson: the red was the sizes-header mirror race (stale shadow `nros_config_generated.h` in incrementally-rebuilt fixture trees → executor placement-new overflow), so any bisect step that wiped caches and rebuilt CLEAN went green regardless of revision — the run "converged" on a docs-only commit. Two rules: (1) a first-bad that cannot plausibly cause the symptom (docs-only, unrelated subsystem) means the verdicts tracked a confounder (build staleness, host load, re-baked ports) — rerun ONE rev N times under both protocols before trusting any boundary; (2) when the same symptom is debugged from two sides in parallel, reconcile root causes before landing a fix — a plausible-but-wrong story (e.g. "load flake" from build-state coincidence) can motivate a fix that MASKS the real defect's loud failure.
- **Build-side stale probes and test-side stale gates must watch the SAME inputs** (issue 0196): the native-rust fixture probe missed `generated/**`, so a month-old museum binary passed every "native OK" sweep while the test gate failed it. When adding a fixture family, source one shared staleness helper.
- **Test greps use `nros_tests::output::*` constants, never literal strings** — example banners/markers get slimmed (phase-277 broke ~10 tests grepping `"Result:"`/`"[OK]"`/old banners). If a test times out, FIRST diff the grep pattern against what the fixture actually prints.
- **Test names describe behavior, not phase numbers** — cross-reference a phase in a doc-comment, never the identifier.

### Multi-Session / Shell Pitfalls

- **Parallel agent sessions push to `main` concurrently.** `git fetch` + check
  `origin/main`'s highest issue id (including `archived/`) immediately before
  filing a `docs/issues/` entry. The generated list is `docs/issues/open.md`, a
  GITIGNORED build artifact (issues 0883/0884), so two agents filing concurrently
  no longer conflict — regenerate it with `python3 scripts/gen-issue-index.py`
  and NEVER `git add` it. `merge=union` was tried first and retired: the driver
  does not run in GitHub's server-side merge or rebase, which is the only place
  the conflict mattered. Renumber only your own files. Stash-wrap local-only files
  (`packages/rmw/zenoh/zpico-sys/c/include/zpico.h`-style) around every rebase.
- **Resolving an issue: the digest goes in `archived/<id>-*.md`, not in
  `docs/issues/README.md`.** That file's "Recently resolved" block is frozen and
  ratcheted by `check-issue-index`. Every entry was prepended at the same offset
  in one shared authored file, so two agents resolving issues on the same day
  conflicted by construction — issue 0883's class, one file over, except this
  one cannot be fixed by generating it because the digests are prose. Read the
  list with `just issues --all --status resolved`, which is derived from the
  files and so cannot drift.
- **Write full logs of background builds/tests to files** and grep afterwards;
  `cmd | tail -N` swallows the mid-log error that explains the failure.
- **`pkill -f <pattern>` matches your OWN wrapper shell** when the pattern
  appears in the command string you are currently running (the agent shell's
  `zsh -c 'bash -c "… just check …"'` self-killed with exit 144). Kill by PID,
  or pick a pattern the current command line cannot contain.
- **zsh gotchas in agent shells:** unmatched globs abort the whole compound
  command (`rm -rf foo* && build` never builds — use `find`), and unquoted
  `$var` does NOT word-split (a loop over `$FILES` sees one giant argument —
  use `xargs` or explicit arrays).

## Editing a justfile recipe or a workflow `run:` block

**A line at column 0 inside either is parsed as SYNTAX, not as text.** So a
heredoc cannot be used — its terminator must sit at column 0 — and `just`'s own
body-dedent then fights any `sed` strip you add to compensate.

Use `printf` with one argument per line and the indentation inside the quotes:

```
recipe:
    #!/usr/bin/env bash
    printf "%s\n" \
        "  first line" \
        "  second line"
```

This cost four separate failures in one day, and none of the errors named the
cause: `unknown start of token '—' (U+2014)`, `expected '*', ':', '$',
identifier, or '+', but found end of line`, `extraneous attribute`, and YAML's
`could not find expected ':'`. If a heredoc in a recipe or a `run:` block
produces a parse error about something unrelated, this is why.

## CLI Install & Submodule Operations

CLI install:

* `~/.cargo/bin/nros` + `~/.nros/bin/nros` are STALE shadows from pre-Phase-218 install paths — remove if present.
* Canonical install: `git submodule update --init packages/cli/third-party/play_launch`, then `just setup-cli`, then `source ./activate.sh`. **This is the CONTRIBUTOR path, not the user one** (RFC-0014 says so, and RFC-0090 says why it is still the only one): a user is meant to download `nros` and never clone this repo. The download is blocked on the codegen-version check, not on packaging. A consequence worth knowing while it lasts — the stale-CLI refusal (`stale_guard.rs`) can only fire for a binary INSIDE a checkout (`checkout_root_of`), so it is a cost contributors pay and users never meet. **NON-recursive on purpose** (phase-332): the CLI's only pin is the `play_launch` repo, whose layer 2 (`src/ros-launch-resolve` = resolver + parser) is REGULAR FILES, and `ros-launch-manifest` is a git-TAG cargo dep (`v0.1.0`) rather than a nested submodule — so nothing under play_launch needs recursing. `--recursive` here would pull play_launch's layer-3 runtime submodules (`src/vendor/*`, container, msgs), which nano-ros never builds (RFC-0060). PATH wires `nros`, `zenohd` from `~/.nros/sdk/*/bin/`; the launch helper `nros-launch-resolve` is built by `just setup-launch-resolve` and invoked by ABSOLUTE PATH, never via `$PATH` (issue 0285).
* `just doctor` FAILs (not warns) on stale shadows + a missing `nros-launch-resolve`. `play_launch_parser` is a TEST-tier prereq only (the `launch_synth` / `self_bringup` / `orchestration_includes` PATH probes), not a CLI one.

Agent-dispatch contract:

* Every `just <plat>` invocation needs `source ./activate.sh` first; dispatch templates MUST source it. The pre-218 `export PATH="$HOME/.nros/bin:$PATH"` is INSUFFICIENT (misses the SDK-provisioned tools). CLAUDE.md “Practices” carries this.

Submodule init landmine:

* Never `git submodule update --init --recursive` from a worktree — the transitive closure pulls QEMU → OpenSSL → pyca-cryptography (~30 min). Init only what the task needs.

## The zenoh router is ROS's, and we ship none

RFC-0075 / phase-362 retired the vendored `zenohd`: `rmw_zenohd` links the same
`libzenohc.so` as the `rmw_zenoh_cpp` a ROS node uses, so it cannot drift from
it, which a pinned copy did (issue 0609 measured the ROS package moving its
zenoh 1.2.0 → 1.8.0 with our pin taking no part in either the failure or the
fix).

Consequences that keep being rediscovered, so they are written here once:

* **`nros setup` does not provision a router**, and `zenohd` is not on `PATH`.
  A leftover `~/.nros/sdk/zenohd/` from an older checkout is a RETIRED entry —
  `just doctor` reports it with the `rm -rf` to remove it, because the SDK store
  accumulates and nothing prunes it (issue 0653).
* **Never spell the install path.** The router's location under the ROS prefix
  is only the THIRD of three resolution steps, so a line naming it is wrong on a
  ROS built from source or installed as a colcon overlay. Gated by
  `check-zenohd-flag-invocations`, which will reject the literal if you write
  it — including in prose explaining why not to.
* **One function per job** (`scripts/dev/zenohd.sh`): `nros_zenohd_bin` resolves,
  `nros_router_exec` starts, `nros_router_hint` prints the line for a human. The
  Rust twin is `nros_tests::process::ros_zenohd_path`, kept in step by
  `check-zenohd-resolution-parity` over a shared table. `just zenohd [locator]`
  is the entry point a human or a doc should name.
* **`rmw_zenohd` parses no argv.** `--listen` and friends are not rejected, they
  are UNREAD — the router silently comes up on its defaults, and the wrong port
  reads as a hang in `Executor::open`. Configuration travels in
  `ZENOH_CONFIG_OVERRIDE`.
