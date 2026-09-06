set working-directory := '..'

# Phase 143 — patched qemu-system-arm built by `just qemu setup-qemu`.
QEMU_BIN := absolute_path("build/qemu") / "bin/qemu-system-arm"

default:
    @just --list --justfile {{source_file()}}

# Install FreeRTOS prerequisites. Phase 197.4 — `nros setup <board>` provisions
# the toolchain + sources (arm-none-eabi-gcc, qemu, FreeRTOS-Kernel, lwIP,
# zenoh-pico, …) from the SDK index; the host-env helper adds the cross Rust
# target. `source setup.bash` afterwards to put the provisioned tools on PATH.
[group("setup")]
setup:
    #!/usr/bin/env bash
    # Every RMW this platform's lane BUILDS, not just zenoh.
    #
    # `nros setup <board>` takes `--rmw`'s zenoh default, so a lane that also
    # builds a cyclonedds coordinate never got its host tools — the freertos
    # nightly died at cmake configure with `idlc (Cyclone DDS IDL compiler, a
    # host tool) not found`. `[rmw.cyclonedds]` declares exactly what that
    # needs and calls itself "a complete provisioner"; nothing requested it.
    #
    # The backend list is DERIVED from `examples/fixtures.toml`, not written
    # here, so it cannot drift from the coordinates the lane actually builds.
    # A SHEBANG recipe: `just` runs each line of a line-based recipe in its
    # own shell, so a sourced helper does not survive to the next line — and
    # the default shell is `sh`, which has no `source` at all.
    set -euo pipefail
    source scripts/build/platform-rmws.sh
    nros_setup_board_rmws qemu-arm-freertos freertos
    "{{justfile_directory()}}/tools/host-env.sh" freertos

# Diagnose FreeRTOS install status (read-only).
[group("setup")]
doctor:
    #!/usr/bin/env bash
    bash scripts/dev/deprecated-scope-alias.sh freertos doctor "just doctor freertos"
    set +e
    fail=0
    if [ -d "$FREERTOS_DIR/include" ]; then
        echo "  [OK] FreeRTOS kernel: $FREERTOS_DIR"
    else
        echo "  [MISSING] FreeRTOS kernel at $FREERTOS_DIR (run: just freertos setup)"
        fail=1
    fi
    if [ -d "$LWIP_DIR/src/include" ]; then
        echo "  [OK] lwIP: $LWIP_DIR"
    else
        echo "  [MISSING] lwIP at $LWIP_DIR (run: just freertos setup)"
        fail=1
    fi
    if command -v arm-none-eabi-gcc &>/dev/null; then
        echo "  [OK] arm-none-eabi-gcc"
    else
        echo "  [MISSING] arm-none-eabi-gcc (run: nros setup --tool arm-none-eabi-gcc — prebuilt, no sudo; or apt gcc-arm-none-eabi) — 0368 F2"
        fail=1
    fi
    exit $fail

# Phase 140 — `install-platform` + `install` recipes deleted along with
# the `build/install/` layout. FreeRTOS examples consume nano-ros via
# `add_subdirectory(<repo-root>)` (Phase 144.5) or via the
# `integrations/nuttx/` etc. shells (Phase 139). The `build` recipe
# below now drives only the example tree.

# Build FreeRTOS examples (the canonical "did the core link clean?" smoke).
[group("main")]
build: build-examples
    @echo "FreeRTOS examples built."

# Build FreeRTOS QEMU MPS2-AN385 Rust examples
[group("main")]
build-examples:
    #!/usr/bin/env bash
    set -e
    # issue 0650 — first step of the lane: clear last run's skip notes, so a
    # lane that is complete this time is not reported SKIPPED from history.
    # shellcheck source=scripts/build/lane-skip.sh
    source scripts/build/lane-skip.sh
    nros_lane_skip_reset freertos
    source scripts/build/cargo.sh
    # issue 1038 — the central `nros-patch.toml` this lane's Rust leaves
    # `include`, made EXPLICIT. This lane already got the file, but only as a
    # side effect of `workspace-fixtures-build.sh` running `nros sync` somewhere
    # earlier; `threadx-riscv64` had no such neighbour and died at manifest
    # parse. An inherited artifact is a working lane, not a declared dependency.
    # Idempotent — a process spawn when the file is already current.
    nros_ensure_central_patch
    # Phase 214.J.2 — guard `nros sync` against stale `generated/` after trait
    # surface drift (see scripts/build/codegen-stamp.sh).
    source scripts/build/codegen-stamp.sh
    cargo_profile_args="$(nros_cargo_profile_arg_string)"
    nros_cli="$(nros_cli_bin)"
    # Phase 214.I.2 — fail-loud prereq guard: the loop below invokes
    # `nros sync`, which the shipped 0.3.7 release predates. Probe
    # once, emit `[PREREQ]` + exit 0 if absent rather than cascading
    # into clap "unrecognized subcommand 'ws'" stack traces per example.
    nros_require_sync "$nros_cli"
    echo "Building FreeRTOS QEMU MPS2-AN385 examples..."
    if [ ! -d "$FREERTOS_DIR/include" ]; then
        echo "ERROR: FreeRTOS not found at $FREERTOS_DIR. Run: just setup-freertos"
        exit 1
    fi
    if [ ! -d "$LWIP_DIR/src/include" ]; then
        echo "ERROR: lwIP not found at $LWIP_DIR. Run: just setup-freertos"
        exit 1
    fi
    # Codegen prep (per-platform); build options come from the SSOT manifest
    # (examples/fixtures.toml) via the shared builder. Phase 181.4.d.
    # phase-338 W2 — the sibling Entry pkgs are gone: each role package now
    # carries BOTH the node logic and the entry, so one `nros sync` per role
    # covers what used to need two (the `[patch.crates-io]` block and the
    # `generated/` msg crates moved in with the entry).
    for example in \
        talker listener service-server service-client action-server action-client; do
        dir="examples/qemu-arm-freertos/rust/$example"
        NROS_REPO_DIR="$PWD" nros_codegen_stamp_check_or_wipe "$dir"
        NROS_REPO_DIR="$PWD" "$nros_cli" sync "$dir" >/dev/null
        NROS_REPO_DIR="$PWD" nros_codegen_stamp_write "$dir"
    done
    # phase-336 / phase-340 P2 — the profile is the `freertos-qemu` carve-out,
    # not a literal: a lightly-optimized zenoh-pico misses its handshake window
    # on the emulated Cortex-M3, and every FreeRTOS test-side locator reads the
    # SAME constant (`nros_cargo_profile::FREERTOS_QEMU_PROFILE`). It reaches
    # the manifest builder the way `just nuttx build-fixtures` passes
    # `nuttx-rust` — through `NROS_CARGO_PROFILE`, which `scripts/build/cargo.sh`
    # already treats as the ambient profile for everything downstream.
    #
    # This ONE call is now the whole freertos rust fixture lane. Until phase-340
    # P2 a bare `cd <leaf> && cargo build` loop ran here as well, prebuilding the
    # same six Entry pkgs into their own `target/`: a build with no manifest row,
    # therefore no coordinate, therefore no shared cargo group — so it re-created
    # `examples/qemu-arm-freertos/rust/*/target/` two minutes after the group dir
    # was written, on a platform B3 had already migrated. The rows now name that
    # build (see the comment on them in examples/fixtures.toml for why the old
    # `target-zenoh` half was the dead one), and the loop is gone.
    NROS_CARGO_PROFILE="$(nros_cargo_platform_profile freertos)" \
        bash scripts/build/fixtures-build.sh freertos rust zenoh
    bash scripts/build/workspace-fixtures-build.sh freertos rust
    # phase-263 C2b — the embedded C workspace LAUNCH entry (issue 0097), consumed by
    # tests/entry_e2e.rs (freertos_c cell). Own build-subdir (one board per CMake configure).
    bash scripts/build/workspace-fixtures-build.sh freertos c
    # phase-263 C2c — the embedded C++ workspace LAUNCH entry, consumed by
    # tests/entry_e2e.rs (freertos_cpp cell).
    bash scripts/build/workspace-fixtures-build.sh freertos cpp
    # phase-263 C2c — the embedded MIXED (C+C++ + no_std Rust) workspace LAUNCH entry,
    # consumed by tests/entry_e2e.rs (freertos_mixed cell).
    bash scripts/build/workspace-fixtures-build.sh freertos mixed
    # phase-370 — the freertos-posix board variant. Its two `[[workspace_fixture]]`
    # rows carry `platform = "freertos-posix"`, a DIFFERENT fixture token from the
    # `freertos` rows above, so nothing here built them: the coordinate was
    # selectable by a lane while no recipe on that lane's path could produce it,
    # which is issue #405's shape and what `matrix_fixture_coverage`'s recipe-graph
    # gate names. The board still lives under `just freertos` (PlatformId
    # ::FreertosPosix maps to this module), so the calls belong here rather than in
    # a module of their own.
    bash scripts/build/workspace-fixtures-build.sh freertos-posix c
    bash scripts/build/workspace-fixtures-build.sh freertos-posix cpp
    echo "FreeRTOS QEMU examples built!"

# phase-370 W3 — the POSIX-simulator board's C and C++ cyclonedds workspace
# rows (`platform = "freertos-posix"` in examples/fixtures.toml).
#
# Its own recipe rather than a line in `build-examples`, because that recipe IS
# the ARM QEMU lane: it hard-fails without lwIP and cross-compiles every leaf,
# and this board does neither — host sockets, host cc, no emulator. Before this
# the two rows had no producer anywhere in the module, so `just freertos
# build-fixtures` finished green on fixtures nothing in it could build and the
# staleness gate then failed on them (issue #405 exactly; caught by
# `matrix_fixture_coverage::every_fixture_token_is_producible_by_the_module_that_owns_it`).
[group("examples")]
build-fixtures-posix:
    #!/usr/bin/env bash
    set -e
    # shellcheck source=scripts/build/lane-skip.sh
    source scripts/build/lane-skip.sh
    source scripts/build/cargo.sh
    source scripts/build/codegen-stamp.sh
    nros_cli="$(nros_cli_bin)"
    nros_require_sync "$nros_cli"
    # The kernel source is the one thing this board still needs; lwIP is not.
    if [ ! -d "$FREERTOS_DIR/include" ]; then
        nros_lane_skip_note freertos \
            "FreeRTOS kernel not found at $FREERTOS_DIR (run: just setup freertos)"
        exit 0
    fi
    bash scripts/build/workspace-fixtures-build.sh freertos-posix c
    bash scripts/build/workspace-fixtures-build.sh freertos-posix cpp

# Build all example binaries used as test fixtures (Rust + C + C++).
[group("full-matrix")]
build-fixtures: build-examples build-fixture-extras build-fixtures-posix
    #!/usr/bin/env bash
    bash scripts/dev/deprecated-scope-alias.sh freertos build-fixtures "just build freertos"
    set -e
    # issue 0650 — the ONE place this lane says it built its fixtures, and it
    # refuses to when a step was skipped. Before this the claim was an
    # unconditional echo, so a lane with no toolchain printed it anyway.
    # shellcheck source=scripts/build/lane-skip.sh
    source scripts/build/lane-skip.sh
    nros_lane_skip_flush freertos "FreeRTOS test fixtures built."

# Internal Phase 178.B target: only fixture artifacts not already covered by
# `build-examples`.
[group("examples")]
build-fixture-extras:
    #!/usr/bin/env bash
    set -e
    # shellcheck source=scripts/build/lane-skip.sh
    source scripts/build/lane-skip.sh
    source scripts/build/cargo.sh
    # Phase 214.J.2 — codegen-stamp helpers for `nros sync` invocations below.
    source scripts/build/codegen-stamp.sh
    cargo_profile_args="$(nros_cargo_profile_arg_string)"
    nros_cli="$(nros_cli_bin)"
    # Phase 214.I.2 — fail-loud prereq guard: the cyclone-fixture loop
    # below invokes `nros sync`. Probe once; emit `[PREREQ]` + exit 0
    # rather than burying the build in clap "unrecognized subcommand 'ws'"
    # stack traces (shipped 0.3.7 release predates the verb).
    nros_require_sync "$nros_cli"
    export cargo_profile_args
    if ! command -v arm-none-eabi-gcc >/dev/null 2>&1; then
        nros_lane_skip_note freertos "arm-none-eabi-gcc not found"; exit 0
    fi
    echo "Building FreeRTOS test fixtures..."
    # Phase 176 — cmake parallelism via the fifo jobserver when present.
    if [ "${NROS_JOBSERVER:-}" = "1" ] || [ "${NROS_INHERIT_JOBSERVER:-}" = "1" ]; then unset CMAKE_BUILD_PARALLEL_LEVEL; else export CMAKE_BUILD_PARALLEL_LEVEL="${NROS_BUILD_JOBS:-8}"; fi
    # The normal Rust role examples are produced by `build-examples`.
    # Phase 88.15.b — nros-log severity-coverage smoke fixture, selected by
    # stable manifest id so `build-examples` stays limited to role examples.
    #
    # phase-340 P2 — the SAME `NROS_CARGO_PROFILE` carve-out `build-examples`
    # passes. This row's `rmw` defaults to zenoh (`row_coord()`), so it also
    # matches `build-examples`'s `freertos rust zenoh` query; without the same
    # profile on both sides the two calls would build it twice, at two profiles,
    # and the resolver could only read one of them.
    NROS_CARGO_PROFILE="$(nros_cargo_platform_profile freertos)" \
        bash scripts/build/fixtures-build.sh freertos rust --id freertos-logging-smoke-mps2

    # Issue #0317 — the wake-latency-cortex-m3 microbench (Phase 141.D), a
    # TWO-IMAGE bench (subscriber `wake-latency-cortex-m3` + publisher
    # `wake-latency-pub`) so the zenohd router delivers a real transport-arrival
    # wake to the subscriber. Build BOTH images at the profile the test locates
    # so a compile regression is caught here. The runtime
    # `wake_latency_cortex_m3` test takes its intended CYCCNT skip on QEMU
    # (Cortex-M3 doesn't emulate the DWT cycle counter); the real P99 validates on
    # hardware. This lane guards the BUILD of both images.
    # issue 0488 residue 1 — the bare in-leaf `cargo build` that used to be here
    # is gone. The pair is a `[[fixture]]` row now, so `build-test-fixtures`
    # builds it into the shared `freertos` group with a coordinate and a
    # staleness probe, and `tests/wake_latency_cortex_m3.rs` resolves it through
    # that row instead of a hand-spelled path.

    toolchain="$(pwd)/cmake/toolchain/arm-freertos-armcm3.cmake"
    # Phase 140 removed the `build/install/bin/nros-codegen` install
    # layout. Cross builds (FreeRTOS) can't build the host codegen
    # tool with their cross toolchain, so build it once with the host
    # cargo and pass the absolute path via `-D_NANO_ROS_CODEGEN_TOOL`.
    echo "  → host nros-codegen (cargo)"
    nros_cargo_ensure_codegen_c
    codegen="$(nros_cargo_codegen_c_bin)"
    # Phase 186 — RMW availability follows config, fully in CMake. The example's
    # nros_provide_cyclonedds() self-provisions Cyclone from the pinned submodule
    # (sccache-accelerated) — no prebuilt install, no `just cyclonedds
    # *-cross-probe` pre-step. The ARM cross toolchain is already guaranteed here
    # (the recipe exited above if `arm-none-eabi-gcc` was missing). idlc resolves
    # host-side via find_program (Phase 186.3).
    #
    # Phase 177.38 — bake this platform's Cyclone role-set onto a distinct ROS
    # domain so its fixtures never collide with other Cyclone fixtures on the
    # derived RTPS ports. talker+listener share (they communicate). freertos=60.
    export NROS_DOMAIN_ID=60
    # Cyclone descriptor codegen needs a host `idlc` (find_program, Phase 186.3);
    # skip the cyclonedds fixtures gracefully when it's absent rather than failing
    # the whole fixture build — mirrors threadx-linux. The zenoh c/cpp/rust
    # fixtures (the primary coverage) are unaffected. Install a ROS 2 / CycloneDDS
    # idlc (or set IDLC_EXECUTABLE) to exercise the Cyclone variants.
    # Phase 220.C (Path B) — FreeRTOS rust cyclonedds fixtures are retired
    # from the cmake/corrosion path. Phase 212.M.5.b deleted the per-example
    # CMakeLists.txt; Phase 214.S.5.b (BSP gate — cyclonedds-sys vendored
    # build against the ARM cross toolchain + FreeRTOS POSIX shim) is the
    # remaining blocker for a pure-cargo cyclonedds path here. Until that
    # lands, the freertos rust cyclonedds e2e tests stay `#[ignore]`d
    # (see packages/testing/nros-tests/tests/freertos_qemu.rs) and the
    # build step is a no-op. The c/cpp cyclonedds cells on freertos are
    # also out of scope (none exist today — see examples/fixtures.toml).
    if command -v idlc >/dev/null 2>&1 || [ -x build/cyclonedds/bin/idlc ]; then
      echo "  → freertos rust cyclonedds fixtures retired (Phase 220.C path B — pure-cargo only; BSP gate 214.S.5.b)"
    else
      echo "  → freertos cyclonedds fixtures skipped (no idlc on PATH — run: nros setup --tool cyclonedds — the dist ships bin/idlc; or a ROS 2 / CycloneDDS install, or set IDLC_EXECUTABLE) — 0368 F2"
    fi
    # Phase 181.5.b — collapsed-shape FreeRTOS C / C++ examples from the SSOT
    # manifest (examples/fixtures.toml). Zenoh-only (no DDS C/C++ siblings
    # existed pre-collapse on FreeRTOS). Per-cell RMW + build-subdir come from
    # the manifest; the platform-wide toolchain / codegen-tool -D defs are
    # injected via the shared builder's NROS_CMAKE_EXTRA_DEFS contract.
    # issue 0805 — one Corrosion cargo dir per (platform, resolved feature
    # inputs) instead of one per leaf; 12 identical per-leaf dirs here (6 C +
    # 6 C++, all at the same hash) collapse to 1. cmake keys the subdirectory
    # itself. `nros_build_dir` verified IN SCOPE in this recipe before wiring
    # (it arrives via `lane-skip.sh`, sourced first — `cargo.sh` dots
    # build-root.sh only INSIDE a function, so it does not export the helper).
    # A missing helper would expand the flag EMPTY and silently keep per-leaf
    # dirs, which is defect 3 from the native wiring.
    export NROS_CMAKE_EXTRA_DEFS="-DNROS_SHARED_CARGO_ROOT=$(nros_build_dir "$NROS_KIND_CORROSION_CARGO" freertos) -DCMAKE_TOOLCHAIN_FILE=$toolchain -DCMAKE_BUILD_TYPE=Release -D_NANO_ROS_CODEGEN_TOOL=$codegen"
    bash scripts/build/fixtures-build.sh freertos c zenoh
    bash scripts/build/fixtures-build.sh freertos cpp zenoh
    unset NROS_CMAKE_EXTRA_DEFS

# Build every FreeRTOS tier: core libraries, examples, and test fixtures.
[group("full-matrix")]
build-all: build-fixtures
    @echo "FreeRTOS build-all completed."

# Run FreeRTOS QEMU integration tests
[group("main")]
test verbose="": build-fixtures
    #!/usr/bin/env bash
    bash scripts/dev/deprecated-scope-alias.sh freertos test "just test freertos"
    set -e
    if [ ! -d "$FREERTOS_DIR/include" ]; then
        echo "ERROR: FreeRTOS not found at $FREERTOS_DIR. Run: just setup-freertos"
        exit 1
    fi
    if [ ! -d "$LWIP_DIR/src/include" ]; then
        echo "ERROR: lwIP not found at $LWIP_DIR. Run: just setup-freertos"
        exit 1
    fi
    # `rtos_e2e`'s `Platform__Freertos` cells, NOT the `freertos_qemu` target.
    #
    # That target holds exactly two tests and BOTH are `#[ignore]`d — the
    # cyclonedds fixtures they drove were retired in phase 220.C path B and
    # phase 214.P, and their own header says where the coverage went: "The
    # per-role binaries are consumed by the `rtos_e2e` Platform__Freertos
    # tests." So this lane selected a binary with nothing runnable in it and
    # nextest ended the job:
    #
    #     Starting 0 tests across 1 binary (2 tests skipped)
    #     error: no tests to run
    #
    # It had been VACUOUS since those ignores landed and only became visible
    # when nextest started treating an empty selection as an error rather than
    # a silent pass — which is the better default, and is what surfaced this.
    # A lane that runs nothing while reporting success is the shape this repo
    # gates against everywhere else.
    #
    # Same move the NuttX lane already made, for the same reason and in the
    # same words (`_nextest-platform`'s `filter` parameter exists for it): the
    # per-platform target was subsumed by the shared matrix, so select the
    # platform's cells out of `rtos_e2e` instead. `test(Freertos)` selects 9 —
    # action / pubsub / service, each in Rust, C and C++.
    just _nextest-platform rtos_e2e '{{verbose}}' '' 'test(Freertos)'

# Run all FreeRTOS tests including networked E2E
[group("full-matrix")]
test-all verbose="":
    #!/usr/bin/env bash
    set -e
    source scripts/build/cargo.sh
    cargo_nextest_args=($(nros_cargo_nextest_args))
    if [ ! -d "$FREERTOS_DIR/include" ]; then
        echo "ERROR: FreeRTOS not found at $FREERTOS_DIR. Run: just freertos setup"
        exit 1
    fi
    if [ ! -d "$LWIP_DIR/src/include" ]; then
        echo "ERROR: lwIP not found at $LWIP_DIR. Run: just freertos setup"
        exit 1
    fi
    # Build tests
    args=(-p nros-tests --test freertos_qemu --no-fail-fast)
    if [ -z "{{verbose}}" ]; then
        args+=(--success-output never --failure-output never)
    fi
    cargo nextest run "${cargo_nextest_args[@]}" "${args[@]}"
    # E2E networking tests (pubsub, service, action across RTOS platforms)
    e2e_args=(-p nros-tests --test rtos_e2e -E 'test(Freertos)' --no-fail-fast)
    if [ -z "{{verbose}}" ]; then
        e2e_args+=(--success-output never --failure-output never)
    fi
    cargo nextest run "${cargo_nextest_args[@]}" "${e2e_args[@]}"

# Phase 121.6 — build the FreeRTOS Posix-port smoke test for
# nros-platform-freertos. Uses the in-tree FreeRTOS-Kernel + a
# minimal FreeRTOSConfig.h. Excludes net.c (no lwIP in this harness).
[group("examples")]
build-c-port:
    #!/usr/bin/env bash
    set -e
    cd tests/freertos-c-smoke
    # nros-cmake-prefix-exempt: pure-C FreeRTOS port smoke test, no Rust.
    cmake -B build >/dev/null
    cmake --build build

# Phase 121.6 — run the FreeRTOS Posix-port smoke test on the host.
# Boots FreeRTOS, runs the smoke task, exits with status 0 on PASS.
[group("debug")]
test-c-port: build-c-port
    #!/usr/bin/env bash
    set -e
    timeout 15 ./tests/freertos-c-smoke/build/nros_freertos_c_smoke

# CI: check prerequisites + test
[group("full-matrix")]
ci: doctor test

# Clean FreeRTOS build artifacts
[group("main")]
clean:
    #!/usr/bin/env bash
    rm -rf build/cmake-freertos-armcm3-zenoh
    for example in talker listener service-server service-client action-server action-client; do
        rm -rf "examples/qemu-arm-freertos/rust/$example/target-zenoh"
        # Phase 220.C path B — `build-cyclonedds` retired from rust freertos
        # (no cmake-bridge build path); leave the rm for legacy artifacts.
        rm -rf "examples/qemu-arm-freertos/rust/$example/build-cyclonedds"
        rm -rf "examples/qemu-arm-freertos/c/$example/build-zenoh"
        rm -rf "examples/qemu-arm-freertos/cpp/$example/build-zenoh"
    done
    echo "FreeRTOS build artifacts cleaned"

# =============================================================================
# Tracing (Tonbandgeraet → Perfetto)
# =============================================================================

# Build and run a FreeRTOS example with tracing, then convert to Perfetto.
#
# Each example gets its own subdirectory so multiple instances can run in
# parallel (e.g., talker + listener with a shared zenohd).
#
# Usage:
#   just freertos trace talker    # single example
#   # For a pair, start zenohd first, then run in separate terminals:
#   #   terminal 1: ZENOH_CONFIG_OVERRIDE='listen/endpoints=["tcp/127.0.0.1:7800"];scouting/multicast/enabled=false' ros2 run rmw_zenoh_cpp rmw_zenohd
#   #   terminal 2: just freertos trace talker
#   #   terminal 3: just freertos trace listener
#
# Output: test-logs/freertos-trace/<example>/trace.pf
trace example="talker":
    #!/usr/bin/env bash
    set -e

    EXAMPLE="{{example}}"
    EXAMPLE_DIR="examples/qemu-arm-freertos/rust/$EXAMPLE"
    TRACE_DIR="test-logs/freertos-trace/$EXAMPLE"
    mkdir -p "$TRACE_DIR"

    if [ ! -d "$EXAMPLE_DIR" ]; then
        echo "ERROR: Example not found: $EXAMPLE_DIR"
        exit 1
    fi

    echo "=== Building $EXAMPLE with tracing ==="
    # phase-336 — the freertos-qemu carve-out, and the path derived from the
    # SAME answer, so the build and the lookup cannot disagree.
    source scripts/build/cargo.sh
    FR_PROFILE="$(nros_cargo_freertos_qemu_profile)"
    FR_ARGS="$(nros_cargo_profile_args_for "$FR_PROFILE")"
    FR_DIR="$(nros_cargo_target_profile_dir_for "$FR_PROFILE")"
    # issue 0488 residue 2 — same shared group as `_run-qemu`, same reason.
    source scripts/build/fixtures-target-dir.sh
    fr_tdir_flag="$(nros_fixture_target_dir_flag freertos "" "")"
    fr_artifacts="$(nros_fixture_row_artifact_dir "$EXAMPLE_DIR" freertos "" "")"
    (cd "$EXAMPLE_DIR" && NROS_TRACE=1 cargo build $FR_ARGS $fr_tdir_flag)

    # Find the binary name from Cargo.toml
    BINARY=$(grep '^name = ' "$EXAMPLE_DIR/Cargo.toml" | head -1 | sed 's/name = "\(.*\)"/\1/')
    BINARY_PATH="$fr_artifacts/thumbv7m-none-eabi/$FR_DIR/$BINARY"
    if [ ! -f "$BINARY_PATH" ]; then
        echo "ERROR: Binary not found: $BINARY_PATH"
        exit 1
    fi

    # Build tband-cli if needed (before cd to trace dir)
    # profile-literal-ok: vendored: tband-cli is built --release above, in a third-party workspace
    TBAND_CLI="third-party/tracing/Tonbandgeraet/tools/target/release/tband-cli"
    if [ ! -f "$TBAND_CLI" ]; then
        echo "Building tband-cli..."
        # Host tool from a vendored third-party workspace — outside the
        # propagation graph, and it has no nros-* profile defined. `--release`
        # stays literal here (phase-336 allow-list).
        # profile-literal-ok: vendored: third-party workspace defines no nros-* profile
        (cd "third-party/tracing/Tonbandgeraet/tools" && cargo build --release -p tband-cli)
    fi

    echo "=== Running $EXAMPLE in QEMU (with tracing) ==="
    echo "    Trace will be dumped to $TRACE_DIR/trace.bin on exit."
    # Run QEMU — semihosting creates trace.bin in CWD
    TRACE_DIR_ABS="$(pwd)/$TRACE_DIR"
    BINARY_ABS="$(pwd)/$BINARY_PATH"
    TBAND_CLI_ABS="$(pwd)/$TBAND_CLI"
    cd "$TRACE_DIR_ABS"
    rm -f trace.bin trace.pf

    # Run QEMU with a trap so Ctrl-C kills it cleanly.
    # QEMU with semihosting ignores SIGINT — we must kill explicitly.
    QEMU_PID=""
    trap 'if [ -n "$QEMU_PID" ]; then kill $QEMU_PID 2>/dev/null; wait $QEMU_PID 2>/dev/null; fi; exit 130' INT TERM
    timeout 60 {{ if path_exists(QEMU_BIN) == "true" { QEMU_BIN } else { "qemu-system-arm" } }} \
        -cpu cortex-m3 -machine mps2-an385 -nographic \
        -icount shift=auto \
        -semihosting-config enable=on,target=native \
        -kernel "$BINARY_ABS" \
        -nic user,model=lan9118 &
    QEMU_PID=$!
    wait $QEMU_PID || true
    QEMU_PID=""
    trap - INT TERM

    if [ ! -f trace.bin ]; then
        echo "ERROR: trace.bin not created — semihosting file I/O may not be working"
        exit 1
    fi

    echo "=== Converting trace to Perfetto format ==="
    "$TBAND_CLI_ABS" conv --format bin --mode free-rtos --core-count 1 \
        --output trace.pf trace.bin

    echo ""
    echo "=== Trace captured ==="
    echo "  Raw:      $TRACE_DIR/trace.bin ($(wc -c < trace.bin) bytes)"
    echo "  Perfetto: $TRACE_DIR/trace.pf"
    echo ""
    echo "Open in browser: https://ui.perfetto.dev"
    echo "  → Open trace file → select $TRACE_DIR/trace.pf"

# =============================================================================
# Run examples (build + launch QEMU MPS2-AN385 with slirp networking)
#
# QEMU with semihosting ignores SIGINT (Ctrl-C). Each recipe runs QEMU in
# the background and traps INT/TERM to kill it explicitly.
# =============================================================================

[no-exit-message]
_run-qemu example binary:
    #!/usr/bin/env bash
    set -e
    source "{{justfile_directory()}}/scripts/build/cargo.sh"
    # issue 0488 residue 2 — the SHARED group, not a per-leaf `target-zenoh/`.
    # `freertos` is a migrated platform, so the fixture lane already compiles
    # this configuration once; the authored dir re-did the whole graph per leaf
    # and per run. Flag and `-kernel` path come from ONE `nros_fixture_group`
    # call so they cannot disagree (phase-340 item 7's failure).
    source "{{justfile_directory()}}/scripts/build/fixtures-target-dir.sh"
    FR_PROFILE="$(nros_cargo_freertos_qemu_profile)"
    FR_DIR="$(nros_cargo_target_profile_dir_for "$FR_PROFILE")"
    FR_LEAF="examples/qemu-arm-freertos/rust/{{example}}"
    fr_tdir_flag="$(nros_fixture_target_dir_flag freertos "" "")"
    fr_artifacts="$(nros_fixture_row_artifact_dir "$FR_LEAF" freertos "" "")"
    cd "{{justfile_directory()}}/$FR_LEAF" && \
        cargo build $(nros_cargo_profile_args_for "$FR_PROFILE") $fr_tdir_flag
    trap 'kill $PID 2>/dev/null; wait $PID 2>/dev/null; exit 130' INT TERM
    {{ if path_exists(QEMU_BIN) == "true" { QEMU_BIN } else { "qemu-system-arm" } }} -cpu cortex-m3 -machine mps2-an385 -nographic \
        -icount shift=auto \
        -semihosting-config enable=on,target=native \
        -kernel "$fr_artifacts/thumbv7m-none-eabi/$FR_DIR/{{binary}}" \
        -nic user,model=lan9118 &
    PID=$!
    wait $PID

# Run FreeRTOS talker on QEMU
talker: (_run-qemu "talker" "talker")

# Run FreeRTOS listener on QEMU
listener: (_run-qemu "listener" "listener")

# Run FreeRTOS service server on QEMU
service-server: (_run-qemu "service-server" "service-server")

# Run FreeRTOS service client on QEMU
service-client: (_run-qemu "service-client" "service-client")

# Run FreeRTOS action server on QEMU
action-server: (_run-qemu "action-server" "action-server")

# Run FreeRTOS action client on QEMU
action-client: (_run-qemu "action-client" "action-client")

# Start zenohd router for FreeRTOS QEMU examples (allocator rust-pubsub port 7800)
zenohd:
    #!/usr/bin/env bash
    set -e
    source scripts/dev/zenohd.sh
    nros_router_exec tcp/127.0.0.1:7800
