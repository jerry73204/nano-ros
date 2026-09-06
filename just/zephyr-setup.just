setup *ARGS:
    #!/usr/bin/env bash
    set -e
    WORKSPACE="{{ZEPHYR_WORKSPACE}}"
    ARGS="{{ARGS}}"
    # Phase 197.1 — provision the in-tree sources the Zephyr build needs the way a
    # user/CI does (`nros setup --source`, index-driven), NOT a hand
    # `git submodule update`: zenoh-pico (zenoh examples), the cyclonedds fork (the
    # patches below operate on the in-tree submodule), and px4-rs (the root
    # workspace's cargo load path-deps px4-sitl-tests). Mirrors the CI workflow so
    # local == CI. nros resolved from $NROS_CLI / PATH / ~/.nros (install.sh).
    source scripts/build/cargo.sh
    nros_bin="$(nros_cli_bin)"
    echo "[zephyr setup] provisioning sources via nros ($nros_bin)..."
    "$nros_bin" setup --source zenoh-pico --source cyclonedds-src --source px4-rs \
        --index "$(pwd)/nros-sdk-index.toml"
    if [ -d "$WORKSPACE/zephyr" ] && [[ "$ARGS" != *--force* ]]; then
        echo "Zephyr workspace already present at $WORKSPACE"
        echo "To reinstall, run: just zephyr setup --force"
    else
        NROS_ZEPHYR_MANIFEST="{{ZEPHYR_MANIFEST}}" NROS_ZEPHYR_WORKSPACE="$WORKSPACE" ./scripts/zephyr/setup.sh $ARGS
    fi
    # Phase 160.B — ensure the Cortex-A9 (qemu_cortex_a9) toolchain
    #   1. `armv7a-none-eabi` rust target (no_std core / compiler_builtins)
    #   2. `cortex-a9-rust-patch.sh`: patches Zephyr's Rust module to
    #      register the Cortex-A9 target + adds the SLCR MMU entry the
    #      Zynq-7000 eth_xlnx_gem driver needs.
    # Both are idempotent — safe to run on every `just zephyr setup`.
    echo "[zephyr setup] ensuring armv7a-none-eabi Rust target..."
    rustup target add armv7a-none-eabi
    # Phase 199.3 — version-dispatched patch set. The per-line patch sequences
    # (formerly an inline `if NROS_ZEPHYR_VERSION = 4.4 … else …` here) live in
    # scripts/zephyr/patches/<version>.sh. Adding a Zephyr line = drop a new
    # sibling there; no edit to this recipe.
    PATCHSET="./scripts/zephyr/patches/{{NROS_ZEPHYR_VERSION}}.sh"
    if [ ! -f "$PATCHSET" ]; then
        echo "[zephyr setup] ERROR: no patch set for Zephyr {{NROS_ZEPHYR_VERSION}} ($PATCHSET)." >&2
        echo "  Supported lines: $(ls ./scripts/zephyr/patches/*.sh | xargs -n1 basename | sed 's/\.sh$//' | tr '\n' ' ')" >&2
        echo "  Add scripts/zephyr/patches/{{NROS_ZEPHYR_VERSION}}.sh (see patches/README.md)." >&2
        exit 1
    fi
    echo "[zephyr setup] applying {{NROS_ZEPHYR_VERSION}} patch set ($PATCHSET)..."
    bash "$PATCHSET" "$WORKSPACE"
    # The Micro-XRCE-DDS Agent is only needed for XRCE *runtime* tests. Builds
    # that use a different RMW (e.g. the zenoh dual-line CI) can skip the heavy
    # Fast-DDS superbuild via NROS_ZEPHYR_SKIP_XRCE_AGENT.
    if [ -n "${NROS_ZEPHYR_SKIP_XRCE_AGENT:-}" ]; then
        echo "[zephyr setup] skipping Micro XRCE-DDS Agent (NROS_ZEPHYR_SKIP_XRCE_AGENT set)"
    else
        echo "[zephyr setup] ensuring Micro XRCE-DDS Agent for Zephyr XRCE tests..."
        just xrce setup
    fi

# Issue 0651. Two shallow clones, NOT a west workspace: Kconfig lives in the
# `zephyr` repo plus the modules west pins, and of those only `zephyr-lang-rust`
# defines a symbol this repo references (`RUST`). A full 4.4 west workspace is
# ~20 modules and filled a CI disk once (issue 0078). Revisions are read from
# `west-4.4.yml`, never restated. Idempotent; `--force` refetches.
#
# Fetch the Zephyr 4.4 source `check::zephyr-kconfig-symbols` needs (~600 MB).
[group("setup")]
kconfig-trees *ARGS:
    @bash scripts/zephyr/fetch-kconfig-trees.sh {{ARGS}}

# Diagnose Zephyr install status (read-only).
[group("setup")]
doctor:
    #!/usr/bin/env bash
    bash scripts/dev/deprecated-scope-alias.sh zephyr doctor "just doctor zephyr"
    set +e
    fail=0
    WORKSPACE="{{ZEPHYR_WORKSPACE}}"
    # issue 0698 follow-up — the Zephyr venv is this lane's, not the session's.
    source scripts/build/zephyr-python.sh
    nros_zephyr_activate
    if command -v west &>/dev/null; then
        echo "  [OK] west"
    else
        echo "  [MISSING] west (run: just zephyr setup)"
        fail=1
    fi
    if [ -d "$WORKSPACE/zephyr" ]; then
        echo "  [OK] Zephyr workspace: $WORKSPACE"
    else
        echo "  [MISSING] Zephyr workspace at $WORKSPACE (run: just zephyr setup)"
        fail=1
    fi
    # The SDK is a per-LINE requirement and a mismatch is otherwise invisible
    # until the first build fails inside FindZephyr-sdk.cmake with an error
    # naming neither the version nor the setup step that chose it.
    # `zephyr/SDK_VERSION` is Zephyr's own statement of what this tree needs.
    if [ -f "$WORKSPACE/zephyr/SDK_VERSION" ]; then
        want="$(cat "$WORKSPACE/zephyr/SDK_VERSION")"
        have=""
        for reg in "$HOME"/.cmake/packages/Zephyr-sdk/*; do
            [ -f "$reg" ] || continue
            case "$(cat "$reg")" in *"zephyr-sdk-$want"*) have="$(cat "$reg")";; esac
        done
        if [ -n "$have" ]; then
            echo "  [OK] Zephyr SDK $want registered ($have)"
        else
            echo "  [MISSING] Zephyr SDK $want — this line's zephyr/SDK_VERSION demands it."
            echo "            Registered instead:"
            for reg in "$HOME"/.cmake/packages/Zephyr-sdk/*; do
                [ -f "$reg" ] && echo "              $(cat "$reg")"
            done
            echo "            Run: just zephyr setup (the SDK is line-dispatched)"
            fail=1
        fi
    fi
    if rustup target list --installed 2>/dev/null | grep -q '^armv7a-none-eabi$'; then
        echo "  [OK] armv7a-none-eabi rust target (qemu_cortex_a9 DDS)"
    else
        echo "  [MISSING] armv7a-none-eabi rust target (run: just zephyr setup)"
        fail=1
    fi
    if just xrce doctor; then
        :
    else
        fail=1
    fi
    # native_sim uses NSOS (host loopback) — no TAP bridge required.
    exit $fail

# phase-350 W3 (issue 0537) — this used to also build
# `build-fvp-aemv8r-cyclonedds{,-rust}`. Those two, their `run-` siblings and
# the two `examples/zephyr/{cpp,rust}/talker-aemv8r` apps were RETIRED: their
# runners (`fvp_runtime.rs` / `fvp_runtime_rust.rs`) were deleted by phase-298
# W4, so for months they built an artifact nothing ran, and the justfile still
# read as a complete lane.
#
# FVP support is wanted LATER, and what that needs is kept — the
# `fvp-aemv8r-smp` board crate, `nano_ros_use_board()`, the board-import
# surface and the workspace-Entry runtime, all of which still have consumers.
# What went is only the part with none. (The historical `build-fvp-aemv8r` base
# lane went the same way in #217: unbuildable since phase-221 dropped its west
# source arg, its app long gone.)
#
# Issue #216 — aggregate FVP build sweep. The FVP lanes are manual-only (no CI
# consumer; each skips cleanly when west / the Zephyr workspace / the aarch64
# SDK is missing), which is exactly how the rust lane rotted silently to E0463
# for weeks. One verb for a periodic local sweep.
[group("examples")]
build-fvp-all: build-fvp-ws-entry

# phase-292 W1.a — the FVP WORKSPACE C++ RT-tiers Entry
# (examples/workspaces/realtime-cpp/src/fvp_entry): proves the ASI
# reference consumer's exact shape in-tree — `nano_ros_use_board(fvp-aemv8r-smp)`
# + `find_package(nano_ros)` (287-W6 verbs) + one
# `nano_ros_add_executable(BOARD zephyr LAUNCH … TYPED)` in a single west
# configure. NO `-b` flag: the board id comes from the board crate. Skips
# identically to the other FVP lanes.
[group("examples")]
build-fvp-ws-entry:
    #!/usr/bin/env bash
    # shellcheck source=scripts/build/lane-skip.sh
    source scripts/build/lane-skip.sh
    set -e
    # issue 0698 follow-up — the Zephyr venv is this lane's, not the session's.
    source scripts/build/zephyr-python.sh
    nros_zephyr_activate
    if ! command -v west >/dev/null 2>&1; then
        nros_lane_skip "west not found"
    fi
    workspace="$(realpath {{ZEPHYR_WORKSPACE}} 2>/dev/null || echo "")"
    if [ -z "$workspace" ] || [ ! -d "$workspace/zephyr" ]; then
        nros_lane_skip "{{ZEPHYR_WORKSPACE}} not set up"
    fi
    sdk="$(pwd)/scripts/zephyr/sdk/zephyr-sdk-0.16.8"
    if [ ! -x "$sdk/aarch64-zephyr-elf/bin/aarch64-zephyr-elf-gcc" ]; then
        nros_lane_skip "Zephyr/FVP-ws-entry skip: aarch64-zephyr-elf toolchain missing"
    fi
    nros_root="$(pwd)"
    echo "  → host nros-codegen (cargo)"
    source scripts/build/cargo.sh
    nros_cargo_ensure_codegen_c
    codegen_tool="$(nros_cargo_codegen_c_bin)"
    make_bin="$(nros sdk-path make 2>/dev/null)/bin/make"
    if [ ! -x "$make_bin" ]; then
        make_bin="$(command -v make)"
    fi
    cd "$workspace"
    export PATH="$(nros sdk-path make)/bin:$(nros sdk-path ninja)/bin:$PATH"
    export ZEPHYR_SDK_INSTALL_DIR="$sdk"
    export NROS_REPO_DIR="$nros_root"
    # phase-298 W3 — bake the fast-sim flag into the configure-time
    # run_armfvp target (Zephyr armfvp.cmake reads ARMFVP_EXTRA_FLAGS at
    # configure; last -C wins over the board default =1). AEMv8-R only.
    export ARMFVP_EXTRA_FLAGS="-C cache_state_modelled=0"
    echo "Building examples/workspaces/realtime-cpp/src/fvp_entry (board from nano_ros_use_board)..."
    west build \
        -d build-fvp-ws-entry -p auto \
        "$nros_root/examples/workspaces/realtime-cpp/src/fvp_entry" \
        -- -D_NANO_ROS_CODEGEN_TOOL="$codegen_tool" -DMAKE="$make_bin"
    echo "phase-292 W1.a FVP AEMv8-R workspace-Entry (cpp/cyclonedds) build smoke OK"

# phase-298 W3.1 (issue 0232) — run the realtime-cpp workspace Entry
# (the canonical ASI-consumption mirror: nano_ros_use_board + MODEL +
# run_tiers) on the FVP via stock `west build -t run`. The publish-success markers are
# `[ctrl] tick=` / `[telem] tick=` (printed only on publish().ok());
# `fvp_runtime_ws.rs` greps them. NROS_REPO_DIR is exported because
# `west build -t run` may reconfigure, and the entry's CMakeLists include()s
# the board helper via $ENV{NROS_REPO_DIR}. Runs until killed — callers
# bound it (the test uses its own timeout).
[group("examples")]
run-fvp-ws-entry:
    #!/usr/bin/env bash
    # shellcheck source=scripts/build/lane-skip.sh
    source scripts/build/lane-skip.sh
    set -e
    # phase-320 W1.d/W1.f — the model itself is license-walled and USER-SUPPLIED,
    # so its absence is a skip, not a failure, and the book promises these
    # recipes "skip with a clear hint".
    #
    # RFC-0064 R5 D4 / phase-215.K.4 — the resolver's output is now EXPORTED and
    # the launch is stock `west build -t run`. It used to be `west fvp run`, a
    # 117-line west extension whose entire body was: read NROS_BOARD_RUNNER from
    # CMakeCache, run this same resolver, export ARMFVP_BIN_PATH, and then
    # `os.execvpe('west', ['west','build','-t','run'])`. Two of those four steps
    # are below; the other two were the verb's whole reason to exist.
    if ! _fvp_bin="$(bash scripts/zephyr/resolve-fvp-bin.sh 2>/dev/null)"; then
        nros_lane_skip "ARM FVP model not resolvable (set ARM_FVP_DIR / ARMFVP_BIN_PATH; license-gated, user-supplied)"
    fi
    # issue 0698 follow-up — the Zephyr venv is this lane's, not the session's.
    source scripts/build/zephyr-python.sh
    nros_zephyr_activate
    if ! command -v west >/dev/null 2>&1; then
        nros_lane_skip "west not found"
    fi
    workspace="$(realpath {{ZEPHYR_WORKSPACE}} 2>/dev/null || echo "")"
    if [ -z "$workspace" ] || [ ! -d "$workspace/zephyr" ]; then
        nros_lane_skip "{{ZEPHYR_WORKSPACE}} not set up"
    fi
    if [ ! -f "$workspace/build-fvp-ws-entry/zephyr/zephyr.elf" ]; then
        nros_lane_skip "Zephyr/FVP-ws-entry run skip: build-fvp-ws-entry/zephyr/zephyr.elf missing — run \`just zephyr build-fvp-ws-entry\` first"
    fi
    nros_root="$(pwd)"
    cd "$workspace"
    export PATH="$(nros sdk-path make)/bin:$(nros sdk-path ninja)/bin:$PATH"
    export ZEPHYR_SDK_INSTALL_DIR="$nros_root/scripts/zephyr/sdk/zephyr-sdk-0.16.8"
    export NROS_REPO_DIR="$nros_root"
    export ARMFVP_EXTRA_FLAGS="-C cache_state_modelled=0"
    export ARMFVP_BIN_PATH="$_fvp_bin"
    west build -d build-fvp-ws-entry -t run

# phase-298 W3.3 (issue 0232) — the maintainer pre-release gate: build the
# ws-entry image, then run the `fvp_runtime_ws` assertion test. One command
# a maintainer (with the license-gated FVP installed) runs before an ASI pin
# bump or release. Skips cleanly (never false-fails) when the model is absent.
[group("examples")]
verify-fvp-runtime:
    #!/usr/bin/env bash
    set -e
    just zephyr build-fvp-ws-entry
    cargo nextest run -p nros-tests --test fvp_runtime_ws --no-capture

# Phase 215.G.1 — build the minimal ASI-shaped board-import fixture
# (`packages/testing/nros-tests/fixtures/board_import_fvp`) for the FVP
# AEMv8-R target THROUGH the Phase 215.B import surface: the fixture's
# CMakeLists carries ONLY `nano_ros_use_board(fvp-aemv8r-smp)` +
# `find_package(Zephyr)` + a trivial `printk` app — no hand-rolled
# BOARD / EXTRA_CONF_FILE / DTC_OVERLAY_FILE / -DNANO_ROS_RMW. This is
# the canonical "is the board crate import surface healthy?" build.
# `nano_ros_use_board()` sets `BOARD` from `board.cmake`, so no `-b`.
# Skips cleanly when west / Zephyr workspace / aarch64 SDK are absent.
[group("examples")]
build-fvp-board-import:
    #!/usr/bin/env bash
    # shellcheck source=scripts/build/lane-skip.sh
    source scripts/build/lane-skip.sh
    set -e
    # issue 0698 follow-up — the Zephyr venv is this lane's, not the session's.
    source scripts/build/zephyr-python.sh
    nros_zephyr_activate
    if ! command -v west >/dev/null 2>&1; then
        nros_lane_skip "west not found"
    fi
    workspace="$(realpath {{ZEPHYR_WORKSPACE}} 2>/dev/null || echo "")"
    if [ -z "$workspace" ] || [ ! -d "$workspace/zephyr" ]; then
        nros_lane_skip "{{ZEPHYR_WORKSPACE}} not set up"
    fi
    sdk="$(pwd)/scripts/zephyr/sdk/zephyr-sdk-0.16.8"
    if [ ! -x "$sdk/aarch64-zephyr-elf/bin/aarch64-zephyr-elf-gcc" ]; then
        nros_lane_skip "Zephyr/FVP board-import skip: aarch64-zephyr-elf toolchain missing"
    fi
    bash ./scripts/zephyr/cortex-a9-rust-patch.sh "$workspace"
    bash ./scripts/zephyr/aarch64-rust-patch.sh "$workspace"
    # issue 0432 — the gpio pair, without which the `zephyr` crate does not
    # compile for any board with gpio nodes in its devicetree.
    bash ./scripts/zephyr/zephyr-lang-rust-gpio-patch.sh "$workspace"
    nros_root="$(pwd)"
    echo "  → host nros-codegen (cargo)"
    source scripts/build/cargo.sh
    nros_cargo_ensure_codegen_c
    codegen_tool="$(nros_cargo_codegen_c_bin)"
    make_bin="$(nros sdk-path make 2>/dev/null)/bin/make"
    if [ ! -x "$make_bin" ]; then
        make_bin="$(command -v make)"
    fi
    src="$nros_root/packages/testing/nros-tests/fixtures/board_import_fvp"
    cd "$workspace"
    export PATH="$(nros sdk-path make)/bin:$(nros sdk-path ninja)/bin:$PATH"
    export ZEPHYR_SDK_INSTALL_DIR="$sdk"
    # Phase 215.B — the fixture's CMakeLists includes
    # `$ENV{NROS_REPO_DIR}/zephyr/cmake/nano_ros_use_board.cmake` BEFORE
    # `find_package(Zephyr)`; export the repo root so the include resolves.
    export NROS_REPO_DIR="$nros_root"
    echo "Building board-import fixture via nano_ros_use_board(fvp-aemv8r-smp)..."
    west build \
        -d build-fvp-board-import -p auto \
        "$src" \
        -- -D_NANO_ROS_CODEGEN_TOOL="$codegen_tool" -DMAKE="$make_bin"
    echo "Phase 215.G.1 FVP board-import fixture build OK"

# Phase 215.G.1 — run the board-import fixture on the FVP THROUGH the
# import surface (the board's `[board.zephyr] runner` reaches `NROS_BOARD_RUNNER` in
# the CMakeCache that `nano_ros_use_board()` cached). Same resolver +
# west delegation as the other `run-fvp-*` recipes; different build dir.
# The Phase 215.G integration test (`phase215_g_fvp_smoke`) drives this
# recipe and scrapes the `nros: smoke ok` UART line.
[group("examples")]
run-fvp-board-import:
    #!/usr/bin/env bash
    # shellcheck source=scripts/build/lane-skip.sh
    source scripts/build/lane-skip.sh
    set -e
    # phase-320 W1.d/W1.f — the model itself is license-walled and USER-SUPPLIED,
    # so its absence is a skip, not a failure, and the book promises these
    # recipes "skip with a clear hint".
    #
    # RFC-0064 R5 D4 / phase-215.K.4 — the resolver's output is now EXPORTED and
    # the launch is stock `west build -t run`. It used to be `west fvp run`, a
    # 117-line west extension whose entire body was: read NROS_BOARD_RUNNER from
    # CMakeCache, run this same resolver, export ARMFVP_BIN_PATH, and then
    # `os.execvpe('west', ['west','build','-t','run'])`. Two of those four steps
    # are below; the other two were the verb's whole reason to exist.
    if ! _fvp_bin="$(bash scripts/zephyr/resolve-fvp-bin.sh 2>/dev/null)"; then
        nros_lane_skip "ARM FVP model not resolvable (set ARM_FVP_DIR / ARMFVP_BIN_PATH; license-gated, user-supplied)"
    fi
    # issue 0698 follow-up — the Zephyr venv is this lane's, not the session's.
    source scripts/build/zephyr-python.sh
    nros_zephyr_activate
    if ! command -v west >/dev/null 2>&1; then
        nros_lane_skip "west not found"
    fi
    workspace="$(realpath {{ZEPHYR_WORKSPACE}} 2>/dev/null || echo "")"
    if [ -z "$workspace" ] || [ ! -d "$workspace/zephyr" ]; then
        nros_lane_skip "{{ZEPHYR_WORKSPACE}} not set up"
    fi
    if [ ! -f "$workspace/build-fvp-board-import/zephyr/zephyr.elf" ]; then
        nros_lane_skip "Zephyr/FVP board-import run skip: build-fvp-board-import/zephyr/zephyr.elf missing — run \`just zephyr build-fvp-board-import\` first"
    fi
    nros_root="$(pwd)"
    cd "$workspace"
    export PATH="$(nros sdk-path make)/bin:$(nros sdk-path ninja)/bin:$PATH"
    export ZEPHYR_SDK_INSTALL_DIR="$nros_root/scripts/zephyr/sdk/zephyr-sdk-0.16.8"
    export ARMFVP_BIN_PATH="$_fvp_bin"
    west build -d build-fvp-board-import -t run

# phase-337 W7.b — the S32Z270-DC RTU0 board left the tree with the other
# scaffolds: `nros-board-s32z270dc2-r52` had ZERO cargo consumers, its
# `build-s32z-board-import` recipe had NO caller, and its `board_import_s32z`
# fixture was never registered in WEST_FIXTURES — so nothing ever built it.
# The board's reference consumer (ASI `zephyr-s32z`) is out-of-tree and
# C++-only; per RFC-0064 a Zephyr board is a conf bundle its integrator owns,
# not a crate here. `build-fvp-board-import` above is the surviving
# `nano_ros_use_board()` import proof.
