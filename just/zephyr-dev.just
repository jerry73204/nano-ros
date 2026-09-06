build:
    @echo "Zephyr has no separate core build; use 'just zephyr build-examples' for the matrix, or 'just zephyr build-one <example> <rmw>' for one."

# Build ONE Zephyr example for the selected line (NROS_ZEPHYR_VERSION).
# Reproduces the Phase 180.A orchestration: host nros-codegen + (4.4) the
# Python 3.12 venv on PATH + host toolchain. Assumes `just zephyr setup`
# already ran (it applies the 3.7 patch set / provisions the 4.4 venv).
#   just zephyr build-one c/talker zenoh
#   NROS_ZEPHYR_VERSION=4.4 just zephyr build-one c/talker zenoh
[group("examples")]
build-one example rmw="zenoh" board="native_sim/native/64":
    #!/usr/bin/env bash
    set -euo pipefail
    source scripts/build/cargo.sh
    # phase-400 W5 — `nros_build_dir` and the KIND constants come from here, and
    # this must happen BEFORE the recipe changes into the west workspace: the
    # path is relative to the repo root.
    source scripts/build/build-root.sh
    # Phase 180.B — examples read the ROS interface package dirs from env
    # (no more hardcoded /opt/ros/humble/share/* PKG_DIR). Default to the
    # stock Humble install; override per package if your ROS lives elsewhere.
    export NROS_STD_MSGS_DIR="${NROS_STD_MSGS_DIR:-/opt/ros/humble/share/std_msgs}"
    export NROS_EXAMPLE_INTERFACES_DIR="${NROS_EXAMPLE_INTERFACES_DIR:-/opt/ros/humble/share/example_interfaces}"
    export NROS_BUILTIN_INTERFACES_DIR="${NROS_BUILTIN_INTERFACES_DIR:-/opt/ros/humble/share/builtin_interfaces}"
    export NROS_UNIQUE_IDENTIFIER_MSGS_DIR="${NROS_UNIQUE_IDENTIFIER_MSGS_DIR:-/opt/ros/humble/share/unique_identifier_msgs}"
    export NROS_ACTION_MSGS_DIR="${NROS_ACTION_MSGS_DIR:-/opt/ros/humble/share/action_msgs}"
    workspace="$(realpath {{ZEPHYR_WORKSPACE}} 2>/dev/null || echo "")"
    if [ -z "$workspace" ] || [ ! -d "$workspace/zephyr" ]; then
        echo "Zephyr {{NROS_ZEPHYR_VERSION}} workspace not set up — run:"
        echo "  NROS_ZEPHYR_VERSION={{NROS_ZEPHYR_VERSION}} just zephyr setup"
        exit 1
    fi
    # 4.4 line: run west through the Python 3.12 venv (Task 2b) + host
    # toolchain (native_sim = host gcc). Invoke west via the venv interpreter
    # EXPLICITLY rather than prepending the venv to PATH — a PATH prepend also
    # shadows `python3` for build subprocesses, which breaks the cyclonedds
    # descriptor codegen (ROS msg2idl needs catkin_pkg/rosidl_adapter from the
    # system ROS python 3.10, not the venv 3.12). west passes WEST_PYTHON to
    # cmake so find_package(Python3>=3.12) is still satisfied.
    venvbin="{{ZEPHYR_VENV_BIN}}"
    # issue 0698 follow-up — the Zephyr venv is this lane's, not the session's.
    source scripts/build/zephyr-python.sh
    nros_zephyr_activate
    west_cmd=(west)
    if [ -n "$venvbin" ]; then
        west_cmd=("$(realpath "$venvbin")/python" -m west)
    fi
    # issues #87 + 0698 — host gcc toolchain for native_sim on ANY line
    # (board-keyed, not version-keyed); real boards name `zephyr` rather than
    # leaving the variant unset, which is what CMake 4 rejects. Shared rule:
    # scripts/build/zephyr-toolchain.sh. The venv python switch above stays
    # 4.4-specific (find_package Python3>=3.12).
    source scripts/build/zephyr-toolchain.sh
    export ZEPHYR_TOOLCHAIN_VARIANT="$(nros_zephyr_toolchain_variant "{{board}}")"
    # Host codegen tool — nros_generate_interfaces() needs it (version-independent).
    nros_cargo_ensure_codegen_c
    codegen_tool="$(nros_cargo_codegen_c_bin)"
    src="$(pwd)/examples/zephyr/{{example}}"
    [ -d "$src" ] || { echo "no example at $src"; exit 1; }
    # Rust examples consume the generated interface crates (std_msgs +
    # transitively builtin_interfaces, …) as cargo path-deps via
    # .cargo/config.toml's [patch.crates-io] — those crates must exist before the
    # cargo-driven west build. The C/C++ paths codegen in cmake via
    # nros_generate_interfaces(); the Rust path has no cmake equivalent, so run
    # `nros generate-rust` here. Needs ROS sourced (AMENT_PREFIX_PATH resolves
    # the .msg defs — CI sources it; locally `source /opt/ros/<distro>/setup.bash`).
    case "{{example}}" in
    rust/*)
        echo "Zephyr: generating Rust interface crates for {{example}}"
        # Phase 214.I.2 — fail-loud prereq guard: `nros sync` is absent
        # from the shipped 0.3.7 release. Probe; emit `[PREREQ]` + exit 0
        # if missing rather than cascading into clap noise.
        nros_require_sync "$codegen_tool"
        # Phase 214.J.2 — wipe `generated/` on trait-surface drift before sync.
        source "$(pwd)/scripts/build/codegen-stamp.sh"
        NROS_REPO_DIR="$(pwd)" nros_codegen_stamp_check_or_wipe "$src"
        # issue 0318 — no `--no-metadata` needed. A zephyr leaf declares
        # `[package.metadata.nros.deploy.zephyr]`, which `nros` now reads as
        # deploy-bound, so it skips the host probe for that component by itself
        # (the probe cannot compile it: the `zephyr` crate's build script wants
        # `DOTCONFIG` from a cmake configure this sync is the prerequisite for).
        # The flag was a blunter version of the same thing — it suppressed
        # probing for EVERY component in the workspace.
        NROS_REPO_DIR="$(pwd)" "$codegen_tool" sync "$src"
        NROS_REPO_DIR="$(pwd)" nros_codegen_stamp_write "$src"
        ;;
    esac
    # CONF_FILE = base + RMW overlay + (native_sim) the version-aware NSOS
    # line overlay. The line overlay (cmake/zephyr/native-sim-line-<v>.conf)
    # supplies the native-eth-driver disable under the right symbol for the
    # line (ETH_NATIVE_POSIX on 3.7, ETH_NATIVE_TAP on 4.4) plus the shared
    # NSOS settings — so NSOS works on BOTH lines. It supersedes the legacy
    # per-example boards/native_sim_*.conf (which hardcodes the 3.7 symbol);
    # that file is left for the not-yet-version-gated build-fixtures path.
    conf="prj.conf;prj-{{rmw}}.conf"
    case "{{board}}" in
    native_sim*)
        line_overlay="$(pwd)/cmake/zephyr/native-sim-line-{{NROS_ZEPHYR_VERSION}}.conf"
        if [ -f "$line_overlay" ]; then
            conf="$conf;$line_overlay"
        else
            echo "WARN: no native_sim line overlay at $line_overlay" >&2
        fi
        ;;
    qemu_cortex_m3*|mps2*)
        # No entropy device on these boards; the net stack needs sys_rand_get.
        # Overlays live here rather than in a per-example `boards/*.conf`
        # because Zephyr only auto-merges those when CONF_FILE is UNSET
        # (cmake/modules/configuration_files.cmake:39) and every build here
        # sets it explicitly — so a `boards/` fragment is silently inert.
        # phase-391 W3 — `mps2*` added: `build-one c/talker zenoh mps2/an385`
        # failed on `z_impl_sys_rand_get` for exactly this reason, while the
        # fixture lane's mps2 images (which get the fragment) link fine. The
        # conf's content is board-agnostic (TEST/TIMER_RANDOM_GENERATOR).
        # NOTE `qemu_cortex_m3` itself is the TI LM3S6965 (256K flash / 64K
        # RAM) — far too small for a zenoh image; mps2/an385 is the Cortex-M
        # board this repo actually targets.
        conf="$conf;$(pwd)/cmake/zephyr/qemu-cortex-m3.conf"
        ;;
    esac
    bd="build-$(echo '{{example}}' | tr '/' '-')-{{rmw}}"
    echo "Zephyr {{NROS_ZEPHYR_VERSION}}: building zephyr/{{example}} ({{rmw}}, {{board}}) -> $workspace/$bd"
    cd "$workspace"
    # phase-400 W5 — share the cargo dependency mass across images.
    #
    # Zephyr was the one platform building every image against its own cargo
    # target dir: 191 of them across 89 west build trees, each recompiling the
    # same ~86 host crates. The six other platforms have passed a shared root
    # since issue 0805; this is the same call they make.
    #
    # Safe here only because the generated headers no longer come from inside
    # that directory (W5.c): they are placed from `$OUT_DIR`, which cargo reports
    # even on a cache hit. Before that, sharing left every image after the first
    # without its headers.
    shared_cargo_root="$(nros_build_dir "$NROS_KIND_CORROSION_CARGO" zephyr)"
    "${west_cmd[@]}" build -b "{{board}}" -d "$bd" "$src" -- \
        -DCONF_FILE="$conf" -D_NANO_ROS_CODEGEN_TOOL="$codegen_tool" \
        -DNROS_SHARED_CARGO_ROOT="$shared_cargo_root"
    echo "Built: $workspace/$bd/zephyr/zephyr.elf"

# Phase 180.B item 5 — copy-out CI check. Copies an example OUT of the
# repo tree and builds it against the nano-ros Zephyr module on the 4.4
# line, proving the module-only consumption contract (no repo-tree
# walk-up). PASS iff it reaches zephyr.elf. Idempotent; cleans its temp
# dir. Args: <example> <rmw> <board> (defaults c/talker zenoh native_sim).
#   just zephyr check-copy-out
#   just zephyr check-copy-out c/listener zenoh
[group("examples")]
check-copy-out example="c/talker" rmw="zenoh" board="native_sim/native/64":
    ./scripts/zephyr/check-copy-out.sh "{{example}}" "{{rmw}}" "{{board}}"

# issue 0805 — these four recipes DELEGATE to the fixture driver instead of
# looping `build-one` per example, which is what they used to do.
#
# The loops were 24 serial `just zephyr build-one` invocations. They resemble the
# serial-dispatch defect fixed on threadx_riscv64, but that fix does NOT transfer
# — fanning these out would be wrong for reasons already written down here:
#
#   * `build-one` passes no `-j` to `west build`, so each leaf's ninja already
#     takes full `nproc`. N-wide dispatch is N x nproc; the fixture driver bounds
#     total concurrency with a shared fifo jobserver and says outright that the
#     naive form "would oversubscribe to N x ninja_jobs".
#   * The issue-0086 rustup race: `zephyr-ci.just` pre-adds the rust-std targets
#     SERIALLY before its fan-out because concurrent cargo collides on
#     `~/.rustup/downloads/<hash>.partial`. `build-one` has no such guard.
#   * `build-one` pins no toolchain-capability cache dir and takes no repo-level
#     flock; the fixture path does both.
#
# So the answer is not a fan-out but the driver, which is correctly parallel and
# already owns every leaf these loops built. Coverage verified as SETS, not
# counts: each filter below selects exactly the six examples its loop named, and
# `build-fixtures` also applies the same cortex-a9 patch and workspace check the
# loops did.
#
# This delegation was written once, REVERTED because the fixture path could not
# build anything (issue 0845 — a de-duplicated `-include` plus a shim that was
# not inert inside picolibc's own build), and re-landed after that was fixed.
# Same shape as issue 0549 below: a second builder for fixtures the manifest
# already owns is a divergence waiting to happen — and it happened, with only the
# CI-facing one broken.
# Build Zephyr Rust examples (all Rust examples for native_sim)
[group("examples")]
build-rust-examples:
    #!/usr/bin/env bash
    set -e
    # Coverage verified as a SET against the loop this replaced.
    NROS_ZEPHYR_FIXTURE_FILTER='build-rust-.*-zenoh' just zephyr build-fixtures

# Build Zephyr C examples
[group("examples")]
build-c:
    #!/usr/bin/env bash
    set -e
    # Coverage verified as a SET against the loop this replaced.
    NROS_ZEPHYR_FIXTURE_FILTER='build-c-.*-zenoh' just zephyr build-fixtures

# issue 0549 — the `build-logging-smoke` recipe lived here and is GONE. It was
# a SECOND builder for a fixture the manifest already declares: it wrote
# `<workspace>/build-logging-smoke/` (which the test reads) while the manifest
# leaf wrote a different dir nothing read, behind a flag no lane passed. The
# leaf builds with `just zephyr build-fixtures` now, into the same dir.

# Build Zephyr C++ examples
[group("examples")]
build-cpp:
    #!/usr/bin/env bash
    set -e
    # Coverage verified as a SET against the loop this replaced.
    NROS_ZEPHYR_FIXTURE_FILTER='build-cpp-.*-zenoh' just zephyr build-fixtures

# Build Zephyr C + C++ examples against the XRCE-DDS backend
[group("examples")]
build-xrce:
    #!/usr/bin/env bash
    set -e
    # Coverage verified as a SET against the loop this replaced.
    NROS_ZEPHYR_FIXTURE_FILTER='build-(c|cpp)-.*-xrce' just zephyr build-fixtures

# Build all Zephyr examples (Rust + C + C++ + XRCE)
[group("main")]
build-examples: build-rust-examples build-c build-cpp build-xrce
    @echo "All Zephyr examples built!"

# Build every Zephyr tier: core artifacts, examples, and test fixtures.
[group("full-matrix")]
build-all: build build-examples build-fixtures
    @echo "Zephyr build-all completed."

# Run Zephyr E2E tests (requires pre-built Zephyr examples; native_sim uses NSOS on host loopback)
#
# Phase 214.Q: `build-fixtures` is sequenced as a prereq so the nextest
# harness never explodes with "Zephyr fixture binary is stale" — recipes
# own the build-then-test sequencing.
[group("main")]
test verbose="": build-fixtures
    #!/usr/bin/env bash
    bash scripts/dev/deprecated-scope-alias.sh zephyr test "just test zephyr"
    set -e
    source scripts/build/cargo.sh
    cargo_nextest_args=($(nros_cargo_nextest_args))
    args=(-p nros-tests --test zephyr --no-fail-fast)
    if [ -z "{{verbose}}" ]; then
        args+=(--success-output never --failure-output never)
    else
        args+=(--success-output immediate --failure-output immediate)
    fi
    cargo nextest run "${cargo_nextest_args[@]}" "${args[@]}"

# Run all Zephyr tests (zenoh + XRCE + C)
[group("full-matrix")]
test-all verbose="":
    #!/usr/bin/env bash
    set +e
    failed=0
    just zephyr test {{verbose}} || failed=1
    just zephyr test-xrce {{verbose}} || failed=1
    just zephyr test-c || failed=1
    if [ $failed -ne 0 ]; then
        echo "FAIL: Some Zephyr tests failed."
        exit 1
    fi

# Phase 121.6 — build the Zephyr native_sim smoke test for
# nros-platform-zephyr. Pulls platform.c + timer.c (not net.c)
# directly into a minimal Zephyr application and links via west.
[group("examples")]
build-c-port:
    #!/usr/bin/env bash
    set -e
    # Phase 220.D — derive workspace from the same {{ZEPHYR_WORKSPACE}}
    # used by every other zephyr recipe; the old hardcoded
    # `zephyr-workspace/env.sh` only worked under the legacy in-tree
    # layout and silently broke under the canonical
    # `../nano-ros-workspace` sibling layout produced by Phase 218 setup.
    WORKSPACE="{{ZEPHYR_WORKSPACE}}"
    if [ ! -f "$WORKSPACE/env.sh" ]; then
        echo "$WORKSPACE/env.sh missing — run: just zephyr setup" >&2
        exit 2
    fi
    make_bin="$(nros sdk-path make 2>/dev/null)/bin/make"
    if [ ! -x "$make_bin" ]; then
        make_bin="$(command -v make)"
    fi
    export PATH="$(nros sdk-path make)/bin:$(nros sdk-path ninja)/bin:$PATH"
    source "$WORKSPACE/env.sh"
    # issue 0698 follow-up — the Zephyr venv is this lane's, not the session's.
    source scripts/build/zephyr-python.sh
    nros_zephyr_activate
    west build -b native_sim/native/64 \
        -d tests/zephyr-c-smoke/build tests/zephyr-c-smoke \
        -- -DMAKE="$make_bin"

# Phase 121.6 — run the Zephyr native_sim smoke test on the host.
[group("debug")]
test-c-port: build-c-port
    #!/usr/bin/env bash
    set -e
    timeout 15 ./tests/zephyr-c-smoke/build/zephyr/zephyr.exe

# Phase 180.A Task 10 — dual-line CI. Build a representative example
# (default c/talker zenoh) on BOTH the 3.7 LTS line and the 4.4 rolling
# line via `build-one`, reporting per-line pass/fail. A line whose
# workspace is not set up SKIPS with a message (never a silent pass);
# a line whose workspace IS set up but fails to build is a hard FAIL.
# Exit non-zero iff any present line failed.
#   just zephyr ci-both
#   just zephyr ci-both cpp/listener zenoh
[group("full-matrix")]
ci-both example="c/talker" rmw="zenoh":
    #!/usr/bin/env bash
    set -uo pipefail
    declare -A WS=( [3.7]="${NROS_ZEPHYR_WORKSPACE_37:-../nano-ros-workspace}"
                    [4.4]="${NROS_ZEPHYR_WORKSPACE_44:-../nano-ros-workspace-4.4}" )
    overall=0
    declare -A RESULT
    for line in 3.7 4.4; do
        ws="${WS[$line]}"
        ws_abs="$(realpath "$ws" 2>/dev/null || echo "")"
        if [ -z "$ws_abs" ] || [ ! -d "$ws_abs/zephyr" ]; then
            echo "=== Zephyr $line: SKIP (workspace not set up at $ws — run: NROS_ZEPHYR_VERSION=$line just zephyr setup)"
            RESULT[$line]="SKIP"
            continue
        fi
        echo "=== Zephyr $line: building zephyr/{{example}} ({{rmw}}) via build-one ==="
        if NROS_ZEPHYR_VERSION="$line" NROS_ZEPHYR_WORKSPACE="$ws_abs" \
            just zephyr build-one "{{example}}" "{{rmw}}"; then
            echo "=== Zephyr $line: PASS ==="
            RESULT[$line]="PASS"
        else
            echo "=== Zephyr $line: FAIL ==="
            RESULT[$line]="FAIL"
            overall=1
        fi
    done
    echo
    echo "================ dual-line summary ================"
    for line in 3.7 4.4; do
        printf "  Zephyr %-4s : %s\n" "$line" "${RESULT[$line]}"
    done
    echo "==================================================="
    if [ "$overall" -ne 0 ]; then
        echo "FAIL: at least one present Zephyr line failed."
        exit 1
    fi
    echo "OK: all present Zephyr lines built (skips reported above)."

# Issue 0651 direction 3 — the 4.4 cell tier 3 runs. `ci-both` is the wrong
# vehicle for a TIER: it SKIPS a line whose workspace is absent, by design, and
# a skip that reports the same colour as a pass is the exact failure this issue
# was filed about. Tier 3 is pre-release and on demand, so it can afford to
# REQUIRE what it claims to cover.
#
# So this asserts the workspace before building, and fails with the remedy when
# it is missing. There is deliberately NO escape hatch: an opt-out here would
# reintroduce "ci-full says tier 3 passed" over a line nobody built. A host that
# cannot provision 4.4 runs tier 2 (`just ci-matrix`), which never claimed to.
#
# One representative example, matching what the nightly's `ci-both` job builds —
# the point is that a Kconfig or API change fails HERE rather than a day later
# attributed to whatever else moved that night.
[group("full-matrix")]
tier3-cell example="c/talker" rmw="zenoh":
    #!/usr/bin/env bash
    set -euo pipefail
    ws="${NROS_ZEPHYR_WORKSPACE_44:-../nano-ros-workspace-4.4}"
    ws_abs="$(realpath "$ws" 2>/dev/null || echo "")"
    if [ -z "$ws_abs" ] || [ ! -d "$ws_abs/zephyr" ]; then
        echo "error: tier 3 covers the Zephyr 4.4 line and its workspace is absent." >&2
        echo "  looked for: $ws/zephyr" >&2
        echo >&2
        echo "  Provision it:  NROS_ZEPHYR_VERSION=4.4 just zephyr setup" >&2
        echo >&2
        echo "  NOT skipped on purpose (issue 0651): a lane that skips when" >&2
        echo "  unprovisioned reports the same colour as one that passed, which is" >&2
        echo "  what let 4.4 breakage reach the nightly unattributed. If this host" >&2
        echo "  cannot provision 4.4, run tier 2 (just ci-matrix) instead — it does" >&2
        echo "  not claim to cover this line." >&2
        exit 1
    fi
    echo "=== tier 3: Zephyr 4.4 — building zephyr/{{example}} ({{rmw}}) ==="
    NROS_ZEPHYR_VERSION=4.4 NROS_ZEPHYR_WORKSPACE="$ws_abs" \
        just zephyr build-one "{{example}}" "{{rmw}}"
    echo "=== tier 3: Zephyr 4.4 OK ==="

# CI: doctor + test
[group("full-matrix")]
ci: doctor test

# Run Zephyr tests with full rebuild
[group("debug")]
test-full verbose="": build-examples
    just zephyr test {{verbose}}

# Run Zephyr XRCE E2E tests
[group("debug")]
test-xrce verbose="":
    #!/usr/bin/env bash
    set -e
    source scripts/build/cargo.sh
    cargo_nextest_args=($(nros_cargo_nextest_args))
    args=(-p nros-tests --test zephyr --no-fail-fast -E 'test(xrce)')
    if [ -z "{{verbose}}" ]; then
        args+=(--success-output never --failure-output never)
    else
        args+=(--success-output immediate --failure-output immediate)
    fi
    cargo nextest run "${cargo_nextest_args[@]}" "${args[@]}"

# An nros zenoh-pico publisher ON TARGET reaches a stock `ros2 topic echo`
# subscriber (issue #141) — the only `interop::CELLS` cell built through the
# west leaves rather than native fixtures, so it has TWO build channels and
# skips on either (phase-433 / issue 1127).
#
# Deliberately NOT sequenced after `build-fixtures` the way `just zephyr test`
# is: this is an aimable unit, and running the whole west lane to reach one
# cell is not something anyone would type. Build the fixture, then aim.
#
# Its baked router port is shared with `entry_e2e`'s zephyr_rust_qos cell, so
# nextest serializes the two in `matrix-consumers-serial`.
# The west-built qos entry comes from `just zephyr build-fixtures`.
# Runs `qos_zephyr_ros2_interop_e2e`; needs ROS 2 + `rmw_zenoh_cpp` + that entry.
[group("debug")]
test-ros2-qos verbose="":
    @just _test-focused 'binary(=qos_zephyr_ros2_interop_e2e)' {{verbose}}

# Run Zephyr C examples test
[group("debug")]
test-c:
    ./tests/zephyr/run-c.sh

# Clean Zephyr build directories
[group("main")]
clean:
    #!/usr/bin/env bash
    WORKSPACE="{{ZEPHYR_WORKSPACE}}"
    rm -rf "$WORKSPACE"/build-{talker,listener,service-server,service-client,action-server,action-client,service-client-async,async-service-client,c-talker,c-listener,cpp-talker,cpp-listener,cpp-service-server,cpp-service-client,cpp-action-server,cpp-action-client,xrce-rs-talker,xrce-rs-listener,xrce-c-talker,xrce-c-listener}
    echo "Zephyr build directories cleaned"

# Force rebuild Zephyr examples
rebuild: clean build-examples

# Show Zephyr help
help:
    @echo "Zephyr Examples"
    @echo "==============="
    @echo ""
    @echo "Prerequisites:"
    @echo "  Set up Zephyr workspace: ./scripts/zephyr/setup.sh"
    @echo "  (native_sim uses NSOS; no TAP bridge/sudo required.)"
    @echo ""
    @echo "Build examples:"
    @echo "  just zephyr build          # Build Zephyr core artifacts"
    @echo "  just zephyr build-examples # Build all Zephyr examples"
    @echo "  just zephyr build-rust-examples # Build Rust zenoh Zephyr examples"
    @echo "  just zephyr build-c        # Build C zenoh Zephyr examples"
    @echo "  just zephyr build-cpp      # Build C++ zenoh Zephyr examples"
    @echo "  just zephyr build-xrce     # Build XRCE Zephyr examples (Rust + C)"
    @echo "  just zephyr build-all      # Build Zephyr core, examples, and fixtures"
    @echo "  just zephyr rebuild        # Clean and rebuild"
    @echo "  just zephyr clean          # Remove all build directories"
    @echo ""
    @echo "Run tests:"
    @echo "  just zephyr test           # Run zenoh Zephyr E2E tests"
    @echo "  just zephyr test-xrce      # Run XRCE Zephyr E2E tests"
    @echo "  just zephyr test-full      # Rebuild and run zenoh Zephyr tests"
    @echo "  just zephyr test-c         # Run Zephyr C examples test"
    @echo ""
    @echo "Manual build (from Zephyr workspace):"

# =============================================================================
# Run examples (build + launch native_sim binary)
#
# Each recipe passes --seed=$RANDOM so that multiple native_sim instances
# get different Zenoh session IDs. Without unique seeds, deterministic
# entropy produces identical ZIDs and the router rejects the second
# session with Close(MAX_LINKS). On real SoCs this isn't needed — the
# hardware RNG provides unique entropy.
# =============================================================================

# Run Zephyr talker
talker:
    #!/usr/bin/env bash
    set -e
    NROS="$(basename "$(pwd)")"
    WORKSPACE="{{ZEPHYR_WORKSPACE}}"
    cd "$WORKSPACE"
    ./build-talker/zephyr/zephyr.exe --seed=$RANDOM

# Run Zephyr listener
listener:
    #!/usr/bin/env bash
    set -e
    NROS="$(basename "$(pwd)")"
    WORKSPACE="{{ZEPHYR_WORKSPACE}}"
    cd "$WORKSPACE"
    ./build-listener/zephyr/zephyr.exe --seed=$RANDOM

# Run Zephyr service server
service-server:
    #!/usr/bin/env bash
    set -e
    NROS="$(basename "$(pwd)")"
    WORKSPACE="{{ZEPHYR_WORKSPACE}}"
    cd "$WORKSPACE"
    ./build-service-server/zephyr/zephyr.exe --seed=$RANDOM

# Run Zephyr service client
service-client:
    #!/usr/bin/env bash
    set -e
    NROS="$(basename "$(pwd)")"
    WORKSPACE="{{ZEPHYR_WORKSPACE}}"
    cd "$WORKSPACE"
    ./build-service-client/zephyr/zephyr.exe --seed=$RANDOM

# Run Zephyr action server
action-server:
    #!/usr/bin/env bash
    set -e
    NROS="$(basename "$(pwd)")"
    WORKSPACE="{{ZEPHYR_WORKSPACE}}"
    cd "$WORKSPACE"
    ./build-action-server/zephyr/zephyr.exe --seed=$RANDOM

# Run Zephyr action client
action-client:
    #!/usr/bin/env bash
    set -e
    NROS="$(basename "$(pwd)")"
    WORKSPACE="{{ZEPHYR_WORKSPACE}}"
    cd "$WORKSPACE"
    ./build-action-client/zephyr/zephyr.exe --seed=$RANDOM

# Run Zephyr async service client
async-service-client:
    #!/usr/bin/env bash
    set -e
    NROS="$(basename "$(pwd)")"
    WORKSPACE="{{ZEPHYR_WORKSPACE}}"
    cd "$WORKSPACE"
    ./build-async-service-client/zephyr/zephyr.exe

# Start zenohd router for Zephyr examples (allocator rust-pubsub port 7400, all interfaces)
zenohd:
    #!/usr/bin/env bash
    set -e
    source scripts/dev/zenohd.sh
    nros_router_exec tcp/0.0.0.0:7400
