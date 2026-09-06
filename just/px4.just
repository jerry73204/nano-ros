set working-directory := '..'

default:
    @just --list --justfile {{ source_file() }}

# Initialise PX4 submodules (shallow). Includes:
#   third-party/px4/px4-rs           — Rust framework for PX4 modules
#   third-party/px4/PX4-Autopilot    — PX4 firmware tree
# Phase 203 — PX4-Autopilot's pin lags `main` and isn't advertised, so a plain
# `--depth 1` submodule update can't reach it (empty checkout, no `Makefile`).
# nros ≥ 0.3.7 falls back to a depth-1 fetch-by-SHA, so the TOP tree is a real
# shallow snapshot (Makefile present). PX4's ~50 OWN sub-submodules (cmake,
# mavlink, ecl, …) are pulled shallow below. Total ~500 MB.
# Required for `just px4 build-sitl-cpp` and `just px4 test-sitl`.
[group("setup")]
setup:
    # Phase 197.3 — px4-rs + PX4-Autopilot are index sources ([source.px4-rs],
    # [source.px4-autopilot]); provision the nano-ros submodules via nros (the
    # SSOT for refs) instead of an inline `git submodule update`.
    nros setup --source px4-rs --source px4-autopilot
    # PX4-Autopilot has ~50 of its OWN sub-submodules (cmake, mavlink, ecl, …) —
    # that recursion is PX4's concern, not nano-ros source provisioning.
    git -C third-party/px4/PX4-Autopilot submodule update --init \
        --depth 1 --recommend-shallow --recursive
    # PX4's Python build deps are CHECKED, not installed (issue 0698 follow-up).
    #
    # This used to `pip3 install --user -r Tools/setup/requirements.txt`,
    # mirroring PX4's own `Tools/setup/ubuntu.sh`. That step is written for
    # Ubuntu, where `--user` works; on a PEP 668 host (Arch, Fedora, Debian 12+)
    # pip refuses `--user` outright and setup dies here — and where it does
    # succeed it installs into whichever interpreter `pip3` happens to be, which
    # need not be the one the build later runs. Same reasoning as
    # scripts/zephyr/setup.sh: nano-ros reports, the host owns its Python.
    #
    # One physical line: this recipe has no shebang, so just runs each line in
    # its own shell and an indented block is a parse error.
    python3 scripts/check-python-deps.py px4-build \
        || { echo "  PX4's own full list: third-party/px4/PX4-Autopilot/Tools/setup/requirements.txt" >&2; \
             echo "  Install it however suits this host (pip --user on Ubuntu, else a venv)," >&2; \
             echo "  then re-run \`just px4 setup\` or set NROS_PYTHON to that interpreter." >&2; \
             exit 1; }

# Diagnose PX4 install status (read-only).
[group("setup")]
doctor:
    #!/usr/bin/env bash
    bash scripts/dev/deprecated-scope-alias.sh px4 doctor "just doctor px4"
    set +e
    fail=0
    if [ -d "third-party/px4/px4-rs/crates" ]; then
        echo "  [OK] px4-rs submodule populated"
    else
        echo "  [MISSING] third-party/px4/px4-rs (run: just px4 setup)"
        fail=1
    fi
    if [ -d "third-party/px4/PX4-Autopilot/Tools" ]; then
        echo "  [OK] PX4-Autopilot submodule populated"
    else
        echo "  [MISSING] third-party/px4/PX4-Autopilot (run: just px4 setup)"
        fail=1
    fi
    for tool in cmake ninja gcc python3; do
        if command -v "$tool" >/dev/null 2>&1; then
            echo "  [OK] $tool"
        else
            echo "  [MISSING] $tool — install via your package manager"
            fail=1
        fi
    done
    if command -v arm-none-eabi-gcc >/dev/null 2>&1; then
        echo "  [OK] arm-none-eabi-gcc (target build available)"
    else
        echo "  [INFO] arm-none-eabi-gcc not installed — only SITL host build available"
    fi
    # PX4's uORB topic-header generator + Kconfig step pull in Python
    # modules listed in `Tools/setup/requirements.txt`. The few that
    # are load-bearing for `build-sitl-cpp` (and which silently
    # break the build with "No module named X" if they're missing
    # after a submodule update) get spot-checked here so doctor
    # surfaces the staleness before the user spends 30s waiting for
    # ninja to fail at step [2/1140].
    for mod in genmsg kconfiglib jinja2 em; do
        if python3 -c "import $mod" >/dev/null 2>&1; then
            echo "  [OK] python3: $mod"
        else
            echo "  [MISSING] python3: $mod — run: just px4 setup"
            fail=1
        fi
    done
    if [ -n "${PX4_AUTOPILOT_DIR:-}" ]; then
        echo "  [INFO] PX4_AUTOPILOT_DIR set to $PX4_AUTOPILOT_DIR (overrides submodule)"
    fi
    exit $fail

# Build PX4 SITL with the nano-ros uORB C++ register-check module
# linked in via EXTERNAL_MODULES_LOCATION. This is the Phase 115.K.4.5
# validation gate: it compiles the entire `nros-rmw-uorb-cpp` library
# inline inside a real `px4_add_module()` build context, including the
# `SubscriptionCallbackWorkItem` push-wake glue that requires
# `<px4_boardconfig.h>` (per-board Kconfig output, only exists during
# a SITL build).
#
# Cold build ~10 min; warm <2 s. Requires `just px4 setup`.
[group("examples")]
build-sitl-cpp:
    #!/usr/bin/env bash
    set -e
    PX4_DIR="${PX4_AUTOPILOT_DIR:-$(pwd)/third-party/px4/PX4-Autopilot}"
    EXT_DIR="$(pwd)/packages/testing/nros-px4-register-check"
    if [ ! -d "$PX4_DIR/Tools" ]; then
        echo "ERROR: PX4-Autopilot tree not found at $PX4_DIR"
        echo "Run: just px4 setup"
        exit 1
    fi
    echo "Building PX4 SITL with EXTERNAL_MODULES_LOCATION=$EXT_DIR"
    make -C "$PX4_DIR" px4_sitl_default \
        EXTERNAL_MODULES_LOCATION="$EXT_DIR"

# Build PX4 SITL with the uORB INTEROP EXAMPLE as EXTERNAL_MODULES_LOCATION
# (phase-325 W2).
#
# Separate from `build-sitl-cpp` because PX4 takes exactly ONE
# EXTERNAL_MODULES_LOCATION per build, and the register-check gate
# (packages/testing/nros-px4-register-check) and this example are different roots.
# Each is ~10 min cold, <2 s warm.
#
# Prereqs the helper checks for at configure time, with the command to fix each:
#   cargo build -p nros-cpp --no-default-features --features std,rmw-cffi --release
#   cmake -S packages/platform/nros-platform-posix -B build/nros-platform-posix
#   cmake --build build/nros-platform-posix
[group("examples")]
build-sitl-example:
    #!/usr/bin/env bash
    set -e
    PX4_DIR="${PX4_AUTOPILOT_DIR:-$(pwd)/third-party/px4/PX4-Autopilot}"
    EXT_DIR="$(pwd)/examples/px4/cpp/firmware"
    if [ ! -d "$PX4_DIR/Tools" ]; then
        echo "ERROR: PX4-Autopilot tree not found at $PX4_DIR"
        echo "Run: just px4 setup"
        exit 1
    fi
    # Issue 1050 -- build the archive this links, instead of inheriting one.
    #
    # This recipe used to be `make` alone, linking whatever `cargo build -p
    # nros-cpp` last left in `target/release/`. That is not a tidiness point:
    # the demo declares `BACKENDS uorb`, but a zenoh-enabled archive ALSO
    # registers zenoh through the `.init_array` ctor that survives on hosted
    # Rust backends -- and on POSIX that ctor runs BEFORE main, hence before the
    # generated `nros_app_register_backends()`. zenoh takes slot 0, `nros::init()`
    # opens the default slot, dials tcp/127.0.0.1:7447, finds nothing, and the
    # module dies with `nros::init() failed` -> PX4_ERROR -> status 255.
    #
    # A uORB-only module, broken by a backend it never declared.
    #
    # The coupling is circular too: the generated header the module compiles
    # against comes from the same ambient archive, so building the BRIDGE and
    # then this produced `undefined reference to
    # nros_cpp_config_variant_..._rmw_zenoh_cffi_...` -- the module compiled for
    # an archive it was not linked against.
    #
    # `rmw-cffi` and NOT `rmw-zenoh-cffi`: this module's outward side IS uORB.
    # `build-bridge-example` does the same thing for the opposite reason (it
    # picks a NETWORKED backend), which is why that recipe never had this bug.
    echo "Building the nano-ros archive this module links (uORB only)"
    # profile-literal-ok: symbol fixture: the PX4 link and this build must name
    # the same profile dir, and PX4 knows nothing about nano-ros profiles.
    cargo build -p nros-cpp --no-default-features \
        --features std,rmw-cffi,platform-posix --release
    # nros-cmake-prefix-exempt: the POSIX platform's C archive -- no Rust target
    # in that project, so no Corrosion to resolve.
    cmake -S "$(pwd)/packages/platform/nros-platform-posix" \
        -B "$(pwd)/build/nros-platform-posix" >/dev/null
    cmake --build "$(pwd)/build/nros-platform-posix" >/dev/null

    echo "Building PX4 SITL with EXTERNAL_MODULES_LOCATION=$EXT_DIR"
    make -C "$PX4_DIR" px4_sitl_default EXTERNAL_MODULES_LOCATION="$EXT_DIR"

# Build PX4 SITL with the uORB->RMW BRIDGE as EXTERNAL_MODULES_LOCATION
# (phase-325 W3, issue 0362).
#
# Its own root because PX4 takes exactly ONE EXTERNAL_MODULES_LOCATION per build.
# Three steps the other px4 recipes don't need, all consequences of publishing on
# a wire rather than into uORB:
#
#   1. generate the CDR px4_msgs C++ headers (their RIHS01 hash is what makes a
#      ROS 2 peer match the topic — a guessed one is invisible, or worse, wrong);
#   2. build the FFI staticlib holding those headers' Rust bodies (a normal CMake
#      consumer gets this from nros_generate_interfaces(LANGUAGE CPP), which a PX4
#      module never runs);
#   3. the outward backend is chosen when the ARCHIVE is built, so libnros_cpp.a
#      must carry `rmw-zenoh-cffi`, and zenoh needs its own platform archive.
#
# TOPICS is the single source of truth for the message set: it drives the
# generator, and the FFI crate's build.rs globs whatever the generator wrote.
[group("examples")]
build-bridge-example topics="debug_key_value" edition="jazzy":
    #!/usr/bin/env bash
    set -e
    ROOT="$(pwd)"
    PX4_DIR="${PX4_AUTOPILOT_DIR:-$ROOT/third-party/px4/PX4-Autopilot}"
    EXT_DIR="$ROOT/examples/px4/cpp/bridge"
    # phase-325 W3.3 — the OUTWARD backend, from the ENVIRONMENT and not from a
    # recipe parameter. `just` has no named-argument syntax: `just px4
    # build-bridge-example rmw=zenoh` binds the string "rmw=zenoh" to the FIRST
    # positional (`topics`), and the run dies with
    #     Error: px4 message `rmw=zenoh` not found under .../msg
    # which names the message and not the mistake. Measured, not assumed -- and
    # the same flaw made this file's own README example
    # (`build-bridge-example topics=vehicle_status`) generate a message called
    # "topics=vehicle_status" for as long as it has existed.
    #
    # The repo is INCONSISTENT here and it is worth knowing which way: archived
    # phase-410 states the rule plainly ("`just ci matrix depth=build` does NOT
    # work. `just` does not parse `name=value`"), while `lane=` DOES work --
    # because `scripts/build/fixture-lane.sh:74` strips the prefix by hand
    # (`lane="${lane#lane=}"`). So `name=value` is an accommodation each recipe
    # makes or does not; this one does not, and says so rather than adding a
    # second idiom nobody can predict from the outside.
    RMW="${NROS_PX4_BRIDGE_RMW:-zenoh}"
    case "$RMW" in
        zenoh|xrce|cyclonedds) ;;
        *) echo "ERROR: NROS_PX4_BRIDGE_RMW='$RMW' is not a networked backend." >&2
           echo "       Use one of: zenoh xrce cyclonedds" >&2
           exit 1 ;;
    esac
    GEN="$ROOT/build/px4-bridge/generated"
    if [ ! -d "$PX4_DIR/Tools" ]; then
        echo "ERROR: PX4-Autopilot tree not found at $PX4_DIR"
        echo "Run: just px4 setup"
        exit 1
    fi

    echo "[1/4] generating CDR px4_msgs C++ ({{topics}})"
    nros generate-px4-msgs --lang cpp --topics '{{topics}}' \
        --ros-edition '{{edition}}' -o "$GEN"

    echo "[2/4] building the generated-message FFI staticlib"
    # `--target-dir` is not cosmetic: without it cargo writes `ffi/target/`, and
    # a bare `target/` under `examples/**` is exactly what
    # `check-example-leaf-target-dirs` fails on (phase-340 P2 residue, issue
    # 0200). That gate is BUILDLESS, so it never saw this — the dir only exists
    # after someone runs this recipe, and then `just check fast` goes red in a
    # tree that merely built what the docs told it to.
    #
    # NOTE the ordering below: `check-build-profile-literals` looks only 3 LINES
    # UP from the `cargo` line for its marker, and the `\` continuation means the
    # marker cannot be appended inline either. Writing the paragraph above
    # BETWEEN the marker and the command pushed it out of range and turned the
    # lane red. Explanations go above the marker, never between it and the call.
    # profile-literal-ok: symbol fixture: the archive PATH is handed to PX4's
    # make via NROS_PX4_BRIDGE_FFI_ARCHIVE, and PX4 knows nothing about nano-ros
    # profiles — build and path must name the same dir, so both stay pinned.
    NROS_PX4_BRIDGE_GEN="$GEN" cargo build --release \
        --manifest-path "$EXT_DIR/ffi/Cargo.toml" \
        --target-dir "$EXT_DIR/ffi/target-bridge-ffi"

    echo "[3/4] building the nano-ros archives (outward backend $RMW baked in here)"
    # `bridge` is what puts nros_init_multi / nros_pubsub_bridge_* (the ABI
    # nros::MultiExecutor calls) into libnros_cpp.a — issue 0436. Without it the
    # module compiles and fails to LINK.
    # `platform-posix` makes the umbrella's bundled zenoh-pico COMPLETE (core +
    # platform layer, 1576 symbols). Without it the link needed a SECOND
    # zenoh-pico archive, and two copies in one image is what broke the session
    # open — issue 0436. One archive, one zenoh-pico.
    # Linked into the same PX4 SITL image as the archive above; one profile for
    # the whole link. Marker is INLINE because check::build-profile-literals.sh
    # only looks 3 lines up, and this comment block is longer than that.
    # phase-325 W3.3 — `rmw=` picks the OUTWARD backend, and it has to reach two
    # places that cannot see each other: this cargo feature (which compiles the
    # backend INTO libnros_cpp.a) and the module's cmake (which registers it and
    # names it in the session spec). One value, exported below, drives both.
    # The archive-vs-BACKENDS half is guarded (an llvm-nm precheck in
    # nros_px4_add_module names the exact cargo command). The
    # BACKENDS-vs-session-NAME half is not: asking for a backend by a name
    # nothing registered opens nothing and reports a transport failure, which
    # reads as a dead network. One value removes that.
    cargo build -p nros-cpp --no-default-features \
        --features "std,rmw-$RMW-cffi,bridge,platform-posix" --release # profile-literal-ok: symbol fixture
    # profile-literal-ok: symbol fixture: same PX4 SITL link as above.
    # The POSIX platform archive nros_px4_add_module links (its guard names this
    # exact command when it is missing).
    # nros-cmake-prefix-exempt: the POSIX platform's C archive — no Rust target
    # in that project, so no Corrosion to resolve.
    cmake -S "$ROOT/packages/platform/nros-platform-posix" -B "$ROOT/build/nros-platform-posix" >/dev/null
    cmake --build "$ROOT/build/nros-platform-posix" >/dev/null

    echo "[4/4] building PX4 SITL with EXTERNAL_MODULES_LOCATION=$EXT_DIR"
    # profile-literal-ok: symbol fixture: pairs with the --release build above.
    NROS_PX4_BRIDGE_GEN="$GEN" \
    NROS_PX4_BRIDGE_RMW="$RMW" \
    NROS_PX4_BRIDGE_FFI_ARCHIVE="$EXT_DIR/ffi/target-bridge-ffi/release/libnros_px4_bridge_ffi.a" \
        make -C "$PX4_DIR" px4_sitl_default EXTERNAL_MODULES_LOCATION="$EXT_DIR"

# Build PX4 examples. (phase-277 W7: the former `build-sitl` recipe pointed
# EXTERNAL_MODULES_LOCATION at the README-only `examples/px4/rust/uorb/`
# placeholder — it compiled no nano-ros module and the placeholder dir is
# retired; `build-sitl-cpp` is the real EXTERNAL_MODULES_LOCATION build.)
[group("main")]
build-examples: build-sitl-cpp
    @echo "PX4 examples built."

# Build PX4 test fixtures.
#
# Phase 233.4 — the XRCE companion host test (`nros-tests::px4_xrce`) runs the
# prebuilt `px4-stub` example. Generate its `px4_msgs` bindings from the PX4
# `.msg` tree, run `nros sync` for the patch config, then build the standalone
# example to `target-xrce/` where `build_example_rmw` resolves it. Companion gets
# the same bindings so a `just px4 build-examples` copy-out builds too.
#
# issue 0510 — this used to say "no `nros sync` path — px4_msgs isn't an ament
# package" and skip sync entirely. True of the CODEGEN half and false of the
# other one: sync also writes the leaf's `[patch.crates-io]`, and these three
# manifests registry-name their nros deps (`nros = { version = "*" }`,
# `nros-platform-cffi`, `nros-rmw-xrce-cffi`). With no `.cargo/config.toml` at
# all, cargo resolved them against the PUBLIC crates.io and the lane died with
#
#     error: no matching package named `nros` found
#     location searched: crates.io index
#
# which reads as a missing crate rather than a missing sync. That failure takes
# `build-test-fixtures lane=all` down with it (rc=2), so no tier needing the full
# existence set can complete. Same class as #378.
[group("full-matrix")]
build-fixtures:
    #!/usr/bin/env bash
    bash scripts/dev/deprecated-scope-alias.sh px4 build-fixtures "just build px4"
    set -e
    source scripts/build/cargo.sh
    PX4_DIR="${PX4_AUTOPILOT_DIR:-$(pwd)/third-party/px4/PX4-Autopilot}"
    source scripts/build/lane-skip.sh   # issue 0599 — SKIPPED, not OK
    nros_lane_platform px4                 # phase-411 W2 — named `px4` must work
    if [ ! -d "$PX4_DIR/msg" ]; then
        nros_lane_skip "PX4-Autopilot .msg tree not found at $PX4_DIR/msg — run: just px4 setup"
    fi
    nros_cli="$(nros_cli_bin)"
    if ! "$nros_cli" generate-px4-msgs --help >/dev/null 2>&1; then
        nros_lane_skip "'$nros_cli' lacks 'generate-px4-msgs' — rebuild the in-tree CLI (just setup-cli)"
    fi
    cargo_profile_args="$(nros_cargo_profile_arg_string)"
    for ex in px4-stub offboard-companion px4-probe; do
        dir="examples/px4/rust/companion/$ex"
        echo "  → generate px4_msgs for $dir"
        "$nros_cli" generate-px4-msgs --px4 "$PX4_DIR" --output "$dir/generated" >/dev/null
        # issue 0510 — the patch config, without which the registry-named nros
        # deps resolve against crates.io. Gitignored, so a fresh clone has none.
        echo "  → nros sync $dir"
        "$nros_cli" sync "$dir" >/dev/null
    done
    # phase-336 W7 — the three companion examples build into their own
    # `target-xrce/` dirs, so they are independent; run them under the jobserver
    # instead of back to back.
    source scripts/build/jobserver-pool.sh
    for ex in px4-stub offboard-companion px4-probe; do
        printf '%s\n' "echo '  → cargo build $ex (rmw-xrce, target-xrce/)' && cd examples/px4/rust/companion/$ex && cargo build $cargo_profile_args --no-default-features --features rmw-xrce --target-dir target-xrce"
    done | nros_pool_run px4-companion
    echo "PX4 XRCE fixtures built."

# Run the Track A uORB interop E2E: a STOCK PX4 `listener` reads a nano-ros
# publication (phase-325 W2.4). Rebuilds SITL against the example first, because
# the register-check gate and this example cannot share one build dir.
[group("debug")]
test-sitl-example: build-sitl-example
    #!/usr/bin/env bash
    set -e
    source scripts/build/cargo.sh
    cargo_nextest_args=($(nros_cargo_nextest_args))
    cd packages/testing/nros-px4-sitl-test
    cargo nextest run "${cargo_nextest_args[@]}" --test px4_uorb_interop_e2e

# Build every PX4 tier: example integrations and fixtures.
#
# phase-321 W1.c — `build` and `build-targets` are gone from this list because
# they built `nros-rmw-uorb` and `nros-px4` as CARGO crates, and both were
# deleted in phase-115.K.4. The surviving uORB backend is the C++ CMake project
# at `packages/rmw/uorb/nros-rmw-uorb`, which `build-examples` covers.
[group("full-matrix")]
build-all: build-examples build-fixtures
    @echo "PX4 build-all completed."

# Run E2E tests against PX4 SITL. Builds SITL + the XRCE fixtures first.
#
#   px4_xrce_e2e  — Track B (real PX4 uxrce_dds_client → agent → nano-ros
#                   companion receives /fmu/out telemetry over nros-rmw-xrce)
#
# Track A (in-firmware uORB) is `px4_uorb_interop_e2e`, run by `test-sitl-example`
# rather than here. It needs a DIFFERENT EXTERNAL_MODULES_LOCATION than the
# register-check gate, PX4 takes one per build, and both write to the same
# build/px4_sitl_default — so the two cannot share a SITL tree. Keeping them in
# separate recipes makes the rebuild explicit instead of silently testing
# whichever root was built last.
[group("debug")]
test-sitl: build-sitl-cpp build-fixtures
    #!/usr/bin/env bash
    # Phase 208.D.3 — these SITL tests live in the out-of-workspace
    # `nros-px4-sitl-test` crate so the root workspace doesn't try to
    # resolve `px4-rs` on a fresh clone. Run them from their own dir.
    set -e
    source scripts/build/cargo.sh
    cargo_nextest_args=($(nros_cargo_nextest_args))
    cd packages/testing/nros-px4-sitl-test
    cargo nextest run "${cargo_nextest_args[@]}" --test px4_xrce_e2e

# Combined CI: build the C++ uORB integration (SITL gated separately via
# test-sitl).
#
# phase-321 W1.c — this was `ci: check test build-targets`, all three of which
# built the CARGO crates `nros-rmw-uorb` / `nros-px4` that phase-115.K.4
# deleted. Only archived phase docs referenced this recipe, so it had been
# unrunnable and unnoticed. What survives is the C++ CMake project, which
# `build-examples` compiles inside a real `px4_add_module()` context.
[group("full-matrix")]
ci: build-examples

# Clean PX4 example build artifacts without touching downloaded submodules.
[group("main")]
clean:
    #!/usr/bin/env bash
    set -e
    PX4_DIR="${PX4_AUTOPILOT_DIR:-$(pwd)/third-party/px4/PX4-Autopilot}"
    # (phase-321 W1.c — the old `rm -rf packages/rmw/nros-rmw-uorb/target
    # packages/platforms/nros-px4/target` line is gone: BOTH cargo crates were
    # deleted in phase-115.K.4, and neither path existed when the line was
    # written. Quoted verbatim on purpose — it is a record of a dead line, so it
    # must NOT be rewritten by path sweeps. `packages/rmw/` does exist now, but
    # only since phase-321 W2.d, and it never held `nros-rmw-uorb` as a crate.)
    if [ -d examples/px4 ]; then
        find examples/px4 -type d -name target -exec rm -rf {} + 2>/dev/null || true
    fi
    if [ -d "$PX4_DIR/build" ]; then
        rm -rf "$PX4_DIR"/build/px4_sitl_default "$PX4_DIR"/build/px4_fmu-v6x_default
    fi
    echo "PX4 build artifacts cleaned"
