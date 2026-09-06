set working-directory := '..'

default:
    @just --list --justfile {{source_file()}}

# CLIPPY_LINTS removed — workspace-wide lint config now lives in root Cargo.toml `[workspace.lints]`

LOG_DIR := "test-logs"
NIGHTLY := `awk '/^channel/ {gsub(/"/, "", $3); print $3; exit}' tools/rust-toolchain.toml`

# =============================================================================
# Rust examples (auto-discovered from examples/{platform}/{lang}/{rmw}/{use-case}/)
# =============================================================================

# Build native core artifacts.
[group("main")]
build:
    @echo "Native example module has no separate core build; use 'just native build-examples' for examples."

# Build all Rust examples (auto-discovered, excludes zephyr + cross-compiled RTOS).
#
# Parallelised via a temporary Make graph — examples are independent standalone
# crates (per the copy-out invariant in AGENTS.md), so each cargo invocation can
# run concurrently without requiring GNU parallel. JOBS defaults to the global
# NROS_BUILD_JOBS budget (itself nproc unless overridden), falling back to 75 %
# of cores. Override per-invocation with `just native build-examples 4` or
# `NROS_BUILD_JOBS=4 just native build-examples` for tighter machines. Just
# recipe parameters are positional; `JOBS=4` is parsed as a literal argument.
[group("main")]
build-examples JOBS=env_var_or_default("NROS_BUILD_JOBS", "75%"):
    #!/usr/bin/env bash
    set -e
    source scripts/build/cargo.sh
    cargo_profile_args="$(nros_cargo_profile_arg_string)"
    export cargo_profile_args
    nros_make_jobs() {
        local jobs="$1"
        case "$jobs" in
            *%)
                local pct="${jobs%\%}"
                if ! [[ "$pct" =~ ^[0-9]+$ ]] || [ "$pct" -lt 1 ]; then
                    echo "invalid JOBS=$jobs; expected positive integer or percent" >&2
                    return 2
                fi
                local cores
                cores="$(nproc 2>/dev/null || echo 1)"
                local computed=$(( cores * pct / 100 ))
                [ "$computed" -ge 1 ] || computed=1
                printf '%s\n' "$computed"
                ;;
            *)
                if ! [[ "$jobs" =~ ^[0-9]+$ ]] || [ "$jobs" -lt 1 ]; then
                    echo "invalid JOBS=$jobs; expected positive integer or percent" >&2
                    return 2
                fi
                printf '%s\n' "$jobs"
                ;;
        esac
    }
    if [ "${NROS_JOBSERVER:-}" = "1" ] || [ "${NROS_INHERIT_JOBSERVER:-}" = "1" ]; then
        cargo_frontends="$(nros_cargo_frontend_jobs)"
    else
        cargo_frontends="$(nros_make_jobs "{{JOBS}}")"
    fi
    echo "Building examples (parallel, cargo-frontends=$cargo_frontends, profile=$(nros_cargo_profile_name))..."
    toml_list="$(mktemp "${TMPDIR:-/tmp}/nros_example_toml.XXXXXX")"
    makefile="$(mktemp "${TMPDIR:-/tmp}/nros_example_build.XXXXXX.mk")"
    trap 'rm -f "$toml_list" "$makefile"' EXIT

    # Phase 181.6 / 181.4.h — qemu-arm-baremetal, esp32 and
    # qemu-esp32-baremetal are excluded here and built from the SSOT manifest
    # below instead, so each has a single enumeration source (matching the
    # test-all staleness probe). The remaining platforms have no manifest rows
    # for this broad find, or are built by their own platform recipes
    # (freertos/nuttx/threadx/zephyr/px4).
    # Enumerate via the git index (tracked files only) — no filesystem
    # traversal, so untracked build/target trees can never slow this down or
    # leak in. `NF>=5` = the old find's `-mindepth 4` (leaf crates, not
    # workspace-root manifests). Selection filters stay as greps.
    git ls-files 'examples/**/Cargo.toml' | awk -F/ 'NF>=5' \
        | grep -vE '/zephyr/|/workspaces/|/qemu-arm-freertos/|/qemu-arm-nuttx/|/threadx-linux/|/qemu-riscv64-threadx/|/qemu-arm-baremetal/|/esp32/|/qemu-esp32-baremetal/|/px4/' \
        | sort > "$toml_list"

    # The ESP platforms (which need the nightly toolchain + SSID/PASSWORD env)
    # are excluded from this find and built from the manifest below, so this
    # loop only sees host examples.
    #
    # phase-340 P2 — but NOT a bare `cargo build`. `linux` is a migrated
    # platform (phase-340 B3), so every default-config `linux` fixture row
    # already builds into `build/cargo-fixtures/linux`; a bare in-leaf build
    # here wrote a SECOND, per-leaf copy of the same artifacts and re-created
    # the `examples/**/target/` dirs R1 exists to remove. The dir comes from the
    # same resolver the manifest build and the staleness probe call — no literal
    # here — so the 22 dirs that DO have a row land exactly where their row
    # lands (this sweep becomes an incremental no-op for them) and the five
    # `examples/templates/**` crates that have no row join the same group. Bin
    # names across this sweep were checked collision-free before it moved.
    #
    # Empty for an unmigrated platform, in which case the leaf `target/` is
    # still correct — the resolver, not this recipe, decides.
    # shellcheck source=scripts/build/fixtures-target-dir.sh
    source scripts/build/fixtures-target-dir.sh
    nros_example_tdir_flag="$(nros_fixture_target_dir_flag linux "" "")"
    export nros_example_tdir_flag
    build_one() {
        local toml="$1"
        local dir="$(dirname "$toml")"
        echo "  build $dir"
        ( cd "$dir" && cargo build $cargo_profile_args $nros_example_tdir_flag )
    }
    export -f build_one

    if [ "${NROS_JOBSERVER:-}" = "1" ] || [ "${NROS_INHERIT_JOBSERVER:-}" = "1" ]; then
        echo "NROS_JOBSERVER=1 — serial launcher; cargo inherits fifo tokens."
        while read -r toml; do build_one "$toml"; done < "$toml_list"
    else
        {
            printf 'SHELL := /bin/bash\n'
            printf '.SHELLFLAGS := -eu -o pipefail -c\n'
            printf '.DELETE_ON_ERROR:\n'
            printf '.PHONY: all'
            idx=0
            while read -r toml; do
                printf ' native-example-%04d' "$idx"
                idx=$((idx + 1))
            done < "$toml_list"
            printf '\n\nall:'
            idx=0
            while read -r toml; do
                printf ' native-example-%04d' "$idx"
                idx=$((idx + 1))
            done < "$toml_list"
            printf '\n\n'
            idx=0
            while read -r toml; do
                printf 'native-example-%04d:\n' "$idx"
                printf '\t+@bash -c '"'"'build_one "$$1"'"'"' _ %q\n\n' "$toml"
                idx=$((idx + 1))
            done < "$toml_list"
        } > "$makefile"
        # issue 0762 — one process group; passthrough when nested under
        # `just build-test-fixtures`, its own group when run directly.
        source scripts/build/subtree-guard.sh
        nros_guard_exec native-examples make -j "$cargo_frontends" -f "$makefile"
    fi
    # Phase 181.6 — qemu-arm-baremetal rust examples from the SSOT manifest
    # (plain default builds; each example's .cargo/config.toml selects the cross
    # target). No codegen prep needed (no generated-msg consumers).
    bash scripts/build/fixtures-build.sh qemu-arm-baremetal rust
    # Phase 181.4.h — esp32 + qemu-esp32-baremetal (ESP32-C3, riscv32imc) from
    # the manifest. They need the workspace nightly + rust-src/build-std (NOT
    # xtensa/espup) + SSID/PASSWORD; inject those as platform env.
    RUSTUP_TOOLCHAIN="{{NIGHTLY}}" SSID="${SSID:-test}" PASSWORD="${PASSWORD:-test}" \
        bash scripts/build/fixtures-build.sh esp32 rust
    RUSTUP_TOOLCHAIN="{{NIGHTLY}}" SSID="${SSID:-test}" PASSWORD="${PASSWORD:-test}" \
        bash scripts/build/fixtures-build.sh qemu-esp32-baremetal rust
    echo "All examples built!"

# Build all example binaries used as test fixtures (Rust + C + C++ + workspaces).
[group("full-matrix")]
build-fixtures: build-fixture-rust build-fixture-extras build-workspace-fixtures
    @bash scripts/dev/deprecated-scope-alias.sh native build-fixtures "just build native"
    @echo "Native test fixtures built."

# Provision what the NATIVE lane needs — phase-413 W5.
#
# `just setup native` expands to the modules of the `native` preset, which is
# `[native]` — and this module had NO `setup` recipe, so the verb ran the common
# preamble (submodule init, CLI, launch resolver) and then printed "native has no
# `setup` recipe — nothing to provision". Meanwhile `host-tests.yml` provisioned
# the native lane in five hand-rolled steps, because the verb genuinely did not
# do it. That is the gap issue 0996 measured as "provisioning verbs `just setup`
# cannot reach".
#
# SOURCES COME FROM THE INDEX, NOT A LIST HERE. `--build-sources` resolves the
# top-level `build_sources` union — which is byte for byte the seven sources the
# workflow was naming by hand (zenoh-pico, mbedtls, cyclonedds-src,
# micro-xrce-dds-client, micro-cdr, px4-rs, nuttx-libc). Restating it here would
# reintroduce the second copy W3 just deleted one level down.
#
# `--prefix build/qemu` is not a CI detail: `nros setup --help` documents it as
# where the test harness already looks for the emulator.
[group("setup")]
setup:
    #!/usr/bin/env bash
    set -euo pipefail
    nros setup --build-sources
    just ci provision-zenohd
    nros setup --tool qemu --prefix build/qemu --index nros-sdk-index.toml
    nros setup --tool xrce-agent --index nros-sdk-index.toml
    # OPTIONAL, and it says so. The workspace-entry fixtures skip without it;
    # every other native fixture builds. `|| echo` is the "continue past a
    # known-optional command" form — a decision that is visible in the log,
    # not a fallback between two verbs.
    nros setup --tool play_launch_parser --index nros-sdk-index.toml \
        || echo "native setup: play_launch_parser unavailable — workspace-entry fixtures will SKIP"
    echo "native lane provisioned."

# Build native workspace fixtures from examples/fixtures.toml.
#
# CODEGEN FIRST — phase-413 W2, and it is issue 0992 in a second place.
#
# A workspace entry reaches its RMW, its ROS edition and its CAPABILITY features
# only through the `generated/nros-selection/<entry>` facade that `nros sync`
# writes. Without it `nros build` emitted the Entry with none of them — and only
# WARNED, because an unsynced workspace was a state it was documented to
# tolerate:
#
#   nros build: warning: no selection facade at …/generated/nros-selection/… —
#   building `native_rust_qos` without its RMW, ROS edition and capability
#   features. Run `nros sync` first if this Entry needs them (issue 0937).
#
# That warning is GONE (issue 0831): a missing facade is now written from the
# entry `nros build` just generated, and refused only if it cannot be. Keep the
# codegen step anyway — sync is also what produces the generated message crates,
# and healing per image is a repair, not the intended order.
#
# `host-tests` called this recipe directly and ran NO codegen step anywhere in
# the job, so the facades never existed there. The features workspace declares
# `features = ["param_services", "lifecycle"]`, so every entry compiled without
# them and the build died in const eval, hundreds of lines away, naming neither
# sync nor the facade:
#
#   error[E0080]: evaluation panicked: this system declares `[param_services]`
#   but this `nros` build does not carry the `param-services` feature
#
# 0992 put codegen in `just build`; this recipe is reachable WITHOUT it, which
# is the same gap one call site over. `_codegen` is idempotent and ~9 s warm, so
# paying it here costs little and removes the whole class for every caller
# rather than for the one workflow that tripped over it.
[group("examples")]
build-workspace-fixtures:
    @just _codegen
    bash scripts/build/workspace-fixtures-build.sh linux

# Build every native Rust test fixture from the SSOT manifest
# (examples/fixtures.toml). Per-fixture build options — features, target-dir,
# env, per-RMW variants — live in the manifest; this recipe owns only the
# codegen prep and the build loop, so the build and the test-all staleness
# probe (scripts/test/rust-fixture-stale.sh) consume identical options.
# Phase 181.3.
[group("examples")]
build-fixture-rust:
    #!/usr/bin/env bash
    set -e
    source scripts/build/cargo.sh
    # issue 1038 — the central `nros-patch.toml` this lane's Rust leaves
    # `include`, made EXPLICIT. This lane already got the file, but only as a
    # side effect of `workspace-fixtures-build.sh` running `nros sync` somewhere
    # earlier; `threadx-riscv64` had no such neighbour and died at manifest
    # parse. An inherited artifact is a working lane, not a declared dependency.
    # Idempotent — a process spawn when the file is already current.
    nros_ensure_central_patch
    # Phase 214.J.2 — codegen-stamp guard for trait-surface drift.
    source scripts/build/codegen-stamp.sh
    nros_cli="$(nros_cli_bin)"
    # Phase 214.I.2 — fail-loud prereq guard: the loop below invokes
    # `nros sync`, absent from the shipped 0.3.7 release. Probe once;
    # emit `[PREREQ]` + exit 0 if missing rather than cascading into
    # clap "unrecognized subcommand 'ws'" stack traces.
    nros_require_sync "$nros_cli"
    # Phase 210.E.3.d.native — pre-cargo codegen via `nros sync` writes
    # the auto-managed [patch.crates-io] block into each example's Cargo.toml
    # + materialises generated msg crates under `<ex>/generated/`.
    # Replaces the legacy `nros generate-rust --force` + hand-managed
    # `.cargo/config.toml [patch.crates-io]` chunks.
    nros_repo_root="$PWD"
    for ex in talker listener service-server service-client service-client-callback action-server \
              action-client custom-transport-talker custom-transport-listener; do
        dir="examples/native/rust/$ex"
        NROS_REPO_DIR="$nros_repo_root" nros_codegen_stamp_check_or_wipe "$dir"
        NROS_REPO_DIR="$nros_repo_root" "$nros_cli" sync "$dir" >/dev/null
        NROS_REPO_DIR="$nros_repo_root" nros_codegen_stamp_write "$dir"
    done
    bash scripts/build/fixtures-build.sh linux rust
    echo "Native rust fixtures built (from examples/fixtures.toml)."

# Issue #29 — like `build-fixture-rust` but `--core-only`: builds just the
# default-config per-example fixtures (no isolated-`target_dir` RMW/feature
# variants). The variant cells (TLS / safety / zero-copy / zenoh / xrce /
# large-buf) each rebuild the full dep graph into their own target dir, and
# that duplication overruns the host-integration runner disk. Those variants
# are covered by other lanes; the host-integration lane uses this recipe and
# lets the variant-spawning tests `skip!` (NROS_FIXTURES_OPTIONAL). The codegen
# prep below covers the default example dirs (the core set).
[group("examples")]
build-fixture-rust-core:
    #!/usr/bin/env bash
    set -e
    source scripts/build/cargo.sh
    source scripts/build/codegen-stamp.sh
    nros_cli="$(nros_cli_bin)"
    nros_require_sync "$nros_cli"
    nros_repo_root="$PWD"
    for ex in talker listener service-server service-client service-client-callback action-server \
              action-client custom-transport-talker custom-transport-listener; do
        dir="examples/native/rust/$ex"
        NROS_REPO_DIR="$nros_repo_root" nros_codegen_stamp_check_or_wipe "$dir"
        NROS_REPO_DIR="$nros_repo_root" "$nros_cli" sync "$dir" >/dev/null
        NROS_REPO_DIR="$nros_repo_root" nros_codegen_stamp_write "$dir"
    done
    bash scripts/build/fixtures-build.sh linux rust --core-only
    echo "Native rust CORE fixtures built (default-config rows only)."

# Internal Phase 178.B target: only fixture artifacts not already covered by
# `build-fixture-role-examples`.
[group("examples")]
build-fixture-extras:
    #!/usr/bin/env bash
    set -e
    source scripts/build/cargo.sh
    # Phase 214.J.2 — codegen-stamp guard for trait-surface drift.
    source scripts/build/codegen-stamp.sh
    source scripts/build/cmake-incremental.sh
    # issue 0805 — `nros_build_dir` / `NROS_KIND_*` for the shared Corrosion
    # cargo dir. cargo.sh sources build-root.sh only from inside one of its
    # functions, so the names are NOT in scope here; without this the
    # `-DNROS_SHARED_CARGO_ROOT=` below expands EMPTY and the cmake side
    # silently degrades to per-leaf dirs — the fix reading as applied while
    # doing nothing. Verified in scope, not assumed.
    # shellcheck source=scripts/build/build-root.sh
    source scripts/build/build-root.sh
    repo_root="$PWD"
    echo "Building native C/C++ + Cyclone test fixtures..."
    # Phase 176 — cmake parallelism via the fifo jobserver when present.
    if [ "${NROS_JOBSERVER:-}" = "1" ] || [ "${NROS_INHERIT_JOBSERVER:-}" = "1" ]; then unset CMAKE_BUILD_PARALLEL_LEVEL; else export CMAKE_BUILD_PARALLEL_LEVEL="${NROS_BUILD_JOBS:-8}"; fi
    nros_cli="$(nros_cli_bin)"
    # Phase 214.I.2 — fail-loud prereq guard: `ensure_native_rust_generated`
    # below invokes `nros sync`. Probe once; emit `[PREREQ]` + exit 0
    # if the verb is missing (shipped 0.3.7 predates it).
    nros_require_sync "$nros_cli"
    nros_cargo_ensure_codegen_c
    codegen_tool="$(nros_cargo_codegen_c_bin)"
    cargo_profile_args="$(nros_cargo_profile_arg_string)"
    # Phase 181.6 — every native *Rust* fixture (the link-tls /
    # unstable-zenoh-api variants; the param / safety / header chatter bins
    # under packages/testing/nros-tests/bins/; the per-RMW target-<rmw>/
    # collapsed roles; the custom-transport pair; the nros-bench
    # binaries; the large-buf stress build) is now a row in
    # examples/fixtures.toml and built by `build-fixture-rust`. This recipe owns
    # only what the manifest can't express: the gated CMake/Corrosion Cyclone
    # cells (no pure-cargo register symbol) and the C/C++ manifest passes.
    ensure_native_rust_generated() {
        case "$1" in
            native/rust/talker|native/rust/listener)
                # Phase 210.E.3.d.native — same swap as build-fixture-role-
                # examples: `nros sync` for the patch-table + codegen flow.
                # Phase 214.J.2 — also wipe `generated/` if trait surface
                # has drifted since the previous sync.
                NROS_REPO_DIR="$repo_root" nros_codegen_stamp_check_or_wipe "examples/$1"
                NROS_REPO_DIR="$repo_root" "$nros_cli" sync "examples/$1" >/dev/null
                NROS_REPO_DIR="$repo_root" nros_codegen_stamp_write "examples/$1" ;;
        esac
    }
    # Phase 220.C (Path B) — native talker/listener Cyclone cells now build via
    # pure-cargo, matching K.7.7 (`fcbc498cc`). The Phase 175.A cmake/corrosion
    # bridge was retired by Phase 212.M.1 (deleted the per-example
    # CMakeLists.txt) once `cargo build --features rmw-cyclonedds` could link
    # `nros_rmw_cyclonedds_register` through `nros-rmw-cyclonedds-sys`. Each
    # example carries an `rmw-cyclonedds = ["nros/rmw-cyclonedds"]` feature
    # that routes the typed-creator hook (K.7.6.b) into the C++ backend.
    for collapsed_case in talker listener; do
        echo "  → native/rust/${collapsed_case} (cargo --features rmw-cyclonedds, target-cyclonedds/)"
        ensure_native_rust_generated "native/rust/${collapsed_case}"
    done
    export NROS_CARGO_PROFILE_ARGS="$cargo_profile_args"
    bash scripts/build/fixture-make-driver.sh linux-cyclonedds-rust
    # Phase 181.5 — collapsed-shape native C / C++ examples from the SSOT
    # manifest (examples/fixtures.toml). Per-cell RMW (`-DNROS_RMW`),
    # per-RMW build-subdir (`build-<rmw>/`) and any cmake target come from
    # the manifest; the platform-wide defs (build type, the C codegen tool,
    # codegen-off, cyclone prefix) are injected here via the shared builder's
    # NROS_CMAKE_EXTRA_DEFS contract. The cpp/parameters cell (target
    # parameters_roundtrip) is a manifest row too, so it builds in the same pass.
    export NROS_CMAKE_EXTRA_DEFS="-DNROS_SHARED_CARGO_ROOT=$(nros_build_dir "$NROS_KIND_CORROSION_CARGO" native) -DCMAKE_BUILD_TYPE=Release -DNANO_ROS_BUILD_CODEGEN=OFF -D_NANO_ROS_CODEGEN_TOOL=${codegen_tool} -DCMAKE_DISABLE_FIND_PACKAGE_CycloneDDS=ON"
    bash scripts/build/fixture-make-driver.sh linux-cmake-rmw
    unset NROS_CMAKE_EXTRA_DEFS
    # Phase 186 — native c/cpp Cyclone cells self-provision from source (static
    # ddsc via nano-ros-posix.cmake; DISABLE_FIND_PACKAGE forces the pinned source
    # over any system Cyclone). No build/install / `just cyclonedds setup` needed.
    # Phase 177.31 fixed the example link gap; the Cyclone *action* e2e is still
    # blocked on a runtime bug (177.32) so its tests skip at runtime, but the
    # fixtures build cleanly here. These cells are manifest rows and run through
    # fixtures-build.sh, preserving the incremental build-cyclonedds directories.
    export NROS_CMAKE_EXTRA_DEFS="-DNROS_SHARED_CARGO_ROOT=$(nros_build_dir "$NROS_KIND_CORROSION_CARGO" native) -DCMAKE_BUILD_TYPE=Release -DNANO_ROS_BUILD_CODEGEN=OFF -D_NANO_ROS_CODEGEN_TOOL=${codegen_tool} -DCMAKE_DISABLE_FIND_PACKAGE_CycloneDDS=ON -DNROS_RMW_CYCLONEDDS_MSG_TO_IDL=${repo_root}/scripts/cyclonedds/msg_to_cyclone_idl.py"
    # The two cyclone leaves run in parallel. The recursive-cargo jobserver
    # deadlock (issue 0022) is fixed at the source — nros-sizes-build strips the
    # jobserver from its nested probe cargo — so no per-leaf workaround is needed
    # here. DO NOT re-add `NROS_BUILD_JOBS=1` (re-serializes for no reason).
    bash scripts/build/fixture-make-driver.sh linux-cyclonedds-cmake
    unset NROS_CMAKE_EXTRA_DEFS

# Build every native tier: core artifacts, Rust/C/C++ examples, and test fixtures.
[group("full-matrix")]
build-all: build build-examples build-c build-c-xrce build-cpp build-fixtures
    @echo "Native build-all completed."

# Format all Rust examples (auto-discovered, parallel)
format:
    #!/usr/bin/env bash
    set -e
    echo "Formatting examples..."
    # Tracked-file enumeration via the git index (no filesystem traversal;
    # untracked build/target trees can never slow this down or leak in).
    git ls-files 'examples/**/Cargo.toml' | awk -F/ 'NF>=5' \
        | grep -vE '/zephyr/|/multi-package-workspace/|/qemu-arm-freertos/|/qemu-arm-nuttx/|/threadx-linux/|/qemu-riscv64-threadx/' \
        | xargs -r -n1 dirname | sort \
    | parallel --halt now,fail=1 --line-buffer \
        'cd {} && cargo +{{NIGHTLY}} fmt && echo "  fmt {}"'
    echo "All examples formatted!"

# Check all Rust examples (auto-discovered; fan-out under the make jobserver).
check JOBS="75%":
    #!/usr/bin/env bash
    set -e
    # phase-336 — resolve the profile ONCE here, not inside check_one: that
    # function is `export -f`d into GNU parallel's subshells, where the
    # cargo.sh helpers are not defined. An exported variable crosses; a
    # function does not.
    source scripts/build/cargo.sh
    export NROS_EXAMPLE_PROFILE_FLAGS="$(nros_cargo_profile_arg_string)"
    # issue 0624 — where this lane's cargo artifacts go. Resolved ONCE here for
    # the same reason as the profile flags above: `check_one` is `export -f`d
    # into subshells where `nros_build_dir` is not defined. An exported variable
    # crosses; a function does not.
    #
    # Without it, `cd <leaf> && cargo clippy` writes `<leaf>/target/` — the
    # exact shape CLAUDE.md names, and the population issue 0488 declared empty
    # and gated. The gate ran BEFORE this lane, so it passed on dirs this lane
    # had not yet re-created and failed on the NEXT run, which is why deleting
    # them "fixed" it for exactly one run.
    source scripts/build/build-root.sh
    export NROS_EXAMPLE_LINT_DIR="$(nros_build_dir "$NROS_KIND_EXAMPLE_LINT")"

    # Fan-out choice.
    #
    #   NROS_JOBSERVER=1  an OUTER pool already owns the tokens. A serial walk
    #                     still saturates the machine, because cargo is itself a
    #                     jobserver client and every one it spawns draws from
    #                     that pool. Fanning out on top would double-count the
    #                     budget — `nros_pool_available` refuses to nest for the
    #                     same reason.
    #
    # Otherwise the units go to `nros_pool_run`, which owns the degrade path
    # itself (serial when no pinned make 4.4). Issue 0466 — GNU parallel is no
    # longer used here, so its absence is no longer a tier-1 prerequisite that
    # silently turned this lane into a ~99-leaf serial walk looking like a hang.
    if [ "${NROS_JOBSERVER:-}" = "1" ] || [ "${NROS_INHERIT_JOBSERVER:-}" = "1" ]; then
        echo "Checking examples (serial; outer jobserver owns the tokens)..."
        SERIAL=1
    else
        SERIAL=0
        echo "Checking examples (jobserver fan-out)..."
    fi

    toml_list="$(mktemp "${TMPDIR:-/tmp}/nros_check_toml.XXXXXX")"
    trap 'rm -f "$toml_list"' EXIT

    # Exclude example trees that can't be host-clippy'd standalone (validated
    # via the fixture-manifest / orchestration path instead, as
    # multi-package-workspace already is):
    #   - ws-sync templates: workspace msg pkgs (`extra_msgs`/`local_msgs`) only
    #     resolve via the `[patch.crates-io]` block `nros sync` writes; cargo
    #     can't find them on crates.io here.
    #   - `examples/workspaces/*`: a multi-target workspace whose entry packages
    #     are cross-compiled (esp32/threadx/nuttx/freertos/zephyr) — e.g.
    #     `esp32_entry` configures `portable-atomic` single-core, which fails to
    #     compile for the host x86_64 target.
    # Tracked-file enumeration via the git index (no filesystem traversal;
    # untracked build/target trees can never slow this down or leak in).
    git ls-files 'examples/**/Cargo.toml' | awk -F/ 'NF>=5' \
        | grep -vE '/zephyr/|/multi-package-workspace/|/templates/local-msg-package/|/templates/multi-node-workspace/|/templates/workspace-shadowing/|/workspaces/|qemu-esp32-baremetal/rust/dds/|/qemu-arm-freertos/|/qemu-arm-nuttx/|/threadx-linux/|/qemu-riscv64-threadx/|/px4/' \
        | sort > "$toml_list"

    # Message-bindings precondition gate (phase-264 W4c follow-up). Example msg
    # crates (`generated/<msg>/`) are BUILD-TIME + ROS-version-dependent, so they
    # are gitignored and materialised by `nros sync` / `just generate-bindings`
    # (scripts/regenerate-bindings.sh) — NOT shipped in git. `just check` does NOT
    # sync them: that would couple a pure static check to a live ROS environment and
    # silently rewrite each example's Cargo.toml `[patch.crates-io]` block. Instead
    # it GATES — any example the fan-out below will check whose `package.xml`
    # declares a message `<depend>` but whose `generated/` is absent fails the
    # precondition with one clear, actionable error rather than a cryptic per-example
    # cargo "failed to read generated/<msg>/Cargo.toml" deep in the parallel output.
    #
    # The predicate is the dependency, NOT the pkg-metadata shape: every self-bringup
    # `[package.metadata.nros.application]` example AND every split Node pkg is
    # covered (the previous `[package.metadata.nros.node]`-only pre-pass silently
    # skipped all `application` examples, so single-pkg examples relied on an
    # undocumented "build-test-fixtures ran first" ordering — issue surfaced 2026-06).
    missing="$(mktemp "${TMPDIR:-/tmp}/nros_missing_bindings.XXXXXX")"
    trap 'rm -f "$toml_list" "$missing"' EXIT
    while IFS= read -r toml; do
        dir="$(dirname "$toml")"
        [ -f "$dir/package.xml" ] || continue
        grep -qE '<(depend|exec_depend|build_depend)>' "$dir/package.xml" || continue
        if [ ! -d "$dir/generated" ] || [ -z "$(ls -A "$dir/generated" 2>/dev/null)" ]; then
            printf '  %s\n' "$dir" >> "$missing"
        fi
    done < "$toml_list"
    if [ -s "$missing" ]; then
        echo "native check: $(wc -l < "$missing" | tr -d ' ') example(s) are missing their generated message bindings:" >&2
        cat "$missing" >&2
        echo "" >&2
        echo "  Bindings are build-time + ROS-version-dependent (gitignored, not shipped in git)." >&2
        echo "  Run 'just generate-bindings' (or 'just build-test-fixtures') first, then re-run 'just check'." >&2
        exit 1
    fi

    check_one() {
        local toml="$1"
        local dir="$(dirname "$toml")"
        local platform="$(echo "$dir" | cut -d/ -f2)"
        local flags=""
        local env_prefix=""
        # phase-336 — embedded examples lint at the ambient profile rather than a
        # hardcoded `--release`; resolved once above and passed in by variable.
        if [ "$platform" != "native" ]; then flags="$NROS_EXAMPLE_PROFILE_FLAGS"; fi
        if [ "$platform" = "esp32" ] || [ "$platform" = "qemu-esp32-baremetal" ]; then
            env_prefix="SSID=${SSID:-test} PASSWORD=${PASSWORD:-test}"
        fi
        # Quiet by default: no per-example label, and cargo --quiet suppresses
        # the "Checking …" progress + the "Blocking waiting for file lock on
        # package cache" spam (every parallel job contends the shared ~/.cargo
        # package-cache lock). A failing clippy still prints its diagnostic with
        # the example's file path, so failures stay identifiable. Set
        # NROS_CHECK_VERBOSE=1 to restore the per-example progress label.
        if [ -n "${NROS_CHECK_VERBOSE:-}" ]; then echo "  check $dir"; fi
        # `-p <leaf>`, not a bare `fmt --check`: run inside the leaf, cargo fmt
        # formats every member of THAT leaf's workspace — which, once a
        # developer has run `nros sync`, includes the `generated/*` message
        # crates. Those are USER-side codegen output (gitignored, regenerated
        # per host), so the gate was checking rustfmt's opinion of a code
        # generator's output and failing `just ci` for exactly the people who
        # followed the setup instructions. A fresh clone has no `generated/`,
        # so CI never saw it and main stayed green.
        pkg="$(sed -n 's/^name[[:space:]]*=[[:space:]]*"\([^"]*\)".*/\1/p' "$dir/Cargo.toml" | head -1)"
        # No `-j`: this path runs one leaf at a time (outer jobserver owns the
        # tokens, and cargo joins it). The old NROS_EXAMPLE_INNER_JOBS cap went
        # with the GNU parallel fan-out that needed it — issue 0466.
        # issue 0624 — one target dir PER LEAF (each is its own workspace root,
        # RFC-0026), never the leaf's own `target/` and never one dir shared by
        # all 39 (issue 0616: a `--target-dir` serves exactly ONE root).
        local lintdir="$NROS_EXAMPLE_LINT_DIR/$(echo "${dir#examples/}" | tr '/' '-')"
        ( cd "$dir" && cargo +{{NIGHTLY}} fmt --check -p "$pkg" && eval $env_prefix cargo clippy --quiet --target-dir "$lintdir" $flags )
    }
    export -f check_one
    export NIGHTLY="{{NIGHTLY}}"

    # Issue 0466 — fan out under the JOBSERVER, not GNU parallel.
    #
    # Every unit here is `cargo clippy`, and cargo is a jobserver CLIENT: given
    # an inherited jobserver it uses that budget instead of creating its own
    # (rust-lang/cargo#4110; rustc does the same via MAKEFLAGS). So N units
    # under one pool draw rustc slots from ONE set of tokens, which is what
    # `NROS_EXAMPLE_INNER_JOBS` was hand-simulating — the cap whose own comment
    # admitted the width "stayed survivable only because each example's dep
    # graph is small, which is luck, not a design". Under the pool the luck is
    # not needed and the knob is gone.
    #
    # `-P N` on top of self-parallelising tools is the multiplication the
    # ecosystem moved off years ago (12 tokens x 4 packages = 48 jobs); one
    # jobserver shared by every descendant is the current answer, and it is
    # already this repo's idiom in `workspace-fixtures-build.sh`,
    # `zephyr-fixture-make-driver.sh` and `compile-check-fixtures.sh`.
    #
    # `nros_pool_run` degrades to a serial walk by itself when a pool is
    # unavailable (outer jobserver, or no pinned make 4.4) — same commands,
    # same order — so this lane no longer needs its own serial branch, and
    # GNU parallel is no longer a tier-1 prerequisite at all.
    if [ "$SERIAL" = "1" ]; then
        while read -r toml; do check_one "$toml"; done < "$toml_list"
    else
        source scripts/build/jobserver-pool.sh
        # ONE self-contained line per leaf. A make recipe is one command per
        # line, so the unit cannot be an exported bash function spliced in with
        # `declare -f` (multi-line, and make runs recipes under /bin/sh). Derive
        # the per-leaf bits here — the same three `check_one` computes — and
        # emit a flat command.
        units="$(mktemp "${TMPDIR:-/tmp}/nros_check_units.XXXXXX")"
        trap 'rm -f "$toml_list" "$missing" "$units"' EXIT
        while read -r toml; do
            d="$(dirname "$toml")"
            plat="$(echo "$d" | cut -d/ -f2)"
            f=""
            [ "$plat" != "native" ] && f="$NROS_EXAMPLE_PROFILE_FLAGS"
            e=""
            case "$plat" in
                esp32 | qemu-esp32-baremetal) e="SSID=${SSID:-test} PASSWORD=${PASSWORD:-test}" ;;
            esac
            pkg="$(sed -n 's/^name[[:space:]]*=[[:space:]]*"\([^"]*\)".*/\1/p' "$d/Cargo.toml" | head -1)"
            # No `-j`: the pool's tokens are the budget now (cargo joins it).
            # issue 0624 — same per-leaf target dir as the serial branch above.
            # BOTH branches must carry it: fixing only one leaves `SERIAL=1`
            # writing the leaf `target/` dirs this gate exists to keep empty.
            ld="$NROS_EXAMPLE_LINT_DIR/$(echo "${d#examples/}" | tr '/' '-')"
            printf 'cd %s && cargo +%s fmt --check -p %s && %s cargo clippy --quiet --target-dir %s %s\n' \
                "$d" "{{NIGHTLY}}" "$pkg" "$e" "$ld" "$f"
        done < "$toml_list" > "$units"
        nros_pool_run check-examples < "$units"
    fi
    # issue 0624 — assert the leaf `target/` population is still empty HERE, at
    # the end of the lane that used to repopulate it. The gate also runs in
    # `check::fast`, but only BEFORE this lane, so a writer added to this recipe
    # produced a failure on the NEXT run, attributed to whatever else changed in
    # between. Checking after the writer makes it same-run and self-evident.
    python3 scripts/check-example-leaf-target-dirs.py
    echo "All examples check passed!"

# Show embedded example binary sizes.
#
# phase-336 — the paths follow the profile `build-examples` actually used. They
# were pinned to `release/` while the builder had moved to a different profile,
# and the `|| echo "build failed"` fallback reported that as a build failure
# rather than a wrong path: the exact silent shape this phase removes.
size: build-examples
    #!/usr/bin/env bash
    set -e
    # phase-336 — the paths follow the profile `build-examples` actually used.
    # They were pinned to `release/` while the builder had moved off it, and the
    # `|| echo "build failed"` fallback reported a wrong path as a broken build.
    source scripts/build/cargo.sh
    pdir="$(nros_cargo_target_profile_dir)"
    echo ""
    echo "Binary sizes ($pdir):"
    echo "======================="
    for img in \
        "packages/reference/stm32f4-porting/rtic/target/thumbv7em-none-eabihf/$pdir/stm32f4-rs-rtic-example" \
        "packages/reference/stm32f4-porting/polling/target/thumbv7em-none-eabihf/$pdir/stm32f4-rs-polling-example" \
        "packages/testing/nros-smoke/stm32f4-smoltcp-echo/target/thumbv7em-none-eabihf/$pdir/stm32f4-rs-smoltcp"; do
        if [ -f "$img" ]; then
            size "$img"
        else
            echo "MISSING: $img"
        fi
    done

# Clean native example build artifacts.
[group("main")]
clean:
    #!/usr/bin/env bash
    find examples/native -type d \( -name target -o -name build -o -name 'build-*' \) \
        -prune -exec rm -rf {} + 2>/dev/null || true
    echo "Native example build artifacts cleaned"

# =============================================================================
# Native C/C++ examples (CMake-based)
# =============================================================================

# Build a single C example with CMake (Phase 140 — add_subdirectory shape).
[no-exit-message]
_build-c-example dir *CMAKE_ARGS:
    #!/usr/bin/env bash
    set -e
    source scripts/build/cargo.sh
    source scripts/build/cmake-incremental.sh
    echo "Building {{dir}}..."
    nros_cargo_ensure_codegen_c
    codegen_tool="$(nros_cargo_codegen_c_bin)"
    nros_cmake_configure_if_needed "{{dir}}" "{{dir}}/build" \
        -DNANO_ROS_PLATFORM=posix -DNANO_ROS_RMW=zenoh -DNROS_RMW=zenoh \
        -DNANO_ROS_BUILD_CODEGEN=OFF -D_NANO_ROS_CODEGEN_TOOL="$codegen_tool" {{CMAKE_ARGS}}
    cmake --build "{{dir}}/build"

# Build C examples only (no tests)
[group("examples")]
build-c:
    #!/usr/bin/env bash
    set -e
    just native _build-c-example examples/native/c/talker
    just native _build-c-example examples/native/c/listener
    just native _build-c-example examples/native/c/custom-msg
    echo "C examples built!"

# Build C XRCE examples only (no tests)
[group("examples")]
build-c-xrce:
    #!/usr/bin/env bash
    set -e
    just native _build-c-example examples/native/c/talker  "-DNANO_ROS_RMW=xrce -DNROS_RMW=xrce"
    just native _build-c-example examples/native/c/listener "-DNANO_ROS_RMW=xrce -DNROS_RMW=xrce"
    echo "C XRCE examples built!"

# Clean C examples build
[group("main")]
clean-c:
    find examples/native/c -type d \( -name build -o -name 'build-*' \) \
        -prune -exec rm -rf {} + 2>/dev/null || true
    @echo "C examples build cleaned"

# Build a single C++ example with CMake (Phase 140 — add_subdirectory shape).
[no-exit-message]
_build-cpp-example dir *CMAKE_ARGS:
    #!/usr/bin/env bash
    set -e
    source scripts/build/cargo.sh
    source scripts/build/cmake-incremental.sh
    echo "Building {{dir}}..."
    nros_cargo_ensure_codegen_c
    codegen_tool="$(nros_cargo_codegen_c_bin)"
    nros_cmake_configure_if_needed "{{dir}}" "{{dir}}/build" \
        -DNANO_ROS_PLATFORM=posix -DNANO_ROS_RMW=zenoh -DNROS_RMW=zenoh \
        -DNANO_ROS_BUILD_CODEGEN=OFF -D_NANO_ROS_CODEGEN_TOOL="$codegen_tool" {{CMAKE_ARGS}}
    cmake --build "{{dir}}/build"

# Build C++ examples only (no tests)
[group("examples")]
build-cpp:
    #!/usr/bin/env bash
    set -e
    just native _build-cpp-example examples/native/cpp/talker
    just native _build-cpp-example examples/native/cpp/listener
    just native _build-cpp-example examples/native/cpp/service-server
    just native _build-cpp-example examples/native/cpp/service-client
    just native _build-cpp-example examples/native/cpp/action-server
    just native _build-cpp-example examples/native/cpp/action-client
    echo "C++ examples built!"

# Clean C++ examples build
[group("main")]
clean-cpp:
    find examples/native/cpp -type d \( -name build -o -name 'build-*' \) \
        -prune -exec rm -rf {} + 2>/dev/null || true
    @echo "C++ examples build cleaned"

# =============================================================================
# Integration tests
# =============================================================================

# Run integration tests only (requires zenohd). Focused ROS 2, large-msg,
# native C/C++, and XRCE C API lanes live below; this default native tier
# keeps those slower slices out of the broad local loop.
#
# Phase 214.Q: `build-fixtures` is sequenced as a prereq so the nextest
# harness never explodes with `nros_tests::fixtures::binaries::*` "Test
# fixture binary not prebuilt" errors. The harness fails loud on missing
# fixtures by design (Phase 181) — recipes own the build-then-test
# sequencing.
[group("main")]
test verbose="": build-fixtures
    #!/usr/bin/env bash
    bash scripts/dev/deprecated-scope-alias.sh native test "just test native"
    set -e
    source scripts/build/cargo.sh
    cargo_nextest_args=($(nros_cargo_nextest_args))
    # Drop ROS 2 / XRCE-interop binaries by their nextest test-groups
    # (`.config/nextest.toml`) so new interop binaries assigned to those
    # groups are excluded automatically — no per-binary chain to maintain.
    # The remaining explicit `binary(...)` excludes are carve-outs with
    # their own focused lane and no shared group: zephyr (native_sim
    # fixtures), esp32_emulator, large_msg (test-large-msg), and the
    # native_api / parameters_roundtrip / c_xrce_api fixture lanes
    # (test-native-api, test-c-xrce). Unlike root `just test`, this lane
    # keeps the QEMU RTOS e2e groups in.
    args=(-p nros-tests --no-fail-fast \
          -E 'not (group(=ros2-interop) or binary(xrce_ros2_interop) or binary(zephyr) or binary(esp32_emulator) or binary(large_msg) or binary(native_api) or binary(parameters_roundtrip) or binary(c_xrce_api))')
    if [ -z "{{verbose}}" ]; then
        args+=(--success-output never --failure-output never)
    fi
    cargo nextest run "${cargo_nextest_args[@]}" "${args[@]}"

# Run all native tests (integration + ROS 2 interop + large_msg + C/C++ API)
[group("full-matrix")]
test-all verbose="":
    #!/usr/bin/env bash
    set +e
    failed=0
    just native test {{verbose}} || failed=1
    just native test-large-msg {{verbose}} || failed=1
    just native test-ros2 {{verbose}} || failed=1
    just native test-ros2-params {{verbose}} || failed=1
    just native test-ros2-lifecycle {{verbose}} || failed=1
    just native test-native-api {{verbose}} || failed=1
    just native test-c-xrce {{verbose}} || failed=1
    just native _test-c-codegen {{verbose}} || failed=1
    if [ $failed -ne 0 ]; then
        echo "FAIL: Some native tests failed."
        exit 1
    fi

# CI: check + test
[group("full-matrix")]
ci: check test

# Run large message & throughput stress tests
[group("debug")]
test-large-msg verbose="":
    #!/usr/bin/env bash
    set -e
    source scripts/build/cargo.sh
    cargo_nextest_args=($(nros_cargo_nextest_args))
    args=(-p nros-tests --test large_msg --no-fail-fast)
    if [ -z "{{verbose}}" ]; then
        args+=(--success-output never --failure-output never)
    fi
    cargo nextest run "${cargo_nextest_args[@]}" "${args[@]}"

[group("debug")]
test-ros2 verbose="":
    #!/usr/bin/env bash
    set -e
    source scripts/build/cargo.sh
    cargo_nextest_args=($(nros_cargo_nextest_args))
    args=(-p nros-tests --test interop_e2e --no-fail-fast)
    if [ -z "{{verbose}}" ]; then
        args+=(--success-output never --failure-output never)
    fi
    cargo nextest run "${cargo_nextest_args[@]}" "${args[@]}"

# Run ROS 2 parameter interop tests.
# Needs the distro's `rmw_zenoh_cpp` (`ros-<distro>-rmw-zenoh-cpp` from apt).
# There is no source overlay to build: issue 0291 refuted the "wire-match our
# zenoh-pico pin" premise (zenoh's wire is proto-0x09-stable across 1.x), so the
# pinned build had no automated consumer and was removed.
[group("debug")]
test-ros2-params verbose="":
    #!/usr/bin/env bash
    set -e
    source scripts/build/cargo.sh
    cargo_nextest_args=($(nros_cargo_nextest_args))
    args=(-p nros-tests --test params --no-fail-fast)
    if [ -z "{{verbose}}" ]; then
        args+=(--success-output never --failure-output never)
    fi
    cargo nextest run "${cargo_nextest_args[@]}" "${args[@]}"

# Run the `ros2 lifecycle …` interop test against an nros lifecycle-node.
# Needs the distro's `rmw_zenoh_cpp` (`ros-<distro>-rmw-zenoh-cpp` from apt).
# There is no source overlay to build: issue 0291 refuted the "wire-match our
# zenoh-pico pin" premise (zenoh's wire is proto-0x09-stable across 1.x), so the
# pinned build had no automated consumer and was removed.
[group("debug")]
test-ros2-lifecycle verbose="":
    #!/usr/bin/env bash
    set -e
    source scripts/build/cargo.sh
    cargo_nextest_args=($(nros_cargo_nextest_args))
    args=(-p nros-tests --test interop_e2e -E 'test(lifecycle)' --no-fail-fast)
    if [ -z "{{verbose}}" ]; then
        args+=(--success-output never --failure-output never)
    fi
    cargo nextest run "${cargo_nextest_args[@]}" "${args[@]}"

# Run ROS 2 interop tests (shell script)
[group("debug")]
test-ros2-shell:
    ./tests/ros2-interop.sh

_test-c-codegen verbose="":
    #!/usr/bin/env bash
    set -e
    just init-test-logs
    v="{{ if verbose != "" { "--verbose" } else { "" } }}"
    ./tests/run-test.sh --name c-msg-gen --log {{LOG_DIR}}/latest/c-msg-gen.log $v -- ./tests/c-msg-gen-tests.sh

# Run the full native C/C++ API nextest lane.
[group("debug")]
test-native-api verbose="":
    #!/usr/bin/env bash
    set -e
    # Consumes prebuilt per-RMW CMake fixtures; run
    # `just build-test-fixtures` first for the full matrix.
    source scripts/build/cargo.sh
    cargo_nextest_args=($(nros_cargo_nextest_args))
    args=(-p nros-tests --test native_api --test parameters_roundtrip --no-fail-fast)
    if [ -z "{{verbose}}" ]; then
        args+=(--success-output never --failure-output never)
    fi
    cargo nextest run "${cargo_nextest_args[@]}" "${args[@]}"

# Run all C tests (integration + codegen)
[group("debug")]
test-c verbose="":
    #!/usr/bin/env bash
    set -e
    source scripts/build/cargo.sh
    cargo_nextest_args=($(nros_cargo_nextest_args))
    just init-test-logs
    args=(-p nros-tests --test native_api --no-fail-fast \
          -E 'test(/Language__C$/) or test(test_c_)')
    if [ -z "{{verbose}}" ]; then
        args+=(--success-output never --failure-output never)
    fi
    cargo nextest run "${cargo_nextest_args[@]}" "${args[@]}"
    just native _test-c-codegen {{verbose}}

# C++ API integration tests (needs cmake + zenohd)
[group("debug")]
test-cpp verbose="":
    #!/usr/bin/env bash
    set -e
    source scripts/build/cargo.sh
    cargo_nextest_args=($(nros_cargo_nextest_args))
    args=(-p nros-tests --no-fail-fast \
          -E '(binary(native_api) and (test(Language__Cpp) or test(test_cpp_))) or binary(parameters_roundtrip)')
    if [ -z "{{verbose}}" ]; then
        args+=(--success-output never --failure-output never)
    fi
    cargo nextest run "${cargo_nextest_args[@]}" "${args[@]}"

# C XRCE-DDS API integration tests (needs cmake + XRCE Agent)
[group("debug")]
test-c-xrce verbose="":
    #!/usr/bin/env bash
    set -e
    source scripts/build/cargo.sh
    cargo_nextest_args=($(nros_cargo_nextest_args))
    args=(-p nros-tests --no-fail-fast -E 'binary(c_xrce_api)')
    if [ -z "{{verbose}}" ]; then
        args+=(--success-output never --failure-output never)
    fi
    cargo nextest run "${cargo_nextest_args[@]}" "${args[@]}"

# Run executor fairness benchmark (requires zenohd)
# `--release` is deliberate and stays literal: a benchmark wants the
# performance profile by name, which is what `release` means since phase-336
# (opt-level 3). It is not the ambient build profile, and following that would
# make the numbers depend on how the tree happened to be configured.
# `--release` is deliberate and stays literal: a benchmark wants the
# performance profile BY NAME, which is what `release` means since phase-336
# (opt-level 3). Following the ambient profile would make the numbers depend on
# how the tree happened to be configured.
# profile-literal-ok: benchmark: wants the performance profile by name
bench-fairness:
    cd packages/testing/nros-bench/executor-fairness && \
        RUST_LOG=warn cargo run --release

# =============================================================================
# Run examples
# =============================================================================

# Run native Rust talker
talker:
    cd examples/native/rust/talker && cargo run

# Run native Rust listener
listener:
    cd examples/native/rust/listener && cargo run

# Run native Rust service server
service-server:
    cd examples/native/rust/service-server && cargo run

# Run native Rust service client
service-client:
    cd examples/native/rust/service-client && cargo run

# Run native Rust action server
action-server:
    cd examples/native/rust/action-server && cargo run

# Run native Rust action client
action-client:
    cd examples/native/rust/action-client && cargo run

# Start zenohd router for native examples (override bind via $ZENOH_LOCATOR)
zenohd:
    #!/usr/bin/env bash
    set -e
    source scripts/dev/zenohd.sh
    nros_router_exec "${ZENOH_LOCATOR:-tcp/127.0.0.1:7447}"
