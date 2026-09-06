# The six Rust roles this lane builds. ONE list: it was written out three times
# (the codegen pre-sync, the Corrosion fan-out, the fixture loop), and a role
# added to two of the three is a leaf that silently never gets synced or never
# gets built — the "one shared spelling, not a second one" rule.
RV64_RUST_ROLES := "talker listener service-server service-client action-server action-client"

set working-directory := '..'

default:
    @just --list --justfile {{source_file()}}

# Install ThreadX RISC-V 64 prerequisites. Phase 197.4 — `nros setup <board>`
# provisions the threadx + netxduo sources + riscv-none-elf-gcc + qemu from the
# SDK index; the host-env helper handles rustup (riscv64 build-std, no cross
# target). Does NOT build example binaries — use `build-examples`.
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
    nros_setup_board_rmws qemu-riscv64-threadx threadx-riscv64
    "{{justfile_directory()}}/tools/host-env.sh" threadx

# Diagnose ThreadX RISC-V 64 install status (read-only).
[group("setup")]
doctor:
    #!/usr/bin/env bash
    bash scripts/dev/deprecated-scope-alias.sh threadx_riscv64 doctor "just doctor threadx_riscv64"
    set +e
    fail=0
    if [ -d "$THREADX_DIR/common/inc" ]; then
        echo "  [OK] ThreadX kernel: $THREADX_DIR"
    else
        echo "  [MISSING] ThreadX kernel at $THREADX_DIR (run: just threadx_riscv64 setup)"
        fail=1
    fi
    if [ -d "$NETX_DIR/common/inc" ]; then
        echo "  [OK] NetX Duo: $NETX_DIR"
    else
        echo "  [MISSING] NetX Duo at $NETX_DIR (run: just threadx_riscv64 setup)"
        fail=1
    fi
    # shellcheck source=scripts/build/riscv64-toolchain.sh
    source scripts/build/riscv64-toolchain.sh
    _rv_prefix="$(nros_riscv64_prefix)"
    if [ -n "$_rv_prefix" ]; then
        echo "  [OK] ${_rv_prefix}-gcc ($(nros_riscv64_tool gcc))"
        # issue 0657 — probe the QUESTION (can this toolchain preprocess a TU
        # that needs libc headers?), not one libc's answer to it. Two shapes
        # reach here and both are fine:
        #
        #   * Ubuntu `gcc-riscv64-unknown-elf` — compiler only, headers come
        #     from a separate picolibc package, reached via `--specs=`;
        #   * the pinned index dist (xPack `riscv-none-elf-gcc`, what
        #     `nros setup` installs for this board) — BUNDLES newlib, so plain
        #     preprocessing already works and `picolibc.specs` does not exist.
        #
        # The old check asked only the first, so the provisioned toolchain
        # reported "[MISSING] riscv64 C library headers" while holding a
        # complete libc. There WAS a second branch saying exactly that about
        # xPack; it sat after an `elif` this one always won.
        _rv_gcc="$(nros_riscv64_tool gcc)"
        if echo '#include <stdint.h>' | "$_rv_gcc" -x c -E - &>/dev/null; then
            echo "  [OK] riscv64 C library headers (bundled with the toolchain)"
        elif echo '#include <stdint.h>' | "$_rv_gcc" --specs=picolibc.specs -x c -E - &>/dev/null; then
            echo "  [OK] riscv64 C library headers (picolibc)"
        else
            echo "  [MISSING] riscv64 C library headers — this toolchain ships none."
            echo "             Pinned dist that bundles its own: nros setup --tool riscv-none-elf-gcc"
            echo "             Or add picolibc for the distro compiler: nros setup --system"
            fail=1
        fi
    else
        echo "  [MISSING] riscv64 bare-metal cross-gcc. Install the pinned index"
        echo "             dist:   nros setup --tool riscv-none-elf-gcc"
        echo "             or distro: just workspace apt-packages"
        echo "             (gcc-riscv64-unknown-elf + picolibc-riscv64-unknown-elf)"
        fail=1
    fi
    if command -v qemu-system-riscv64 &>/dev/null; then
        echo "  [OK] qemu-system-riscv64"
    else
        echo "  [MISSING] qemu-system-riscv64 (run: nros setup --tool qemu — prebuilt, no sudo; or just threadx_riscv64 setup) — 0368 F2"
        fail=1
    fi
    # Host `idlc` (Cyclone DDS IDL compiler) — needed only for the Cyclone
    # fixtures (`build-fixture-extras`), which self-provision Cyclone with
    # BUILD_IDLC=OFF and so cannot produce idlc themselves. Advisory (not
    # fail=1): the Cyclone fixtures are opt-out via
    # NROS_THREADX_RV64_CYCLONEDDS_FIXTURES=0, and the zenoh path needs no idlc.
    if command -v idlc &>/dev/null; then
        echo "  [OK] idlc (Cyclone DDS IDL compiler): $(command -v idlc)"
    elif [ -x "$PWD/build/cyclonedds/bin/idlc" ]; then
        echo "  [OK] idlc (project): $PWD/build/cyclonedds/bin/idlc (folded onto PATH by build-fixture-extras)"
    else
        echo "  [INFO] idlc not found — ThreadX-RV64 Cyclone fixtures will skip."
        echo "         Provision it the way a USER would:  nros setup --tool cyclonedds"
        echo "         (the index builds Cyclone with BUILD_IDLC=ON, and activate.sh puts"
        echo "          the store bin on PATH — issue 0663), then re-source ./activate.sh."
        echo "         In-repo alternative: just cyclonedds setup (builds build/cyclonedds/bin/idlc)."
        echo "         Opt out entirely: NROS_THREADX_RV64_CYCLONEDDS_FIXTURES=0."
    fi
    exit $fail

# Phase 140 — `install` recipe deleted. ThreadX RISC-V 64 examples
# consume nano-ros via `add_subdirectory(<repo-root>)` (Phase 144.7).

# Build ThreadX RISC-V 64 examples (canonical core-link smoke).
[group("main")]
build: build-examples
    @echo "ThreadX RISC-V 64 examples built."

# Build ThreadX QEMU RISC-V 64 examples
#
# issue 1038 — `nros_ensure_central_patch` FIRST. Every Rust leaf here reaches the
# universal-trio patches through `include = ["…/nros-patch.toml"]`, and that
# file is written by `nros sync`, which needs a workspace. This lane builds only
# standalone leaves, so nothing here ever wrote it: the nightly cell died on
# `failed to load config include ../../../../../nros-patch.toml` at MANIFEST
# PARSE, before any compilation. Every other platform lane runs
# `workspace-fixtures-build.sh` somewhere and got the file by side effect, which
# is why the identical config resolved for them and not here.
[group("main")]
build-examples:
    #!/usr/bin/env bash
    set -e
    # issue 0650 — first step of the lane: clear last run's skip notes, so a
    # lane that is complete this time is not reported SKIPPED from history.
    # shellcheck source=scripts/build/lane-skip.sh
    source scripts/build/lane-skip.sh
    nros_lane_skip_reset threadx-riscv64
    source scripts/build/cargo.sh
    # issue 1038 — before ANY cargo invocation in this lane (see the header).
    nros_ensure_central_patch
    cargo_profile_args="$(nros_cargo_profile_arg_string)"
    if [ ! -d "$THREADX_DIR/common/inc" ] || [ ! -d "$NETX_DIR/common/inc" ]; then
        nros_lane_skip_note threadx-riscv64 "Skipping ThreadX QEMU RISC-V examples (ThreadX/NetX not found). Run: just threadx-linux setup"
        exit 0
    fi
    # issue 0657 — resolve, do not spell. `nros setup` provisions xPack's
    # `riscv-none-elf-gcc` for this board, so a guard naming only Ubuntu's
    # binary skipped the lane on a fully provisioned host.
    # shellcheck source=scripts/build/riscv64-toolchain.sh
    source scripts/build/riscv64-toolchain.sh
    # issue 0657 — cc-rs derives its compiler from the TARGET TRIPLE when no
    # `CC_<triple>` says otherwise, and its derivation for
    # `riscv64gc-unknown-none-elf` is `riscv64-unknown-elf-gcc` — a binary the
    # provisioned toolchain does not contain. Exported here so the lane's cargo
    # builds and its cmake ones use the same compiler.
    nros_riscv64_export_cc
    if [ -z "$(nros_riscv64_prefix)" ]; then
        nros_lane_skip_note threadx-riscv64 "Skipping ThreadX QEMU RISC-V examples (no riscv64 bare-metal gcc: run nros setup qemu-riscv64-threadx)"
        exit 0
    fi
    echo "Building ThreadX QEMU RISC-V 64 examples..."
    # Board config dirs are platform env (same for every fixture) — export them
    # for the shared builder; per-fixture options come from the SSOT manifest
    # (examples/fixtures.toml). Phase 181.4.g.
    rv_config="$(pwd)/packages/boards/nros-board-threadx-qemu-riscv64/config"
    export THREADX_CONFIG_DIR="$rv_config" NETX_CONFIG_DIR="$rv_config"
    # phase-369 W2 — the six rust example leaves are no longer cargo rows.
    #
    # They are cmake rows now (`builder = "cmake"`, `build-zenoh/`), built by
    # `build_threadx_cmake_rmw` in `build-fixture-extras` alongside their
    # cyclonedds twins — one mechanism for both RMWs, which is the point of
    # issue 0666. Calling the cargo builder here as well would build them a
    # second time, by the path this phase is removing.
    #
    # `logging-smoke-threadx-riscv64` is still a genuine cargo row and is built
    # by id in `build-fixture-extras`.
    echo "ThreadX QEMU RISC-V rust leaves build via cmake (phase-369); see build-fixture-extras."
    echo "ThreadX QEMU RISC-V examples built!"

# Build all example binaries used as test fixtures (Rust + C + C++).
[group("full-matrix")]
build-fixtures: build-examples build-fixture-extras
    #!/usr/bin/env bash
    bash scripts/dev/deprecated-scope-alias.sh threadx_riscv64 build-fixtures "just build threadx_riscv64"
    set -e
    # issue 0650 — the ONE place this lane says it built its fixtures, and it
    # refuses to when a step was skipped. Before this the claim was an
    # unconditional echo, so a lane with no toolchain printed it anyway.
    # shellcheck source=scripts/build/lane-skip.sh
    source scripts/build/lane-skip.sh
    nros_lane_skip_flush threadx-riscv64 "ThreadX-RV64 test fixtures built."

# Internal Phase 178.B target: only fixture artifacts not already covered by
# `build-examples`. Mirrors the cmake -D set used by the test fixture builders
# in `nros_tests::fixtures::binaries::threadx_riscv64::build_cmake_example`.
[group("examples")]
build-fixture-extras:
    #!/usr/bin/env bash
    # shellcheck source=scripts/build/lane-skip.sh
    source scripts/build/lane-skip.sh
    set -e
    source scripts/build/cargo.sh
    source scripts/build/fixture-matrix.sh
    # issue 1038 — this is where the Rust leaves are actually built (the
    # phase-369 cmake path), so it is where the nightly died at manifest parse.
    # Also called in `build-examples`, which runs first under `build-fixtures`;
    # the call is idempotent, and repeating it means running THIS recipe alone
    # works too.
    nros_ensure_central_patch
    cargo_profile_args="$(nros_cargo_profile_arg_string)"
    export cargo_profile_args
    cargo_frontends="$(nros_cargo_frontend_jobs)"
    cmake_frontends="$(nros_cmake_frontend_jobs)"
    # shellcheck source=scripts/build/riscv64-toolchain.sh
    source scripts/build/riscv64-toolchain.sh
    # issue 0657 — cc-rs derives its compiler from the TARGET TRIPLE when no
    # `CC_<triple>` says otherwise, and its derivation for
    # `riscv64gc-unknown-none-elf` is `riscv64-unknown-elf-gcc` — a binary the
    # provisioned toolchain does not contain. Exported here so the lane's cargo
    # builds and its cmake ones use the same compiler.
    nros_riscv64_export_cc
    if [ -z "$(nros_riscv64_prefix)" ]; then
        nros_lane_skip_note threadx-riscv64 "no riscv64 bare-metal gcc (run: nros setup qemu-riscv64-threadx)"; exit 0
    fi
    # issue 1038 follow-up — sync the six rust roles' message crates BEFORE
    # cmake ever configures them.
    #
    # `nros_ensure_central_patch` above (and again, idempotently, before the
    # `logging-smoke` cargo row below) fixes the include-target half of issue
    # 0463 for this lane, but talker/listener/service-{server,client}/
    # action-{server,client} each also patch a generated message crate
    # (`std_msgs`, `builtin_interfaces`, …) that ONLY `nros sync` — not
    # `ws central-patch` — materialises. They build through
    # `nros_threadx_rv64_rust_app` (Corrosion/CMake), the one seam in this repo
    # that never goes through `fixtures-build.sh`'s per-row presync, so nothing
    # else in this lane ever ran it. Serial, once per role, before the six
    # leaves fan out below — same shape as `nros_presync_row_dirs`.
    for _rc_sync in {{RV64_RUST_ROLES}}; do
        _rc_sync_dir="examples/qemu-riscv64-threadx/rust/$_rc_sync"
        if [ -d "$_rc_sync_dir" ]; then
            echo "  → (role codegen) $_rc_sync_dir"
            nros_ensure_leaf_synced "$_rc_sync_dir"
        fi
    done
    echo "Building ThreadX-RV64 test fixtures..."
    # Phase 176 — cmake parallelism via the fifo jobserver when present.
    if [ "${NROS_JOBSERVER:-}" = "1" ] || [ "${NROS_INHERIT_JOBSERVER:-}" = "1" ]; then unset CMAKE_BUILD_PARALLEL_LEVEL; else export CMAKE_BUILD_PARALLEL_LEVEL="${NROS_BUILD_JOBS:-8}"; fi
    root="$(pwd)"
    toolchain="$root/cmake/toolchain/riscv64-threadx.cmake"
    threadx_dir="${THREADX_DIR:-$root/third-party/threadx/kernel}"
    netx_dir="${NETX_DIR:-$root/third-party/threadx/netxduo}"
    config_dir="$root/packages/boards/nros-board-threadx-qemu-riscv64/config"
    board_dir="$root/packages/boards/nros-board-threadx-qemu-riscv64/c"
    virtio_dir="$root/packages/drivers/net/virtio-net-netx"
    # Phase 88.15.d — nros-log severity-coverage smoke fixture. It needs THIS
    # board's NetX port headers rather than the threadx-linux defaults the
    # top-level justfile exports (the global-default-shadows-a-per-board-value
    # class). The env vars stay: the C/C++ lane below consumes them too.
    # (This comment named `threadx-netx-sys` until phase-400 deleted that crate —
    # nothing here ever depended on it; the fixture is a plain Rust build.)
    THREADX_CONFIG_DIR="$config_dir" NETX_CONFIG_DIR="$config_dir" \
        bash scripts/build/fixtures-build.sh threadx-riscv64 rust --id threadx-riscv64-logging-smoke
    # Phase 160.J — ThreadX RV64 C/C++ examples now build via cmake.
    # Corrosion handles the riscv64gc-unknown-none-elf cross under
    # the bundled toolchain file; the ThreadX + NetX Duo include
    # paths the staticlib needs flow in via `-D` cache vars matching
    # the Rust path's `.cargo/config.toml` env. Host nros-codegen is
    # still required for `nros_generate_interfaces()`.
    echo "  → host nros-codegen (cargo)"
    nros_cargo_ensure_codegen_c
    codegen="$(nros_cargo_codegen_c_bin)"
    # Phase 118.C — collapsed-shape ThreadX-RV64 C / C++. The Rust
    # zenoh role binaries are produced by `build-examples`.
    # Phase 175.B adds CMake/Corrosion Cyclone cells for talker/listener
    # coverage because Cyclone is a C++ backend and cannot link from
    # cargo alone.

    build_threadx_cmake_rmw() {
        local dir="$1"
        local rmw="$2"
        local build_dir="$3"
        echo "  → $dir (-DNROS_RMW=$rmw, $build_dir/)"
        # phase-295 W4 (ex Phase 177.38) — Cyclone RUST role builds bake the
        # allocator domain (nros_tests::alloc::domain_of(ThreadxRiscv64, Rust,
        # Pubsub) = 127; env for the Rust/Corrosion build + -D for the C/C++
        # glue); fold it into the cache sig so a domain change forces a
        # rebuild. Empty for non-Cyclone (keeps the config.toml default).
        local _dom=""
        [ "$rmw" = "cyclonedds" ] && _dom="127"
        local sig
        sig="$(printf '%s\n' \
          "platform=threadx_riscv64" \
          "rmw=$rmw" \
          "domain=${_dom}" \
          "src=$dir" \
          "toolchain=$toolchain" \
          "threadx_dir=$threadx_dir" \
          "netx_dir=$netx_dir" \
          "config_dir=$config_dir" \
          "codegen=$codegen" \
          "cyclonedds_prefix=${cyclonedds_prefix:-}")"
        # Export the domain (not an inline VAR=val prefix — a prefix produced by
        # ${_dom:+…} expansion is parsed as a command, not an assignment). The
        # Rust/Corrosion build reads it via option_env!; the -D below covers the
        # C/C++ glue. Only set for Cyclone (_dom non-empty).
        [ -n "$_dom" ] && export NROS_DOMAIN_ID="$_dom"
        # issue 0805 — one Corrosion cargo dir per (platform, resolved feature
        # inputs) instead of one per leaf. cmake keys the subdirectory itself:
        # the leaf's capabilities are not knowable here, and keying on
        # "platform + rmw" alone would let two leaves with different
        # capabilities overwrite each other's unhashed `libnros_c.a`.
        THREADX_DIR="$threadx_dir" NETX_DIR="$netx_dir" \
        THREADX_CONFIG_DIR="$config_dir" NETX_CONFIG_DIR="$config_dir" \
          nros_cmake_fixture_build "$dir" "$dir/$build_dir" "$sig" \
            -DNROS_SHARED_CARGO_ROOT="$shared_cargo_root" \
            -DNANO_ROS_PLATFORM=threadx_riscv64 \
            -DNROS_RMW="$rmw" \
            ${_dom:+-DNROS_DOMAIN_ID=$_dom} \
            -DCMAKE_TOOLCHAIN_FILE="$toolchain" \
            -DTHREADX_DIR="$threadx_dir" \
            -DNETX_DIR="$netx_dir" \
            -DTHREADX_CONFIG_DIR="$config_dir" \
            -DNETX_CONFIG_DIR="$config_dir" \
            -DCMAKE_BUILD_TYPE=Release \
            -D_NANO_ROS_CODEGEN_TOOL="$codegen" \
            -DNROS_RMW_CYCLONEDDS_MSG_TO_IDL="$root/scripts/cyclonedds/msg_to_cyclone_idl.py" > /dev/null
    }

    # Phase 181.5.e — collapsed-shape ThreadX-RV64 C / C++ from the SSOT
    # manifest. The Rust zenoh role binaries are produced by `build-examples`.
    # The example CMakeLists read the NetX / ThreadX include paths from env at
    # configure time (matching the Rust path's .cargo/config.toml), so export
    # them; the rest of the -D set goes through NROS_CMAKE_EXTRA_DEFS. The
    # IDLC / msg_to_idl defs are harmless for the zenoh pass.
    export THREADX_DIR="$threadx_dir" NETX_DIR="$netx_dir" \
           THREADX_CONFIG_DIR="$config_dir" NETX_CONFIG_DIR="$config_dir"
    # 287-W6 dropped the leaves' `set(NANO_ROS_PLATFORM threadx)` (ament shape);
    # the platform must now come from the invocation (mirrors threadx-linux's
    # recipe). Without it the root default "posix" configures
    # nros-platform-posix under the riscv64 cross toolchain → FindThreads dies.
    # issue 0805 — the C/C++ leaves reach cmake through `fixtures-build.sh`
    # via NROS_CMAKE_EXTRA_DEFS, a different path from the Rust leaves'
    # `build_threadx_cmake_rmw`. BOTH need the shared root or only half the
    # leaves collapse — measured: wiring only the Rust path left 17 of 29
    # per-leaf cargo dirs in place.
    shared_cargo_root="$(nros_build_dir "$NROS_KIND_CORROSION_CARGO" threadx-riscv64)"
    base_defs="-DNROS_SHARED_CARGO_ROOT=$shared_cargo_root -DNANO_ROS_PLATFORM=threadx_riscv64 -DCMAKE_TOOLCHAIN_FILE=$toolchain -DTHREADX_DIR=$threadx_dir -DNETX_DIR=$netx_dir -DTHREADX_CONFIG_DIR=$config_dir -DNETX_CONFIG_DIR=$config_dir -DCMAKE_BUILD_TYPE=Release -D_NANO_ROS_CODEGEN_TOOL=$codegen -DNROS_RMW_CYCLONEDDS_MSG_TO_IDL=$root/scripts/cyclonedds/msg_to_cyclone_idl.py"
    export NROS_CMAKE_EXTRA_DEFS="$base_defs"
    bash scripts/build/fixtures-build.sh threadx-riscv64 c zenoh
    bash scripts/build/fixtures-build.sh threadx-riscv64 cpp zenoh

    cyclonedds_prefix="$root/build/cyclonedds-threadx-rv64-install"
    # Phase 203 decision — build the ThreadX-RV64 Cyclone fixtures BY DEFAULT so a
    # clean `build-all` compiles every fixture (the runtime works e2e since Phase
    # 177.26; Phase 186 self-provisions Cyclone from source, no separate probe).
    # Opt out with NROS_THREADX_RV64_CYCLONEDDS_FIXTURES=0 (e.g. a fast local loop).
    if [ "${NROS_THREADX_RV64_CYCLONEDDS_FIXTURES:-1}" = "0" ]; then
        echo "  → skipping ThreadX-RV64 Cyclone fixtures (NROS_THREADX_RV64_CYCLONEDDS_FIXTURES=0 opt-out)"
    elif [ -z "$(nros_riscv64_prefix)" ]; then
        echo "  → skipping ThreadX-RV64 Cyclone fixtures (no riscv64 bare-metal gcc — cannot self-provision Cyclone)"
    else
        # Phase 186 — self-provision Cyclone from source via CMake
        # (nros_provide_cyclonedds() + the ThreadX flags staged in
        # cmake/platform/nano-ros-threadx.cmake); no prebuilt install, no
        # `just cyclonedds threadx-cross-probe`. idlc resolves host-side (186.3).
        #
        # The self-provisioned Cyclone build keeps BUILD_IDLC=OFF, so the host
        # `idlc` (Cyclone DDS IDL compiler) it needs for descriptor codegen is
        # NOT produced here — it must already be on the host, either on PATH
        # (a ROS 2 / CycloneDDS install) or as the project's own
        # build/cyclonedds/bin/idlc (`just cyclonedds setup`). Without it the
        # Cyclone type-support cmake aborts — a fresh configure errors loudly
        # (NrosRmwCycloneddsTypeSupport.cmake), but an incremental build dir with
        # a stale cached idlc path dies as an opaque rc 127 when the missing
        # binary is invoked as a build-time custom command (known-issues #16).
        # Guard here so the absence is an actionable skip, not a 127. Fold the
        # project-local idlc onto PATH when present so the existing cmake
        # find_program(idlc) resolves it for every Cyclone cell unchanged.
        if ! command -v idlc >/dev/null 2>&1 && [ -x "$root/build/cyclonedds/bin/idlc" ]; then
            export PATH="$root/build/cyclonedds/bin:$PATH"
        fi
        if ! command -v idlc >/dev/null 2>&1; then
            echo "  → skipping ThreadX-RV64 Cyclone fixtures: host 'idlc' (Cyclone DDS IDL compiler) not found."
            echo "     Provision it with: just cyclonedds setup   (builds build/cyclonedds/bin/idlc),"
            echo "     or put an idlc on PATH (e.g. a ROS 2 / CycloneDDS install),"
            echo "     or set NROS_THREADX_RV64_CYCLONEDDS_FIXTURES=0 to skip Cyclone fixtures entirely."
        else
            # phase-295 W4 (ex Phase 177.38) — each ThreadX-RV64 Cyclone lang
            # pass bakes ITS allocator domain (alloc::domain_of(ThreadxRiscv64,
            # lang, Pubsub): c=128, cpp=129; rust roles get 127 via the
            # helper's env + -D). A pair (talker+listener) shares its lang's
            # domain; different langs stay SPDP-disjoint.
            export NROS_CMAKE_EXTRA_DEFS="$base_defs -DNROS_DOMAIN_ID=128"
            bash scripts/build/fixtures-build.sh threadx-riscv64 c cyclonedds
            export NROS_CMAKE_EXTRA_DEFS="$base_defs -DNROS_DOMAIN_ID=129"
            bash scripts/build/fixtures-build.sh threadx-riscv64 cpp cyclonedds
            # phase-369 W2 — BOTH RMWs go through this seam now.
            #
            # The zenoh image used to be built by cargo from `src/main.rs`
            # (`fixtures-build.sh threadx-riscv64 rust`), which is what gave
            # these six leaves two entry points and two build paths — issue 0666,
            # and the reason 0688's panic policy was missed on the CMake side.
            # The seam is RMW-neutral since W1, so one loop covers both.
            # issue 0805 — dispatch these CONCURRENTLY, at the same width the
            # C/C++ leaves already get from `fixtures-build.sh`'s `make -j`.
            #
            # This was a plain serial for-loop, and a per-leaf span measurement
            # showed it WAS the critical path: 6-7 leaves ran together for the
            # first ~60 s, then 12 rust builds went one at a time for 256 s of a
            # 331 s wall — 77%, at ~20 s each, on a 32-core box. Every earlier
            # component fix in this issue (cargo recompiles, archive
            # post-processing) shaved overlapped work and moved the wall barely
            # at all, because THIS is what bounded it.
            #
            # Failures must still be fatal: a backgrounded job escapes `set -e`,
            # so each records its own status and the loop below re-raises. A
            # silent pass here would be the "reports PASS on unmet
            # preconditions" failure this repo gates against elsewhere.
            _tx_jobs="$(nros_cargo_frontend_jobs)"
            _tx_status_dir="$(mktemp -d)"
            _tx_n=0
            for _rc in {{RV64_RUST_ROLES}}; do
                if [ -d "examples/qemu-riscv64-threadx/rust/$_rc" ]; then
                    for _tx_pair in "cyclonedds build-cyclonedds" "zenoh build-zenoh"; do
                        # shellcheck disable=SC2086
                        set -- $_tx_pair
                        while [ "$(jobs -rp | wc -l)" -ge "$_tx_jobs" ]; do wait -n || true; done
                        _tx_n=$((_tx_n + 1))
                        (
                            if build_threadx_cmake_rmw \
                                "examples/qemu-riscv64-threadx/rust/$_rc" "$1" "$2"; then
                                : > "$_tx_status_dir/$_rc-$1.ok"
                            else
                                : > "$_tx_status_dir/$_rc-$1.fail"
                            fi
                        ) &
                    done
                fi
            done
            wait
            _tx_ok="$(find "$_tx_status_dir" -name '*.ok' | wc -l)"
            _tx_bad="$(find "$_tx_status_dir" -name '*.fail' | wc -l)"
            rm -rf "$_tx_status_dir"
            if [ "$_tx_bad" -ne 0 ] || [ "$_tx_ok" -ne "$_tx_n" ]; then
                echo "  ThreadX-RV64 rust leaves: $_tx_ok/$_tx_n ok, $_tx_bad failed" >&2
                exit 1
            fi
        fi
    fi
    unset NROS_CMAKE_EXTRA_DEFS

# Build every ThreadX RISC-V 64 tier: core libraries, examples, and test fixtures.
[group("full-matrix")]
build-all: build-fixtures
    @echo "ThreadX RISC-V 64 build-all completed."

# Run ThreadX QEMU RISC-V 64 integration tests
[group("main")]
test verbose="": build-fixtures
    #!/usr/bin/env bash
    bash scripts/dev/deprecated-scope-alias.sh threadx_riscv64 test "just test threadx_riscv64"
    set -e
    if [ ! -d "$THREADX_DIR/common/inc" ]; then
        echo "ERROR: ThreadX not found at $THREADX_DIR. Run: just threadx-linux setup"
        exit 1
    fi
    if [ ! -d "$NETX_DIR/common/inc" ]; then
        echo "ERROR: NetX Duo not found at $NETX_DIR. Run: just threadx-linux setup"
        exit 1
    fi
    rv_config="$(pwd)/packages/boards/nros-board-threadx-qemu-riscv64/config"
    export THREADX_CONFIG_DIR="$rv_config"
    export NETX_CONFIG_DIR="$rv_config"
    just _nextest-platform threadx_riscv64_qemu '{{verbose}}'

# Run all ThreadX RISC-V tests including networked E2E
[group("full-matrix")]
test-all verbose="":
    #!/usr/bin/env bash
    set -e
    source scripts/build/cargo.sh
    cargo_nextest_args=($(nros_cargo_nextest_args))
    args=(-p nros-tests --test threadx_riscv64_qemu --no-fail-fast)
    if [ -z "{{verbose}}" ]; then
        args+=(--success-output never --failure-output never)
    fi
    cargo nextest run "${cargo_nextest_args[@]}" "${args[@]}"
    e2e_args=(-p nros-tests --test rtos_e2e -E 'test(ThreadxRiscv64)' --no-fail-fast)
    if [ -z "{{verbose}}" ]; then
        e2e_args+=(--success-output never --failure-output never)
    fi
    cargo nextest run "${cargo_nextest_args[@]}" "${e2e_args[@]}"

# CI: check prerequisites + test
[group("full-matrix")]
ci: doctor test

# =============================================================================
# Run examples (build + launch QEMU RISC-V 64 virt with slirp networking)
#
# QEMU ignores SIGINT — each recipe runs QEMU in the background and
# traps INT/TERM to kill it explicitly (Ctrl-C works).
# peer_index: 0=talker/server (MAC :56), 1=listener/client (MAC :57)
# =============================================================================

[no-exit-message]
_run-qemu example binary peer_index:
    #!/usr/bin/env bash
    set -e
    rv_config="$(pwd)/packages/boards/nros-board-threadx-qemu-riscv64/config"
    # phase-336 — ambient profile, and the kernel path below derived from it.
    source "{{justfile_directory()}}/scripts/build/cargo.sh"
    # issue 0488 residue 2 — the SHARED group, not a per-leaf `target-zenoh/`.
    # `threadx-riscv64` is migrated, so the fixture lane compiles this exact
    # configuration once for the platform. Flag and `-kernel` path come from ONE
    # `nros_fixture_group` call, so this recipe cannot look where it did not
    # write (phase-340 item 7's failure, on esp32).
    source "{{justfile_directory()}}/scripts/build/fixtures-target-dir.sh"
    TX_DIR="$(nros_cargo_target_profile_dir)"
    TX_ARGS="--no-default-features --features rmw-zenoh"
    TX_LEAF="examples/qemu-riscv64-threadx/rust/{{example}}"
    tx_tdir_flag="$(nros_fixture_target_dir_flag threadx-riscv64 "$TX_ARGS" "")"
    tx_artifacts="$(nros_fixture_row_artifact_dir "$TX_LEAF" threadx-riscv64 "$TX_ARGS" "")"
    cd "{{justfile_directory()}}/$TX_LEAF"
    # shellcheck disable=SC2086
    THREADX_CONFIG_DIR="$rv_config" NETX_CONFIG_DIR="$rv_config" cargo build $(nros_cargo_profile_arg_string) $TX_ARGS $tx_tdir_flag
    MAC="52:54:00:12:34:$(printf '%02x' $((0x56 + {{peer_index}})))"
    trap 'kill $PID 2>/dev/null; wait $PID 2>/dev/null; exit 130' INT TERM
    qemu-system-riscv64 -M virt -m 256M -bios none -nographic \
        -global virtio-mmio.force-legacy=false \
        -kernel "$tx_artifacts/riscv64gc-unknown-none-elf/$TX_DIR/{{binary}}" \
        -netdev user,id=net0 \
        -device "virtio-net-device,netdev=net0,bus=virtio-mmio-bus.0,mac=$MAC" &
    PID=$!
    wait $PID

# Run ThreadX RISC-V talker on QEMU
talker: (_run-qemu "talker" "qemu-riscv64-threadx-talker" "0")

# Run ThreadX RISC-V listener on QEMU
listener: (_run-qemu "listener" "qemu-riscv64-threadx-listener" "1")

# Run ThreadX RISC-V service server on QEMU
service-server: (_run-qemu "service-server" "qemu-riscv64-threadx-service-server" "0")

# Run ThreadX RISC-V service client on QEMU
service-client: (_run-qemu "service-client" "qemu-riscv64-threadx-service-client" "1")

# Run ThreadX RISC-V action server on QEMU
action-server: (_run-qemu "action-server" "qemu-riscv64-threadx-action-server" "0")

# Run ThreadX RISC-V action client on QEMU
action-client: (_run-qemu "action-client" "qemu-riscv64-threadx-action-client" "1")

# Start zenohd router for ThreadX RISC-V examples (allocator rust-pubsub port 9400)
zenohd:
    #!/usr/bin/env bash
    set -e
    source scripts/dev/zenohd.sh
    nros_router_exec tcp/127.0.0.1:9400

# Clean ThreadX RISC-V build artifacts
[group("main")]
clean:
    #!/usr/bin/env bash
    rm -rf build/cmake-threadx-riscv64-zenoh
    for example in {{RV64_RUST_ROLES}}; do
        rm -rf "examples/qemu-riscv64-threadx/rust/$example/target" \
               "examples/qemu-riscv64-threadx/rust/$example/target-zenoh"
        rm -rf "examples/qemu-riscv64-threadx/c/$example/build-zenoh" \
               "examples/qemu-riscv64-threadx/cpp/$example/build-zenoh"
    done
    echo "ThreadX RISC-V build artifacts cleaned"
