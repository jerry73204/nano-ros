#!/usr/bin/env bash

# Shared Cargo build knobs for broad build recipes.
#
# NROS_CARGO_PROFILE names the cargo profile; unset means the development
# default. The profile -> flags / target-dir derivations are NOT spelled here —
# phase-336 moved them behind `nros profile`, because this file, nros-tests and
# a just literal each carried their own copy and they were free to drift.
#
#   nros profile resolve --build-type <T>   CMAKE_BUILD_TYPE -> profile
#   nros profile args    <profile>          cargo build flags
#   nros profile dir     <profile>          target/ subdirectory
#   nros profile env     <profile>          the definition, as env pairs
#
# NROS_CARGO_FRONTENDS caps independent Cargo frontend processes. The
# compiler work inside each frontend still uses Cargo/rustc's native
# jobserver when MAKEFLAGS carries one.

# nros_scoped_target_dir <suffix>
# A dedicated Cargo target dir that STAYS inside whatever base is active —
# `$CARGO_TARGET_DIR` when set, else `$PWD/target`. issue 0400: recipes that
# hard-set a RELATIVE dir (`CARGO_TARGET_DIR=target-embedded`) override the ROS
# distrobox's `CARGO_TARGET_DIR=$HOME/.cargo-target-box` redirect and land back
# in the shared checkout, so host and box reuse each other's build-script
# binaries and die on a GLIBC mismatch. Rooting the suffix at the active base
# keeps host (`$PWD/target-<suffix>`, unchanged) and box
# (`$HOME/.cargo-target-box-<suffix>`, box-private) from ever sharing a tree.
# Only for EPHEMERAL scratch dirs (clippy/test) with no consumer that reads a
# fixed relative path — fixture dirs consumed at `target-<x>/…` are coupled and
# not fixed this way.
nros_scoped_target_dir() {
    printf '%s' "${CARGO_TARGET_DIR:-$PWD/target}-$1"
}

# Ask the CLI, once per (verb, argument) pair — every helper below is called
# repeatedly inside build loops and the answer cannot change mid-run.
declare -A _NROS_PROFILE_CACHE 2>/dev/null || true
_nros_profile_query() {
    local key="$*"
    if [ -n "${_NROS_PROFILE_CACHE[$key]+set}" ]; then
        printf '%s\n' "${_NROS_PROFILE_CACHE[$key]}"
        return 0
    fi
    local bin out
    bin="$(nros_cli_bin)" || return 1
    out="$("$bin" profile "$@")" || return 1
    _NROS_PROFILE_CACHE[$key]="$out"
    printf '%s\n' "$out"
}

nros_cargo_profile_name() {
    if [ -n "${NROS_CARGO_PROFILE:-}" ]; then
        printf '%s\n' "$NROS_CARGO_PROFILE"
        return 0
    fi
    # No profile named and no CMake build type in play -> the development
    # default, which only the table knows.
    _nros_profile_query resolve --build-type "${CMAKE_BUILD_TYPE:-}"
}

# Word-split form, for `cargo build "${args[@]}"`. Empty for `dev`.
nros_cargo_profile_args() {
    local flags
    flags="$(_nros_profile_query args "$(nros_cargo_profile_name)")" || return 1
    [ -n "$flags" ] && printf '%s\n' $flags
    return 0
}

# The `test-threads` ceiling `.config/nextest.toml` declares.
#
# READ, never copied: that number is the DOMAIN partition's slot count (issue
# 0838 — slot `s` owns Cyclone domains `[s*4, s*4+3]`), and
# `domain_partition_matches_the_nextest_cap` in `nros-tests` already fails if it
# drifts from the Rust constants. A second copy here would be a third place for
# the same fact. Empty if the key is absent, which the caller treats as "no
# ceiling to respect".
nros_nextest_thread_ceiling() {
    local root
    root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
    sed -n 's/^[[:space:]]*test-threads[[:space:]]*=[[:space:]]*\([0-9]\{1,\}\).*/\1/p' \
        "$root/.config/nextest.toml" 2>/dev/null | head -1
}

# This run's share of the machine, as `--test-threads`, or nothing.
#
# `run-gates-parallel.sh` fans out to `nproc` gates and each gate starts its own
# `cargo nextest`, which takes its thread count from `.config/nextest.toml` and
# knows nothing about its 11 siblings. On a 12-core host that is up to 12 x 25
# test threads for 12 cores, and a test whose assertion is a wall-clock RATE
# then measures the load instead of the code: `periodic_timer_fires_repeatedly`
# reported `got 2` for a `>= 4` bound, twice in three runs, passing 3 of 3 idle.
#
# So the runner publishes each gate's share and this is where it lands. Two
# properties matter:
#
#   * It only ever LOWERS. The ceiling above is a CORRECTNESS bound (domain
#     blocks must stay disjoint), never a performance knob, so a share larger
#     than the ceiling is clamped rather than honoured.
#   * It applies only INSIDE a fan-out. `NROS_GATE_CPU_SHARE` is exported by the
#     parallel runner and by nothing else, so `just test-unit`, `just test-all`
#     and a bare `cargo nextest` are unchanged — they own the machine and the
#     config's value is the right one for them.
#
# A caller that passes its own `--test-threads` still wins: these args are
# spliced BEFORE the caller's, and nextest takes the last occurrence.
nros_nextest_cpu_budget_args() {
    local share ceiling
    share="${NROS_GATE_CPU_SHARE:-}"
    [ -n "$share" ] || return 0
    [ "$share" -ge 1 ] 2>/dev/null || return 0
    ceiling="$(nros_nextest_thread_ceiling)"
    if [ -n "$ceiling" ] && [ "$share" -gt "$ceiling" ]; then
        share="$ceiling"
    fi
    printf '%s\n' "--test-threads=$share"
}

nros_cargo_nextest_args() {
    local flags
    flags="$(_nros_profile_query args --nextest "$(nros_cargo_profile_name)")" || return 1
    [ -n "$flags" ] && printf '%s\n' $flags
    nros_nextest_cpu_budget_args
    return 0
}

# Single-string form, for callers that splice into a command line.
nros_cargo_profile_arg_string() {
    _nros_profile_query args "$(nros_cargo_profile_name)"
}

nros_cargo_target_profile_dir() {
    _nros_profile_query dir "$(nros_cargo_profile_name)"
}

# The profile a platform carve-out forces, ignoring the ambient one. Same
# constants the Rust fixture resolvers read — see `nros profile carve-out`.
nros_cargo_nuttx_profile() {
    _nros_profile_query carve-out nuttx-rust
}

nros_cargo_freertos_qemu_profile() {
    _nros_profile_query carve-out freertos-qemu
}

# nros_cargo_platform_profile <platform>
# The profile a PLATFORM's fixture lane builds at: its carve-out when it has
# one, else the ambient profile. Echoes a profile NAME, for
# `NROS_CARGO_PROFILE=` on a `fixtures-build.sh` / `workspace-fixtures-build.sh`
# call.
#
# phase-340 P2 — this mapping existed three times and in three shapes: a helper
# call in `just/freertos.just`, the bare literal `nros-minsizerel` in eight
# `just/nuttx.just` lines, and NOTHING in the staleness probe, which always used
# the ambient profile. The third one is the dangerous one: the probe rebuilds a
# row with its exact flags to ask cargo whether it is fresh, so probing a
# profile the builder never wrote means a full rebuild into a second profile dir
# and a permanent false-STALE — the failure mode of issue 0196 and of phase-226.D's
# original target-dir split. It went unnoticed only because every nuttx row is
# `skip_probe = true` (nightly + build-std), so `freertos` was the first lane to
# move onto a carve-out with the probe watching.
#
# The PLATFORM key is what the probe has (`fixtures-manifest.py list
# --with-platform` prefixes it) — the carve-out NAME is an internal spelling of
# the profile table.
nros_cargo_platform_profile() {
    case "$1" in
        freertos) nros_cargo_freertos_qemu_profile ;;
        nuttx | nuttx-riscv) nros_cargo_nuttx_profile ;;
        *) nros_cargo_profile_name ;;
    esac
}

# Flags for a NAMED profile (a carve-out), rather than the ambient one.
#
# Emits ONE FLAG PER LINE, like `nros_cargo_profile_args` — so a caller can
# `mapfile` it straight into an argv array. The table stores them as a single
# string (`--profile nros-minsizerel`), and a caller that mapfiles the raw query
# gets that as ONE argv element, which cargo rejects:
#
#     error: unexpected argument '--profile nros-minsizerel' found
#     tip: a similar argument exists: '--profile'
#
# That is exactly what broke the NuttX workspace fixtures: the nuttx carve-out
# in `workspace-fixtures-build.sh` called `_nros_profile_query args` directly
# while every other path went through the ambient helper, which splits. The
# split now lives with the accessor rather than in each caller.
nros_cargo_profile_args_for() {
    local flags
    flags="$(_nros_profile_query args "$1")" || return 1
    # Unquoted on purpose: word-splitting is the point.
    # shellcheck disable=SC2086
    [ -n "$flags" ] && printf '%s\n' $flags
    return 0
}

# Target subdirectory for a NAMED profile.
nros_cargo_target_profile_dir_for() {
    _nros_profile_query dir "$1"
}

# The profile's DEFINITION as `KEY=VALUE` lines — empty unless nano-ros owns
# the profile (see the ownership rule in `nros-cargo-profile`). Callers export
# these so a workspace with no `[profile.*]` block still resolves the name.
nros_cargo_profile_env() {
    _nros_profile_query env "$(nros_cargo_profile_name)"
}

# The size probe's target dir (`nros-sizes-build`). One SHARED dir instead of
# one per OUT_DIR.
#
# The probe compiles the crate under test in an isolated target dir, because a
# nested cargo pointed at the outer dir deadlocks on cargo's exclusive flock.
# Isolation means no artifact sharing, so the default (`$OUT_DIR/…`) rebuilt the
# same ~60-crate graph once per consumer per build dir — measured at ~420 MB and
# a full cold compile each, ~30 times in a native lane.
#
# Sharing keeps the probe's feature-exact nested build (so the sizes are still
# correct by construction, not by mtime luck); only the CACHE is reused.
# `nros-sizes-build` appends the rustc slug itself, so two toolchains cannot mix
# rmeta in here.
#
# NOT `nros_scoped_target_dir`: that roots at `$PWD`, and the fixture builders
# invoke cargo from inside each example dir, so every leaf would get its own
# "shared" dir. Root at the active CARGO_TARGET_DIR when set (keeps the ROS
# distrobox's box-private tree separate from the host's — issue 0400), else at
# the repo.
nros_sizes_probe_dir() {
    if [ -n "${CARGO_TARGET_DIR:-}" ]; then
        printf '%s' "${CARGO_TARGET_DIR}-sizes-probe"
        return 0
    fi
    # An INHERITED `NROS_REPO_DIR` may name a different checkout.
    #
    # It is exported by `activate.sh` (phase-218.C), so a shell that activated
    # another nano-ros — or a git worktree, the case `build-root.sh` already
    # warns about — carries it in. This used to be trusted blindly and then
    # `source`d, so pointing at a checkout without `scripts/build/build-root.sh`
    # produced a raw bash error plus `nros_build_dir: command not found` on
    # stderr. Harmless to the build, but it is UNCONDITIONAL noise: any gate
    # asserting a command prints nothing then fails on output it did not cause,
    # and the failure names the gate rather than the stale variable.
    #
    # So validate the inherited value the same way `build-root.sh` reasons about
    # its own fallback: `scripts/build/` is two levels below the root by
    # construction, so deriving from `BASH_SOURCE` always names the checkout
    # this file was read from.
    local repo_root="${NROS_REPO_DIR:-}"
    if [ -n "$repo_root" ] && [ ! -f "$repo_root/scripts/build/build-root.sh" ]; then
        repo_root=""
    fi
    if [ -z "$repo_root" ]; then
        local _self="${BASH_SOURCE[0]:-$0}"
        repo_root="$(cd "$(dirname "$_self")/../.." 2>/dev/null && pwd)" || repo_root=""
    fi
    if [ -n "$repo_root" ] && [ -f "$repo_root/scripts/build/build-root.sh" ]; then
        # phase-334 W2.b step 2 — derived, not a second spelling of the root.
        source "$repo_root/scripts/build/build-root.sh"
        printf '%s' "$(nros_build_dir "$NROS_KIND_SIZES_PROBE")"
    fi
}

# Export it for every child (cargo, and the cmake/corrosion builds that spawn
# cargo). An explicit setting from the caller always wins.
if [ -z "${NROS_SIZES_PROBE_TARGET_DIR:-}" ]; then
    _nros_probe_dir="$(nros_sizes_probe_dir)"
    [ -n "$_nros_probe_dir" ] && export NROS_SIZES_PROBE_TARGET_DIR="$_nros_probe_dir"
    unset _nros_probe_dir
fi

nros_cargo_frontend_jobs() {
    local jobs="${NROS_CARGO_FRONTENDS:-}"
    if [ -z "$jobs" ]; then
        if [ "${NROS_JOBSERVER:-}" = "1" ]; then
            jobs=4
        else
            jobs="${NROS_BUILD_JOBS:-8}"
        fi
    fi
    if ! [[ "$jobs" =~ ^[0-9]+$ ]] || [ "$jobs" -lt 1 ]; then
        echo "Invalid NROS_CARGO_FRONTENDS=$jobs; expected positive integer" >&2
        return 2
    fi
    printf '%s\n' "$jobs"
}

# How wide may EACH cargo run, when N of them run side by side under a fan-out
# that is NOT a jobserver?
#
# `scripts/build/jobserver-pool.sh` explains why a jobserver is the right answer
# when the units spawn their own parallel tools: `-P N` MULTIPLIES, because each
# unit's cargo assumes the whole machine. Where a pool is available that reason
# stands and this helper is not what you want.
#
# The example lanes fan out with GNU parallel instead, and cannot use a pool:
# it needs pinned make 4.4 for `--jobserver-style=fifo` (the system 4.3's
# pipe-based jobserver cannot be joined by child tools), which is a provisioned
# dependency those lanes deliberately do not carry. So they get the arithmetic
# instead of the token pool — divide the budget up front rather than let every
# frontend claim all of it.
#
# Accepts what GNU parallel's `-j` accepts (an integer or a "NN%" of cores) so
# one spec drives both halves and they cannot drift apart.
#
# Only for the no-jobserver case. Under NROS_JOBSERVER=1 pass NOTHING: cargo is
# itself a jobserver client, an explicit `-j` OVERRIDES the inherited pool, and
# capping there would fight the very mechanism that already bounds the total.
nros_cargo_inner_jobs() {
    local spec="${1:-}" cores frontends inner
    cores="$(nproc 2>/dev/null || echo 4)"
    case "$spec" in
        *%)          frontends=$(( cores * ${spec%\%} / 100 )) ;;
        ''|*[!0-9]*) frontends="$cores" ;;
        *)           frontends="$spec" ;;
    esac
    [ "$frontends" -lt 1 ] && frontends=1
    inner=$(( cores / frontends ))
    [ "$inner" -lt 1 ] && inner=1
    printf '%s\n' "$inner"
}

nros_cmake_frontend_jobs() {
    local jobs="${NROS_CMAKE_FRONTENDS:-}"
    if [ -z "$jobs" ]; then
        if [ "${NROS_JOBSERVER:-}" = "1" ]; then
            jobs=4
        else
            jobs="${NROS_BUILD_JOBS:-4}"
        fi
    fi
    if ! [[ "$jobs" =~ ^[0-9]+$ ]] || [ "$jobs" -lt 1 ]; then
        echo "Invalid NROS_CMAKE_FRONTENDS=$jobs; expected positive integer" >&2
        return 2
    fi
    printf '%s\n' "$jobs"
}

nros_cargo_fetch_root() {
    cargo fetch --locked
}

# NOTE (issue 0363): the freshness guard that used to live here is GONE, and
# deliberately not replaced. It compared mtimes and only covered callers that
# reached the CLI through this function — so `nros sync` straight off PATH, the
# documented recovery step, bypassed it, and CMake's `find_program(nros)` never
# saw it either.
#
# The binary now checks ITSELF before dispatching a guarded verb
# (`nros-cli-core/src/stale_guard.rs`), which covers every invocation style
# including the two this function could not. Re-adding a check here would be a
# second spelling of one predicate, which is what issue 0363 set out to remove.

nros_cli_bin() {
    # Phase 218.D.3 — resolution order:
    #   1. $NROS_CLI                                — explicit override
    #   2. nros on PATH                             — activate.sh / shell env
    #   3. packages/cli/target/release/nros         — per-checkout binary (preferred)
    #   4. ${NROS_HOME:-~/.nros}/bin/nros           — transitional, pre-218 install
    # Per-checkout wins over ~/.nros/bin so each worktree carries its own
    # CLI, no global PATH skew across trees.
    if [ -n "${NROS_CLI:-}" ]; then
        if [ -x "$NROS_CLI" ]; then
            printf '%s\n' "$NROS_CLI"
            return 0
        fi
        echo "NROS_CLI points to a non-executable path: $NROS_CLI" >&2
        return 2
    fi
    if command -v nros >/dev/null 2>&1; then
        local _path_nros
        _path_nros="$(command -v nros)"
        printf '%s\n' "$_path_nros"
        return 0
    fi
    # Per-checkout binary at packages/cli/target/release/nros. Use
    # $NROS_REPO_DIR (exported by activate.sh, Phase 218.C) when set;
    # otherwise walk up from this script to find the repo root so callers
    # without activate.sh sourced still resolve correctly.
    local repo_root="${NROS_REPO_DIR:-}"
    if [ -z "$repo_root" ]; then
        # This file lives at <repo>/scripts/build/cargo.sh.
        local _self
        _self="${BASH_SOURCE[0]:-$0}"
        if [ -n "$_self" ]; then
            repo_root="$(cd "$(dirname "$_self")/../.." 2>/dev/null && pwd)" || repo_root=""
        fi
    fi
    if [ -n "$repo_root" ] && [ -x "$repo_root/packages/cli/target/release/nros" ]; then
        printf '%s\n' "$repo_root/packages/cli/target/release/nros"
        return 0
    fi
    local home_nros="${NROS_HOME:-$HOME/.nros}/bin/nros"
    if [ -x "$home_nros" ]; then
        printf '%s\n' "$home_nros"
        return 0
    fi
    echo "nros CLI not found." >&2
    echo "Run: just setup-cli   (builds packages/cli/target/release/nros), or" >&2
    echo "Set NROS_CLI=/path/to/nros." >&2
    return 2
}

# Phase 195.D: the codegen host tool is the canonical, *installed* `nros`
# binary (`nros codegen …`) — resolved from $NROS_CLI / PATH / ~/.nros, NOT
# built from the packages/codegen submodule target dir. The standalone
# `nros-codegen` (nros-codegen-c) was merged into `nros codegen` in 195.A.
# Function names keep `codegen_c` for callsite stability; the returned path is
# absolute, so recipes use it directly (no `$(pwd)/`/`$root/` prefix).
nros_cargo_codegen_c_bin() {
    nros_cli_bin
}

nros_cargo_ensure_codegen_c() {
    # Installed binary — nothing to build. Resolve it so callers fail loudly
    # (with install guidance) instead of passing an empty -D_NANO_ROS_CODEGEN_TOOL.
    nros_cargo_codegen_c_bin >/dev/null
}

# Phase 214.I.2 — probe whether the installed `nros` CLI exposes the `sync`
# verb (added post-0.3.7 by Phase 210.D.1 / 210.E.3.d.native; promoted from
# `nros sync` to top-level `nros sync` by phase-265 W5). The shipped 0.3.7
# release predates it; without this guard every fixture-build recipe cascades
# into a noisy `clap` "unrecognized subcommand" stack.
#
# Returns 0 if `sync` is available, 1 otherwise. Argument: optional path
# to the `nros` binary (defaults to `$(nros_cli_bin)`). Accepts either the
# new top-level `nros sync` or the deprecated `nros sync` alias.
nros_cli_ws_sync_available() {
    local bin="${1:-}"
    if [ -z "$bin" ]; then
        bin="$(nros_cli_bin 2>/dev/null)" || return 1
    fi
    [ -x "$bin" ] || return 1
    # Top-level `nros sync` (phase-265) OR the hidden `nros sync` alias. The
    # grep on a failure path returns 1 too, so the chained pipes are safe.
    "$bin" help 2>/dev/null | grep -q '^[[:space:]]*sync\b' ||
        "$bin" help ws 2>/dev/null | grep -q '^[[:space:]]*sync\b'
}

# Phase 214.I.2 — fail-loud guard. Call once at the top of any recipe /
# script section that will invoke `nros sync`. On success: silent.
# On failure: emits a `[PREREQ]` one-liner naming the missing verb and
# exits 0 (skip, not fail) so a pre-pin checkout doesn't bury the build
# in cargo / clap stack traces. Honors the same NROS_CLI / PATH / ~/.nros
# resolution as `nros_cli_bin`.
nros_require_sync() {
    local bin="${1:-}"
    if [ -z "$bin" ]; then
        # No arg: resolve fresh. `nros_cli_bin` now runs the STALE guard itself
        # (see `_nros_cli_assert_fresh`) and fails loud + non-zero on a stale
        # in-tree CLI — do NOT suppress it here (issue #197). A pre-resolved `$1`
        # was already fresh-checked by the caller's own `nros_cli_bin`.
        bin="$(nros_cli_bin)" || exit 3
    fi
    if nros_cli_ws_sync_available "$bin"; then
        return 0
    fi
    # issue #181 — a fixture build REQUIRES `nros sync` to generate the
    # per-example `generated/` msg crates; without it the rust lane produces
    # NOTHING. Exiting 0 here silently skipped the whole rust half of the sweep
    # (freertos/threadx-linux/native), which then surfaced downstream as
    # `test-all` reds ("fixture not prebuilt") that look like runtime bugs. Fail
    # LOUD instead: a stale/wrong CLI is an actionable setup error, not a
    # skippable lane.
    echo "[ERROR] nros sync verb unavailable (CLI at '$bin' lacks Phase 210.D.1 / 210.E.3.d.native; rebuild via 'just setup-cli', or set \$NROS_CLI to a binary that carries the verb). Cannot build rust fixtures without it — failing loud (issue #181) rather than skipping the lane and failing downstream tests." >&2
    exit 1
}

nros_cargo_fetch_standalone_manifests() {
    local manifest
    local manifest_dir
    local list
    list="$(mktemp "${TMPDIR:-/tmp}/nros_cargo_fetch.XXXXXX")"
    trap 'rm -f "$list"' RETURN

    # `examples/templates/**` are copy-out recipes, not built by any fixture
    # row or broad-build recipe (absent from examples/fixtures.toml and
    # build-all.mk). Some (e.g. multi-node-workspace) carry a gitignored
    # `[patch.crates-io]` path-dep on `generated/<msg-crate>` that only
    # `nros sync` materialises, so `cargo fetch` here would hard-fail on a
    # missing manifest. Skip them — same rationale as the `examples/zephyr/**`
    # exclusion (known-issues #14).
    rg --files \
        examples \
        packages/testing/nros-tests/bins \
        packages/testing/nros-bench \
        -g Cargo.toml \
        -g '!examples/zephyr/**' \
        -g '!examples/templates/**' \
        -g '!**/target/**' \
        -g '!**/generated/**' \
        -g '!**/build/**' \
        -g '!**/build-*/**' \
        -g '!**/_deps/**' \
        | sort > "$list"

    while IFS= read -r manifest; do
        manifest_dir="$(dirname "$manifest")"
        if [ -f "$manifest_dir/Cargo.lock" ]; then
            # No --locked: these are standalone examples/fixtures whose
            # Cargo.lock is gitignored (not reproducibility-critical), and a
            # clean+setup can leave them stale (deps shrank/bumped). `--locked`
            # made the prefetch hard-fail ("cannot update the lock file …")
            # instead of refreshing them; this prefetch is just cache-warming
            # for the offline fanout, so allow the lock to refresh here.
            ( cd "$manifest_dir" && cargo fetch --quiet )
        fi
    done < "$list"
}

# issue 1038 — ensure the central `nros-patch.toml` exists.
#
# 47 tracked leaf `.cargo/config.toml` files reach the universal-trio patches
# through `include = ["…/nros-patch.toml"]`, and a missing include target is a
# HARD cargo error during MANIFEST PARSE — before any compilation, four frames
# deep, naming neither the leaf nor `nros sync`.
#
# Only `nros sync` wrote that file, and sync needs a WORKSPACE. A lane building
# standalone leaves has no workspace to sync, so it got the file as a SIDE
# EFFECT of whichever neighbouring lane ran `workspace-fixtures-build.sh` first.
# Six lanes do; `threadx-riscv64` does not, which is why its leaves were the
# ones that failed while an identical config in a freertos leaf resolved.
#
# `nros ws central-patch` is the SAME writer sync calls, so the table cannot
# drift from sync's. Idempotent — skip-write when unchanged.
#
# A shell function rather than a `just` recipe because `just` MODULES cannot
# depend on a root-level recipe, and the alternative was spelling these two
# lines once per platform module.
nros_ensure_central_patch() {
    NROS_REPO_DIR="${NROS_REPO_DIR:-$PWD}" "$(nros_cli_bin)" ws central-patch >/dev/null
}

# issue 1038 follow-up — ensure ONE leaf's `generated/<pkg>` message crates
# exist, not just the central include target above.
#
# `nros_ensure_central_patch` closes the manifest-parse failure for a leaf
# whose `.cargo/config.toml` only reaches `nros-patch.toml`. A leaf that ALSO
# patches a message crate (any `std_msgs = { path = "generated/std_msgs" }`
# row — RFC-0067) fails the SAME way one path deeper: cargo cannot read a
# `generated/` dir that was never materialised, central table or not. Measured
# by reproducing it (`cargo metadata` on `examples/qemu-riscv64-threadx/rust/
# talker` after `nros ws central-patch` alone: parse still fails, now on
# `generated/std_msgs/Cargo.toml`).
#
# Every OTHER lane's rust leaves reach `fixtures-build.sh`'s cargo-row path,
# which pre-syncs each row directory once before it fans out
# (`nros_presync_row_dirs`). `threadx-riscv64`'s six rust roles build through
# Corrosion/CMake instead (`nros_threadx_rv64_rust_app` /
# `nros_cmake_fixture_build`), a seam `fixtures-build.sh` never runs for them,
# so nothing there ever synced them.
#
# `nros sync <dir>` is a strict superset of `ws central-patch` — same central
# table, plus the leaf's own `generated/` crates — so this is the ONE call a
# CMake-driven message-dependent leaf needs, not two. Idempotent.
nros_ensure_leaf_synced() {
    local dir="$1"
    NROS_REPO_DIR="${NROS_REPO_DIR:-$PWD}" "$(nros_cli_bin)" sync "$dir" >/dev/null
}
