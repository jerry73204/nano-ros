set working-directory := '..'

default:
    @just --list --justfile {{source_file()}}

# Install XRCE prerequisites: build Micro-XRCE-DDS Agent from source.
[group("setup")]
setup:
    #!/usr/bin/env bash
    set -e
    # phase-334 W2.b step 2 — path from the ONE derivation (RFC-0070 R3).
    source scripts/build/build-root.sh
    # Issue 0741 — ALWAYS delegate. This used to short-circuit on
    # `[ -x "$adir/MicroXRCEAgent" ]`, which made the agent's own freshness
    # logic unreachable on exactly the hosts that need it.
    #
    # `scripts/xrce-agent/build.sh` already keys its stamp on
    # "$agent_ref $ros_prefix" and rebuilds when either moves. That stamp is
    # what issue 0741's 2026-08-24 mitigation (`ca224e271`) is made of: build
    # the agent against the SOURCED ROS's own Fast-CDR, because an agent
    # carrying a different Fast-CDR MAJOR mis-frames the 24-byte SampleIdentity
    # on a service reply. A file-existence test cannot see a ROS prefix change,
    # so any host that had ever published an agent kept the skewed one and the
    # mitigation never ran there -- including this one, still carrying the
    # pre-mitigation wrapper dated Aug 23.
    #
    # The recipe must not re-implement that decision: build.sh prints
    # "already built" itself when the stamp matches, so the fast path is
    # preserved without a second, weaker copy of the rule.
    ./scripts/xrce-agent/build.sh

# Diagnose XRCE install status (read-only).
[group("setup")]
doctor:
    #!/usr/bin/env bash
    bash scripts/dev/deprecated-scope-alias.sh xrce doctor "just doctor xrce"
    set +e
    source scripts/build/build-root.sh
    adir="$(nros_build_dir "$NROS_KIND_XRCE_AGENT")"
    if [ -x "$adir/MicroXRCEAgent" ]; then
        echo "  [OK] Micro-XRCE-DDS Agent: $adir/MicroXRCEAgent"
        exit 0
    else
        echo "  [MISSING] Micro-XRCE-DDS Agent (run: just xrce setup)"
        exit 1
    fi

# Build Micro-XRCE-DDS Agent from source (alias for `just xrce setup`).
[group("main")]
build-agent:
    ./scripts/xrce-agent/build.sh

# Clean XRCE Agent build
[group("main")]
clean-agent:
    ./scripts/xrce-agent/build.sh --clean

# Phase 140 — the legacy `build-rmw` recipe (standalone CMake build
# + install) is gone. The Rust-side `nros-rmw-xrce-cffi` shim is
# what the root CMake xrce branch consumes; build it via cargo or
# via the root cmake configure with NANO_ROS_RMW=xrce. The
# header-only `nros-rmw-xrce` C++ project under
# packages/rmw/xrce/nros-rmw-xrce/ keeps its CTest harness for ad-hoc
# in-tree validation.

# Run XRCE-DDS integration tests (requires: just xrce build-agent)
[group("main")]
test verbose="":
    #!/usr/bin/env bash
    bash scripts/dev/deprecated-scope-alias.sh xrce test "just test xrce"
    set -e
    source scripts/build/cargo.sh
    cargo_nextest_args=($(nros_cargo_nextest_args))
    args=(-p nros-tests --no-fail-fast -E 'binary(xrce)')
    if [ -z "{{verbose}}" ]; then
        args+=(--success-output never --failure-output never)
    fi
    # `nros_tests::skip!` panics with `[SKIPPED]` for unmet preconditions
    # (e.g. xrce agent not present). Nextest records those as failures.
    # Phase 214.R.1 rewrites them to <skipped> + the real-failure tally
    # promotes a SKIPPED-only run to a pass.
    set +e
    cargo nextest run "${cargo_nextest_args[@]}" "${args[@]}"
    rc=$?
    set -e
    just _rewrite-skipped-junit || true
    [ $rc -eq 0 ] && exit 0
    real="$(just _count-real-failures)"
    just _test-summary || true
    if [ "$real" -ne 0 ]; then
        echo "ERROR: $real real (non-[SKIPPED]) test failure(s)."
        exit 1
    fi
    echo "All failures were [SKIPPED] preconditions — treating as pass."

# XRCE ↔ ROS 2 DDS interop tests (needs XRCE Agent + ROS 2 + rmw_fastrtps)
[group("debug")]
test-ros2 verbose="":
    #!/usr/bin/env bash
    set -e
    source scripts/build/cargo.sh
    cargo_nextest_args=($(nros_cargo_nextest_args))
    args=(-p nros-tests --no-fail-fast -E 'binary(xrce_ros2_interop)')
    if [ -z "{{verbose}}" ]; then
        args+=(--success-output never --failure-output never)
    fi
    set +e
    cargo nextest run "${cargo_nextest_args[@]}" "${args[@]}"
    rc=$?
    set -e
    # Phase 214.R.1: SKIPPED → <skipped> + skip-aware tail.
    just _rewrite-skipped-junit || true
    [ $rc -eq 0 ] && exit 0
    real="$(just _count-real-failures)"
    just _test-summary || true
    if [ "$real" -ne 0 ]; then
        echo "ERROR: $real real (non-[SKIPPED]) test failure(s)."
        exit 1
    fi
    echo "All failures were [SKIPPED] preconditions — treating as pass."

# C XRCE-DDS API integration tests (needs cmake + XRCE Agent)
[group("debug")]
test-c verbose="":
    #!/usr/bin/env bash
    set -e
    source scripts/build/cargo.sh
    cargo_nextest_args=($(nros_cargo_nextest_args))
    args=(-p nros-tests --no-fail-fast -E 'binary(c_xrce_api)')
    if [ -z "{{verbose}}" ]; then
        args+=(--success-output never --failure-output never)
    fi
    set +e
    cargo nextest run "${cargo_nextest_args[@]}" "${args[@]}"
    rc=$?
    set -e
    # Phase 214.R.1: SKIPPED → <skipped> + skip-aware tail.
    just _rewrite-skipped-junit || true
    [ $rc -eq 0 ] && exit 0
    real="$(just _count-real-failures)"
    just _test-summary || true
    if [ "$real" -ne 0 ]; then
        echo "ERROR: $real real (non-[SKIPPED]) test failure(s)."
        exit 1
    fi
    echo "All failures were [SKIPPED] preconditions — treating as pass."

# Issue 1097 — the `check-rust-rmw` recipe is GONE. It ran `cargo check` in
# `packages/rmw/xrce/nros-rmw-xrce`, and that directory has never held a
# `Cargo.toml` in this repo's history: the crate was deleted 2026-05-12
# (`f5aeb8faf`, phase-115.K.2) and the recipe was written 2026-05-13
# (`68c89568c`, phase-121.7.i), whose roadmap entry claims "CI now exercises
# it". It was born dead and stayed that way for four months, because it was
# `[group("debug")]`, on no lane, and named by nothing.
#
# What it looked like it did is not what it did. `cargo` walks UP for a
# manifest, so the missing `Cargo.toml` never produced "could not find
# Cargo.toml" — cargo resolved the ROOT workspace and the recipe became a
# full-workspace `cargo check`, i.e. everything EXCEPT the xrce crate it
# claimed to check (`nros-rmw-xrce-cffi` is in `HOST_UNCHECKABLE`). Measured:
# it dies on `nros-c`'s duplicate `panic_impl` lang item, a red that says
# nothing about XRCE.
#
# The coverage it named — "catch drift after platform-API or trait-surface
# changes" — is `just check rmw-xrce` (on the `build-serial` lane): it runs
# `cargo build -p nros-rmw-xrce-cffi`, then cmake-builds this C project
# against that lane's own archive and runs its CTests. Respelling this as
# `cargo check -p nros-rmw-xrce-cffi` would only be a second, weaker spelling
# of that gate's first step.
#
# Gate: `check-just-recipe-paths` — a literal path a recipe names must exist
# and carry the manifest its tool needs.

# Clean XRCE build artifacts
[group("main")]
clean: clean-agent
