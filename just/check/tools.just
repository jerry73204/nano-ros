# nano-ros gates: tools
#
# Behind `just check` but asserting nothing -- developer tools.
#
# One of the topic files `just/check.just` imports. The names stay FLAT -- a
# gate is still `just check <name>` -- because `import` merges definitions
# rather than namespacing them, and ~350 references across docs, scripts and
# workflows spell them that way.
#
# Adding a gate here does not touch any other file, which is the point: for
# most of this repo's life every new gate appended to ONE file, and
# `just/check.just` became the worst conflict surface in the tree at 48 of
# 300 commits.

# Quarantine registry gate — every entry has an open issue and an unexpired
# review date. Expiry is a HARD failure on purpose: quarantine without expiry is
# deletion with extra steps.
[private]
flake-quarantine:
    @python3 scripts/test/quarantine.py --check

# Phase 392 W1 — the declared arithmetic has to survive contact with a linker.
# `check-pool-inventory` proves the FORMULA is present and evaluates; this proves
# it equals the bytes the compiler actually reserved. Source-only here (the
# selftest), because `check-build` runs before any fixture exists; the
# fixture-backed comparison is `static_memory_declared_pools`, which resolves a
# built image through the harness and so inherits the staleness probe.
[private]
mem-report:
    @python3 scripts/nros-mem-report.py --selftest

# Check Python code: formatting + linting with ruff
[private]
python:
    @echo "No in-tree Python package to check (nros-cli owns the colcon extension)."

# Issue 0882 / the stale-`build.ninja` class -- a manifest error is raised at
# LOAD, before any rule runs, so ninja can never re-run cmake to repair itself.
# The dir is wedged and every ninja invocation in it reports the same error
# whatever you ask for. `just reconfigure-stale` repairs such dirs in place by
# re-running cmake; it never deletes one, because a wedged build dir is the
# only reproduction of whatever wedged it.
#
# This gate is the NEGATIVE CONTROL for that script. On a healthy tree the
# script prints "OK, N build dir(s) load", which is exactly what a probe that
# can never fail would print -- so the check that matters is whether it still
# detects a dir that is genuinely unloadable. Real cmake + real ninja,
# `project(... NONE)`, no compiler, ~2 s.
reconfigure-stale:
    #!/usr/bin/env bash
    set -euo pipefail
    _missing=""
    for _t in cmake ninja; do
        command -v "$_t" >/dev/null 2>&1 || _missing="$_missing $_t"
    done
    if [ -n "$_missing" ]; then
        # shellcheck source=scripts/build/check-skip.sh
        source scripts/build/check-skip.sh
        nros_check_skip reconfigure-stale "missing host tool(s):$_missing"
        exit 0
    fi
    ./tests/cmake-reconfigure-stale-tests.sh

# Analyze per-function stack usage (requires nightly + llvm-tools)
# Usage: just check stack [example-dir] [top]
# Default: examples/qemu/rs-wcet-bench, top 30
[group("debug")]
stack example="packages/testing/nros-bench/wcet-cycles-qemu" top="30":
    ./scripts/stack-analysis.sh {{example}} --top {{top}}

# Analyze stack usage of all examples (requires nightly + llvm-tools + cmake)
# Covers: QEMU ARM, native Rust, and native C examples
# ESP32/STM32F4 excluded (need platform-specific SDKs)
[group("debug")]
stack-all top="10":
    #!/usr/bin/env bash
    set -euo pipefail
    failed=0
    # Rust examples (QEMU ARM — no exclude, show full picture)
    for example in \
        packages/testing/nros-bench/wcet-cycles-qemu \
        packages/testing/nros-tests/bins/cdr-roundtrip-qemu \
        examples/qemu-arm-baremetal/rust/talker \
        examples/qemu-arm-baremetal/rust/listener \
    ; do
        echo "================================================================"
        ./scripts/stack-analysis.sh "$example" --top {{top}} || { echo "[FAIL] $example"; failed=$((failed + 1)); }
        echo ""
    done
    # Rust examples (native — exclude tracing/regex infrastructure noise)
    for example in \
        examples/native/rust/talker \
        examples/native/rust/listener \
        examples/native/rust/custom-msg \
        examples/native/rust/service-server \
        examples/native/rust/service-client \
        examples/native/rust/action-server \
        examples/native/rust/action-client \
    ; do
        echo "================================================================"
        ./scripts/stack-analysis.sh "$example" --top {{top}} --exclude "regex_automata|regex_syntax|aho_corasick|env_filter|env_logger|driftsort" || { echo "[FAIL] $example"; failed=$((failed + 1)); }
        echo ""
    done
    # C examples (native)
    for example in \
        examples/native/c/talker \
        examples/native/c/listener \
        examples/native/c/custom-msg \
        examples/native/c/custom-transport-loopback \
    ; do
        echo "================================================================"
        ./scripts/stack-analysis-c.sh "$example" --top {{top}} || { echo "[FAIL] $example"; failed=$((failed + 1)); }
        echo ""
    done
    if [ "$failed" -gt 0 ]; then
        echo "[WARN] $failed example(s) failed"
        exit 1
    fi
    echo "[OK] All stack analyses complete"

# Analyze stack usage of C examples (requires cmake + gcc)
# Usage: just check stack-c [example-dir] [top]
# Default: examples/native/c/talker, top 30
[group("debug")]
stack-c example="examples/native/c/talker" top="30":
    ./scripts/stack-analysis-c.sh {{example}} --top {{top}}

# Analyze stack usage of a pre-built ELF (e.g. Zephyr west build output)
# Usage: just check stack-elf <path-to-elf> [top]
[group("debug")]
stack-elf elf top="30":
    ./scripts/stack-analysis.sh --elf {{elf}} --top {{top}}

# Issue 1052 — the ESP32 stack is the LINKER LEFTOVER after `.bss`
# (`_stack_start - _stack_end`), so every byte a static gains is a byte the
# stack loses, and there is no runtime overflow guard on the target: the image
# writes frames into `.bss` and dies later as a wild jump somewhere unrelated.
# `nros-board-esp32-qemu/src/node.rs` has instructed "check `.stack` after
# changing ANY large static" since issue 0190; a comment cannot be run on a
# schedule, and the talker duly shipped with 18,572 B against a ~67 KB budget.
#
# Only the artifact-free half runs here — the real per-image assertion lives in
# `just esp32 build-qemu`, on the ELF that step just linked. Resolving a built
# image from a `check` lane would need a fixture (banned) or would skip when
# absent, and a skip is exactly the silence this gate exists to end.
[private]
stack-floor:
    @python3 scripts/check-stack-floor.py --selftest
    @python3 scripts/check-stack-floor.py --claims

# Issue 0466 — report EVERY unmet tier precondition at once (CLI stamp, leaf
# includes, build sources, fixtures for the lane) instead of one per ~40-minute
# attempt. Run at the head of `ci`; callable directly to check a tree before
# committing to a run.
tier-preconditions:
    @bash scripts/check-tier-preconditions.sh
