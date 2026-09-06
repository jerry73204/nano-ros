# nano-ros gates: cmake
#
# The cmake layer and the formatters -- configure-time behaviour,
# template hygiene, and clang-format/rustfmt agreement.
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

# Check C formatting only (clang-format) — BUILDLESS, source-free → push lane.
[private]
c-fmt:
    #!/usr/bin/env bash
    set -e
    source scripts/dev/clang-format.sh
    CF="$(nros_clang_format)"
    echo "Checking C formatting... ($CF)"
    echo "  - clang-format (nros-c headers)"
    find packages/api/nros-c/include -name '*.h' -not -name 'nros_generated.h' -print0 | xargs -0 "$CF" --dry-run --Werror
    echo "  - clang-format (zpico C)"
    "$CF" --dry-run --Werror packages/rmw/zenoh/zpico-zephyr/src/*.c packages/rmw/zenoh/zpico-zephyr/include/*.h
    echo "  - clang-format (C examples, ALL platforms)"
    git ls-files -z 'examples/*/c/**/*.c' 'examples/*/c/**/*.h' ':!examples/px4/**' | xargs -0 "$CF" --dry-run --Werror
    echo "C formatting OK."

# #310 — gate the in-tree cli sub-workspace's rustfmt-cleanliness (it is outside
# the root workspace that `check-workspace`'s `cargo fmt --check` covers).
#
# issue 0337 — `rustfmt` DIRECTLY, not `cargo fmt`, for the same reason as
# `check-example-fmt`: `cargo fmt` runs `cargo metadata`, which has to resolve
# `ros-launch-manifest-model` (historically through a nested launch submodule;
# since phase-332 a git-tag dep that needs a network fetch). This gate is in
# `check-fast`, which is source-free — inits no submodules, fetches nothing —
# so on CI's push lane resolution failed and the gate died before formatting
# anything. Formatting needs no dependency graph.
#
# Scope matches what `cargo fmt` covered: the workspace MEMBERS only. Test
# fixture crates under `tests/fixtures/` are separate packages, not members,
# and were never formatted by this gate — several are deliberately on older
# editions.
[private]
cli-fmt:
    #!/usr/bin/env bash
    set -euo pipefail
    ws_edition="$(sed -n 's/^edition[[:space:]]*=[[:space:]]*"\([0-9]*\)".*/\1/p' packages/cli/Cargo.toml | head -1)"
    ws_edition="${ws_edition:-2024}"
    # Members, read from the manifest rather than duplicated here.
    members="$(sed -n '/^members = \[/,/^]/p' packages/cli/Cargo.toml \
        | sed -n 's/^[[:space:]]*"\([^"]*\)".*/\1/p')"
    for m in $members; do
        toml="packages/cli/$m/Cargo.toml"
        [ -f "$toml" ] || continue
        edition="$(sed -n 's/^edition[[:space:]]*=[[:space:]]*"\([0-9]*\)".*/\1/p' "$toml" | head -1)"
        edition="${edition:-$ws_edition}"   # `edition.workspace = true` inherits
        mapfile -t files < <(git ls-files "packages/cli/$m/*.rs" "packages/cli/$m/**/*.rs" \
            | grep -v '/tests/fixtures/' || true)
        [ "${#files[@]}" -eq 0 ] && continue
        echo "  fmt $m (edition $edition, ${#files[@]} files)"
        rustfmt "+{{NIGHTLY}}" --check --edition "$edition" "${files[@]}"
    done

# Issue 0726 — `find_program(VAR ...)` is a NO-OP when VAR is already defined,
# so the "initialise then search" spelling never searches. All three sites in
# this tree paired it with a quiet fallback (FetchContent clone, "no SDK
# libstdc++", host-idlc), so none failed loudly.
cmake-find-program-shadowed:
    @python3 scripts/check-cmake-find-program-shadowed.py --self-test
    @python3 scripts/check-cmake-find-program-shadowed.py

# issue 1033 — a RETIRED cmake API keyword must survive in no caller.
#
# Our verbs retire a keyword by keeping it PARSED and raising a FATAL_ERROR
# naming the replacement, so an old caller fails loudly instead of having the
# keyword fall into UNPARSED_ARGUMENTS and become source files nobody can find.
# Right shape, but it only fires when somebody CONFIGURES the caller — and the
# six standalone `examples/zephyr/cpp` leaves are configured by no lane a pull
# request runs. phase-412 retired `ENTITIES`, migrated the six workspace
# packages and missed those six, so all of them (and the 19 fixture rows over
# them, across all three RMWs, since the error precedes the RMW) could not
# configure on main and nothing said so. The retirement is a good mechanism
# reporting to nobody; this is the reader.
#
# The retired set is DISCOVERED from the cmake sources, so the next retirement
# arms this gate by itself and a moved wording makes it BLIND, not permissive.
retired-cmake-keywords:
    @python3 scripts/check-retired-cmake-keywords.py

# issue 1136 — the same question from the GENERATOR's end. The sibling above
# scans tracked cmake for a caller still passing a retired keyword; a generated
# root lives under `build/<coord>/` and is tracked by nothing, so the one caller
# that runs in CI is the one it cannot see. phase-405 W1 dropped `LAUNCH_ARGS`
# from `nano_ros_entry` on a survey that excluded generated CMakeLists, and
# every workspace-fixture configure died on `Cannot find source file:
# LAUNCH_ARGS` — for six days, because host-tests runs on no gating event.
#
# Both sides are literals, so the comparison is static: what `cmake_root.rs`
# emits, against the keyword lists of every cmake frame the call traverses.
[private]
generated-cmake-keywords:
    @python3 scripts/check-generated-cmake-keywords.py

# issue 0719 — the C/C++ half of the same question. `check-image-panic-policy`
# reads what a RUST image declares (`nros::main!(panic = …)`) and says outright
# that it cannot see "the C/C++ side, where the policy is a cargo feature on the
# staticlib". This is that side: every cmake path that links `NanoRos::NanoRos*`
# into an executable must apply an ending, directly or through the entry verbs.
cmake-image-policy:
    @python3 scripts/check-cmake-image-policy.py

# RFC-0054 (phase-299 W3) — the committed bindgen output must match a fresh
# regeneration from the C-header SSoT packages. Loud skip when bindgen-cli
# (pinned; maintainer/CI-only dep) is absent — embedded consumers never
# need it.
# phase-383 W7.e/W7.f (RFC-0065 D12) — `nano_ros_support_library()` must emit the
# force-link flag AND the `LINK_DEPENDS` that gives it a rebuild edge (issue
# 0475), and must route `.ld` fragments to whichever seam the platform has. The
# test configures throwaway projects against the HOST compiler only — no
# nano-ros build, no SDK, seconds — so it belongs on the fast line where every
# task runs it. Loud skip when the host has no cmake/ninja/compiler: `check-fast`
# is documented to run green on a pristine worktree.
cmake-support-library:
    #!/usr/bin/env bash
    set -euo pipefail
    _missing=""
    for _t in cmake ninja nm ar "${CC:-cc}"; do
        command -v "$_t" >/dev/null 2>&1 || _missing="$_missing $_t"
    done
    if [ -n "$_missing" ]; then
        # shellcheck source=scripts/build/check-skip.sh
        source scripts/build/check-skip.sh
        nros_check_skip cmake-support-library \
            "missing host tool(s):$_missing"
        exit 0
    fi
    ./tests/cmake-support-library-tests.sh

# Check C++ formatting only (clang-format) — BUILDLESS, source-free → push lane.
[private]
cpp-fmt:
    #!/usr/bin/env bash
    set -e
    source scripts/dev/clang-format.sh
    CF="$(nros_clang_format)"
    echo "Checking C++ formatting... ($CF)"
    echo "  - clang-format (nros-cpp headers)"
    "$CF" --dry-run --Werror packages/api/nros-cpp/include/nros/*.hpp
    echo "  - clang-format (C++ examples, ALL platforms)"
    git ls-files -z 'examples/*/cpp/**/*.cpp' 'examples/*/cpp/**/*.hpp' ':!examples/px4/**' | xargs -0 "$CF" --dry-run --Werror
    echo "C++ formatting OK."

# Issue 0332 — nros-cpp public headers must not include a hosted STL header
# (`<string>`/`<vector>`/…) outside an `#ifdef NROS_CPP_STD` region. Source-level
# gate: the `-ffreestanding` compile probe runs against the host's full
# libstdc++ and cannot see the 0112 class.
[private]
cpp-freestanding-includes:
    @bash scripts/check-cpp-freestanding-includes.sh

# RFC-0088 D5 / phase-421 W6 — `NROS_SERIALIZATION_FORMAT{,_ID}` names the
# format of THE image's single backend. A bridge image links two and has no
# single answer, so a bridge-linked TU referencing the macro is a wrong answer
# that compiles. Pure source scan (~1 s, no build, no fixture, no SDK), so it
# is affordable on the fast line — and it carries its own vacuity guards, since
# a gate whose subject set emptied reports the same green as one that passed.
[private]
format-macro-scope:
    @python3 scripts/check-format-macro-scope.py

reconfigure-on-change:
    #!/usr/bin/env bash
    set -euo pipefail
    _missing=""
    for _t in cmake ninja; do
        command -v "$_t" >/dev/null 2>&1 || _missing="$_missing $_t"
    done
    if [ -n "$_missing" ]; then
        # shellcheck source=scripts/build/check-skip.sh
        source scripts/build/check-skip.sh
        nros_check_skip reconfigure-on-change "missing host tool(s):$_missing"
        exit 0
    fi
    ./tests/cmake-reconfigure-tests.sh

# Root-workspace rustfmt. `check-example-fmt` and `check-cli-fmt` already sit in
# the fast tier; the ROOT workspace's `fmt --check` was the one left in the build
# tier, so an unformatted file you just wrote survived `check-fast` — which is
# how `model_location.rs` (phase-330 W3.b) shipped needing a reflow.
#
# Buildless and seconds-long: rustfmt parses, it does not compile.
#
# NIGHTLY, always: `rustfmt.toml` enables nightly-only options and stable
# produces different output (CLAUDE.md).
[private]
workspace-fmt:
    cargo +{{NIGHTLY}} fmt --check

# Issue 1118 — nothing may DECLARE a C++ standard below the one nros-cpp needs.
#
# The tree said C++14 in 95 places while `component_node.hpp` used `if
# constexpr`, which is C++17. GCC only warns on that, so the declared minimum
# was a claim the code violated and no build ever failed. The number lived in
# four vocabularies, two of which already disagreed with their own siblings
# (`arm-freertos-armcr52.cmake` at 17 among four toolchains at 14;
# `fvp_entry/prj.conf` at CPP17 beside its workspace's CPP14).
#
# The gate holds NO version number: it reads the floor out of
# `target_compile_features(nros-cpp-headers INTERFACE cxx_std_NN)`, so raising
# the floor is one edit and this follows. Covers `CMAKE_CXX_STANDARD`, the
# `cxx_std_N` feature and Zephyr's `CONFIG_STD_CPPN`; the `-std=` in a
# toolchain's `CMAKE_CXX_FLAGS_INIT` is deliberately out of scope and the
# script's header says why.
#
# A site that consumes no nros-cpp header and whose own build imposes an older
# dialect (uORB inside PX4 at `-std=gnu++14 -Werror`; the Cyclone wrapper on
# Zephyr at `-std=c++11`) carries `# nros-cxx-floor-exempt: <reason>`. The
# reason is required, and the exempt sites print on the GREEN path.
[group("check")]
cxx-standard-floor:
    @python3 scripts/check-cxx-standard-floor.py
