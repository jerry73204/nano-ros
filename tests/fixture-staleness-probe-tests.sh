#!/bin/bash
# tests/fixture-staleness-probe-tests.sh -- issue 0835
#
# Gates the DECISION RULE of the two differential fixture staleness probes:
#
#   scripts/test/cmake-fixture-stale.sh    (120 c/cpp manifest rows)
#   scripts/test/rust-fixture-stale.sh     (117 cargo-builder rows)
#
# THE CONTRACT: a probe's verdict is a function of the ARTIFACT BYTES, never of
# whether the build did work.
#
# WHY THAT SENTENCE IS THE WHOLE ISSUE. Both probes used to ask "did the build
# do work?", and for these fixtures the answer is permanently yes, for reasons
# that are by design and will not go away:
#
#   * a cmake cell links nros-c / nros-cpp through Corrosion, whose cargo step
#     is an always-dirty custom command feeding the link, so ninja recompiles
#     and relinks on EVERY build;
#   * cargo rows share one `--target-dir` per platform (the phase-340 group) and
#     rows in a group evict each other's units, so `"fresh":false` fires on a
#     row nobody touched.
#
# The linked executable is byte-identical throughout. So the probe reported 17
# cmake cells + 23 rust rows STALE on every run of a tree whose `lane=all` build
# had just gone green — a fixed oscillation, not a treadmill converging, and the
# reason `just ci matrix` failed ~190 tests with "fixture is stale" on runs
# either side of an unrelated change. Fixed in 2fa1ed09f by hashing the
# artifact; NOTHING guarded it, which is what this file is.
#
# It matters beyond its own two scripts. Phase-424 groups eight freshness
# issues, and every one of them is tempting to close by widening what a prober
# watches. The property asserted here is what makes cross-family re-staling
# impossible: family B may rebuild, touch, evict or relink whatever family A
# reads, and A stays silent as long as A's bytes do not move. Widen a watch set
# without that property and 0835 comes straight back.
#
# WHAT IS ASSERTED:
#
#   A. cmake — an always-dirty edge that recompiles and RELINKS on every build
#      leaves the probe silent, three runs running. A real source edit reports
#      stale EXACTLY ONCE, then silent. (Silence alone would be satisfied by a
#      probe that says nothing ever, so both halves are required.)
#
#   B. cargo — a unit re-run that produces identical bytes leaves the probe
#      silent. A real source edit reports stale exactly once, then silent.
#
#   C. CROSS-FAMILY, which is the issue's title. The cmake cell's build TOUCHES
#      the cargo row's source, so building the cmake family forces the cargo
#      family to re-run a unit. Three alternating rounds: both stay silent.
#
#   D. THE NEGATIVE CONTROL. The pre-2fa1ed09f decision rules — grep the cmake
#      output for "Building/Linking/Compiling", grep cargo's json for
#      `"fresh":false` — are re-applied to the SAME unchanged trees and must
#      FIRE. Without this, A/B/C would also pass against a probe that had been
#      quietly broken into never reporting anything, and the whole file would be
#      the mistake it exists to prevent. It also proves the fixtures below
#      really do reproduce the hazard rather than modelling something easier.
#
# The projects are the smallest ones with the right SHAPE: a cmake cell whose
# always-dirty custom command byproduct is a header the executable includes
# (ninja therefore recompiles + relinks every build, printing the exact strings
# the old grep matched), and a cargo leaf with one binary and no dependencies.
# No nano-ros build, no SDK, no codegen, no fixture manifest. Seconds.
#
# PRECONDITIONS ARE HARD FAILURES. A green here reads as "the staleness probes
# cannot oscillate"; skipping belongs in the just recipe, which checks for the
# host tools before calling this at all.

set -uo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root" || exit 1

# `nros_grep_q` — 0 match / 1 no-match / exit 2 when grep could not run, so a
# tool failure never becomes a finding (issue 0726). Call it with a HERE-STRING,
# NEVER through a pipe: builtin `printf` flushes per LINE and `grep -q` stops at
# the first hit, so the writer's next write takes SIGPIPE and `pipefail` turns a
# MATCH into a MISS. That is issue 1077, measured at 13 of 300 runs here.
# shellcheck source=../scripts/lib/grep-q.sh
source "$repo_root/scripts/lib/grep-q.sh"

CMAKE_PROBE="scripts/test/cmake-fixture-stale.sh"
RUST_PROBE="scripts/test/rust-fixture-stale.sh"

failures=0
checks=0

fail() {
    failures=$((failures + 1))
    echo "  [FAIL] $*" >&2
}
ok() {
    echo "  [ok]   $*"
}
check_eq() {
    # check_eq <what> <expected> <actual>
    checks=$((checks + 1))
    if [ "$2" = "$3" ]; then
        ok "$1"
    else
        fail "$1: expected [$2], got [$3]"
    fi
}

for tool in cmake ninja cargo cc; do
    command -v "$tool" >/dev/null 2>&1 || {
        echo "FATAL: $tool is required — this script never skips, see header" >&2
        exit 2
    }
done
[ -x "$CMAKE_PROBE" ] || [ -r "$CMAKE_PROBE" ] || {
    echo "FATAL: $CMAKE_PROBE not found" >&2
    exit 2
}

work="$(mktemp -d)" || exit 1
trap 'rm -rf "$work"' EXIT

# ---------------------------------------------------------------------------
# The two fixtures.
# ---------------------------------------------------------------------------

# The cmake cell. `OUTPUT` names a stamp the command never creates, so ninja
# runs the edge on every build ("output ... doesn't exist"); its BYPRODUCT is a
# header main.c includes, whose mtime therefore moves, so main.c recompiles and
# the executable relinks. That is Corrosion's shape (its `_cargo-build_*` stamp
# is never created and its declared `.a` feeds the link) with no Rust in it.
mkdir -p "$work/cell"
cat > "$work/cell/CMakeLists.txt" <<'CMAKEEOF'
cmake_minimum_required(VERSION 3.20)
project(probe_cell C)
set(_gen "${CMAKE_CURRENT_BINARY_DIR}/gen.h")
# NROS_PROBE_TOUCH is the cross-family lever (case C): when set, this edge also
# dates a file OUTSIDE this project forward, exactly as a shared cargo group
# does to a sibling row.
add_custom_command(
    OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/never-created.stamp"
    BYPRODUCTS "${_gen}"
    COMMAND "${CMAKE_COMMAND}" -E echo "#define PROBE_CONST 7" > "${_gen}"
    COMMAND "${CMAKE_COMMAND}" -E touch "${_gen}"
    COMMAND "${CMAKE_COMMAND}"
            -DNROS_PROBE_TOUCH=$ENV{NROS_PROBE_TOUCH}
            -P "${CMAKE_CURRENT_SOURCE_DIR}/touch-foreign.cmake"
    COMMENT "always-dirty (models Corrosion's cargo step)"
    VERBATIM)
add_custom_target(always_dirty DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/never-created.stamp")
add_executable(probe_cell main.c)
target_include_directories(probe_cell PRIVATE "${CMAKE_CURRENT_BINARY_DIR}")
add_dependencies(probe_cell always_dirty)
CMAKEEOF
cat > "$work/cell/touch-foreign.cmake" <<'CMAKEEOF'
if(NROS_PROBE_TOUCH AND EXISTS "${NROS_PROBE_TOUCH}")
    execute_process(COMMAND "${CMAKE_COMMAND}" -E touch "${NROS_PROBE_TOUCH}")
endif()
CMAKEEOF
printf '#include "gen.h"\nint main(void){return PROBE_CONST - 7;}\n' > "$work/cell/main.c"

# The cargo leaf. `[workspace]` detaches it from any enclosing workspace; the
# lock is written by hand because the `scripts/bin/cargo` shim injects
# `--locked` project-wide and a dependency-free crate's lock is three lines.
mkdir -p "$work/leaf/src"
cat > "$work/leaf/Cargo.toml" <<'TOMLEOF'
[workspace]

[package]
name = "nros-staleness-probe-leaf"
version = "0.0.0"
edition = "2021"

[[bin]]
name = "nros_staleness_probe_leaf"
path = "src/main.rs"
TOMLEOF
cat > "$work/leaf/Cargo.lock" <<'LOCKEOF'
version = 3

[[package]]
name = "nros-staleness-probe-leaf"
version = "0.0.0"
LOCKEOF
printf 'fn main() { println!("probe"); }\n' > "$work/leaf/src/main.rs"

# The probe builds a row at its PLATFORM's profile (`nros_cargo_platform_profile`,
# phase-340 P2), and this leaf is detached from the workspace that defines the
# repo's custom profiles. Resolve the name the SAME way the probe does and give
# the leaf that profile, rather than hardcoding today's answer — a profile table
# change would otherwise make every case here fail as `profile ... is not
# defined`, which reads as a probe defect and is not one.
# shellcheck source=scripts/build/cargo.sh
source scripts/build/cargo.sh 2>/dev/null || {
    echo "FATAL: cannot source scripts/build/cargo.sh" >&2
    exit 2
}
probe_profile="$(nros_cargo_platform_profile linux)" || probe_profile=""
[ -n "$probe_profile" ] || {
    echo "FATAL: could not resolve the linux fixture profile (is the nros CLI built?" >&2
    echo "       \`just setup-cli\`)" >&2
    exit 2
}
case "$probe_profile" in
    dev | release | test | bench) ;;
    *) printf '\n[profile.%s]\ninherits = "release"\n' "$probe_profile" >> "$work/leaf/Cargo.toml" ;;
esac

# The probes derive the cargo group dir through `nros_build_dir`, so pointing
# NROS_BUILD_ROOT at the scratch tree keeps every byte this test writes inside
# it — the repo's own build/ is untouched.
export NROS_BUILD_ROOT="$work/build-root"

cmake_record="$(printf '%s\x1fbuild-probe\x1f\x1f' "$work/cell")"
rust_record="$(printf 'linux\x1f%s\x1f\x1f' "$work/leaf")"

cmake_probe() { bash "$CMAKE_PROBE" "$cmake_record"; }
rust_probe() { bash "$RUST_PROBE" "$rust_record"; }

# Configure + first build, so both fixtures start FRESH. A probe run would also
# do it, but then case A's first assertion would be measuring the initial build.
cmake -S "$work/cell" -B "$work/cell/build-probe" -G Ninja > "$work/configure.log" 2>&1 || {
    echo "FATAL: cmake configure failed" >&2
    cat "$work/configure.log" >&2
    exit 2
}
cmake --build "$work/cell/build-probe" > "$work/build.log" 2>&1 || {
    echo "FATAL: cmake build failed" >&2
    cat "$work/build.log" >&2
    exit 2
}
[ -x "$work/cell/build-probe/probe_cell" ] || {
    echo "FATAL: the cell produced no executable — the probe would have nothing to hash" >&2
    exit 2
}
out="$(rust_probe)"
[ -n "$out" ] || {
    echo "FATAL: the cargo leaf's first probe reported FRESH — it had never been built," >&2
    echo "       so the probe cannot be distinguishing anything. $out" >&2
    exit 2
}

echo "== A. cmake: an always-dirty relink is not staleness =="

# The premise, asserted rather than assumed: every build really does recompile
# and relink. If cmake/ninja ever stopped doing that, cases A and D would both
# pass while measuring nothing.
cmake --build "$work/cell/build-probe" > "$work/relink.log" 2>&1
if grep -qE 'Building C object' "$work/relink.log" && grep -qE 'Linking C executable' "$work/relink.log"; then
    ok "premise: the cell recompiles and relinks on an unchanged tree"
else
    fail "premise: the cell did NOT relink — cases A and D no longer model the hazard"
    sed 's/^/    /' "$work/relink.log" >&2
fi
checks=$((checks + 1))

for i in 1 2 3; do
    check_eq "run $i on an unchanged cell is silent" "" "$(cmake_probe)"
done

printf '#include "gen.h"\nint main(void){return PROBE_CONST - 6;}\n' > "$work/cell/main.c"
check_eq "a real source edit reports the cell stale" \
    "$work/cell/build-probe" "$(cmake_probe)"
check_eq "and is silent on the next run" "" "$(cmake_probe)"

echo "== B. cargo: a unit re-run with identical bytes is not staleness =="

check_eq "the freshly built leaf is silent" "" "$(rust_probe)"

# `touch` is the smallest thing that makes cargo re-run a unit without changing
# what it produces. In the tree it is group eviction; the probe cannot tell the
# two apart and must not need to.
before="$(md5sum "$NROS_BUILD_ROOT"/cargo-fixtures/linux/*/nros_staleness_probe_leaf 2>/dev/null | awk '{print $1}' | head -1)"
touch "$work/leaf/src/main.rs"
check_eq "a touched source (unit re-runs, bytes identical) is silent" "" "$(rust_probe)"
after="$(md5sum "$NROS_BUILD_ROOT"/cargo-fixtures/linux/*/nros_staleness_probe_leaf 2>/dev/null | awk '{print $1}' | head -1)"
check_eq "premise: the rebuilt binary is byte-identical" "$before" "$after"

printf 'fn main() { println!("probe changed"); }\n' > "$work/leaf/src/main.rs"
check_eq "a real source edit reports the leaf stale" \
    "$work/leaf" "$(rust_probe)"
check_eq "and is silent on the next run" "" "$(rust_probe)"

echo "== C. cross-family: building one family does not stale the other =="

# Arm the lever: from here the cmake cell's always-dirty edge dates the cargo
# leaf's source forward on every build, so running the cmake probe forces the
# cargo probe's next build to re-run a unit. That is what "the two families
# re-stale each other" means, reduced to two files.
export NROS_PROBE_TOUCH="$work/leaf/src/main.rs"
cmake -S "$work/cell" -B "$work/cell/build-probe" -G Ninja >> "$work/configure.log" 2>&1 || {
    echo "FATAL: cmake re-configure failed" >&2
    exit 2
}
cmake_probe > /dev/null # settle the re-configure's own rebuild
rust_probe > /dev/null

for round in 1 2 3; do
    check_eq "round $round: cmake probe silent" "" "$(cmake_probe)"
    check_eq "round $round: rust probe silent after the cmake build touched its source" \
        "" "$(rust_probe)"
done
unset NROS_PROBE_TOUCH

echo "== D. negative control: the pre-fix rules FIRE on these same trees =="

# Everything above would also pass against a probe that had been broken into
# reporting nothing at all. These two blocks apply the decision rules the probes
# used before 2fa1ed09f to the SAME unchanged fixtures; each must report stale,
# which is simultaneously the proof that the fixtures reproduce the hazard and
# the proof that the assertions above are not vacuous.

echo "== E. the fallback branch ANNOUNCES itself (issue 0945 item 6) =="

# Cases A-D all have the artifact PRESENT, so none of them reaches the branch
# taken when the locator finds NOTHING — which is the branch an on-disk layout
# change produces, and the one that silently reverts the verdict to the
# pre-`2fa1ed09f` rule. That rule is permanently "stale" for rows sharing a
# phase-340 cargo group, so a layout change would reproduce issue 0835's ~190
# failures while reading as ordinary self-healing.
#
# A LIBRARY-ONLY leaf is the real shape, not a contrived one:
# `packages/testing/qemu-smoltcp-bridge` is on this branch in the live tree
# today, because its Cargo.toml names no `[[bin]]` the glob can find.
mkdir -p "$work/libleaf/src"
cat > "$work/libleaf/Cargo.toml" <<'TOMLEOF'
[workspace]

[package]
name = "nros-staleness-probe-libleaf"
version = "0.0.0"
edition = "2021"

[lib]
name = "nros_staleness_probe_libleaf"
path = "src/lib.rs"
TOMLEOF
cat > "$work/libleaf/Cargo.lock" <<'LOCKEOF'
version = 3

[[package]]
name = "nros-staleness-probe-libleaf"
version = "0.0.0"
LOCKEOF
printf 'pub fn probe() -> u8 { 7 }\n' > "$work/libleaf/src/lib.rs"
case "$probe_profile" in
    dev | release | test | bench) ;;
    *) printf '\n[profile.%s]\ninherits = "release"\n' "$probe_profile" >> "$work/libleaf/Cargo.toml" ;;
esac

libleaf_record="$(printf 'linux\x1f%s\x1f\x1f' "$work/libleaf")"
out="$(bash "$RUST_PROBE" "$libleaf_record")"
if nros_grep_q '^DEGRADED'"$(printf '\t')" <<<"$out"; then
    ok "a leaf whose artifact the locator cannot find reports DEGRADED"
else
    fail "the fallback branch stayed SILENT — a layout change would revert every row to the pre-0835 rule with nothing saying so. got: ${out:-<empty>}"
fi
checks=$((checks + 1))

# And the reason has to be usable: a bare marker with no cause is a line nobody
# can act on, which is how the silent state survived in the first place.
if nros_grep_q 'falls back to' <<<"$out"; then
    ok "the DEGRADED line names what the verdict fell back to"
else
    fail "the DEGRADED line carries no reason: ${out:-<empty>}"
fi
checks=$((checks + 1))

echo ""
echo "== the OLD rules still fire on this fixture (negative control) =="

out="$(cmake --build "$work/cell/build-probe" 2>&1)"
if nros_grep_q -E "Building (C|CXX|ASM) object|Linking (C|CXX|CXX shared)|Compiling [a-z0-9_-]+ v" <<<"$out"; then
    ok "the old cmake rule (grep the build chatter) reports STALE — as it did forever"
else
    fail "the old cmake rule did NOT fire: this fixture no longer models the defect"
fi
checks=$((checks + 1))

touch "$work/leaf/src/main.rs"
out="$(cd "$work/leaf" && cargo build --profile "$probe_profile" \
    --target-dir "$NROS_BUILD_ROOT/cargo-fixtures/linux" \
    --message-format=json --quiet 2>/dev/null)"
if nros_grep_q '"fresh":false' <<<"$out"; then
    ok "the old cargo rule (\"fresh\":false) reports STALE — as it did forever"
else
    fail "the old cargo rule did NOT fire: this fixture no longer models the defect"
fi
checks=$((checks + 1))

echo ""
if [ "$failures" -eq 0 ]; then
    echo "fixture-staleness-probe-tests: $checks check(s) passed"
    exit 0
fi
echo "fixture-staleness-probe-tests: $failures of $checks check(s) FAILED" >&2
echo "" >&2
echo "  A probe that decides from the build's ACTIVITY instead of the artifact's" >&2
echo "  BYTES cannot reach a fixed point on this tree: Corrosion's cargo step is" >&2
echo "  an always-dirty edge and the phase-340 cargo groups evict rows nobody" >&2
echo "  touched, so it reports work on every run forever (issue 0835)." >&2
exit 1
