#!/bin/bash
# tests/cmake-support-library-tests.sh — phase-383 W7.e / W7.f (RFC-0065 D12)
#
# Exercises `nano_ros_support_library()` / `nano_ros_link_support_libraries()`
# from `cmake/NanoRosSupportLibrary.cmake` against throwaway projects that use
# the HOST compiler only — no nano-ros build, no toolchain, no SDK.
#
# WHAT IS ACTUALLY ASSERTED, and why each assertion exists:
#
#   A. Force-linking WORKS. A vendor init table nothing references survives into
#      the executable with WHOLE_ARCHIVE and is ABSENT without it. Asserting
#      both directions is the point: a test that only checks the symbol is
#      present passes even if the keyword became a no-op and ld happened to
#      keep the member for another reason.
#
#   B. Force-linking carries a REBUILD EDGE — issue 0475. The archive must be an
#      IMPLICIT (`|`) input of the link edge, never only an order-only (`||`)
#      one, AND editing a support source must actually relink the executable.
#      0475's museum binaries passed every "does it contain the flag?" check.
#
#   C. The attach is ORDER-INDEPENDENT. The consuming target is created in a
#      subdirectory processed BEFORE the support package's, which is the
#      deferred-attach case D4 forces (the entry package is generated, so the
#      builder chooses the subdirectory order, not the user).
#
#   D. A PREBUILT `.a` (ARCHIVE) force-links the same way.
#
#   E. LINKER_FRAGMENTS (W7.f) put the fragment's directory on the linker search
#      path so a board script's `INCLUDE <name>.ld` resolves, and put the
#      fragment itself on the link rule's dependencies so editing it relinks.
#
#   F. Every declaration error is a FATAL_ERROR at CONFIGURE time with guidance,
#      not an unresolved-symbol wall three minutes into a compile.
#
# PRECONDITIONS ARE HARD FAILURES. This script never prints-and-returns: a
# missing cmake/ninja/cc must be a red, because a green here is read as "the
# force-link seam works". Skipping belongs in the `just` recipe's check ledger
# (`nros_check_skip`), which is a different claim from "the test passed".
#
# Usage: ./tests/cmake-support-library-tests.sh
# Exit:  0 all assertions held; 1 otherwise.

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# shellcheck source=lib/common.sh
source "$SCRIPT_DIR/lib/common.sh"

# `nros_grep_q` — 0 match / 1 no-match / exit 2 when grep could not run, so a
# tool failure never becomes a finding (issue 0726). Call it with a HERE-STRING,
# NEVER through a pipe: builtin `printf` flushes per LINE and `grep -q` stops at
# the first hit, so the writer's next write takes SIGPIPE and `pipefail` turns a
# MATCH into a MISS. That is issue 1077, measured at 13 of 300 runs here.
# shellcheck source=../scripts/lib/grep-q.sh
source "$PROJECT_ROOT/scripts/lib/grep-q.sh"

MODULE="$PROJECT_ROOT/cmake/NanoRosSupportLibrary.cmake"

FAILURES=0
CHECKS=0

fail() {
    log_error "$*"
    FAILURES=$((FAILURES + 1))
}

pass() {
    CHECKS=$((CHECKS + 1))
    log_success "$*"
}

check() {
    # check <description> <predicate...>
    local desc="$1"; shift
    if "$@"; then
        pass "$desc"
    else
        fail "$desc"
    fi
}

assert_contains() {
    # assert_contains <description> <file> <needle>
    local desc="$1" file="$2" needle="$3"
    if [ ! -f "$file" ]; then
        fail "$desc — file missing: $file"
        return
    fi
    if grep -qF -- "$needle" "$file"; then
        pass "$desc"
    else
        fail "$desc — '$needle' not found in $file"
    fi
}

# CMake hard-wraps `message(FATAL_ERROR …)` at ~76 columns and double-spaces
# after a period, so a two-word needle is routinely split across lines. Match on
# collapsed whitespace instead — otherwise the negative cases fail for a reason
# that has nothing to do with the guard they are checking.
log_says() {
    # log_says <file> <needle>
    # Captured, not piped: `tr … | grep -qF` lets grep exit at the first match
    # while tr still has output to write, and tr's SIGPIPE then becomes the
    # pipeline's status under `pipefail` — a MATCH reported as a MISS. 1077.
    nros_grep_q -F -- "$2" <<<"$(tr -s '[:space:]' ' ' < "$1")"
}

assert_not_contains() {
    local desc="$1" file="$2" needle="$3"
    if [ ! -f "$file" ]; then
        fail "$desc — file missing: $file"
        return
    fi
    if grep -qF -- "$needle" "$file"; then
        fail "$desc — '$needle' unexpectedly present in $file"
    else
        pass "$desc"
    fi
}

cleanup() {
    cleanup_test_tmpdir
}
trap cleanup EXIT

# ---------------------------------------------------------------------------
# Preconditions — every one of these is fatal, by design (see the header).
# ---------------------------------------------------------------------------
log_header "Preconditions"

if [ ! -f "$MODULE" ]; then
    log_error "cmake/NanoRosSupportLibrary.cmake not found at $MODULE"
    exit 1
fi
for tool in cmake ninja nm ar; do
    if ! command -v "$tool" >/dev/null 2>&1; then
        log_error "required tool '$tool' is not on PATH — this test cannot verify \
the force-link seam without it. (A host without the tool should SKIP in the \
just recipe's check ledger, not pass here.)"
        exit 1
    fi
done
CC_BIN="${CC:-cc}"
if ! command -v "$CC_BIN" >/dev/null 2>&1; then
    log_error "no C compiler ('$CC_BIN') on PATH"
    exit 1
fi
log_info "cmake:  $(cmake --version | head -1)"
log_info "ninja:  $(ninja --version)"
log_info "cc:     $CC_BIN"

init_test_tmpdir "nros-support-library"
log_info "scratch: $TEST_TMPDIR"

# ---------------------------------------------------------------------------
# Shared fixture sources.
# ---------------------------------------------------------------------------
write_support_pkg() {
    # write_support_pkg <dir>
    local d="$1"
    mkdir -p "$d/include"
    cat > "$d/include/vendor.h" <<'EOF'
#ifndef VENDOR_H
#define VENDOR_H
extern const int nros_test_vendor_init_table[4];
#endif
EOF
    cat > "$d/vendor_init.c" <<'EOF'
/* The D12 case: an init table NOTHING references. Dead-code elimination — and
 * ld's "only pull an archive member that resolves an undefined symbol" rule —
 * drops it unless the member is force-linked. */
#include "vendor.h"
const int nros_test_vendor_init_table[4] = {1, 2, 3, 4};
EOF
    cat > "$d/frag.ld" <<'EOF'
/* A linker-script fragment a board's top-level script would `INCLUDE`.
 * Deliberately inert so a host link is unaffected: W7.f's assertion is that the
 * fragment is REACHABLE (-L) and DEPENDED ON, not what it says. */
EOF
}

write_main() {
    cat > "$1" <<'EOF'
int main(void) { return 0; }
EOF
}

# ---------------------------------------------------------------------------
# Project builder. The consumer subdirectory is added BEFORE the support one,
# so every positive case also exercises the deferred attach (assertion C).
# ---------------------------------------------------------------------------
make_project() {
    # make_project <dir> <support-args...>
    local proj="$1"; shift
    mkdir -p "$proj/app" "$proj/support"
    write_support_pkg "$proj/support"
    write_main "$proj/app/main.c"

    cat > "$proj/CMakeLists.txt" <<EOF
cmake_minimum_required(VERSION 3.22)
project(nros_support_probe C)
include("${MODULE}")
# app FIRST: the consumer exists before the support package is even read.
add_subdirectory(app)
add_subdirectory(support)
EOF

    cat > "$proj/app/CMakeLists.txt" <<'EOF'
add_executable(image main.c)
nano_ros_link_support_libraries(image)
EOF

    {
        printf 'nano_ros_support_library(vendor_init\n'
        for a in "$@"; do printf '    %s\n' "$a"; done
        printf ')\n'
    } > "$proj/support/CMakeLists.txt"
}

configure() {
    # configure <proj> <build> ; echoes nothing, returns cmake's status
    local proj="$1" build="$2"
    cmake -S "$proj" -B "$build" -G Ninja \
        -DCMAKE_BUILD_TYPE=Debug > "$build.configure.log" 2>&1
}

# The link EDGE, read straight out of `build.ninja`:
#
#   build app/image: C_EXECUTABLE_LINKER__image_ <explicit…> | <implicit…> || <order-only…>
#
# Issue 0475 is EXACTLY the difference between the last two groups — an
# order-only (`||`) edge says "must exist before linking" and an implicit (`|`)
# edge says "relink when it changes" — so the test must read the MARKER, not
# merely find the filename somewhere in the output. `ninja -t query image`
# cannot answer this: cmake makes the bare target name a PHONY alias whose only
# input is the real output path, so the query returns the alias, not the edge.
link_edge_line() {
    # link_edge_line <build-dir>
    grep -E '^build [^:]*image: C(XX)?_EXECUTABLE_LINKER' "$1/build.ninja" | head -1
}

implicit_inputs() {
    # implicit_inputs <build-dir> — the `|` group of the link edge, one per line
    link_edge_line "$1" |
        sed -e 's/ || .*$//' -e 's/^[^|]*| *//' |
        tr ' ' '\n' | sed '/^$/d'
}

order_only_inputs() {
    # order_only_inputs <build-dir> — the `||` group, one per line
    link_edge_line "$1" |
        sed -n 's/^.* || //p' |
        tr ' ' '\n' | sed '/^$/d'
}

link_command() {
    # link_command <build-dir> — the full link rule text for `image`
    ninja -C "$1" -t commands image 2>/dev/null | tail -1
}

# ===========================================================================
log_header "A/B/C/E — SRCS + WHOLE_ARCHIVE + LINKER_FRAGMENTS, deferred attach"
# ===========================================================================
P1="$TEST_TMPDIR/whole"
B1="$TEST_TMPDIR/whole-build"
make_project "$P1" \
    "SRCS vendor_init.c" \
    "INCLUDES include" \
    "DEFINES NROS_TEST_VENDOR=1" \
    "WHOLE_ARCHIVE" \
    "LINKER_FRAGMENTS frag.ld"

if ! configure "$P1" "$B1"; then
    log_error "configure failed; log follows"
    sed 's/^/    /' "$B1.configure.log"
    exit 1
fi
pass "configures with WHOLE_ARCHIVE + LINKER_FRAGMENTS (deferred attach)"

# --- the flag reached the link line -----------------------------------------
LINK_CMD_FILE="$TEST_TMPDIR/link-cmd.txt"
link_command "$B1" > "$LINK_CMD_FILE"
assert_contains "the force-link flag is on the link line" \
    "$LINK_CMD_FILE" "--whole-archive"
assert_contains "the closing --no-whole-archive scopes the group" \
    "$LINK_CMD_FILE" "--no-whole-archive"
assert_contains "the support archive is named inside the flag" \
    "$LINK_CMD_FILE" "libvendor_init.a"

# --- W7.f: the fragment's dir is on the linker search path -------------------
assert_contains "LINKER_FRAGMENTS put the fragment's dir on -L (INCLUDE resolves)" \
    "$LINK_CMD_FILE" "-L$P1/support"

# --- issue 0475: the archive is an IMPLICIT dep, not order-only --------------
if [ -z "$(link_edge_line "$B1")" ]; then
    log_error "could not find the executable's link edge in $B1/build.ninja — \
the marker assertions below would vacuously pass"
    exit 1
fi
IMPL="$TEST_TMPDIR/implicit.txt"
ORDER="$TEST_TMPDIR/order.txt"
implicit_inputs "$B1" > "$IMPL"
order_only_inputs "$B1" > "$ORDER"

if grep -q 'libvendor_init\.a' "$IMPL"; then
    pass "issue 0475: the archive is an IMPLICIT (|) input of the link edge"
else
    fail "issue 0475: the archive is NOT an implicit input of the link edge — \
LINK_DEPENDS missing. Implicit inputs were: $(tr '\n' ' ' < "$IMPL")"
fi

if grep -q 'frag\.ld' "$IMPL"; then
    pass "W7.f: the linker fragment is an IMPLICIT input of the link edge"
else
    fail "W7.f: frag.ld is NOT an implicit input — editing it would not relink. \
Implicit inputs were: $(tr '\n' ' ' < "$IMPL")"
fi

# --- build it, and prove the unreferenced table survived ---------------------
if ! ninja -C "$B1" > "$TEST_TMPDIR/whole-build.log" 2>&1; then
    log_error "build failed; log follows"
    sed 's/^/    /' "$TEST_TMPDIR/whole-build.log"
    FAILURES=$((FAILURES + 1))
else
    pass "the project builds"
    if nros_grep_q 'nros_test_vendor_init_table' <<<"$(nm "$B1/app/image" 2>/dev/null)"; then
        pass "WHOLE_ARCHIVE kept the unreferenced vendor init table in the image"
    else
        fail "WHOLE_ARCHIVE did NOT keep the unreferenced symbol — force-linking \
is a no-op"
    fi
fi

# --- issue 0475, the behavioural half: an edit must relink --------------------
BEFORE_MTIME="$(stat -c %Y "$B1/app/image" 2>/dev/null || echo 0)"
sleep 1
cat >> "$P1/support/vendor_init.c" <<'EOF'
const int nros_test_vendor_second_table[2] = {5, 6};
EOF
if ! ninja -C "$B1" > "$TEST_TMPDIR/whole-rebuild.log" 2>&1; then
    log_error "rebuild failed; log follows"
    sed 's/^/    /' "$TEST_TMPDIR/whole-rebuild.log"
    FAILURES=$((FAILURES + 1))
else
    AFTER_MTIME="$(stat -c %Y "$B1/app/image" 2>/dev/null || echo 0)"
    if [ "$AFTER_MTIME" != "$BEFORE_MTIME" ]; then
        pass "issue 0475: editing a support source RELINKS the executable"
    else
        fail "issue 0475 REPRODUCED: the archive rebuilt but the executable did \
not relink (museum binary)"
    fi
    if nros_grep_q 'nros_test_vendor_second_table' <<<"$(nm "$B1/app/image" 2>/dev/null)"; then
        pass "the relinked executable carries the NEW object"
    else
        fail "the executable kept the OLD object after a support-source edit"
    fi
fi

# --- control: the SAME project without WHOLE_ARCHIVE drops the symbol --------
P2="$TEST_TMPDIR/plain"
B2="$TEST_TMPDIR/plain-build"
make_project "$P2" \
    "SRCS vendor_init.c" \
    "INCLUDES include"
if ! configure "$P2" "$B2"; then
    log_error "control configure failed; log follows"
    sed 's/^/    /' "$B2.configure.log"
    FAILURES=$((FAILURES + 1))
elif ! ninja -C "$B2" > "$TEST_TMPDIR/plain-build.log" 2>&1; then
    log_error "control build failed; log follows"
    sed 's/^/    /' "$TEST_TMPDIR/plain-build.log"
    FAILURES=$((FAILURES + 1))
else
    pass "the control project (no WHOLE_ARCHIVE) builds"
    if nros_grep_q 'nros_test_vendor_init_table' <<<"$(nm "$B2/app/image" 2>/dev/null)"; then
        fail "CONTROL BROKEN: the unreferenced table survived WITHOUT \
WHOLE_ARCHIVE, so assertion A proves nothing on this host/linker"
    else
        pass "control: without WHOLE_ARCHIVE the unreferenced table is dropped"
    fi
    CONTROL_CMD="$TEST_TMPDIR/control-cmd.txt"
    link_command "$B2" > "$CONTROL_CMD"
    assert_not_contains "control: no force-link flag when WHOLE_ARCHIVE is absent" \
        "$CONTROL_CMD" "--whole-archive"
fi

# ===========================================================================
log_header "D — a PREBUILT archive (ARCHIVE) force-links the same way"
# ===========================================================================
PRE="$TEST_TMPDIR/prebuilt"
mkdir -p "$PRE"
cat > "$PRE/rom_table.c" <<'EOF'
const int nros_test_prebuilt_rom_table[3] = {7, 8, 9};
EOF
if ! ( cd "$PRE" && "$CC_BIN" -c rom_table.c -o rom_table.o && \
       ar rcs librom.a rom_table.o ) > "$TEST_TMPDIR/prebuilt.log" 2>&1; then
    log_error "could not build the prebuilt archive fixture; log follows"
    sed 's/^/    /' "$TEST_TMPDIR/prebuilt.log"
    exit 1
fi

P3="$TEST_TMPDIR/archive"
B3="$TEST_TMPDIR/archive-build"
mkdir -p "$P3/app" "$P3/support"
write_main "$P3/app/main.c"
cat > "$P3/CMakeLists.txt" <<EOF
cmake_minimum_required(VERSION 3.22)
project(nros_support_archive C)
include("${MODULE}")
add_subdirectory(app)
add_subdirectory(support)
EOF
cat > "$P3/app/CMakeLists.txt" <<'EOF'
add_executable(image main.c)
nano_ros_link_support_libraries(image LIBRARIES vendor_rom)
EOF
cat > "$P3/support/CMakeLists.txt" <<EOF
nano_ros_support_library(vendor_rom
    ARCHIVE "${PRE}/librom.a"
    WHOLE_ARCHIVE)
EOF

if ! configure "$P3" "$B3"; then
    log_error "ARCHIVE configure failed; log follows"
    sed 's/^/    /' "$B3.configure.log"
    FAILURES=$((FAILURES + 1))
else
    pass "configures with a PREBUILT ARCHIVE"
    A_CMD="$TEST_TMPDIR/archive-cmd.txt"
    link_command "$B3" > "$A_CMD"
    assert_contains "prebuilt archive is force-linked" "$A_CMD" "librom.a"
    A_IMPL="$TEST_TMPDIR/archive-implicit.txt"
    implicit_inputs "$B3" > "$A_IMPL"
    if grep -q 'librom\.a' "$A_IMPL"; then
        pass "issue 0475: the prebuilt archive is an IMPLICIT input of the link edge"
    else
        fail "issue 0475: the prebuilt archive is not an implicit input. \
Implicit inputs were: $(tr '\n' ' ' < "$A_IMPL")"
    fi
    if ninja -C "$B3" > "$TEST_TMPDIR/archive-build.log" 2>&1; then
        pass "the ARCHIVE project builds"
        if nros_grep_q 'nros_test_prebuilt_rom_table' <<<"$(nm "$B3/app/image" 2>/dev/null)"; then
            pass "WHOLE_ARCHIVE kept the prebuilt archive's unreferenced table"
        else
            fail "the prebuilt archive's unreferenced table was dropped"
        fi
    else
        log_error "ARCHIVE build failed; log follows"
        sed 's/^/    /' "$TEST_TMPDIR/archive-build.log"
        FAILURES=$((FAILURES + 1))
    fi
fi

# ===========================================================================
log_header "E2 — W7.f: on Zephyr the fragment goes through zephyr_linker_sources()"
# ===========================================================================
#
# The Zephyr seam is selected by `if(COMMAND zephyr_linker_sources)`, so a stand
# in with the same signature exercises the real branch — the module cannot tell
# the difference, which is the property being tested. Asserting the NEGATIVE
# half matters as much: on Zephyr the snippet mechanism does the `#include`
# itself, so adding `-L` would be a second, silently-diverging placement path.

P4="$TEST_TMPDIR/zephyr"
B4="$TEST_TMPDIR/zephyr-build"
mkdir -p "$P4/app" "$P4/support"
write_support_pkg "$P4/support"
write_main "$P4/app/main.c"
cat > "$P4/CMakeLists.txt" <<EOF
cmake_minimum_required(VERSION 3.22)
project(nros_support_zephyr C)

# Stand-in for Zephyr's extensions.cmake command. Records its calls so the test
# can assert BOTH that it ran and what location it was handed.
function(zephyr_linker_sources location)
    foreach(_f IN LISTS ARGN)
        file(APPEND "\${CMAKE_BINARY_DIR}/linker-sources.log"
             "\${location} \${_f}\n")
    endforeach()
endfunction()

include("${MODULE}")
add_subdirectory(app)
add_subdirectory(support)
EOF
cat > "$P4/app/CMakeLists.txt" <<'EOF'
add_executable(image main.c)
nano_ros_link_support_libraries(image)
EOF
cat > "$P4/support/CMakeLists.txt" <<'EOF'
nano_ros_support_library(vendor_init
    SRCS vendor_init.c
    INCLUDES include
    LINKER_FRAGMENTS frag.ld
    ZEPHYR_SECTION NOCACHE_SECTION)
EOF

if ! configure "$P4" "$B4"; then
    log_error "Zephyr-seam configure failed; log follows"
    sed 's/^/    /' "$B4.configure.log"
    FAILURES=$((FAILURES + 1))
else
    pass "configures against a Zephyr-shaped build"
    assert_contains "W7.f: the fragment reached zephyr_linker_sources()" \
        "$B4/linker-sources.log" "frag.ld"
    assert_contains "W7.f: ZEPHYR_SECTION selected the location" \
        "$B4/linker-sources.log" "NOCACHE_SECTION"
    Z_CMD="$TEST_TMPDIR/zephyr-cmd.txt"
    link_command "$B4" > "$Z_CMD"
    assert_not_contains "W7.f: no -L on the Zephyr seam (the snippet is #included)" \
        "$Z_CMD" "-L$P4/support"
    Z_IMPL="$TEST_TMPDIR/zephyr-implicit.txt"
    implicit_inputs "$B4" > "$Z_IMPL"
    if grep -q 'frag\.ld' "$Z_IMPL"; then
        pass "W7.f: the fragment still relinks the image on the Zephyr seam"
    else
        fail "W7.f: frag.ld is not an implicit input on the Zephyr seam. \
Implicit inputs were: $(tr '\n' ' ' < "$Z_IMPL")"
    fi
fi

# ===========================================================================
log_header "F — declaration errors are configure-time FATAL_ERRORs with guidance"
# ===========================================================================
expect_configure_error() {
    # expect_configure_error <label> <needle-in-message> <support-args...>
    local label="$1" needle="$2"; shift 2
    local proj="$TEST_TMPDIR/neg-$label" build="$TEST_TMPDIR/neg-$label-build"
    make_project "$proj" "$@"
    if configure "$proj" "$build"; then
        fail "negative case '$label' CONFIGURED successfully — the guard is missing"
        return
    fi
    if log_says "$build.configure.log" "$needle"; then
        pass "negative case '$label' fails at configure and says why"
    else
        fail "negative case '$label' failed, but the message never mentions \
'$needle':
$(tail -20 "$build.configure.log" | sed 's/^/        /')"
    fi
}

expect_configure_error "srcs-and-archive" "mutually exclusive" \
    "SRCS vendor_init.c" "ARCHIVE frag.ld"
expect_configure_error "whole-without-code" "there is no archive to force-link" \
    "LINKER_FRAGMENTS frag.ld" "WHOLE_ARCHIVE"
expect_configure_error "nothing-to-contribute" "nothing to contribute" \
    "INCLUDES include"
expect_configure_error "missing-archive" "does not exist" \
    "ARCHIVE no_such_lib.a"
expect_configure_error "missing-fragment" "does not exist" \
    "SRCS vendor_init.c" "LINKER_FRAGMENTS no_such.ld"
expect_configure_error "empty-glob" "matched no files" \
    "SRCS no_such_dir/*.c"
# The stray token comes FIRST on purpose: `cmake_parse_arguments` folds an
# unknown token that FOLLOWS a multi-value keyword into that keyword's values,
# so `SRCS a.c SOURCES a.c` never reaches UNPARSED_ARGUMENTS at all. That is
# cmake's rule, not ours — the guard can only catch what parsing leaves over.
expect_configure_error "unknown-keyword" "unexpected argument" \
    "SOURCES" "SRCS vendor_init.c"
expect_configure_error "zephyr-section-off-zephyr" "not a Zephyr build" \
    "SRCS vendor_init.c" "LINKER_FRAGMENTS frag.ld" "ZEPHYR_SECTION NOCACHE_SECTION"

# A consumer naming a support library nobody declared must be caught at the
# flush, not silently ignored — a silently-ignored LIBRARIES entry is an image
# quietly missing its vendor layer.
NEG="$TEST_TMPDIR/neg-unknown-lib"
NEGB="$TEST_TMPDIR/neg-unknown-lib-build"
make_project "$NEG" "SRCS vendor_init.c"
cat > "$NEG/app/CMakeLists.txt" <<'EOF'
add_executable(image main.c)
nano_ros_link_support_libraries(image LIBRARIES not_declared_anywhere)
EOF
if configure "$NEG" "$NEGB"; then
    fail "negative case 'unknown-library' CONFIGURED successfully"
else
    if log_says "$NEGB.configure.log" "not_declared_anywhere"; then
        pass "negative case 'unknown-library' names the missing library"
    else
        fail "negative case 'unknown-library' failed without naming the library"
    fi
fi

# ===========================================================================
log_header "Summary"
# ===========================================================================
if [ "$CHECKS" -eq 0 ]; then
    log_error "no assertions ran — that is a failure, not a pass"
    exit 1
fi
if [ "$FAILURES" -ne 0 ]; then
    log_error "$FAILURES assertion(s) failed out of $((CHECKS + FAILURES))"
    exit 1
fi
log_success "all $CHECKS assertions held"
exit 0
