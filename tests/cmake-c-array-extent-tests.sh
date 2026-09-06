#!/bin/bash
# tests/cmake-c-array-extent-tests.sh -- issue 1015
#
# Exercises `nros_assert_c_array_extent` from `cmake/NanoRosCArrayExtents.cmake`
# in `cmake -P` script mode. Sibling of `tests/cmake-entity-inventory-tests.sh`,
# and shaped like it.
#
# WHY THIS EXISTS. phase-412 W1 derives `MAX_QUERYABLES = service servers +
# 3 * action servers`. The reference island declares neither, so it derived
# exactly 0, and `queryable_entry_t queryables[ZPICO_MAX_QUERYABLES]` became a
# zero-length array that is NOT the last member of its struct -- a GNU extension
# that compiles silently and changes what the struct IS. Measured: the board
# transmitted 0 bytes in 15 s. No panic, no log line, no fault, core in WFI. The
# same image at 4 transmitted 110 bytes, and every gate was green either way
# because 0 was derived correctly and delivered faithfully.
#
# The C `#error` beside the arrays catches it at COMPILE. This function catches
# it at CONFIGURE, which is earlier, and which is where the diagnostic can still
# name the KNOB and the DECLARATION instead of a struct nobody edited.
#
# BOTH DIRECTIONS ARE ASSERTED, and the passing direction is the one that costs
# something to get wrong: a correctly-declared image whose derivation lands on
# the floor must still configure. The derivation floors its pools at 1, so `1`
# is the value a real single-service image arrives with, and a check that
# refused it would trade a silent board for a broken build.
#
# WHAT IS ASSERTED:
#
#   A. A usable size passes, quietly. 1 (the floor a real derivation produces),
#      and a large one.
#   B. ZERO is a FATAL_ERROR, and the message NAMES the macro. The whole defect
#      was a failure nobody could attribute; a refusal that does not name the
#      knob repeats it one level up.
#   C. The refusal explains that a zero is almost never a DELIBERATE zero --
#      it is a derivation that found nothing -- and says where to declare.
#   D. The `-1` DERIVE SENTINEL is refused with its OWN explanation. It means
#      "nobody stated a number", not "size the array -1", and reaching a
#      compiler it produces `size of array is negative` on a struct nobody
#      edited.
#   E. An EMPTY value is refused. `-D<M>=` reaches the compiler and fails as
#      `flexible array member not at end of struct`, naming neither the knob nor
#      the step that dropped it. A knob left unresolved is legal only where the
#      consumer supplies a default, and a C `-D` has none.
#   F. A non-numeric value is refused.
#   G. The optional KCONFIG and DECLARE arguments reach the message. They are
#      the "where do I fix this" half, and a diagnostic without them is the
#      silent board with extra steps.
#
# PRECONDITIONS ARE HARD FAILURES. This script never prints-and-returns: a green
# here is read as "the floor holds".
#
# Usage: ./tests/cmake-c-array-extent-tests.sh
# Exit:  0 all assertions held; 1 otherwise.

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# shellcheck source=lib/common.sh
source "$SCRIPT_DIR/lib/common.sh"

MODULE="$PROJECT_ROOT/cmake/NanoRosCArrayExtents.cmake"

FAILURES=0
CHECKS=0

fail() {
    log_error "$*"
    FAILURES=$((FAILURES + 1))
}

check() {
    CHECKS=$((CHECKS + 1))
}

if [ ! -f "$MODULE" ]; then
    fail "module not found: $MODULE"
    exit 1
fi
if ! command -v cmake >/dev/null 2>&1; then
    fail "cmake is not on PATH -- this test cannot report a verdict without it"
    exit 1
fi

init_test_tmpdir "nros-c-array-extent"
trap 'cleanup_test_tmpdir' EXIT

# Run one assertion through `cmake -P`, setting $RC and $OUT.
#
# Called as a plain statement, never captured with a command substitution: `$(...)`
# runs the function in a SUBSHELL, so an rc it assigns never reaches the caller
# and every case reads as rc 0 -- which made the first version of this file
# report five PASSES for five correct refusals, the exact shape of "a check that
# cannot fail". The whole question here is whether cmake FAILED, so the rc has
# to survive.
RC=0
OUT=""
assert_extent() {
    local body="$1"
    local script="$TEST_TMPDIR/case.cmake"
    local outfile="$TEST_TMPDIR/case.out"
    {
        echo "include(\"$MODULE\")"
        echo "$body"
        echo 'message(STATUS "REACHED_THE_END")'
    } > "$script"
    cmake -P "$script" > "$outfile" 2>&1
    RC=$?
    OUT="$(cat "$outfile")"
}

# ---------------------------------------------------------------------------
# A. A usable size passes.
#
# 1 first, and deliberately: that is what the derivation's own floor produces
# for an image with exactly one service server, so it is the value the FIX
# ships. A check that rejected it would be worse than the bug.
# ---------------------------------------------------------------------------
log_info "A. a usable size configures"
for value in 1 4 8 65536; do
    check
    assert_extent "nros_assert_c_array_extent(
        MACRO ZPICO_MAX_QUERYABLES
        VALUE \"$value\"
        ARRAY \"queryable_entry_t queryables[ZPICO_MAX_QUERYABLES]\"
        KCONFIG CONFIG_NROS_MAX_QUERYABLES)"
    if [ "$RC" -ne 0 ]; then
        fail "A: a pool of $value was refused -- $OUT"
    fi
    check
    if ! grep -q "REACHED_THE_END" <<<"$OUT"; then
        fail "A: a pool of $value stopped the script -- $OUT"
    fi
done

# ---------------------------------------------------------------------------
# B/C. ZERO is fatal, names the macro, and explains what a zero MEANS.
# ---------------------------------------------------------------------------
log_info "B. a derived zero is refused, loudly"
assert_extent 'nros_assert_c_array_extent(
    MACRO ZPICO_MAX_QUERYABLES
    VALUE "0"
    ARRAY "queryable_entry_t queryables[ZPICO_MAX_QUERYABLES]"
    KCONFIG CONFIG_NROS_MAX_QUERYABLES
    DECLARE "Declare them in the ENTITIES list of nano_ros_node_register().")'
check
if [ "$RC" -eq 0 ]; then
    fail "B: a pool of ZERO configured successfully -- $OUT"
fi
check
if grep -q "REACHED_THE_END" <<<"$OUT"; then
    fail "B: a pool of ZERO did not stop the configure -- $OUT"
fi
check
if ! grep -q "ZPICO_MAX_QUERYABLES" <<<"$OUT"; then
    fail "B: the refusal does not NAME the macro -- $OUT"
fi
check
if ! grep -q "1015" <<<"$OUT"; then
    fail "B: the refusal does not name the issue -- $OUT"
fi
log_info "C. the refusal says what a zero means and where to fix it"
check
if ! grep -q "ALMOST NEVER A DELIBERATE ZERO" <<<"$OUT"; then
    fail "C: the refusal does not say a zero is a derivation that found nothing -- $OUT"
fi
check
if ! grep -q "queryable_entry_t queryables" <<<"$OUT"; then
    fail "C: the refusal does not say WHICH array it sizes -- $OUT"
fi
check
if ! grep -q "ENTITIES list of nano_ros_node_register" <<<"$OUT"; then
    fail "C: the refusal does not say where to declare -- $OUT"
fi
check
if ! grep -q "CONFIG_NROS_MAX_QUERYABLES" <<<"$OUT"; then
    fail "C: the refusal does not name the Kconfig option -- $OUT"
fi

# ---------------------------------------------------------------------------
# D. The DERIVE SENTINEL gets its own explanation.
#
# `-1` is not a small pool either; it is the spelling of "nobody stated a
# number", and reaching a compiler it sizes the array NEGATIVE. It has escaped
# its resolver three times in this campaign (check-knob-resolved-once), so the
# message points at that rather than at the declaration.
# ---------------------------------------------------------------------------
log_info "D. the -1 DERIVE sentinel is refused with its own reason"
assert_extent 'nros_assert_c_array_extent(
    MACRO ZPICO_MAX_SUBSCRIBERS VALUE "-1"
    ARRAY "subscriber_entry_t subscribers[ZPICO_MAX_SUBSCRIBERS]")'
check
if [ "$RC" -eq 0 ]; then
    fail "D: the -1 sentinel configured successfully -- $OUT"
fi
check
if ! grep -q "DERIVE SENTINEL" <<<"$OUT"; then
    fail "D: the refusal does not identify -1 as the sentinel -- $OUT"
fi

# ---------------------------------------------------------------------------
# E. An EMPTY value is refused as its own case.
# ---------------------------------------------------------------------------
log_info "E. an unresolved knob is refused before it becomes -DX="
assert_extent 'nros_assert_c_array_extent(
    MACRO ZPICO_MAX_PUBLISHERS VALUE ""
    ARRAY "publisher_entry_t publishers[ZPICO_MAX_PUBLISHERS]")'
check
if [ "$RC" -eq 0 ]; then
    fail "E: an empty value configured successfully -- $OUT"
fi
check
if ! grep -q "resolved to NOTHING" <<<"$OUT"; then
    fail "E: the refusal does not distinguish EMPTY from zero -- $OUT"
fi
check
if ! grep -q "flexible array member" <<<"$OUT"; then
    fail "E: the refusal does not name the compiler error it prevents -- $OUT"
fi

# ---------------------------------------------------------------------------
# F. A non-numeric value is refused.
# ---------------------------------------------------------------------------
log_info "F. a value that is not a number is refused"
assert_extent 'nros_assert_c_array_extent(
    MACRO ZPICO_MAX_LIVELINESS VALUE "eight")'
check
if [ "$RC" -eq 0 ]; then
    fail "F: a non-numeric value configured successfully -- $OUT"
fi
check
if ! grep -q "is not an integer" <<<"$OUT"; then
    fail "F: the refusal does not say the value is not a number -- $OUT"
fi

# ---------------------------------------------------------------------------
# G. The optional arguments are OPTIONAL -- a call with neither still works.
#
# Asserted because a required-argument regression would turn every unadorned
# call site into a configure failure, which is the loudest possible way to make
# a safety check unwelcome.
# ---------------------------------------------------------------------------
log_info "G. KCONFIG and DECLARE are optional"
check
assert_extent 'nros_assert_c_array_extent(MACRO ZPICO_MAX_SESSIONS VALUE "1")'
if [ "$RC" -ne 0 ]; then
    fail "G: a bare MACRO/VALUE call failed -- $OUT"
fi
check
if ! grep -q "REACHED_THE_END" <<<"$OUT"; then
    fail "G: a bare MACRO/VALUE call stopped the script -- $OUT"
fi
# A caller that forgets MACRO is a bug in the CALLER, and it must say so rather
# than silently checking nothing.
check
assert_extent 'nros_assert_c_array_extent(VALUE "0")'
if [ "$RC" -eq 0 ]; then
    fail "G: a call with no MACRO passed instead of reporting a caller bug -- $OUT"
fi

# ---------------------------------------------------------------------------
if [ "$FAILURES" -eq 0 ]; then
    log_success "cmake-c-array-extent: $CHECKS assertion(s) held"
    exit 0
fi
log_error "cmake-c-array-extent: $FAILURES of $CHECKS assertion(s) failed"
exit 1
