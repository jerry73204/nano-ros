#!/bin/bash
# tests/cmake-reconfigure-tests.sh -- issue 0991
#
# Gates `cmake/NanoRosReconfigure.cmake`: the mechanism that makes a fragment
# written LATE in a configure reach the readers that already ran, inside the
# same build.
#
# WHY THIS IS A REAL PROJECT AND NOT `cmake -P`. Its two siblings
# (`cmake-message-bounds-tests.sh`, `cmake-entity-inventory-tests.sh`) drive
# their modules in script mode, because what they assert is a DERIVATION -- a
# pure function of a fragment. What this module does is not a derivation: it is
# a claim about what NINJA does with `build.ninja` and an mtime. Script mode
# cannot observe that, and the defect being fixed is precisely a mechanism that
# READ as working in every review and never fired. So the assertion has to be a
# real configure followed by a real `ninja`, and the project below is the
# smallest one that has the shape: a fragment read early, rewritten late.
#
# It needs cmake + ninja and NOTHING else -- no compiler (`project(... NONE)`),
# no nano-ros build, no SDK, no codegen. Two seconds.
#
# NOTHING HERE MAY DEPEND ON THE CLOCK OR ON HOST SPEED. The probe projects
# declare `cmake_minimum_required(VERSION 3.20)` and must behave identically on
# every cmake at or above it: `string(TIMESTAMP ... "%f")` is a cmake >= 3.23
# feature that DEGRADES SILENTLY to second granularity below, which made case E
# pass on cmake 4.3 and fail on an older one -- a green that depended on the
# reviewer's toolchain. Anything that must differ between two passes uses a
# cache counter.
#
# WHAT IS ASSERTED:
#
#   A. THE BUG. Without the mechanism -- `CMAKE_CONFIGURE_DEPENDS` plus a write
#      during the configure, which is exactly what three call sites claimed was
#      self-healing -- `ninja` does NOT re-run cmake, and the build proceeds
#      with the placeholder. This case is the control. It is here because the
#      whole issue is that the absent mechanism was indistinguishable from a
#      present one; a fix with no failing control is the same mistake again.
#
#   B. THE FIX. With `nros_reconfigure_on_change`, the same project re-runs
#      cmake during `ninja` and the build proceeds with the REAL answer.
#
#   C. TERMINATION. It re-runs EXACTLY ONCE. A mechanism that dates a file
#      forward can loop until the wall clock catches up -- measured at 100
#      re-configures before `nros_reconfigure_settle` existed -- so "it
#      converges" is not a detail, it is the difference between a fix and an
#      outage. A second `ninja` re-runs cmake zero times.
#
#   D. IDENTICAL BYTES ARM NOTHING. A producer that rewrites the same content
#      every configure must not re-arm, or every build re-configures forever.
#
#   E. THE BOUND HOLDS. With `NROS_RECONFIGURE_MAX_PASSES` exhausted by a
#      deliberately non-convergent producer, the build completes with a WARNING
#      rather than looping. An escape hatch that cannot be exhausted is not a
#      bound.
#
#   F. `nros_reconfigure_settle` is a no-op on an ordinary file. It runs on
#      every reader on every configure, so "cheap and silent when nothing is
#      armed" is part of the contract.
#
#   J. THE PRODUCER RUNS ONCE PER PACKAGE, and the answer must not depend on
#      how many packages there are. `nros_find_interfaces()` is called from
#      every leaf that declares an interface dependency, so cases G/H -- which
#      call it once -- test a shape no image in this tree has. Issue 1119: the
#      second call settled the date the first had armed and snapshotted the
#      bytes the first had written, so a two-package image got 2 configures
#      where it needs 3 and shipped the pass-2 size.
#
# PRECONDITIONS ARE HARD FAILURES. This script never prints-and-returns: a green
# here is read as "the re-configure mechanism works". Skipping belongs in the
# `just` recipe's check ledger, which is a different claim from "it passed".
#
# Usage: ./tests/cmake-reconfigure-tests.sh
# Exit:  0 all assertions held; 1 otherwise.

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# shellcheck source=lib/common.sh
source "$SCRIPT_DIR/lib/common.sh"

MODULE="$PROJECT_ROOT/cmake/NanoRosReconfigure.cmake"

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
for _t in cmake ninja touch; do
    if ! command -v "$_t" >/dev/null 2>&1; then
        fail "$_t is not on PATH -- this test cannot report a verdict without it"
        exit 1
    fi
done

init_test_tmpdir "nros-reconfigure"
trap 'cleanup_test_tmpdir' EXIT

# ---------------------------------------------------------------------------
# The project under test.
#
# `$1` selects the producer's behaviour, so one source tree covers every case:
#
#   armed         -- the module's mechanism (the fix)
#   bare          -- CMAKE_CONFIGURE_DEPENDS alone (the bug, case A)
#   identical     -- rewrites the same bytes every configure (case D)
#   never-settles -- writes a DIFFERENT answer every configure (case E)
#
# The `go` target echoes the answer the configure that generated it had read,
# so what the BUILD used is observable in ninja's output rather than inferred
# from the fragment on disk. That distinction is the entire issue: the fragment
# was always correct; what was wrong was the answer the build was sized from.
# ---------------------------------------------------------------------------
write_project() {
    local dir="$1" mode="$2"
    mkdir -p "$dir"
    cat > "$dir/CMakeLists.txt" <<EOF
cmake_minimum_required(VERSION 3.20)
project(nros_reconfigure_probe NONE)

include("$MODULE")

set(_frag "\${CMAKE_BINARY_DIR}/frag.cmake")
set(_mode "$mode")

# ---- the READER, early in the configure -------------------------------------
if(NOT EXISTS "\${_frag}")
    file(WRITE "\${_frag}" "set(ANSWER placeholder)\n")
endif()
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "\${_frag}")
if(NOT _mode STREQUAL "bare")
    nros_reconfigure_settle("\${_frag}")
endif()
include("\${_frag}")
message(STATUS "PROBE_READ=\${ANSWER}")

# ---- the PRODUCER, later in the same configure -------------------------------
nros_reconfigure_snapshot("\${_frag}" _before)
if(_mode STREQUAL "identical")
    file(WRITE "\${_frag}" "set(ANSWER placeholder)\n")
elseif(_mode STREQUAL "never-settles")
    # DETERMINISTIC non-convergence -- a counter in the cache, never a clock.
    # This used string(TIMESTAMP) with a %f microsecond field, which needs
    # cmake 3.23 or newer. Below that it does not error, it silently gives
    # SECOND granularity, so two re-configures inside one second write
    # IDENTICAL bytes: the producer that must look non-convergent looks
    # convergent, nothing arms the second pass, the bound is never reached and
    # case E fails on its own assertion. A test for a bound must not depend on
    # how fast the host is, nor on which cmake the reviewer happens to run.
    #
    # NOTE this block is inside an UNQUOTED heredoc, so backticks and bare $
    # are shell-expanded. Write no backticks here.
    if(NOT DEFINED NROS_PROBE_SEQ)
        set(NROS_PROBE_SEQ 0 CACHE INTERNAL "probe: configures so far")
    endif()
    math(EXPR _seq "\${NROS_PROBE_SEQ} + 1")
    set(NROS_PROBE_SEQ "\${_seq}" CACHE INTERNAL "probe: configures so far")
    file(WRITE "\${_frag}" "set(ANSWER moving_\${_seq})\n")
else()
    file(WRITE "\${_frag}" "set(ANSWER real)\n")
endif()
if(NOT _mode STREQUAL "bare")
    nros_reconfigure_on_change("\${_frag}" "\${_before}" LABEL "the probe answer")
endif()

add_custom_target(go ALL
    COMMAND \${CMAKE_COMMAND} -E echo "PROBE_BUILT=\${ANSWER}")
EOF
}

# Run `ninja` once and report what the build actually used plus how many times
# cmake re-ran. `timeout` is a guard, not a convenience: the failure mode this
# mechanism can have IS an unbounded re-configure loop, and a test that hangs
# reports nothing.
run_build() {
    local build="$1" out
    out="$(timeout 120 ninja -C "$build" 2>&1)"
    BUILD_RC=$?
    BUILD_OUT="$out"
    BUILD_USED="$(printf '%s\n' "$out" | sed -n 's/.*PROBE_BUILT=\([A-Za-z0-9_]*\).*/\1/p' | tail -1)"
    BUILD_RERUNS="$(printf '%s\n' "$out" | grep -c 'Re-running CMake')"
}

log_header "A. the control: CONFIGURE_DEPENDS alone does NOT re-run cmake"

SRC_BARE="$TEST_TMPDIR/bare-src"
write_project "$SRC_BARE" "bare"
cmake -G Ninja -S "$SRC_BARE" -B "$TEST_TMPDIR/bare-build" >/dev/null 2>&1
run_build "$TEST_TMPDIR/bare-build"
check
if [ "$BUILD_USED" = "placeholder" ] && [ "$BUILD_RERUNS" -eq 0 ]; then
    log_success "bare CONFIGURE_DEPENDS: 0 re-runs, built with the placeholder (the bug reproduces)"
else
    fail "the control did not reproduce the bug: used='$BUILD_USED' re-runs=$BUILD_RERUNS.
  If ninja has started honouring a same-configure write, this module is
  obsolete rather than broken -- check that before deleting the assertion."
fi

log_header "B/C. the fix: exactly one re-configure, and the build uses the real answer"

SRC_ARMED="$TEST_TMPDIR/armed-src"
write_project "$SRC_ARMED" "armed"
CONFIGURE_OUT="$(cmake -G Ninja -S "$SRC_ARMED" -B "$TEST_TMPDIR/armed-build" 2>&1)"
check
if printf '%s\n' "$CONFIGURE_OUT" | grep -q 'PROBE_READ=placeholder'; then
    log_success "first pass read the placeholder, as it must"
else
    fail "first pass did not read the placeholder:
$CONFIGURE_OUT"
fi

check
if printf '%s\n' "$CONFIGURE_OUT" | grep -q 'cmake will run once more'; then
    log_success "the changed answer announced the re-configure"
else
    fail "a changed answer armed nothing, or said nothing about it:
$CONFIGURE_OUT"
fi

run_build "$TEST_TMPDIR/armed-build"
check
if [ "$BUILD_RC" -ne 0 ]; then
    fail "the build failed (rc=$BUILD_RC):
$BUILD_OUT"
elif [ "$BUILD_USED" = "real" ]; then
    log_success "the build used the REAL answer, not the placeholder"
else
    fail "the build used '$BUILD_USED', expected 'real':
$BUILD_OUT"
fi

check
if [ "$BUILD_RERUNS" -eq 1 ]; then
    log_success "exactly 1 re-configure -- it converges"
else
    fail "expected exactly 1 re-configure, saw $BUILD_RERUNS.
  More than one is the future-dated-mtime loop this mechanism must not have:
  \`nros_reconfigure_settle\` is what clears the date on the next pass."
fi

# A settled tree must stay settled. This is the half that a future-dated mtime
# gets wrong for as long as the clock is behind it.
run_build "$TEST_TMPDIR/armed-build"
check
if [ "$BUILD_RERUNS" -eq 0 ] && [ "$BUILD_RC" -eq 0 ]; then
    log_success "a second build re-configures 0 times -- the date was cleared"
else
    fail "a second build re-ran cmake $BUILD_RERUNS time(s) (rc=$BUILD_RC); the future date was not cleared:
$BUILD_OUT"
fi

log_header "D. identical bytes arm nothing"

SRC_SAME="$TEST_TMPDIR/identical-src"
write_project "$SRC_SAME" "identical"
CONFIGURE_OUT="$(cmake -G Ninja -S "$SRC_SAME" -B "$TEST_TMPDIR/identical-build" 2>&1)"
check
if printf '%s\n' "$CONFIGURE_OUT" | grep -q 'cmake will run once more'; then
    fail "a producer that rewrote IDENTICAL bytes armed a re-configure -- every build would re-configure forever"
else
    log_success "identical bytes armed nothing"
fi
run_build "$TEST_TMPDIR/identical-build"
check
if [ "$BUILD_RERUNS" -eq 0 ]; then
    log_success "and the build re-ran cmake 0 times"
else
    fail "identical bytes still caused $BUILD_RERUNS re-configure(s)"
fi

log_header "E. the bound holds: a non-convergent producer warns, it does not loop"

SRC_LOOP="$TEST_TMPDIR/loop-src"
write_project "$SRC_LOOP" "never-settles"
cmake -G Ninja -S "$SRC_LOOP" -B "$TEST_TMPDIR/loop-build" \
    -DNROS_RECONFIGURE_MAX_PASSES=2 >/dev/null 2>&1
run_build "$TEST_TMPDIR/loop-build"
check
if [ "$BUILD_RC" -eq 124 ]; then
    fail "a producer whose answer never settles LOOPED until the timeout -- NROS_RECONFIGURE_MAX_PASSES did not bound it"
elif [ "$BUILD_RC" -ne 0 ]; then
    fail "the bounded build failed for another reason (rc=$BUILD_RC):
$BUILD_OUT"
elif [ "$BUILD_RERUNS" -le 2 ]; then
    log_success "bounded at $BUILD_RERUNS re-configure(s) with MAX_PASSES=2"
else
    fail "MAX_PASSES=2 allowed $BUILD_RERUNS re-configures"
fi

check
if printf '%s\n' "$BUILD_OUT" | grep -q 'NROS_RECONFIGURE_MAX_PASSES'; then
    log_success "and it said WHY it stopped, naming the knob"
else
    fail "the bound was hit silently -- a build sized from a stale answer must say so:
$BUILD_OUT"
fi

log_header "F. settle is a no-op on an ordinary file"

SETTLE_PROBE="$TEST_TMPDIR/settle-probe.cmake"
ORDINARY="$TEST_TMPDIR/ordinary.txt"
echo "content" > "$ORDINARY"
BEFORE_MTIME="$(stat -c %Y "$ORDINARY")"
cat > "$SETTLE_PROBE" <<EOF
include("$MODULE")
nros_reconfigure_settle("$ORDINARY")
nros_reconfigure_settle("$TEST_TMPDIR/does-not-exist.txt")
message(STATUS "SETTLE_OK")
EOF
# The past mtime makes "unchanged" observable: `file(TOUCH)` would move it to
# now, so an equal mtime is proof the no-op path ran rather than the clearing
# one landing on the same second.
touch -d "@$((BEFORE_MTIME - 3600))" "$ORDINARY"
BEFORE_MTIME="$(stat -c %Y "$ORDINARY")"
SETTLE_OUT="$(cmake -P "$SETTLE_PROBE" 2>&1)"
AFTER_MTIME="$(stat -c %Y "$ORDINARY")"
check
if printf '%s\n' "$SETTLE_OUT" | grep -q 'SETTLE_OK' && [ "$BEFORE_MTIME" = "$AFTER_MTIME" ]; then
    log_success "settle left an ordinary file alone, and a missing file is not an error"
else
    fail "settle was not a no-op: before=$BEFORE_MTIME after=$AFTER_MTIME
$SETTLE_OUT"
fi

log_header "G. INTEGRATION: a clean build dir sizes itself from the SUBSCRIBED set"

# The acceptance issue 0991 names, at the level this gate can reach: configure a
# CLEAN build dir once, build, and require that the build was sized from the
# image's declaration rather than from the "nothing composed yet" placeholder.
#
# This drives the REAL modules in the REAL order -- `nros_derive_message_bound_
# knobs` (the reader, standing in for the end of `nros_find_interfaces()`) and
# then `nros_derive_entity_inventory_knobs` (the producer, standing in for
# `nano_ros_entry()`) -- so it fails if either call site loses its
# snapshot/on_change pair, which a module-only test cannot see.
#
# The island's own symptom was one step further down: the closure basis set the
# small payload class from a `std_msgs/Float64MultiArray` the image links
# through and never receives, and the image overflowed RAM by 103160 bytes at
# LINK. The linking half needs a cross toolchain and a 320 KiB part; the BASIS
# half is the cause and is checkable here.

INT_SRC="$TEST_TMPDIR/integration-src"
INT_BUILD="$TEST_TMPDIR/integration-build"
mkdir -p "$INT_SRC"

# Two bounded types. `Float64MultiArray` is the big one the image LINKS and does
# not subscribe to; `Int32` is what it actually receives. The closure basis
# picks the first, the subscribed basis the second -- so the two bases are
# distinguishable by the number alone.
BOUNDS_FRAG="$TEST_TMPDIR/integration-bounds.cmake"
cat > "$BOUNDS_FRAG" <<'EOF'
set(NROS_MESSAGE_BOUNDS_SCHEMA_VERSION 1)
list(APPEND NROS_MESSAGE_BOUND_PACKAGES "std_msgs")
list(APPEND NROS_MESSAGE_BOUND_TYPES "std_msgs/msg/Float64MultiArray")
set(NROS_MESSAGE_BOUND_std_msgs_msg_Float64MultiArray_STATE "bounded")
set(NROS_MESSAGE_BOUND_std_msgs_msg_Float64MultiArray_TX 1496)
set(NROS_MESSAGE_BOUND_std_msgs_msg_Float64MultiArray_RX 1496)
list(APPEND NROS_MESSAGE_BOUND_TYPES "std_msgs/msg/Int32")
set(NROS_MESSAGE_BOUND_std_msgs_msg_Int32_STATE "bounded")
set(NROS_MESSAGE_BOUND_std_msgs_msg_Int32_TX 880)
set(NROS_MESSAGE_BOUND_std_msgs_msg_Int32_RX 880)
list(REMOVE_DUPLICATES NROS_MESSAGE_BOUND_PACKAGES)
list(REMOVE_DUPLICATES NROS_MESSAGE_BOUND_TYPES)
EOF

# The entity fragment the stubbed `nros ws entity-inventory` composes: this
# image subscribes to Int32 only.
# The schema version is READ FROM THE MODULE, never hardcoded. Its sibling
# `cmake-message-bounds-tests.sh` records why in its own words: a literal here
# "silently stopped testing anything it claimed to" the moment the supported
# version moved. It moved again on 2026-09-03 (2 -> 3, phase-403 step 2's
# `@depth=`), and a hardcoded 2 turned this case into a configure FATAL rather
# than a wrong answer -- loud, but for the wrong reason.
ENTITY_SCHEMA="$(sed -n 's/^set(NROS_ENTITY_INVENTORY_SCHEMA_SUPPORTED \([0-9]*\).*/\1/p' \
    "$PROJECT_ROOT/cmake/NanoRosEntityInventory.cmake" | head -1)"
if [ -z "$ENTITY_SCHEMA" ]; then
    fail "could not read NROS_ENTITY_INVENTORY_SCHEMA_SUPPORTED from the module"
    exit 1
fi

ENTITY_BODY="$TEST_TMPDIR/integration-entity.cmake"
cat > "$ENTITY_BODY" <<EOF
set(NROS_ENTITY_INVENTORY_SCHEMA_VERSION ${ENTITY_SCHEMA})
set(NROS_ENTITY_INVENTORY_STATUS "derived")
set(NROS_ENTITY_INVENTORY_COMPONENT_COUNT 1)
set(NROS_ENTITY_INVENTORY_ENTITY_TOTAL 1)
set(NROS_ENTITY_COUNT_SUBSCRIPTION 1)
set(NROS_DERIVED_EXECUTOR_MAX_CBS 1)
set(NROS_ENTITY_SUBSCRIBED_TYPES_STATUS "resolved")
set(NROS_ENTITY_SUBSCRIBED_TYPES "std_msgs/msg/Int32")
set(NROS_ENTITY_SUBSCRIBED_TYPE_COUNTS "std_msgs/msg/Int32=1")
set(NROS_ENTITY_SUBSCRIBED_ENTITY_COUNT 1)
set(NROS_ENTITY_RECEIVED_TYPES_STATUS "resolved")
set(NROS_ENTITY_RECEIVED_TYPES "std_msgs/msg/Int32")
set(NROS_ENTITY_RECEIVED_TYPE_COUNTS "std_msgs/msg/Int32=1")
set(NROS_ENTITY_RECEIVED_ENTITY_COUNT 1)
EOF

INT_STUB="$TEST_TMPDIR/integration-nros"
cat > "$INT_STUB" <<'STUB_EOF'
#!/bin/bash
out=""
prev=""
for a in "$@"; do
    if [ "$prev" = "--output-cmake" ]; then out="$a"; fi
    prev="$a"
done
if [ -n "$out" ]; then
    mkdir -p "$(dirname "$out")"
    cp "$NROS_STUB_BODY" "$out"
fi
exit 0
STUB_EOF
chmod +x "$INT_STUB"

# `nros_derive_entity_inventory_knobs` refuses without a metadata file, exactly
# as a launch-only image would -- so give it one. Its CONTENT is the stub's
# business, not this module's.
INT_META="$TEST_TMPDIR/integration-metadata.json"
echo '{}' > "$INT_META"

cat > "$INT_SRC/CMakeLists.txt" <<EOF
cmake_minimum_required(VERSION 3.20)
project(nros_reconfigure_integration NONE)

include("$PROJECT_ROOT/cmake/NanoRosMessageBounds.cmake")
include("$PROJECT_ROOT/cmake/NanoRosEntityInventory.cmake")
include("$PROJECT_ROOT/cmake/NanoRosReconfigure.cmake")

nros_entity_inventory_knobs_file(_entity)
nros_message_bounds_knobs_file(_bounds)
nros_entity_inventory_seed_knobs_file("\${_entity}")
nros_message_bounds_seed_knobs_file("\${_bounds}")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "\${_entity}" "\${_bounds}")

# ---- the READER half, as \`nros_find_interfaces()\` runs it ---------------------
nros_reconfigure_settle("\${_entity}")
nros_reconfigure_settle("\${_bounds}")
nros_reconfigure_snapshot("\${_bounds}" _bounds_before)
nros_derive_message_bound_knobs(
    FRAGMENTS "$BOUNDS_FRAG"
    OUTPUT_FILE "\${_bounds}"
    ENTITY_INVENTORY "\${_entity}")
nros_reconfigure_on_change("\${_bounds}" "\${_bounds_before}"
    LABEL "this image's derived message-bound sizes")

# ---- the PRODUCER half, as \`nano_ros_entry()\` runs it ------------------------
nros_reconfigure_snapshot("\${_entity}" _entity_before)
nros_derive_entity_inventory_knobs(CLI "$INT_STUB" METADATA "$INT_META" QUIET)
nros_reconfigure_on_change("\${_entity}" "\${_entity_before}"
    LABEL "this image's entity inventory")

add_custom_target(go ALL COMMAND \${CMAKE_COMMAND} -E echo
    "PROBE_BASIS=\${NROS_MESSAGE_BOUNDS_BASIS} PROBE_SMALL=\${NROS_DERIVED_SUBSCRIBER_BUFFER_SIZE}")
EOF

NROS_STUB_BODY="$ENTITY_BODY" \
    cmake -G Ninja -S "$INT_SRC" -B "$INT_BUILD" >/dev/null 2>&1
INT_OUT="$(NROS_STUB_BODY="$ENTITY_BODY" timeout 120 ninja -C "$INT_BUILD" 2>&1)"
INT_RC=$?
INT_BASIS="$(printf '%s\n' "$INT_OUT" | sed -n 's/.*PROBE_BASIS=\([a-z]*\).*/\1/p' | tail -1)"
INT_SMALL="$(printf '%s\n' "$INT_OUT" | sed -n 's/.*PROBE_SMALL=\([0-9]*\).*/\1/p' | tail -1)"

check
if [ "$INT_RC" -ne 0 ]; then
    fail "the integration build failed (rc=$INT_RC):
$INT_OUT"
elif [ "$INT_BASIS" = "subscribed" ]; then
    log_success "one build from nothing: basis=subscribed (not the closure placeholder)"
else
    fail "a CLEAN build dir built on basis='$INT_BASIS', expected 'subscribed'.
  That is issue 0991 exactly: the pass that BUILT was sized from the entity
  fragment's placeholder, so the payload classes came from the linked closure.
$INT_OUT"
fi

check
if [ "$INT_SMALL" = "880" ]; then
    log_success "and the small payload class is 880 -- the type it receives, not the 1496 it merely links"
else
    fail "the small payload class built as '$INT_SMALL', expected 880.
  1496 is the closure answer (std_msgs/msg/Float64MultiArray), which this image
  links through and never receives -- the island's 103160-byte overflow.
$INT_OUT"
fi

log_header "H. the value the BUILD USES settles, not just the fragment (issue 1002)"

# Case G above drives the two PRODUCERS. It cannot see issue 1002, and does not:
# what it reads is `NROS_DERIVED_SUBSCRIBER_BUFFER_SIZE` as
# `nros_derive_message_bound_knobs` left it in the CALLER's scope -- the
# FRAGMENT's answer, which is already right on pass 2. The value a build is
# COMPILED with comes from somewhere else entirely: `nros_resolve_knobs()`,
# which runs inside `find_package(Zephyr)`, i.e. before EITHER producer, and
# therefore reads the fragment the PREVIOUS pass wrote.
#
# So the real chain has three links, not two:
#
#   STAGE R   earliest reader   nros_resolve_knobs   (find_package(Zephyr))
#   STAGE P1  mid producer      nros_find_interfaces (bounds fragment)
#   STAGE P2  latest producer   nano_ros_entry       (entity fragment)
#
# and the delivered value settles on the THIRD configure:
#
#   pass 1  entity=placeholder  bounds=closure(1496)     delivered: nothing
#   pass 2  entity=real         bounds=subscribed(880)   delivered: 1496  <- stale
#   pass 3  entity=real         bounds=subscribed(880)   delivered: 880
#
# MUTATION THIS CATCHES, and case G does not: set
# NROS_RECONFIGURE_MAX_PASSES=1 -- which is exactly what this module's own
# prose used to claim ("exactly ONE extra configure"). The build then stops
# after pass 2 and compiles at 1496, the island's 103160-byte overflow, while
# case G reports 13/13 green. Measured, 2026-09-05.

# `$2` is HOW MANY TIMES the bounds producer runs in one configure, which is
# case J's whole subject: `nros_find_interfaces()` is called once per package
# that declares an interface dependency, not once per image. Case H passes 1.
write_chain_project() {
    local dir="$1" calls="${2:-1}"
    mkdir -p "$dir"
    cat > "$dir/CMakeLists.txt" <<EOF
cmake_minimum_required(VERSION 3.20)
project(nros_reconfigure_chain NONE)

include("$PROJECT_ROOT/cmake/NanoRosMessageBounds.cmake")
include("$PROJECT_ROOT/cmake/NanoRosEntityInventory.cmake")
include("$PROJECT_ROOT/cmake/NanoRosReconfigure.cmake")

nros_entity_inventory_knobs_file(_entity)
nros_message_bounds_knobs_file(_bounds)
nros_entity_inventory_seed_knobs_file("\${_entity}")
nros_message_bounds_seed_knobs_file("\${_bounds}")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "\${_entity}" "\${_bounds}")

# ---- STAGE R: the EARLIEST reader, as \`nros_resolve_knobs()\` runs it ---------
# Settling belongs here for the reason the Zephyr loaders settle here: it is the
# first thing in the configure that touches either fragment, so the window in
# which a file is future-dated is as short as it can be.
nros_reconfigure_settle("\${_bounds}")
nros_reconfigure_settle("\${_entity}")
include("\${_bounds}")
set(_delivered "\${NROS_DERIVED_SUBSCRIBER_BUFFER_SIZE}")
if("\${_delivered}" STREQUAL "")
    set(_delivered "unset")
endif()

# ---- STAGE P1: the bounds producer, as \`nros_find_interfaces()\` runs it ------
# Body and ORDER copied from \`nros_find_interfaces()\`: settle both fragments,
# snapshot the bounds one, derive, arm. Wrapped in a function and called
# \${calls} times because that verb runs ONCE PER PACKAGE.
function(probe_find_interfaces)
    nros_reconfigure_settle("\${_entity}")
    nros_reconfigure_settle("\${_bounds}")
    nros_reconfigure_snapshot("\${_bounds}" _bounds_before)
    nros_derive_message_bound_knobs(
        FRAGMENTS "$BOUNDS_FRAG"
        OUTPUT_FILE "\${_bounds}"
        ENTITY_INVENTORY "\${_entity}")
    nros_reconfigure_on_change("\${_bounds}" "\${_bounds_before}"
        LABEL "this image's derived message-bound sizes")
endfunction()

foreach(_call RANGE 1 $calls)
    probe_find_interfaces()
endforeach()

# ---- STAGE P2: the entity producer, as \`nano_ros_entry()\` runs it ------------
nros_reconfigure_snapshot("\${_entity}" _entity_before)
nros_derive_entity_inventory_knobs(CLI "$INT_STUB" METADATA "$INT_META" QUIET)
nros_reconfigure_on_change("\${_entity}" "\${_entity_before}"
    LABEL "this image's entity inventory")

add_custom_target(go ALL COMMAND \${CMAKE_COMMAND} -E echo
    "PROBE_DELIVERED=\${_delivered}")
EOF
}

# Configure + build the chain project and report what the build was SIZED FROM.
run_chain() {
    local src="$1" build="$2" body="$3" out
    out="$(NROS_STUB_BODY="$body" timeout 180 ninja -C "$build" 2>&1)"
    CHAIN_RC=$?
    CHAIN_OUT="$out"
    CHAIN_DELIVERED="$(printf '%s\n' "$out" | sed -n 's/.*PROBE_DELIVERED=\([A-Za-z0-9_]*\).*/\1/p' | tail -1)"
    CHAIN_RERUNS="$(printf '%s\n' "$out" | grep -c 'Re-running CMake')"
}

CHAIN_SRC="$TEST_TMPDIR/chain-src"
CHAIN_BUILD="$TEST_TMPDIR/chain-build"
write_chain_project "$CHAIN_SRC" 1
NROS_STUB_BODY="$ENTITY_BODY" \
    cmake -G Ninja -S "$CHAIN_SRC" -B "$CHAIN_BUILD" >/dev/null 2>&1
run_chain "$CHAIN_SRC" "$CHAIN_BUILD" "$ENTITY_BODY"

check
if [ "$CHAIN_RC" -ne 0 ]; then
    fail "the chain build failed (rc=$CHAIN_RC):
$CHAIN_OUT"
elif [ "$CHAIN_DELIVERED" = "880" ]; then
    log_success "one build from nothing: the build was SIZED FROM 880, the subscribed class"
else
    fail "the build was sized from '$CHAIN_DELIVERED', expected 880.
  This is issue 1002: the FRAGMENT holds 880 by pass 2 (case G checks that and
  passes), but the value a reader running BEFORE the producers hands to the
  compile is one pass behind it. 1496 means the chain stopped one link short --
  check NROS_RECONFIGURE_MAX_PASSES and that both producers still arm.
$CHAIN_OUT"
fi

# The pass count is MEASURED here rather than asserted from memory, because
# "two passes" was folklore for as long as it was written down. It is the
# chain's DEPTH: one arm per producer upstream of the reader. If a third
# producer joins the chain this number moves, and it must move HERE -- where a
# reader also sees how much headroom is left against NROS_RECONFIGURE_MAX_PASSES.
check
if [ "$CHAIN_RERUNS" -eq 2 ]; then
    log_success "2 automatic re-configures = 3 configures total, the chain's depth (bound is $(sed -n 's/^set(NROS_RECONFIGURE_MAX_PASSES \([0-9]*\).*/\1/p' "$MODULE" | head -1) consecutive arms per fragment)"
else
    fail "expected 2 automatic re-configures for a two-producer chain, saw $CHAIN_RERUNS.
  Fewer means a link stopped arming and the build is sized from a stale answer.
  More means a producer is not settling -- and if it approaches
  NROS_RECONFIGURE_MAX_PASSES the build will start shipping the previous answer
  with a warning.
$CHAIN_OUT"
fi

# FIXPOINT. Issue 1002's "Not covered" asked for exactly this and it is the one
# assertion that does not need anyone to know the right pass count in advance.
run_chain "$CHAIN_SRC" "$CHAIN_BUILD" "$ENTITY_BODY"
check
if [ "$CHAIN_RERUNS" -eq 0 ] && [ "$CHAIN_DELIVERED" = "880" ]; then
    log_success "and the sequence reached a FIXPOINT -- a second build re-configures 0 times"
else
    fail "no fixpoint: a second build re-ran cmake $CHAIN_RERUNS time(s), delivered '$CHAIN_DELIVERED':
$CHAIN_OUT"
fi

log_header "I. the bound counts CONSECUTIVE arms, not the build dir's lifetime (issue 1002)"

# The counter lives in the CACHE, whose lifetime is the build dir. Reading that
# as the bound's SCOPE is a defect with a short fuse, and this is the
# regression: a clean build of the chain above spends 2 of the 3 allowed arms,
# the FIRST declaration edit spends the third, and from the SECOND edit on the
# bound is exhausted for the life of the directory -- every later edit warns and
# compiles the PREVIOUS answer.
#
# Measured on the unfixed module, same project, editing which type the image
# subscribes to:
#
#   clean          -> 880   (correct)
#   after edit 1   -> 1496  (correct)
#   after edit 2   -> 1496  WRONG, declaration says 880, bound exhausted
#   after edit 3   -> 880   WRONG, declaration says 1496 -- and this direction is
#                           UNDER-sized, the half issue 0991 calls unsafe
#
# So this is not "the number is off by one". A directory that has been edited
# twice stops delivering, and which way it fails depends on the edit.

edit_subscribed_type() {   # <out-file> <type>
    local out="$1" t="$2"
    sed -e "s|^set(NROS_ENTITY_SUBSCRIBED_TYPES \".*\")|set(NROS_ENTITY_SUBSCRIBED_TYPES \"$t\")|" \
        -e "s|^set(NROS_ENTITY_SUBSCRIBED_TYPE_COUNTS \".*\")|set(NROS_ENTITY_SUBSCRIBED_TYPE_COUNTS \"$t=1\")|" \
        -e "s|^set(NROS_ENTITY_RECEIVED_TYPES \".*\")|set(NROS_ENTITY_RECEIVED_TYPES \"$t\")|" \
        -e "s|^set(NROS_ENTITY_RECEIVED_TYPE_COUNTS \".*\")|set(NROS_ENTITY_RECEIVED_TYPE_COUNTS \"$t=1\")|" \
        "$ENTITY_BODY" > "$out"
}

EDIT_BODY_BIG="$TEST_TMPDIR/entity-big.cmake"
EDIT_BODY_SMALL="$TEST_TMPDIR/entity-small.cmake"
edit_subscribed_type "$EDIT_BODY_BIG" "std_msgs/msg/Float64MultiArray"
edit_subscribed_type "$EDIT_BODY_SMALL" "std_msgs/msg/Int32"

# Four edits in the SAME build dir, alternating so a stale answer is always
# visibly the wrong one. A declaration change is a CMakeLists change, so the
# touch is what a real edit does, not a way of forcing the issue.
LIFETIME_OK=1
LIFETIME_LOG=""
for _step in 1:big:1496 2:small:880 3:big:1496 4:small:880; do
    _n="${_step%%:*}"; _rest="${_step#*:}"; _which="${_rest%%:*}"; _want="${_rest##*:}"
    if [ "$_which" = "big" ]; then _body="$EDIT_BODY_BIG"; else _body="$EDIT_BODY_SMALL"; fi
    touch "$CHAIN_SRC/CMakeLists.txt"
    run_chain "$CHAIN_SRC" "$CHAIN_BUILD" "$_body"
    LIFETIME_LOG="$LIFETIME_LOG
  edit $_n ($_which): delivered=$CHAIN_DELIVERED want=$_want reruns=$CHAIN_RERUNS"
    if [ "$CHAIN_DELIVERED" != "$_want" ]; then
        LIFETIME_OK=0
    fi
    if printf '%s\n' "$CHAIN_OUT" | grep -q 'NROS_RECONFIGURE_MAX_PASSES'; then
        LIFETIME_OK=0
        LIFETIME_LOG="$LIFETIME_LOG  <-- hit the bound"
    fi
done

check
if [ "$LIFETIME_OK" -eq 1 ]; then
    log_success "four declaration edits in one build dir all delivered their own answer"
else
    fail "the build dir stopped converging as it aged:$LIFETIME_LOG

  The per-fragment counter is not being reset when a fragment SETTLES, so it is
  counting the build dir's lifetime instead of one convergence episode. A clean
  build of this chain already spends 2 of NROS_RECONFIGURE_MAX_PASSES, which
  leaves room for exactly one edit."
fi

log_header "J. a producer called ONCE PER PACKAGE still delivers (issue 1119)"

# Case H drives the chain with ONE bounds producer call per configure. No image
# in this tree has that shape: `nros_find_interfaces()` runs from every leaf
# that declares an interface dependency, so a two-package image calls it twice
# in one configure and a workspace calls it once per node package.
#
# MEASURED on the safety island (MR-CANHUBK344, clean build dir, one
# `west build`) and reproduced by this case exactly:
#
#   configures actually run     2      <- should be 3
#   re-configures armed         3
#   value the build was sized from  1496, the pass-2 answer
#
# Two per-CALL side effects did it, and either alone is enough:
#
#   * the SETTLE at the head of the verb cleared the future date the PREVIOUS
#     call in the same configure had armed, so `build.ninja` came out newer
#     than the fragment and ninja had nothing stale to re-run cmake for;
#   * the per-call snapshot took the bytes the PREVIOUS call wrote as its
#     "before", so a fragment that HAD changed since the readers ran compared
#     equal, reported itself settled, and cleared the pass counter.
#
# On the island that shipped a 70296-byte arena where 46272 is correct: 24 KB
# on a 320 KB part, and both images boot, which is why nobody saw it.
#
# WHAT THIS ASSERTS THAT CASE H CANNOT: the number of configures and the
# delivered value are invariant in HOW MANY TIMES the producer runs per pass.
# That is the property, not "three configures" -- three is the chain's depth and
# case H owns it. Run at 2 and 3 calls, because 1 is case H and the defect is
# "the second call undoes the first".

for _calls in 2 3; do
    _j_src="$TEST_TMPDIR/multi$_calls-src"
    _j_build="$TEST_TMPDIR/multi$_calls-build"
    write_chain_project "$_j_src" "$_calls"
    NROS_STUB_BODY="$ENTITY_BODY" \
        cmake -G Ninja -S "$_j_src" -B "$_j_build" >/dev/null 2>&1
    run_chain "$_j_src" "$_j_build" "$ENTITY_BODY"

    check
    if [ "$CHAIN_RC" -ne 0 ]; then
        fail "the $_calls-package chain build failed (rc=$CHAIN_RC):
$CHAIN_OUT"
    elif [ "$CHAIN_DELIVERED" = "880" ] && [ "$CHAIN_RERUNS" -eq 2 ]; then
        log_success "$_calls producer calls per configure: still 3 configures, still SIZED FROM 880"
    else
        fail "with the bounds producer called $_calls times per configure the build was
  sized from '$CHAIN_DELIVERED' after $((CHAIN_RERUNS + 1)) configure(s); expected 880 after 3.

  That is issue 1119. The pass BASELINE and the ARM must belong to the
  configure, not to one call of the producer: a later call must not settle a
  date an earlier one armed, and must not measure itself against what the
  earlier one wrote. 1496 with 2 configures is the island's measurement.
$CHAIN_OUT"
    fi

    # And the extra calls must not cost extra passes of the bound, or an image
    # with enough leaves exhausts NROS_RECONFIGURE_MAX_PASSES on a CLEAN build
    # dir and ships the previous answer with a warning.
    check
    if printf '%s\n' "$CHAIN_OUT" | grep -q 'NROS_RECONFIGURE_MAX_PASSES'; then
        fail "$_calls producer calls per configure exhausted the pass bound on a clean build dir:
$CHAIN_OUT"
    else
        log_success "and $_calls calls per configure spent no more of the bound than 1 does"
    fi

    run_chain "$_j_src" "$_j_build" "$ENTITY_BODY"
    check
    if [ "$CHAIN_RERUNS" -eq 0 ] && [ "$CHAIN_DELIVERED" = "880" ]; then
        log_success "and it reaches the same FIXPOINT -- a second build re-configures 0 times"
    else
        fail "no fixpoint at $_calls calls: a second build re-ran cmake $CHAIN_RERUNS time(s), delivered '$CHAIN_DELIVERED':
$CHAIN_OUT"
    fi

    # Case I's question at this shape. The bound is what a per-CALL arm burned
    # fastest -- N leaves spent N of it per configure -- so an aged build dir is
    # where the two defects meet. Two alternating edits, same dir.
    _j_ok=1
    _j_log=""
    for _step in big:1496 small:880; do
        _which="${_step%%:*}"; _want="${_step##*:}"
        if [ "$_which" = "big" ]; then _body="$EDIT_BODY_BIG"; else _body="$EDIT_BODY_SMALL"; fi
        touch "$_j_src/CMakeLists.txt"
        run_chain "$_j_src" "$_j_build" "$_body"
        _j_log="$_j_log
  edit ($_which): delivered=$CHAIN_DELIVERED want=$_want reruns=$CHAIN_RERUNS"
        [ "$CHAIN_DELIVERED" = "$_want" ] || _j_ok=0
        if printf '%s\n' "$CHAIN_OUT" | grep -q 'NROS_RECONFIGURE_MAX_PASSES'; then
            _j_ok=0
            _j_log="$_j_log  <-- hit the bound"
        fi
    done
    check
    if [ "$_j_ok" -eq 1 ]; then
        log_success "and two declaration edits in that dir each delivered their own answer"
    else
        fail "at $_calls producer calls per configure the dir stopped converging as it aged:$_j_log

  The arm is being counted per CALL again, so an image with N interface leaves
  spends N of NROS_RECONFIGURE_MAX_PASSES per configure instead of one."
    fi
done

log_header "Result"
if [ "$FAILURES" -eq 0 ]; then
    log_success "$CHECKS/$CHECKS assertions held"
    exit 0
fi
log_error "$FAILURES of $CHECKS assertions failed"
exit 1
