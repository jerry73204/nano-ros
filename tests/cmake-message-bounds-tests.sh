#!/bin/bash
# tests/cmake-message-bounds-tests.sh -- phase-403 W8 (issue 0940)
#
# Exercises the READER for the exported message-bound inventory:
# `cmake/NanoRosMessageBounds.cmake`, driven in `cmake -P` script mode against
# hand-written fragments. No nano-ros build, no toolchain, no codegen -- the
# fragments are the codegen CONTRACT, and writing them by hand is what lets a
# case like "one package is a schema ahead of the other" exist at all.
#
# WHAT IS ACTUALLY ASSERTED, and why each assertion exists:
#
#   A. A bounded closure DERIVES all four numbers, and derives the ones the
#      island bring-up produced by eye. The small class is the largest bound at
#      or under the split; the large count is how many types are above it; the
#      large size is the biggest of those; the take buffer is the biggest of
#      all. Asserting the VALUES and not just "it ran" is the point -- W6's
#      prototype was a one-off `cmake -P` whose answer nobody re-derived.
#
#   B. COMPOSITION over several packages. The image-wide answer is a property
#      of the closure, not of any one package: two fragments whose largest
#      types differ must produce the larger. A reader that took the last
#      fragment's answer would pass a single-package test.
#
#   C. An UNBOUNDED type REFUSES the whole derivation and names the type and
#      the member that costs it. This is requirement 3 of the wave and the one
#      that is tempting to get wrong: deriving over the bounded subset yields a
#      plausible number that a real sample can exceed, and the drop is silent
#      on the C++ arena path. The negative control matters as much -- the same
#      closure with the type bounded DOES derive.
#
#   D. An UNRESOLVED type refuses too. It is a different fact from unbounded
#      (a search-path problem, not a property of the message) and licenses the
#      same action: nothing may be sized from it.
#
#   E. A SCHEMA VERSION the reader does not understand is a FATAL_ERROR, not a
#      field-by-field read on the hope that nothing moved. Covers both shapes:
#      a wrong version, and a fragment that states none at all.
#
#   F. A MISSING fragment refuses rather than fataling -- on the canonical lane
#      codegen is a build-time custom command, so on a clean tree the file is a
#      promise. Refusing keeps that lane building; fataling would break it.
#
#   G. MAX_LARGE_SUBSCRIBERS = 0 is an ANSWER (every type fits the small
#      class), and the large SIZE is then deliberately NOT derived -- with zero
#      blocks the pool is zero bytes whatever size it names, and naming one
#      would be inventing a number. This is the case the reference image is in,
#      against a hand-set 2 / 2560.
#
#   I. The CONSUMER SEAM in a real configure + build. The knobs file is read
#      through CMAKE_CONFIGURE_DEPENDS, and that has a failure mode `cmake -P`
#      cannot show: a ninja input with no producing rule is `missing and no
#      known rule to make it`, raised at LOAD, which makes the whole build dir
#      unusable. The Zephyr lane is not buildable on a bare host; this seam is
#      generic cmake+ninja and is.
#
#   J. The REGISTRY path -- the one `nros_find_interfaces()` takes, where the
#      generators register fragments as they emit them and the composer is
#      called with none. Every other case passes fragments explicitly and would
#      keep passing with the registry broken.
#
#   L. THE JOIN (phase-403 step 1). The three payload-class knobs derive over
#      the types the image RECEIVES, not over everything it links. Asserted as
#      a BEFORE/AFTER on one closure so the difference is the entity
#      declaration and nothing else -- the island's own shape, where the small
#      class is set by a `Float64MultiArray` that is linked and never received.
#      Includes the take buffer NOT moving: it is also DEFAULT_TX_BUF and every
#      raw entity's default buffer, so narrowing it would size a buffer too
#      small, which is the failure this campaign exists to remove.
#
#   M. THE JOIN REFUSES rather than widening. Three shapes, all of them "the
#      image DID declare and the join still cannot answer": the subscribed set
#      itself refused, a named type the bound inventory does not price (which
#      is every `pkg/srv/*` and `pkg/action/*` today -- the inventory records
#      MESSAGES only), and a fragment that states the schema and not the field.
#      None may fall back to the closure: that publishes a number derived over
#      types the image never receives while the status still reads `derived`.
#
#   N. BACK-COMPAT. An image that declares nothing -- no fragment, or a
#      fragment whose own status is `refused` -- keeps W8's closure answer
#      byte for byte, labelled `basis = closure`. Refusing there would take
#      every pre-W9 image from a derived number back to the hand-set ones.
#      A fragment from a STALE CLI is the same case with a warning.
#
#   H. The OUTPUT FILE carries the answer AND the provenance, and is
#      write-if-changed. The second is load-bearing rather than tidy: the
#      consumer registers the file with CMAKE_CONFIGURE_DEPENDS, so rewriting
#      identical bytes every configure would re-arm a reconfigure forever.
#
# PRECONDITIONS ARE HARD FAILURES. This script never prints-and-returns: a
# green here is read as "the derivation is sound". Skipping belongs in the
# `just` recipe's check ledger, which is a different claim from "it passed".
#
# Usage: ./tests/cmake-message-bounds-tests.sh
# Exit:  0 all assertions held; 1 otherwise.

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# shellcheck source=lib/common.sh
source "$SCRIPT_DIR/lib/common.sh"

MODULE="$PROJECT_ROOT/cmake/NanoRosMessageBounds.cmake"

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

init_test_tmpdir "nros-message-bounds"
trap 'cleanup_test_tmpdir' EXIT

# ---------------------------------------------------------------------------
# Fragment fixtures. Byte-for-byte the shape `rosidl_codegen::bounds`'s
# `BoundInventory::to_cmake` emits (see its `to_cmake` and the golden tests
# beside it) -- if that emitter changes shape, these stop matching and this
# test is where it is noticed.
# ---------------------------------------------------------------------------
frag() {
    # frag <path> <package> ; then rows on stdin as: <type>|<state>|<tx>|<rx>|<reason>
    local path="$1" pkg="$2"
    {
        echo "set(NROS_MESSAGE_BOUNDS_SCHEMA_VERSION 1)"
        echo "list(APPEND NROS_MESSAGE_BOUND_PACKAGES \"$pkg\")"
        while IFS='|' read -r t state tx rx reason; do
            [ -z "$t" ] && continue
            local key
            key="$(echo "$t" | sed 's/[^A-Za-z0-9]/_/g')"
            echo "list(APPEND NROS_MESSAGE_BOUND_TYPES \"$t\")"
            echo "set(NROS_MESSAGE_BOUND_${key}_STATE \"$state\")"
            if [ "$state" = "bounded" ]; then
                echo "set(NROS_MESSAGE_BOUND_${key}_TX $tx)"
                echo "set(NROS_MESSAGE_BOUND_${key}_RX $rx)"
            else
                echo "set(NROS_MESSAGE_BOUND_${key}_REASON \"$reason\")"
            fi
        done
        echo "list(REMOVE_DUPLICATES NROS_MESSAGE_BOUND_PACKAGES)"
        echo "list(REMOVE_DUPLICATES NROS_MESSAGE_BOUND_TYPES)"
    } > "$path"
}

# Run the derivation. $1 = ";"-joined fragment list, rest = extra -D args.
# Captures stdout+stderr; the caller greps it.
derive() {
    local frags="$1"; shift
    cmake -DNROS_BOUNDS_FRAGMENTS="$frags" "$@" -P "$MODULE" 2>&1
}

# phase-403 step 1. An ENTITY-inventory fragment, byte-for-byte the shape
# `nros_cli_core::entity_inventory::EntityInventory::to_cmake` emits -- the same
# contract-by-hand-written-fixture the `frag()` helper above is, and for the
# same reason: a case like "the fragment is a schema behind" cannot exist if the
# producer writes it.
#
#   entity_frag <path> <inventory-status> <subscribed-status> [<type>=<count>...]
entity_frag() {
    local path="$1" inv_status="$2" sub_status="$3"; shift 3
    {
        echo "set(NROS_ENTITY_INVENTORY_SCHEMA_VERSION 3)"
        echo "set(NROS_ENTITY_INVENTORY_STATUS \"$inv_status\")"
        echo "set(NROS_ENTITY_INVENTORY_COMPONENT_COUNT 1)"
        echo "set(NROS_ENTITY_SUBSCRIBED_TYPES_STATUS \"$sub_status\")"
        if [ "$sub_status" = "resolved" ]; then
            local names="" pairs=""
            for e in "$@"; do
                names="${names:+$names;}${e%=*}"
                pairs="${pairs:+$pairs;}$e"
            done
            echo "set(NROS_ENTITY_SUBSCRIBED_TYPES \"$names\")"
            echo "set(NROS_ENTITY_SUBSCRIBED_TYPE_COUNTS \"$pairs\")"
        else
            echo "set(NROS_ENTITY_SUBSCRIBED_TYPES_REASON \"a component declared no ENTITIES\")"
        fi
    } > "$path"
}

T="$TEST_TMPDIR"

# ---------------------------------------------------------------------------
# A + B + G. The reference image's shape: a bounded closure over two packages,
# every type under the 2048 B split.
#
# The numbers are the island's own derived bounds (phase-403 W6/W7 measured
# them against /opt/ros/humble): Control 114, VelocityReport 108, Odometry 880,
# the rest 21-27. The hand-set answers for this closure were
# MAX_LARGE_SUBSCRIBERS=2 and SUBSCRIBER_LARGE_SIZE=2560, read off C++ headers
# that state an ESTIMATE; the derived answers are 0 and "not derived".
# ---------------------------------------------------------------------------
log_header "A/B/G -- a fully bounded closure derives, and answers ZERO large"
frag "$T/a1.cmake" "autoware_control_msgs" <<'ROWS'
autoware_control_msgs/msg/Control|bounded|78|114|
ROWS
frag "$T/a2.cmake" "nav_msgs" <<'ROWS'
nav_msgs/msg/Odometry|bounded|836|880|
autoware_vehicle_msgs/msg/VelocityReport|bounded|96|108|
autoware_planning_msgs/msg/RouteState|bounded|13|21|
ROWS
OUT="$(derive "$T/a1.cmake;$T/a2.cmake")"

check
if ! nros_grep_q "NROS_MESSAGE_BOUNDS_STATUS=derived" <<< "$OUT"; then
    fail "A: a fully bounded closure did not derive:"; echo "$OUT"
fi
check
if ! nros_grep_q "NROS_DERIVED_SUBSCRIPTION_BUFFER_SIZE=880" <<< "$OUT"; then
    fail "A: take buffer is not the largest bound in the closure (880):"; echo "$OUT"
fi
check
if ! nros_grep_q "NROS_DERIVED_SUBSCRIBER_BUFFER_SIZE=880" <<< "$OUT"; then
    fail "A: small class is not the largest bound under the split (880):"; echo "$OUT"
fi
check
if ! nros_grep_q "NROS_DERIVED_MAX_LARGE_SUBSCRIBERS=0" <<< "$OUT"; then
    fail "G: no type is over the split, so the large count must be 0:"; echo "$OUT"
fi
check
if nros_grep_q "NROS_DERIVED_SUBSCRIBER_LARGE_SIZE=" <<< "$OUT"; then
    fail "G: the large SIZE must not be derived when the count is 0 (a size for a class that does not exist is an invented number):"; echo "$OUT"
fi
check
# B is the composition claim: the answer came from a1's package AND a2's. A
# reader that kept only the last fragment would say 880 too, so assert the
# type COUNT, which only composition can reach.
if ! nros_grep_q "NROS_MESSAGE_BOUNDS_TYPE_COUNT=4" <<< "$OUT"; then
    fail "B: fragments did not compose -- expected 4 types across 2 packages:"; echo "$OUT"
fi

# ---------------------------------------------------------------------------
# A (large half). Add one type above the split and both large knobs appear.
# ---------------------------------------------------------------------------
log_header "A -- a type over the split drives the large class"
frag "$T/b1.cmake" "sensor_msgs" <<'ROWS'
sensor_msgs/msg/JointState|bounded|4204|4208|
sensor_msgs/msg/Imu|bounded|300|320|
sensor_msgs/msg/LaserScan|bounded|2500|2600|
ROWS
OUT="$(derive "$T/a2.cmake;$T/b1.cmake")"
check
if ! nros_grep_q "NROS_DERIVED_MAX_LARGE_SUBSCRIBERS=2" <<< "$OUT"; then
    fail "A: two types exceed 2048, so the large count must be 2:"; echo "$OUT"
fi
check
if ! nros_grep_q "NROS_DERIVED_SUBSCRIBER_LARGE_SIZE=4208" <<< "$OUT"; then
    fail "A: the large class must hold the largest type above the split (4208):"; echo "$OUT"
fi
check
if ! nros_grep_q "NROS_DERIVED_SUBSCRIBER_BUFFER_SIZE=880" <<< "$OUT"; then
    fail "A: the small class must ignore the types routed large (880):"; echo "$OUT"
fi
check
if ! nros_grep_q "NROS_DERIVED_SUBSCRIPTION_BUFFER_SIZE=4208" <<< "$OUT"; then
    fail "A: the take buffer is one global size and must hold the LARGEST type (4208):"; echo "$OUT"
fi

# The split is a POLICY input, so moving it must move the classification. A
# reader that hardcoded 2048 passes every assertion above.
log_header "A -- the class split is an input, not a constant"
OUT="$(derive "$T/a2.cmake;$T/b1.cmake" -DNROS_BOUNDS_CEILING=512)"
check
if ! nros_grep_q "NROS_DERIVED_MAX_LARGE_SUBSCRIBERS=3" <<< "$OUT"; then
    fail "A: at a 512 B split, three types are large:"; echo "$OUT"
fi
check
if ! nros_grep_q "NROS_DERIVED_SUBSCRIBER_BUFFER_SIZE=320" <<< "$OUT"; then
    fail "A: at a 512 B split, the small class is the largest bound <= 512 (320):"; echo "$OUT"
fi

# ---------------------------------------------------------------------------
# C. One unbounded type refuses the WHOLE derivation.
# ---------------------------------------------------------------------------
log_header "C -- one unbounded type refuses everything, and says which"
frag "$T/c1.cmake" "std_msgs" <<'ROWS'
std_msgs/msg/Header|unbounded|||unbounded member: frame_id (string)
ROWS
OUT="$(derive "$T/a2.cmake;$T/c1.cmake")"
check
if ! nros_grep_q "NROS_MESSAGE_BOUNDS_STATUS=refused" <<< "$OUT"; then
    fail "C: an unbounded type in the closure must refuse the derivation:"; echo "$OUT"
fi
for v in NROS_DERIVED_SUBSCRIBER_BUFFER_SIZE NROS_DERIVED_SUBSCRIBER_LARGE_SIZE \
         NROS_DERIVED_MAX_LARGE_SUBSCRIBERS NROS_DERIVED_SUBSCRIPTION_BUFFER_SIZE; do
    check
    if nros_grep_q "^-- $v=" <<< "$OUT"; then
        fail "C: $v was published despite an unbounded type -- deriving over the bounded subset publishes a maximum a real sample can exceed:"; echo "$OUT"
    fi
done
check
if ! nros_grep_q "std_msgs/msg/Header" <<< "$OUT"; then
    fail "C: the refusal must NAME the type:"; echo "$OUT"
fi
check
if ! nros_grep_q "frame_id" <<< "$OUT"; then
    fail "C: the refusal must name the MEMBER that costs the bound:"; echo "$OUT"
fi
check
if ! nros_grep_q -qi "nros-codegen.toml" <<< "$OUT"; then
    fail "C: the refusal must name a remedy the user can act on:"; echo "$OUT"
fi
# Negative control: the same closure with that type bounded DOES derive. A
# refusal that fires unconditionally would pass every assertion above.
frag "$T/c2.cmake" "std_msgs" <<'ROWS'
std_msgs/msg/Header|bounded|84|92|
ROWS
OUT="$(derive "$T/a2.cmake;$T/c2.cmake")"
check
if ! nros_grep_q "NROS_MESSAGE_BOUNDS_STATUS=derived" <<< "$OUT"; then
    fail "C control: bounding the offending type must let the derivation through:"; echo "$OUT"
fi

# ---------------------------------------------------------------------------
# D. `unresolved` is a different fact and licenses the same refusal.
# ---------------------------------------------------------------------------
log_header "D -- an unresolved type refuses too"
frag "$T/d1.cmake" "some_pkg" <<'ROWS'
some_pkg/msg/Thing|unresolved|||nested type `other_pkg/Widget` could not be resolved
ROWS
OUT="$(derive "$T/a2.cmake;$T/d1.cmake")"
check
if ! nros_grep_q "NROS_MESSAGE_BOUNDS_STATUS=refused" <<< "$OUT"; then
    fail "D: an unresolved type must refuse the derivation:"; echo "$OUT"
fi
check
if ! nros_grep_q "could not be resolved" <<< "$OUT"; then
    fail "D: the refusal must carry the unresolved reason verbatim:"; echo "$OUT"
fi

# ---------------------------------------------------------------------------
# E. Schema version. Both shapes.
# ---------------------------------------------------------------------------
log_header "E -- an unknown schema version is refused, not read anyway"
sed 's/SCHEMA_VERSION 1/SCHEMA_VERSION 99/' "$T/a1.cmake" > "$T/e1.cmake"
OUT="$(derive "$T/e1.cmake")"
check
if ! nros_grep_q "CMake Error" <<< "$OUT"; then
    fail "E: schema version 99 must be a FATAL_ERROR:"; echo "$OUT"
fi
check
# `version 99` and not `schema version 99`: cmake hard-wraps a message() body,
# so an assertion on a phrase that spans the wrap point tests the wrap.
if ! nros_grep_q "version 99" <<< "$OUT"; then
    fail "E: the error must state the version it found:"; echo "$OUT"
fi

grep -v "SCHEMA_VERSION" "$T/a1.cmake" > "$T/e2.cmake"
OUT="$(derive "$T/e2.cmake")"
check
if ! nros_grep_q "CMake Error" <<< "$OUT"; then
    fail "E: a fragment stating NO schema version must be a FATAL_ERROR -- it is indistinguishable from a file that is not a fragment at all:"; echo "$OUT"
fi

# A MIXED-version set must fail on the bad one even though a good one precedes
# it. A reader that checked only the first fragment would pass E above.
cat "$T/a1.cmake" > "$T/e3.cmake"
OUT="$(derive "$T/e3.cmake;$T/e1.cmake")"
check
if ! nros_grep_q "CMake Error" <<< "$OUT"; then
    fail "E: the version is checked PER fragment, not once:"; echo "$OUT"
fi

# ---------------------------------------------------------------------------
# F. A fragment that does not exist yet refuses; it does not fatal.
# ---------------------------------------------------------------------------
log_header "F -- a not-yet-written fragment refuses rather than fataling"
OUT="$(derive "$T/a1.cmake;$T/does-not-exist.cmake")"
check
if nros_grep_q "CMake Error" <<< "$OUT"; then
    fail "F: a build-time fragment that has not been written must not break the configure -- the canonical lane emits it as a custom-command output:"; echo "$OUT"
fi
check
if ! nros_grep_q "NROS_MESSAGE_BOUNDS_STATUS=refused" <<< "$OUT"; then
    fail "F: a missing fragment must refuse:"; echo "$OUT"
fi

# ---------------------------------------------------------------------------
# H. The output file: the answer, the provenance, and write-if-changed.
# ---------------------------------------------------------------------------
log_header "H -- the written answer carries provenance and is write-if-changed"
KNOBS="$T/knobs.cmake"
derive "$T/a1.cmake;$T/a2.cmake" -DNROS_BOUNDS_OUTPUT="$KNOBS" >/dev/null
check
if [ ! -f "$KNOBS" ]; then
    fail "H: OUTPUT_FILE was not written"
else
    check
    if ! nros_grep_q "set(NROS_DERIVED_SUBSCRIPTION_BUFFER_SIZE 880)" "$KNOBS"; then
        fail "H: the written file must carry the derived value:"; cat "$KNOBS"
    fi
    check
    if ! nros_grep_q "nav_msgs/msg/Odometry" "$KNOBS"; then
        fail "H: the written file must name the type the number came from -- a number a reader cannot account for is a number they will 'fix':"; cat "$KNOBS"
    fi
    check
    if ! nros_grep_q -qi "UPPER BOUND" "$KNOBS"; then
        fail "H: the written file must state the over-approximation where a user reads it, not only in a report:"; cat "$KNOBS"
    fi
    check
    if ! nros_grep_q -qi "DEFAULT" "$KNOBS"; then
        fail "H: the written file must say these are defaults a stated value overrides:"; cat "$KNOBS"
    fi
    BEFORE="$(stat -c '%Y.%y' "$KNOBS")"
    sleep 1
    derive "$T/a1.cmake;$T/a2.cmake" -DNROS_BOUNDS_OUTPUT="$KNOBS" >/dev/null
    AFTER="$(stat -c '%Y.%y' "$KNOBS")"
    check
    if [ "$BEFORE" != "$AFTER" ]; then
        fail "H: identical content rewrote the file -- a consumer registers it with CMAKE_CONFIGURE_DEPENDS, so this re-arms a reconfigure forever"
    fi
    # And a CHANGED answer must actually land, or the write-if-changed guard
    # would be indistinguishable from never writing.
    derive "$T/a2.cmake;$T/b1.cmake" -DNROS_BOUNDS_OUTPUT="$KNOBS" >/dev/null
    check
    if ! nros_grep_q "set(NROS_DERIVED_SUBSCRIBER_LARGE_SIZE 4208)" "$KNOBS"; then
        fail "H: a changed answer must be written:"; cat "$KNOBS"
    fi
fi

# The refused file must be readable too: a consumer that includes it gets a
# status and a reason and NO numbers.
derive "$T/a2.cmake;$T/c1.cmake" -DNROS_BOUNDS_OUTPUT="$T/refused.cmake" >/dev/null 2>&1
check
if ! nros_grep_q "set(NROS_MESSAGE_BOUNDS_STATUS \"refused\")" "$T/refused.cmake"; then
    fail "H: a refusal must still write a readable status:"; cat "$T/refused.cmake"
fi
check
if nros_grep_q "set(NROS_DERIVED_" "$T/refused.cmake"; then
    fail "H: a refusal must publish no derived value at all:"; cat "$T/refused.cmake"
fi

# ---------------------------------------------------------------------------
# I. The consumer seam, in a REAL configure.
#
# The Zephyr knob resolver reads the composed answer through
# `CMAKE_CONFIGURE_DEPENDS`, and that is the one part of the wiring with a
# failure mode `cmake -P` cannot show: a ninja input that does not exist and
# has no rule producing it is `missing and no known rule to make it`, raised at
# LOAD before any rule runs, so the whole build dir is unusable. The Zephyr lane
# is not buildable on a bare host, but this seam is generic cmake+ninja and is.
#
# Asserts BOTH halves: the seeded placeholder makes the first configure and
# build work, and rewriting the file with a different answer actually re-runs
# cmake so the new value is picked up.
# ---------------------------------------------------------------------------
log_header "I -- the CONFIGURE_DEPENDS seam survives a real configure + build"
if ! command -v ninja >/dev/null 2>&1; then
    fail "I: ninja is not on PATH -- this assertion cannot report a verdict without it"
else
    P="$T/proj"
    mkdir -p "$P"
    cat > "$P/CMakeLists.txt" <<CMAKEEOF
cmake_minimum_required(VERSION 3.20)
project(nros_bounds_seam NONE)
include("$MODULE")
nros_message_bounds_knobs_file(_knobs)
nros_message_bounds_seed_knobs_file("\${_knobs}")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "\${_knobs}")
include("\${_knobs}")
if(NOT DEFINED NROS_MESSAGE_BOUNDS_STATUS)
    message(FATAL_ERROR "the knobs file set no status")
endif()
file(WRITE "\${CMAKE_BINARY_DIR}/observed.txt"
     "\${NROS_MESSAGE_BOUNDS_STATUS}|\${NROS_DERIVED_SUBSCRIPTION_BUFFER_SIZE}")
add_custom_target(seam ALL COMMAND \${CMAKE_COMMAND} -E true)
CMAKEEOF
    B="$T/proj-build"
    check
    if ! cmake -G Ninja -S "$P" -B "$B" > "$T/i-configure.log" 2>&1; then
        fail "I: first configure failed:"; cat "$T/i-configure.log"
    elif ! ninja -C "$B" > "$T/i-build.log" 2>&1; then
        fail "I: first build failed -- a seeded-but-unproduced CONFIGURE_DEPENDS input is a ninja load error:"; cat "$T/i-build.log"
    fi
    check
    if [ "$(cat "$B/observed.txt" 2>/dev/null)" != "refused|" ]; then
        fail "I: the placeholder must read as refused with no derived value, got: $(cat "$B/observed.txt" 2>/dev/null)"
    fi
    # Now write a real answer into it, exactly as nros_find_interfaces() would,
    # and confirm the next bare `ninja` re-runs cmake and observes it.
    derive "$T/a1.cmake;$T/a2.cmake" -DNROS_BOUNDS_OUTPUT="$B/nros/message_bound_knobs.cmake" >/dev/null
    check
    if ! ninja -C "$B" > "$T/i-build2.log" 2>&1; then
        fail "I: the rebuild after the answer changed failed:"; cat "$T/i-build2.log"
    fi
    check
    if [ "$(cat "$B/observed.txt" 2>/dev/null)" != "derived|880" ]; then
        fail "I: a bare ninja did not re-configure and pick the derived answer up, got: $(cat "$B/observed.txt" 2>/dev/null)"
    fi
fi

# ---------------------------------------------------------------------------
# J. The REGISTRY path -- the one `nros_find_interfaces()` actually takes.
#
# The generators register each fragment as they emit it and the composer is
# then called with NO `FRAGMENTS`, so it reads the registry. Everything above
# passes fragments explicitly and would keep passing if the registry were
# broken. Exercised across an `add_subdirectory()` boundary because that is the
# shape a real workspace has (an interfaces package, then the entry) and it is
# what rules out a normal variable as the carrier.
# ---------------------------------------------------------------------------
log_header "K -- a first include from INSIDE a function frame is survivable"
if ! command -v ninja >/dev/null 2>&1; then
    fail "K: ninja is not on PATH -- this assertion cannot report a verdict without it"
else
    # `include_guard(GLOBAL)` plus an include that happens first from inside a
    # function frame is a live shape in this tree (NanoRosWorkspace.cmake
    # includes NanoRosCodegenCore.cmake inside `nros_resolve_cli`'s branch). A
    # plain file-scope `set()` would land in that frame, vanish when it pops,
    # and never come back, because the guard makes every later include a no-op.
    # The functions survive; only the variables go, so the failure lands on the
    # schema check of a perfectly well-formed fragment.
    K="$T/kfn"
    mkdir -p "$K"
    cat > "$K/CMakeLists.txt" <<CMAKEEOF
cmake_minimum_required(VERSION 3.20)
project(nros_bounds_frame NONE)
function(first_include_from_a_frame)
    include("$MODULE")
endfunction()
first_include_from_a_frame()
include("$MODULE")   # a no-op: the guard has already fired
# The JOIN is the second consumer of a module-scope constant that a function
# frame can eat: it resolves its sibling NanoRosEntityInventory.cmake through
# \`_NROS_MESSAGE_BOUNDS_DIR\` to read that module's schema number, rather than
# restating the number and drifting from it. An empty dir would make the
# include silently fail and every joined image fall back to the closure.
nros_derive_message_bound_knobs(
    FRAGMENTS "$T/a1.cmake" "$T/a2.cmake"
    ENTITY_INVENTORY "$T/k-ents.cmake" QUIET)
file(WRITE "\${CMAKE_BINARY_DIR}/observed.txt"
     "\${NROS_MESSAGE_BOUNDS_STATUS}|\${NROS_DERIVED_SUBSCRIPTION_BUFFER_SIZE}|\${NROS_MESSAGE_BOUNDS_BASIS}|\${NROS_DERIVED_SUBSCRIBER_BUFFER_SIZE}")
CMAKEEOF
    # a1/a2's closure tops out at Odometry 880; subscribe only to the 114 B
    # Control, so the joined small class must differ from the closure's.
    entity_frag "$T/k-ents.cmake" derived resolved \
        "autoware_control_msgs/msg/Control=1"
    KB="$T/kfn-build"
    check
    if ! cmake -G Ninja -S "$K" -B "$KB" > "$T/k.log" 2>&1; then
        fail "K: configure failed -- the module's constants did not survive a first include from a function frame:"; cat "$T/k.log"
    fi
    check
    if [ "$(cat "$KB/observed.txt" 2>/dev/null)" != "derived|880|subscribed|114" ]; then
        fail "K: expected derived|880|subscribed|114 -- the take buffer on the closure, the small class on the join, both after a first include from a function frame. Got: $(cat "$KB/observed.txt" 2>/dev/null)"
    fi
fi

log_header "J -- the composer reads the fragment registry, across a subdirectory"
if ! command -v ninja >/dev/null 2>&1; then
    fail "J: ninja is not on PATH -- this assertion cannot report a verdict without it"
else
    R="$T/reg"
    mkdir -p "$R/sub"
    cat > "$R/sub/CMakeLists.txt" <<CMAKEEOF
nros_message_bounds_register_fragment("$T/a1.cmake")
nros_message_bounds_register_fragment("$T/a2.cmake")
CMAKEEOF
    cat > "$R/CMakeLists.txt" <<CMAKEEOF
cmake_minimum_required(VERSION 3.20)
project(nros_bounds_registry NONE)
include("$MODULE")
add_subdirectory(sub)
# Registering the SAME fragment twice must not double-count it -- a package
# reached through two dependency paths is the normal case.
nros_message_bounds_register_fragment("$T/a1.cmake")
nros_derive_message_bound_knobs(OUTPUT_FILE "\${CMAKE_BINARY_DIR}/knobs.cmake")
file(WRITE "\${CMAKE_BINARY_DIR}/observed.txt"
     "\${NROS_MESSAGE_BOUNDS_STATUS}|\${NROS_MESSAGE_BOUNDS_TYPE_COUNT}|\${NROS_DERIVED_SUBSCRIPTION_BUFFER_SIZE}")
# A SECOND call, this time over a closure that refuses, from the same caller
# scope. Every derived value must be gone in both the returned variables and
# the written file -- a function inherits its caller's variables, so a stale
# one survives unless it is cleared locally as well as in the parent, and the
# result reads as a refusal carrying the previous run's numbers.
nros_derive_message_bound_knobs(
    FRAGMENTS "$T/a1.cmake" "$T/c1.cmake"
    OUTPUT_FILE "\${CMAKE_BINARY_DIR}/knobs2.cmake" QUIET)
file(WRITE "\${CMAKE_BINARY_DIR}/observed2.txt"
     "\${NROS_MESSAGE_BOUNDS_STATUS}|\${NROS_DERIVED_SUBSCRIPTION_BUFFER_SIZE}")
CMAKEEOF
    RB="$T/reg-build"
    check
    if ! cmake -G Ninja -S "$R" -B "$RB" > "$T/j.log" 2>&1; then
        fail "J: configure failed:"; cat "$T/j.log"
    fi
    check
    if [ "$(cat "$RB/observed.txt" 2>/dev/null)" != "derived|4|880" ]; then
        fail "J: the registry did not carry the fragments across the subdirectory (expected derived|4|880), got: $(cat "$RB/observed.txt" 2>/dev/null)"
    fi
    check
    if [ "$(cat "$RB/observed2.txt" 2>/dev/null)" != "refused|" ]; then
        fail "J: a refusal after a successful call left the previous derived value standing, got: $(cat "$RB/observed2.txt" 2>/dev/null)"
    fi
    check
    if nros_grep_q "set(NROS_DERIVED_" "$RB/knobs2.cmake"; then
        fail "J: the refusal file carries a value from the previous call:"; cat "$RB/knobs2.cmake"
    fi
fi

# ---------------------------------------------------------------------------
# L. THE JOIN -- the reference island's shape, before and after.
#
# One closure, two runs, and the ONLY difference is whether an entity
# declaration is present. `std_msgs/msg/Float64MultiArray` at 1496 B is linked
# and never received; `nav_msgs/msg/Odometry` at 880 B is the largest that is.
# These are the island's own derived bounds, and 1496 -> 880 is the measured
# 71,808 -> 42,240 bytes of `SMALL_PAYLOADS` at its 12 subscribers x 4 ring
# depth.
# ---------------------------------------------------------------------------
log_header "L -- the payload classes derive over what the image RECEIVES"
frag "$T/l1.cmake" "island" <<'ROWS'
std_msgs/msg/Float64MultiArray|bounded|1488|1496|
nav_msgs/msg/Odometry|bounded|836|880|
autoware_control_msgs/msg/Control|bounded|78|114|
autoware_vehicle_msgs/msg/GearCommand|bounded|13|21|
ROWS
# The island subscribes to Odometry and Control; it PUBLISHES GearCommand and
# neither publishes nor subscribes Float64MultiArray (a linked sibling type).
entity_frag "$T/l-ents.cmake" derived resolved \
    "nav_msgs/msg/Odometry=1" "autoware_control_msgs/msg/Control=1"

OUT_BEFORE="$(derive "$T/l1.cmake")"
OUT_AFTER="$(derive "$T/l1.cmake" -DNROS_BOUNDS_ENTITY_INVENTORY="$T/l-ents.cmake")"

check
if ! nros_grep_q "NROS_DERIVED_SUBSCRIBER_BUFFER_SIZE=1496" <<< "$OUT_BEFORE"; then
    fail "L: without a declaration the small class must still be the closure's 1496:"; echo "$OUT_BEFORE"
fi
check
if ! nros_grep_q "NROS_MESSAGE_BOUNDS_BASIS=closure" <<< "$OUT_BEFORE"; then
    fail "L: the closure answer must be LABELLED as such:"; echo "$OUT_BEFORE"
fi
check
if ! nros_grep_q "NROS_DERIVED_SUBSCRIBER_BUFFER_SIZE=880" <<< "$OUT_AFTER"; then
    fail "L: with a declaration the small class must be the largest SUBSCRIBED type (880, Odometry) -- a type the image links and never receives cannot set it:"; echo "$OUT_AFTER"
fi
check
if ! nros_grep_q "NROS_MESSAGE_BOUNDS_BASIS=subscribed" <<< "$OUT_AFTER"; then
    fail "L: the joined answer must be labelled subscribed:"; echo "$OUT_AFTER"
fi
check
if ! nros_grep_q "NROS_MESSAGE_BOUNDS_SUBSCRIPTION_COUNT=2" <<< "$OUT_AFTER"; then
    fail "L: the joined answer must record how many subscriptions it saw:"; echo "$OUT_AFTER"
fi
# The take buffer does NOT narrow. It is also DEFAULT_TX_BUF and the default
# buffer of every raw service/client/action entity, so a type this image only
# publishes still has to fit. Narrowing it is the under-derivation.
check
if ! nros_grep_q "NROS_DERIVED_SUBSCRIPTION_BUFFER_SIZE=1496" <<< "$OUT_AFTER"; then
    fail "L: the take buffer must stay on the CLOSURE basis (1496) -- it is DEFAULT_TX_BUF too:"; echo "$OUT_AFTER"
fi

# And the entity count, not the type count, drives the large class: two
# subscriptions on one large type need two BLOCKS.
log_header "L -- MAX_LARGE_SUBSCRIBERS counts blocks, so it counts ENTITIES"
frag "$T/l2.cmake" "big" <<'ROWS'
sensor_msgs/msg/JointState|bounded|4204|4208|
std_msgs/msg/Int32|bounded|4|8|
ROWS
entity_frag "$T/l2-ents.cmake" derived resolved \
    "sensor_msgs/msg/JointState=3" "std_msgs/msg/Int32=1"
OUT="$(derive "$T/l2.cmake" -DNROS_BOUNDS_ENTITY_INVENTORY="$T/l2-ents.cmake")"
check
if ! nros_grep_q "NROS_DERIVED_MAX_LARGE_SUBSCRIBERS=3" <<< "$OUT"; then
    fail "L: three subscriptions on one large type need three blocks, not one:"; echo "$OUT"
fi
check
if ! nros_grep_q "NROS_DERIVED_SUBSCRIBER_LARGE_SIZE=4208" <<< "$OUT"; then
    fail "L: the large class size is the largest RECEIVED type over the split:"; echo "$OUT"
fi
check
if ! nros_grep_q "NROS_DERIVED_SUBSCRIBER_BUFFER_SIZE=8" <<< "$OUT"; then
    fail "L: the small class is the largest received type UNDER the split:"; echo "$OUT"
fi

# An image that declares entities and subscribes to nothing is an ANSWER, not
# a refusal: its payload pools are genuinely unused, so zero blocks is right
# and the small size is not derivable from an empty set.
log_header "L -- an image that receives nothing answers ZERO blocks"
entity_frag "$T/l3-ents.cmake" derived resolved
OUT="$(derive "$T/l1.cmake" -DNROS_BOUNDS_ENTITY_INVENTORY="$T/l3-ents.cmake")"
check
if ! nros_grep_q "NROS_DERIVED_MAX_LARGE_SUBSCRIBERS=0" <<< "$OUT"; then
    fail "L: an image with no subscriptions reserves no large blocks:"; echo "$OUT"
fi
check
if nros_grep_q "NROS_DERIVED_SUBSCRIBER_BUFFER_SIZE=" <<< "$OUT"; then
    fail "L: nothing is received, so the small class size is not derivable -- naming one would invent a number:"; echo "$OUT"
fi
check
if ! nros_grep_q "NROS_MESSAGE_BOUNDS_PAYLOAD_STATUS=derived" <<< "$OUT"; then
    fail "L: an empty received set is an ANSWER, not a refusal:"; echo "$OUT"
fi

# ---------------------------------------------------------------------------
# M. THE JOIN REFUSES -- and never widens to the closure.
# ---------------------------------------------------------------------------
log_header "M -- a type the bound inventory cannot price REFUSES the classes"
# Today this is structural rather than a typo: `BoundInventory::record_message`
# is called for `.msg` files and for nothing else, so no `pkg/srv/*` or
# `pkg/action/*` has an entry however well-formed the declaration is.
entity_frag "$T/m1-ents.cmake" derived resolved \
    "nav_msgs/msg/Odometry=1" "tier4_system_msgs/srv/OperateMrm_Request=1"
OUT="$(derive "$T/l1.cmake" -DNROS_BOUNDS_ENTITY_INVENTORY="$T/m1-ents.cmake")"
check
if ! nros_grep_q "NROS_MESSAGE_BOUNDS_PAYLOAD_STATUS=refused" <<< "$OUT"; then
    fail "M: an unpriced received type must refuse the payload classes:"; echo "$OUT"
fi
check
if nros_grep_q -qE "NROS_DERIVED_(SUBSCRIBER_BUFFER_SIZE|MAX_LARGE_SUBSCRIBERS|SUBSCRIBER_LARGE_SIZE)=" <<< "$OUT"; then
    fail "M: a refusal published a payload-class number:"; echo "$OUT"
fi
check
if nros_grep_q "NROS_MESSAGE_BOUNDS_BASIS=" <<< "$OUT"; then
    fail "M: a refusal must NOT widen to the closure -- that is the wrong row wearing a derived status:"; echo "$OUT"
fi
check
if ! nros_grep_q "OperateMrm_Request" <<< "$OUT"; then
    fail "M: the refusal does not name the type it could not price:"; echo "$OUT"
fi
# The take buffer is a different question with a different basis, so it still
# derives. A reader that collapsed the two statuses would lose it.
check
if ! nros_grep_q "NROS_DERIVED_SUBSCRIPTION_BUFFER_SIZE=1496" <<< "$OUT"; then
    fail "M: the take buffer derives over the closure and must survive a payload-class refusal:"; echo "$OUT"
fi

log_header "M -- an unresolved subscribed set REFUSES the classes"
entity_frag "$T/m2-ents.cmake" derived refused
OUT="$(derive "$T/l1.cmake" -DNROS_BOUNDS_ENTITY_INVENTORY="$T/m2-ents.cmake")"
check
if ! nros_grep_q "NROS_MESSAGE_BOUNDS_PAYLOAD_STATUS=refused" <<< "$OUT"; then
    fail "M: an unresolved subscribed set must refuse:"; echo "$OUT"
fi
check
if nros_grep_q "NROS_DERIVED_SUBSCRIBER_BUFFER_SIZE=" <<< "$OUT"; then
    fail "M: an unresolved subscribed set published a small class anyway:"; echo "$OUT"
fi

log_header "M -- a current schema that states no subscribed set REFUSES"
{
    echo "set(NROS_ENTITY_INVENTORY_SCHEMA_VERSION 3)"
    echo "set(NROS_ENTITY_INVENTORY_STATUS \"derived\")"
} > "$T/m3-ents.cmake"
OUT="$(derive "$T/l1.cmake" -DNROS_BOUNDS_ENTITY_INVENTORY="$T/m3-ents.cmake")"
check
if ! nros_grep_q "NROS_MESSAGE_BOUNDS_PAYLOAD_STATUS=refused" <<< "$OUT"; then
    fail "M: a fragment claiming the current schema with no subscribed set must refuse, not read an absent list as an empty one:"; echo "$OUT"
fi
check
if ! nros_grep_q "malformed" <<< "$OUT"; then
    fail "M: the refusal does not say the fragment is malformed:"; echo "$OUT"
fi

# ---------------------------------------------------------------------------
# N. BACK-COMPAT -- an image that declares nothing does not move.
#
# This is the requirement the join is easiest to get wrong on in the OTHER
# direction: refusing here would take every image built before phase-403 W9
# from a derived (over-approximate, safe) number back to the hand-set numbers
# W8 exists to replace, which is a regression rather than a safety property.
# ---------------------------------------------------------------------------
log_header "N -- an image that declares nothing keeps W8's closure answer"
# A fragment whose own status is `refused` -- the shape W9 writes when any
# component in the image states no ENTITIES, which is every pre-W9 image.
entity_frag "$T/n1-ents.cmake" refused refused
OUT="$(derive "$T/l1.cmake" -DNROS_BOUNDS_ENTITY_INVENTORY="$T/n1-ents.cmake")"
check
if [ "$OUT_BEFORE" != "$OUT" ]; then
    fail "N: an image whose entity inventory refused must derive EXACTLY what it derives with no fragment at all. Diff:"
    diff <(echo "$OUT_BEFORE") <(echo "$OUT") || true
fi
check
if ! nros_grep_q "NROS_MESSAGE_BOUNDS_BASIS=closure" <<< "$OUT"; then
    fail "N: the unchanged answer must still be labelled closure:"; echo "$OUT"
fi

log_header "N -- a fragment from a STALE CLI warns and keeps the closure answer"
# NOT a FATAL_ERROR, deliberately: the fragment's producer (`nano_ros_entry()`)
# runs LATER in the same configure than this reader, so a fatal aborts before
# the stale fragment can ever be rewritten and the build dir is stuck until
# someone deletes the file by hand.
{
    echo "set(NROS_ENTITY_INVENTORY_SCHEMA_VERSION 1)"
    echo "set(NROS_ENTITY_INVENTORY_STATUS \"derived\")"
} > "$T/n2-ents.cmake"
OUT="$(derive "$T/l1.cmake" -DNROS_BOUNDS_ENTITY_INVENTORY="$T/n2-ents.cmake")"
check
if nros_grep_q -qi "CMake Error" <<< "$OUT"; then
    fail "N: a stale-schema fragment must not abort the configure -- its producer runs later in it:"; echo "$OUT"
fi
check
if ! nros_grep_q "IGNORED" <<< "$OUT"; then
    fail "N: a stale-schema fragment must say loudly that it was not read:"; echo "$OUT"
fi
check
if ! nros_grep_q "NROS_MESSAGE_BOUNDS_BASIS=closure" <<< "$OUT"; then
    fail "N: a stale-schema fragment must leave the closure answer standing:"; echo "$OUT"
fi
check
if ! nros_grep_q "NROS_DERIVED_SUBSCRIBER_BUFFER_SIZE=1496" <<< "$OUT"; then
    fail "N: a stale-schema fragment must not change any number:"; echo "$OUT"
fi

log_header "N -- the OUTPUT FILE records which basis each number used"
derive "$T/l1.cmake" -DNROS_BOUNDS_ENTITY_INVENTORY="$T/l-ents.cmake" \
    -DNROS_BOUNDS_OUTPUT="$T/n3-knobs.cmake" >/dev/null
check
if ! nros_grep_q 'set(NROS_MESSAGE_BOUNDS_BASIS "subscribed")' "$T/n3-knobs.cmake"; then
    fail "N: the written answer does not record its basis:"; cat "$T/n3-knobs.cmake"
fi
check
if ! nros_grep_q "2 SUBSCRIPTIONS this image declares" "$T/n3-knobs.cmake"; then
    fail "N: the written answer does not say what it was derived over:"; cat "$T/n3-knobs.cmake"
fi
derive "$T/l1.cmake" -DNROS_BOUNDS_ENTITY_INVENTORY="$T/m1-ents.cmake" \
    -DNROS_BOUNDS_OUTPUT="$T/n4-knobs.cmake" >/dev/null 2>&1
check
if nros_grep_q "set(NROS_DERIVED_SUBSCRIBER_BUFFER_SIZE" "$T/n4-knobs.cmake"; then
    fail "N: a payload-class refusal wrote a class size into the answer file:"; cat "$T/n4-knobs.cmake"
fi
check
if ! nros_grep_q 'set(NROS_MESSAGE_BOUNDS_PAYLOAD_STATUS "refused")' "$T/n4-knobs.cmake"; then
    fail "N: the answer file does not record the payload-class refusal:"; cat "$T/n4-knobs.cmake"
fi
check
if ! nros_grep_q "set(NROS_DERIVED_SUBSCRIPTION_BUFFER_SIZE 1496)" "$T/n4-knobs.cmake"; then
    fail "N: the take buffer must still be written on a payload-class refusal:"; cat "$T/n4-knobs.cmake"
fi

# ---------------------------------------------------------------------------
log_header "O -- the join refuses a subscribed type that carries no bound"
# ---------------------------------------------------------------------------
#
# issue 0963. This guard CANNOT fire through `nros_derive_message_bound_knobs`
# today: the closure-wide open-type check returns before the join runs, and the
# subscribed set is a subset of the closure. It is tested by calling the join
# DIRECTLY, because a check that has never executed is indistinguishable from
# one that does not work -- and the thing that makes this one reachable is
# exactly the change 0963 asks for (deriving the payload classes on an image
# whose closure has an unbounded type it never receives).
cat > "$T/o-join.cmake" <<EOF
include("$MODULE")
set(NROS_MESSAGE_BOUND_std_msgs_msg_Int32_STATE "bounded")
set(NROS_MESSAGE_BOUND_std_msgs_msg_Int32_RX 12)
set(NROS_MESSAGE_BOUND_demo_msg_Open_STATE "unbounded")
set(NROS_MESSAGE_BOUND_demo_msg_Open_RX "")
_nros_bounds_join_subscribed("\${FRAG}" 2048 b st why cnt small ltypes lmax lcount)
message(STATUS "basis=\${b} status=\${st} why=\${why}")
EOF
# The join iterates TYPE_COUNTS (it needs each type's count), NOT TYPES -- so a
# fixture that lists a type only in TYPES is never visited and the case passes
# for the wrong reason. It did, on the first run of this test.
_o_frag() {
    {
        printf 'set(NROS_ENTITY_INVENTORY_SCHEMA_VERSION 3)\n'
        printf 'set(NROS_ENTITY_INVENTORY_STATUS "derived")\n'
        printf 'set(NROS_ENTITY_SUBSCRIBED_TYPES_STATUS "resolved")\n'
        printf 'set(NROS_ENTITY_SUBSCRIBED_TYPES "%s")\n' "$1"
        printf 'set(NROS_ENTITY_SUBSCRIBED_TYPE_COUNTS "%s")\n' "$2"
        printf 'set(NROS_ENTITY_SUBSCRIBED_ENTITY_COUNT 1)\n'
    } > "$3"
}
_o_frag "std_msgs/msg/Int32" "std_msgs/msg/Int32=1" "$T/o-ok.cmake"
_o_frag "std_msgs/msg/Int32;demo/msg/Open" \
        "std_msgs/msg/Int32=1;demo/msg/Open=1" "$T/o-bad.cmake"

_o_out=$(cmake -DFRAG="$T/o-ok.cmake" -P "$T/o-join.cmake" 2>&1)
if ! nros_grep_q "status=derived" <<<"$_o_out"; then
    fail "O: every subscribed type is bounded and the join still refused:"
    printf '%s\n' "$_o_out"
fi
check
_o_out=$(cmake -DFRAG="$T/o-bad.cmake" -P "$T/o-join.cmake" 2>&1)
if ! nros_grep_q "status=refused" <<<"$_o_out"; then
    fail "O: a subscribed type with no bound did NOT refuse -- a payload class \
would be sized from a blank _RX:"
    printf '%s\n' "$_o_out"
fi
check
if ! nros_grep_q "demo/msg/Open" <<<"$_o_out"; then
    fail "O: the refusal does not name the offending type:"
    printf '%s\n' "$_o_out"
fi
check

# ---------------------------------------------------------------------------
log_header "P -- an open CLOSURE type does not poison the payload classes"
# ---------------------------------------------------------------------------
#
# issue 0963, the fix case O was written against. The take buffer and the
# payload classes answer different questions:
#
#   * NROS_SUBSCRIPTION_BUFFER_SIZE is one global size, aliased by
#     DEFAULT_TX_BUF, so a type the image only PUBLISHES must fit it -- an open
#     type anywhere in the closure genuinely poisons it;
#   * the payload classes size staging pools for what the image RECEIVES, which
#     an unbounded type nothing subscribes to cannot reach.
#
# So: closure carries an unbounded type, every SUBSCRIBED type is bounded.
# Expected -- overall `refused`, payload classes `derived`.
_p_pkg="$T/p-pkgs"
mkdir -p "$_p_pkg"
cat > "$_p_pkg/bounds.cmake" <<'EOF'
set(NROS_MESSAGE_BOUNDS_SCHEMA_VERSION 1)
list(APPEND NROS_MESSAGE_BOUND_PACKAGES "demo")
list(APPEND NROS_MESSAGE_BOUND_TYPES "std_msgs/msg/Int32")
set(NROS_MESSAGE_BOUND_std_msgs_msg_Int32_STATE "bounded")
set(NROS_MESSAGE_BOUND_std_msgs_msg_Int32_TX 12)
set(NROS_MESSAGE_BOUND_std_msgs_msg_Int32_RX 12)
list(APPEND NROS_MESSAGE_BOUND_TYPES "demo/msg/Open")
set(NROS_MESSAGE_BOUND_demo_msg_Open_STATE "unbounded")
set(NROS_MESSAGE_BOUND_demo_msg_Open_REASON "unbounded member: data (string)")
EOF
# Subscribes to the BOUNDED type only. The unbounded one is linked, never received.
{
    printf 'set(NROS_ENTITY_INVENTORY_SCHEMA_VERSION 3)\n'
    printf 'set(NROS_ENTITY_INVENTORY_STATUS "derived")\n'
    printf 'set(NROS_ENTITY_SUBSCRIBED_TYPES_STATUS "resolved")\n'
    printf 'set(NROS_ENTITY_SUBSCRIBED_TYPES "std_msgs/msg/Int32")\n'
    printf 'set(NROS_ENTITY_SUBSCRIBED_TYPE_COUNTS "std_msgs/msg/Int32=1")\n'
    printf 'set(NROS_ENTITY_SUBSCRIBED_ENTITY_COUNT 1)\n'
} > "$T/p-entities.cmake"

cat > "$T/p-run.cmake" <<EOF
include("$MODULE")
nros_derive_message_bound_knobs(
    FRAGMENTS "$_p_pkg/bounds.cmake"
    ENTITY_INVENTORY "$T/p-entities.cmake"
    OUTPUT_FILE "$T/p-out.cmake"
    QUIET)
message(STATUS "status=\${NROS_MESSAGE_BOUNDS_STATUS}")
message(STATUS "payload=\${NROS_MESSAGE_BOUNDS_PAYLOAD_STATUS}")
message(STATUS "basis=\${NROS_MESSAGE_BOUNDS_BASIS}")
message(STATUS "why=\${NROS_MESSAGE_BOUNDS_PAYLOAD_REASON}")
message(STATUS "small=\${NROS_DERIVED_SUBSCRIBER_BUFFER_SIZE}")
message(STATUS "takebuf=\${NROS_DERIVED_SUBSCRIPTION_BUFFER_SIZE}")
EOF
_p_out=$(cmake -P "$T/p-run.cmake" 2>&1)

if ! nros_grep_q "payload=derived" <<<"$_p_out"; then
    fail "P: every SUBSCRIBED type is bounded, so the payload classes must derive \
even though the closure refused:"
    printf '%s\n' "$_p_out"
fi
check
if ! nros_grep_q "basis=subscribed" <<<"$_p_out"; then
    fail "P: the payload classes derived on the wrong basis -- a `closure` basis \
here would be derived over the bounded types ONLY, which is the under-derivation \
the refusal exists to prevent:"
    printf '%s\n' "$_p_out"
fi
check
if ! nros_grep_q "small=12" <<<"$_p_out"; then
    fail "P: the small class was not sized from the one subscribed type:"
    printf '%s\n' "$_p_out"
fi
check
# The take buffer must STILL refuse -- that is the half an open closure type
# genuinely poisons, and the whole reason this is not just "stop refusing".
if nros_grep_q "takebuf=[0-9]" <<<"$_p_out"; then
    fail "P: the take buffer was derived over a closure with an unbounded type -- \
DEFAULT_TX_BUF aliases it, so a published type could exceed it silently:"
    printf '%s\n' "$_p_out"
fi
check

# The OUTPUT FILE is the transport -- a knob published in memory but absent from
# it does not reach the consumer, which is exactly the gap the in-memory
# assertions above cannot see. The refused branch of the writer emitted only the
# reason until issue 0963.
if ! nros_grep_q 'NROS_DERIVED_SUBSCRIBER_BUFFER_SIZE' "$T/p-out.cmake"; then
    fail "P: the payload classes were derived but the OUTPUT FILE does not carry \
them, so nros_cargo_build.cmake will never see them:"
    cat "$T/p-out.cmake"
fi
check
if nros_grep_q 'NROS_DERIVED_SUBSCRIPTION_BUFFER_SIZE' "$T/p-out.cmake"; then
    fail "P: the output file carries the TAKE BUFFER on a refused closure -- that \
is the knob an unbounded closure type genuinely poisons:"
    cat "$T/p-out.cmake"
fi
check
if ! nros_grep_q 'NROS_MESSAGE_BOUNDS_BASIS "subscribed"' "$T/p-out.cmake"; then
    fail "P: the file does not record WHICH basis the payload classes used:"
    cat "$T/p-out.cmake"
fi
check

# ...and with NO entity inventory, an open closure still publishes NOTHING: the
# `closure` basis would derive the classes over the bounded types only.
cat > "$T/p-run2.cmake" <<EOF
include("$MODULE")
nros_derive_message_bound_knobs(
    FRAGMENTS "$_p_pkg/bounds.cmake"
    OUTPUT_FILE "$T/p-out2.cmake"
    QUIET)
message(STATUS "small=\${NROS_DERIVED_SUBSCRIBER_BUFFER_SIZE}")
message(STATUS "basis=\${NROS_MESSAGE_BOUNDS_BASIS}")
EOF
_p_out2=$(cmake -P "$T/p-run2.cmake" 2>&1)
if nros_grep_q "small=[0-9]" <<<"$_p_out2"; then
    fail "P: with no entity inventory the classes were derived over the BOUNDED \
types only -- exactly the under-derivation the refusal exists to prevent:"
    printf '%s\n' "$_p_out2"
fi
check

# ...and the no-inventory case must still write nothing derived.
#
# issue 1173 — this ran BEFORE the `cmake -P` above, so `p-out2.cmake` did not
# exist yet and `grep` exited 2. The old `if … grep -q …` spelling reads a tool
# error as "no match", so the assertion silently did not fire, for as long as it
# has existed. `nros_grep_q` refuses to answer from a grep that did not run,
# which is how the ordering surfaced.
if nros_grep_q -qE '^set\(NROS_DERIVED_' "$T/p-out2.cmake"; then
    fail "P: with no entity inventory the output file carries derived knobs:"
    cat "$T/p-out2.cmake"
fi
check

# ---------------------------------------------------------------------------
log_header "Q. the derived count CROSSES THE LANE BOUNDARY (issue 1122)"

# `nros_derive_message_bound_knobs` writes the answer on EVERY lane; before 1122
# its only consumer was `zephyr/cmake/nros_cargo_build.cmake`, so a FreeRTOS
# image linked 131,072 B of `LARGE_PAYLOADS` while its own build dir held
# `set(NROS_DERIVED_MAX_LARGE_SUBSCRIBERS 0)`. `_nros_payload_facts_env` in
# `cmake/NanoRosEntityFacts.cmake` is the second road. It reads the FILE this
# module writes, so it is tested here, beside the writer.
#
# The two GUARD conditions are the whole safety argument and each gets a case:
# a `closure` basis counts large TYPES in the linked closure, which under-counts
# an image with two subscriptions on one large type, and a `refused` status has
# no answer at all. Getting either wrong under-sizes the pool, which is a
# `SubscriberCreationFailed` at `create_subscription`.
FACTS="$PROJECT_ROOT/cmake/NanoRosEntityFacts.cmake"

payload_env() {
    # payload_env <status> <basis> [count]  -> what the carrier would emit
    local dir="$T/carrier"
    rm -rf "$dir"; mkdir -p "$dir/nros"
    if [ "$1" != "NOFILE" ]; then
        {
            printf 'set(NROS_MESSAGE_BOUNDS_PAYLOAD_STATUS "%s")\n' "$1"
            printf 'set(NROS_MESSAGE_BOUNDS_BASIS "%s")\n' "$2"
            [ -n "${3:-}" ] && printf 'set(NROS_DERIVED_MAX_LARGE_SUBSCRIBERS %s)\n' "$3"
        } > "$dir/nros/message_bound_knobs.cmake"
    fi
    cat > "$dir/run.cmake" <<EOF
include("$MODULE")
include("$FACTS")
_nros_payload_facts_env(_out)
message(STATUS "CARRIER=\${_out}")
EOF
    # `cmake -P` sets CMAKE_BINARY_DIR to the CWD, and the carrier resolves the
    # knobs file through `nros_message_bounds_knobs_file()`, which is
    # `${CMAKE_BINARY_DIR}/nros/message_bound_knobs.cmake`. Run from the fixture
    # dir so that path lands on the file this case just wrote.
    (cd "$dir" && cmake -P run.cmake 2>&1) | sed -n 's/^-- CARRIER=//p'
}

_got="$(payload_env derived subscribed 0)"
if [ "$_got" != "NROS_DECLARED_LARGE_SUBSCRIBERS=0" ]; then
    fail "Q: a derived+subscribed answer crosses as a DECLARED fact -- wanted \"NROS_DECLARED_LARGE_SUBSCRIBERS=0\", got ${_got:-<empty>}"
fi
check
# Zero is the whole point: it is a CLAIM (every type fits the small class), not
# an absence, and it is the value the measured image needed.
_got="$(payload_env derived subscribed 3)"
if [ "$_got" != "NROS_DECLARED_LARGE_SUBSCRIBERS=3" ]; then
    fail "Q: a non-zero count crosses unchanged -- wanted \"NROS_DECLARED_LARGE_SUBSCRIBERS=3\", got ${_got:-<empty>}"
fi
check
_got="$(payload_env derived closure 0)"
if [ "$_got" != "" ]; then
    fail "Q: the CLOSURE basis is refused -- it under-counts -- wanted \"\", got ${_got:-<empty>}"
fi
check
_got="$(payload_env refused subscribed 0)"
if [ "$_got" != "" ]; then
    fail "Q: a refused status carries nothing -- wanted \"\", got ${_got:-<empty>}"
fi
check
_got="$(payload_env derived subscribed)"
if [ "$_got" != "" ]; then
    fail "Q: derived+subscribed with NO count carries nothing -- wanted \"\", got ${_got:-<empty>}"
fi
check
_got="$(payload_env NOFILE)"
if [ "$_got" != "" ]; then
    fail "Q: no knobs file at all carries nothing -- wanted \"\", got ${_got:-<empty>}"
fi
check

# ---------------------------------------------------------------------------
log_header "Summary"
if [ "$FAILURES" -eq 0 ]; then
    log_success "$CHECKS assertions held"
    exit 0
fi
log_error "$FAILURES of $CHECKS assertions failed"
exit 1
