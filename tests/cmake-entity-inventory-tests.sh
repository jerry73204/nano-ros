#!/bin/bash
# tests/cmake-entity-inventory-tests.sh -- phase-403 W9 (issue 0965)
#
# Exercises the READER for the entity inventory: `cmake/NanoRosEntityInventory.
# cmake`, driven in `cmake -P` script mode. Sibling of
# `tests/cmake-message-bounds-tests.sh`, and shaped like it.
#
# THE CLI IS STUBBED, DELIBERATELY. The derivation itself -- the counting rule,
# the per-kind slot cost, the refusal -- lives in
# `nros_cli_core::entity_inventory` and is unit-tested there, including the
# island's 33-entities-19-slots case. What CMAKE owns is a different set of
# facts, and every one of them is a failure mode that has bitten this tree
# before: does a refusal publish NO number, does an unknown schema FATAL instead
# of being read field-by-field, does a missing input refuse rather than break a
# clean-tree configure, and does the answer reach the CALLER's scope at all.
# A stub that emits fragments lets each of those be a case; requiring a built
# `nros` would make this gate need a cargo build, which is exactly what
# `check-lane-contracts` forbids on an affordability lane.
#
# WHAT IS ASSERTED:
#
#   A. A DERIVED fragment reaches the caller's scope: status, component count,
#      entity total, per-kind counts and NROS_DERIVED_EXECUTOR_MAX_CBS. The
#      scope half is the one that reads as working while being broken --
#      `include()` inside a function keeps its `set()`s in that frame, and
#      publishing to one of the two scopes is the bug `_nros_bounds_publish`
#      was written to prevent.
#
#   B. A REFUSED fragment publishes the reason and NO number. This is the
#      wave's central requirement: a consumer either reads a value the
#      inventory derived or reads nothing, because a slot count composed over
#      part of an image is SMALLER than the image needs and a short MAX_CBS
#      fails entity creation at boot.
#
#   C. A REFUSAL AFTER A DERIVATION leaves no stale number standing. The
#      function is called twice in one script; the second call must clear what
#      the first published, in both scopes.
#
#   D. An unknown SCHEMA VERSION is a FATAL_ERROR, and so is a fragment that
#      states none. Reading fields that may have moved is how two green tools
#      come to disagree.
#
#   E. A MISSING metadata file refuses rather than fataling -- an image that
#      has registered no component is the state every build was in before this
#      wave, and it is the permanent state of a launch-only image.
#
#   F. A MISSING CLI refuses rather than fataling, for the same reason.
#
#   G. A CLI that EXITS NON-ZERO is fatal. That is a broken declaration -- an
#      unknown entity kind, a component claiming NONE beside real entities --
#      and it names a component the user can fix. Distinguishing it from E/F is
#      the whole point: "you have not declared" and "what you declared is
#      wrong" license different actions.
#
#   A2. The JOIN KEY crosses the same boundary (phase-403 step 1). The
#      subscribed-type set and the wider received set are what
#      `nros_derive_message_bound_knobs` narrows the payload classes with, so
#      an absent list is not "nothing published" -- it reads as "this image
#      receives nothing" and derives a class over an empty set.
#
#   A3. The DECLARED DEPTHS cross the same boundary (phase-403 step 2), and so
#      does the count of endpoints that declared NONE. Depth multiplies the
#      type bound, so an absent list read as "every endpoint is depth 0" sizes
#      an arena an order of magnitude short; and a list read WITHOUT its
#      undeclared count sizes an image from the subset of it that happened to
#      be annotated. Asserted in A (published), B (refused, so absent) and C
#      (cleared by a later refusal).
#
#   H. SEEDING. A refusal writes a placeholder fragment, because the consumer
#      registers the path with CMAKE_CONFIGURE_DEPENDS and a ninja input with
#      no producing rule is `missing and no known rule to make it` at LOAD,
#      before any rule runs. And seeding never overwrites an existing answer.
#
# PRECONDITIONS ARE HARD FAILURES. This script never prints-and-returns: a
# green here is read as "the reader is sound". Skipping belongs in the `just`
# recipe's check ledger, which is a different claim from "it passed".
#
# Usage: ./tests/cmake-entity-inventory-tests.sh
# Exit:  0 all assertions held; 1 otherwise.

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# shellcheck source=lib/common.sh
source "$SCRIPT_DIR/lib/common.sh"

MODULE="$PROJECT_ROOT/cmake/NanoRosEntityInventory.cmake"

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

init_test_tmpdir "nros-entity-inventory"
trap 'cleanup_test_tmpdir' EXIT

# ---------------------------------------------------------------------------
# A stub `nros`. It writes the fragment named by --output-cmake, copying a body
# this script supplies through NROS_STUB_BODY, and exits with NROS_STUB_RC.
#
# The bodies below are byte-for-byte the shape
# `nros_cli_core::entity_inventory::EntityInventory::to_cmake` emits (see its
# unit tests) -- if that emitter changes shape, these stop matching and this
# test is where it is noticed.
# ---------------------------------------------------------------------------
STUB="$TEST_TMPDIR/nros-stub"
cat > "$STUB" <<'STUB_EOF'
#!/bin/bash
out=""
prev=""
for a in "$@"; do
    if [ "$prev" = "--output-cmake" ]; then out="$a"; fi
    prev="$a"
done
if [ -n "${NROS_STUB_BODY:-}" ] && [ -n "$out" ]; then
    mkdir -p "$(dirname "$out")"
    cp "$NROS_STUB_BODY" "$out"
fi
if [ -n "${NROS_STUB_STDERR:-}" ]; then echo "$NROS_STUB_STDERR" >&2; fi
exit "${NROS_STUB_RC:-0}"
STUB_EOF
chmod +x "$STUB"

DERIVED_BODY="$TEST_TMPDIR/derived.cmake"
cat > "$DERIVED_BODY" <<'EOF'
set(NROS_ENTITY_INVENTORY_SCHEMA_VERSION 3)
set(NROS_ENTITY_INVENTORY_SOURCE "meta.json")
set(NROS_ENTITY_INVENTORY_STATUS "derived")
set(NROS_ENTITY_INVENTORY_COMPONENT_COUNT 4)
set(NROS_ENTITY_INVENTORY_ENTITY_TOTAL 33)
set(NROS_ENTITY_COUNT_PUBLISHER 14)
set(NROS_ENTITY_COUNT_SUBSCRIPTION 11)
set(NROS_ENTITY_COUNT_TIMER 4)
set(NROS_ENTITY_COUNT_SERVICE_SERVER 2)
set(NROS_ENTITY_COUNT_SERVICE_CLIENT 2)
set(NROS_DERIVED_EXECUTOR_MAX_CBS 19)
set(NROS_ENTITY_SUBSCRIBED_TYPES_STATUS "resolved")
set(NROS_ENTITY_SUBSCRIBED_TYPES "nav_msgs/msg/Odometry;std_msgs/msg/Int32")
set(NROS_ENTITY_SUBSCRIBED_TYPE_COUNTS "nav_msgs/msg/Odometry=1;std_msgs/msg/Int32=2")
set(NROS_ENTITY_SUBSCRIBED_ENTITY_COUNT 3)
set(NROS_ENTITY_RECEIVED_TYPES_STATUS "resolved")
set(NROS_ENTITY_RECEIVED_TYPES "demo/srv/Op_Request;nav_msgs/msg/Odometry;std_msgs/msg/Int32")
set(NROS_ENTITY_RECEIVED_TYPE_COUNTS "demo/srv/Op_Request=1;nav_msgs/msg/Odometry=1;std_msgs/msg/Int32=2")
set(NROS_ENTITY_RECEIVED_ENTITY_COUNT 4)
set(NROS_ENTITY_DECLARED_DEPTH_STATUS "resolved")
set(NROS_ENTITY_DECLARED_DEPTHS "nav_msgs/msg/Odometry|/localization/kinematic_state=1;std_msgs/msg/Int32|/chatter=10")
set(NROS_ENTITY_DECLARED_DEPTH_COUNT 2)
set(NROS_ENTITY_UNDECLARED_DEPTH_COUNT 3)
EOF

REFUSED_BODY="$TEST_TMPDIR/refused.cmake"
cat > "$REFUSED_BODY" <<'EOF'
set(NROS_ENTITY_INVENTORY_SCHEMA_VERSION 3)
set(NROS_ENTITY_INVENTORY_STATUS "refused")
set(NROS_ENTITY_INVENTORY_COMPONENT_COUNT 4)
set(NROS_ENTITY_INVENTORY_REASON "1 of 4 components in this image declare no entities:\n    demo::legacy (demo::Legacy)")
set(NROS_ENTITY_SUBSCRIBED_TYPES_STATUS "refused")
set(NROS_ENTITY_SUBSCRIBED_TYPES_REASON "the entity inventory itself did not compose")
set(NROS_ENTITY_RECEIVED_TYPES_STATUS "refused")
set(NROS_ENTITY_RECEIVED_TYPES_REASON "the entity inventory itself did not compose")
set(NROS_ENTITY_DECLARED_DEPTH_STATUS "refused")
set(NROS_ENTITY_DECLARED_DEPTH_REASON "the entity inventory itself did not compose")
EOF

# A schema this reader does NOT understand. Written as "one past supported"
# rather than a literal, because the literal was `2` until phase-403 step 1
# made 2 the supported version -- at which point the case silently stopped
# testing anything it claimed to. Step 2 moved it again, to 3/4.
BAD_SCHEMA_BODY="$TEST_TMPDIR/bad-schema.cmake"
sed 's/SCHEMA_VERSION 3/SCHEMA_VERSION 4/' "$DERIVED_BODY" > "$BAD_SCHEMA_BODY"

NO_SCHEMA_BODY="$TEST_TMPDIR/no-schema.cmake"
grep -v SCHEMA_VERSION "$DERIVED_BODY" > "$NO_SCHEMA_BODY"

META="$TEST_TMPDIR/nros-metadata.json"
echo '{"components": []}' > "$META"

# Run the derivation through the module's `cmake -P` entry point.
#   derive <body-or-empty> <rc> <metadata> <output> [extra env]
derive() {
    local body="$1" rc="$2" meta="$3" out="$4" cli="${5:-$STUB}"
    NROS_STUB_BODY="$body" NROS_STUB_RC="$rc" \
        cmake -DNROS_ENTITY_CLI="$cli" \
              -DNROS_ENTITY_METADATA="$meta" \
              -DNROS_ENTITY_OUTPUT="$out" \
              -P "$MODULE" 2>&1
}

# ---------------------------------------------------------------------------
# A. A derived fragment reaches the caller's scope, with every field.
# ---------------------------------------------------------------------------
log_info "A. a derived inventory publishes its numbers"
OUT="$(derive "$DERIVED_BODY" 0 "$META" "$TEST_TMPDIR/a.cmake")"
check
if ! nros_grep_q "NROS_ENTITY_INVENTORY_STATUS=derived" <<<"$OUT"; then
    fail "A: status not derived -- $OUT"
fi
check
if ! nros_grep_q "NROS_DERIVED_EXECUTOR_MAX_CBS=19" <<<"$OUT"; then
    fail "A: MAX_CBS did not reach the caller's scope -- $OUT"
fi
check
if ! nros_grep_q "NROS_ENTITY_INVENTORY_ENTITY_TOTAL=33" <<<"$OUT"; then
    fail "A: the entity total did not reach the caller's scope -- $OUT"
fi
check
# The entity total and the slot demand are DIFFERENT numbers and both are
# published. Collapsing them is the island's 33-vs-19 error.
if ! nros_grep_q "NROS_ENTITY_COUNT_PUBLISHER=14" <<<"$OUT"; then
    fail "A: per-kind counts did not reach the caller's scope -- $OUT"
fi
check
if ! nros_grep_q "publishers, which claim no callback slot" <<<"$OUT"; then
    fail "A: the status line does not say publishers claim no slot -- $OUT"
fi
# phase-403 step 1. The JOIN KEY has to cross the same function boundary the
# counts do; it is the input `nros_derive_message_bound_knobs` narrows the
# payload classes with, and an absent list there reads as "receives nothing".
check
if ! nros_grep_q "NROS_ENTITY_SUBSCRIBED_TYPES_STATUS=resolved" <<<"$OUT"; then
    fail "A: the subscribed-type status did not reach the caller's scope -- $OUT"
fi
check
if ! nros_grep_q "NROS_ENTITY_SUBSCRIBED_TYPE_COUNTS=nav_msgs/msg/Odometry=1;std_msgs/msg/Int32=2" <<<"$OUT"; then
    fail "A: the per-type ENTITY counts did not reach the caller's scope -- $OUT"
fi
# The two views are different sets and both travel. Collapsing them would
# either price a service request against a pool it never allocates from, or
# leave the arena blind to four receiving kinds.
check
if ! nros_grep_q "NROS_ENTITY_RECEIVED_TYPES=demo/srv/Op_Request;" <<<"$OUT"; then
    fail "A: the wider RECEIVED set did not reach the caller's scope -- $OUT"
fi
# phase-403 step 2. Depth MULTIPLIES the type bound above, so the arena cannot
# be derived without it, and it has to cross the same boundary the rest does.
check
if ! nros_grep_q "NROS_ENTITY_DECLARED_DEPTHS=nav_msgs/msg/Odometry|/localization/kinematic_state=1;" <<<"$OUT"; then
    fail "A: the declared depths did not reach the caller's scope -- $OUT"
fi
# The one that is easy to leave out and expensive to leave out. An image where
# three endpoints stated no depth is not an image with three depth-0 endpoints,
# and a consumer that reads only the LIST would size it from the two that
# happened to be annotated.
check
if ! nros_grep_q "NROS_ENTITY_UNDECLARED_DEPTH_COUNT=3" <<<"$OUT"; then
    fail "A: the UNDECLARED depth count did not reach the caller's scope. \
Without it a consumer cannot tell a fully-declared image from a partly-declared \
one, which is the whole reason the count is published -- $OUT"
fi

# ---------------------------------------------------------------------------
# B. A refusal publishes the reason and NO number.
# ---------------------------------------------------------------------------
log_info "B. a refusal publishes no number"
OUT="$(derive "$REFUSED_BODY" 0 "$META" "$TEST_TMPDIR/b.cmake")"
check
if ! nros_grep_q "NROS_ENTITY_INVENTORY_STATUS=refused" <<<"$OUT"; then
    fail "B: status not refused -- $OUT"
fi
check
if nros_grep_q "NROS_DERIVED_EXECUTOR_MAX_CBS=" <<<"$OUT"; then
    fail "B: a refusal published a MAX_CBS -- $OUT"
fi
check
if ! nros_grep_q "declare no entities" <<<"$OUT"; then
    fail "B: the refusal reason did not reach the caller -- $OUT"
fi
# phase-403 step 2 -- and a refusal publishes NO depth list either. An absent
# list read as "every endpoint is depth 0" would size an arena an order of
# magnitude short, which is the same failure the absent TYPE list would cause
# one line up.
check
if nros_grep_q "NROS_ENTITY_DECLARED_DEPTHS=" <<<"$OUT"; then
    fail "B: a refusal published a depth list -- $OUT"
fi
check
if ! nros_grep_q "NROS_ENTITY_DECLARED_DEPTH_STATUS=refused" <<<"$OUT"; then
    fail "B: the depth view published no status, so a reader cannot tell \
\"refused\" from \"this fragment predates the field\" -- $OUT"
fi

# ---------------------------------------------------------------------------
# C. A refusal AFTER a derivation leaves no stale number standing.
# ---------------------------------------------------------------------------
log_info "C. a second, refusing call clears the first call's answer"
SEQ="$TEST_TMPDIR/seq.cmake"
cat > "$SEQ" <<EOF
include("$MODULE")
nros_derive_entity_inventory_knobs(CLI "$STUB" METADATA "$META"
    OUTPUT_FILE "$TEST_TMPDIR/c1.cmake" QUIET)
message(STATUS "first=\${NROS_DERIVED_EXECUTOR_MAX_CBS}")
set(ENV{NROS_STUB_BODY} "$REFUSED_BODY")
nros_derive_entity_inventory_knobs(CLI "$STUB" METADATA "$META"
    OUTPUT_FILE "$TEST_TMPDIR/c2.cmake" QUIET)
message(STATUS "second=[\${NROS_DERIVED_EXECUTOR_MAX_CBS}]")
message(STATUS "second_depths=[\${NROS_ENTITY_DECLARED_DEPTHS}]")
EOF
OUT="$(NROS_STUB_BODY="$DERIVED_BODY" cmake -P "$SEQ" 2>&1)"
check
if ! nros_grep_q "first=19" <<<"$OUT"; then
    fail "C: the first call did not publish -- $OUT"
fi
check
if ! nros_grep_q "second=\[\]" <<<"$OUT"; then
    fail "C: a refusal left the previous number standing -- $OUT"
fi
# phase-403 step 2 -- the depth list is cleared by the same rule. A stale depth
# table is worse than a stale count: it names topics that may no longer be
# subscribed, at depths the current declaration never stated.
check
if ! nros_grep_q "second_depths=\[\]" <<<"$OUT"; then
    fail "C: a refusal left the previous DEPTH LIST standing -- $OUT"
fi

# ---------------------------------------------------------------------------
# D. An unrecognised schema is FATAL, both shapes.
# ---------------------------------------------------------------------------
log_info "D. an unrecognised schema refuses to be read"
# cmake re-wraps a `message(FATAL_ERROR)` body at ~72 columns, so the phrases
# below are matched against the output with newlines squashed. Grepping the raw
# text passes only by accident of where the wrap lands.
flat() { tr '\n' ' ' | tr -s ' '; }
OUT="$(derive "$BAD_SCHEMA_BODY" 0 "$META" "$TEST_TMPDIR/d1.cmake" | flat)"
check
if ! nros_grep_q "states entity-inventory schema version 4" <<<"$OUT"; then
    fail "D: a future schema was read rather than refused -- $OUT"
fi
check
if ! nros_grep_q -qi "CMake Error" <<<"$OUT"; then
    fail "D: a future schema did not raise a FATAL_ERROR -- $OUT"
fi
OUT="$(derive "$NO_SCHEMA_BODY" 0 "$META" "$TEST_TMPDIR/d2.cmake" | flat)"
check
if ! nros_grep_q "sets no NROS_ENTITY_INVENTORY_SCHEMA_VERSION" <<<"$OUT"; then
    fail "D: a fragment with no schema was read rather than refused -- $OUT"
fi

# ---------------------------------------------------------------------------
# E. A missing metadata file refuses; it does not break the configure.
# ---------------------------------------------------------------------------
log_info "E. a missing metadata file refuses rather than fataling"
OUT="$(derive "$DERIVED_BODY" 0 "$TEST_TMPDIR/absent.json" "$TEST_TMPDIR/e.cmake")"
check
if nros_grep_q -qi "CMake Error" <<<"$OUT"; then
    fail "E: a missing metadata file was fatal -- $OUT"
fi
check
if ! nros_grep_q "NROS_ENTITY_INVENTORY_STATUS=refused" <<<"$OUT"; then
    fail "E: a missing metadata file did not refuse -- $OUT"
fi
check
if ! nros_grep_q "nothing in this image called nano_ros_node_register" <<<"$OUT"; then
    fail "E: the refusal does not say what is missing -- $OUT"
fi

# ---------------------------------------------------------------------------
# F. A missing CLI refuses too.
# ---------------------------------------------------------------------------
log_info "F. a missing CLI refuses rather than fataling"
OUT="$(derive "$DERIVED_BODY" 0 "$META" "$TEST_TMPDIR/f.cmake" "$TEST_TMPDIR/no-such-nros")"
check
if nros_grep_q -qi "CMake Error" <<<"$OUT"; then
    fail "F: a missing CLI was fatal -- $OUT"
fi
check
if ! nros_grep_q "NROS_ENTITY_INVENTORY_STATUS=refused" <<<"$OUT"; then
    fail "F: a missing CLI did not refuse -- $OUT"
fi

# ---------------------------------------------------------------------------
# G. A CLI that fails is FATAL -- a broken declaration, not an absent one.
# ---------------------------------------------------------------------------
log_info "G. a broken declaration is fatal and names the fix"
OUT="$(NROS_STUB_STDERR="component \`demo::n\` declares \`publsher\`" \
       derive "" 1 "$META" "$TEST_TMPDIR/g.cmake")"
check
if ! nros_grep_q "not readable" <<<"$OUT"; then
    fail "G: a failing CLI was not fatal -- $OUT"
fi
check
if ! nros_grep_q "publsher" <<<"$OUT"; then
    fail "G: the CLI's own message was swallowed -- $OUT"
fi

# ---------------------------------------------------------------------------
# H. Seeding: a refusal leaves a well-formed fragment, and never clobbers one.
# ---------------------------------------------------------------------------
log_info "H. a refusal seeds a fragment, and seeding never clobbers an answer"
SEED="$TEST_TMPDIR/h.cmake"
derive "$DERIVED_BODY" 0 "$TEST_TMPDIR/absent.json" "$SEED" >/dev/null
check
if [ ! -f "$SEED" ]; then
    fail "H: no fragment was seeded -- a CMAKE_CONFIGURE_DEPENDS input with no rule is a ninja load error"
fi
check
if ! nros_grep_q "NROS_ENTITY_INVENTORY_SCHEMA_VERSION" "$SEED"; then
    fail "H: the seeded fragment states no schema version, so reading it later FATALs"
fi
check
if ! nros_grep_q 'NROS_ENTITY_INVENTORY_STATUS "refused"' "$SEED"; then
    fail "H: the seeded fragment does not read as a refusal"
fi
KEEP="$TEST_TMPDIR/h-keep.cmake"
cp "$DERIVED_BODY" "$KEEP"
CMD="$TEST_TMPDIR/h-seed.cmake"
cat > "$CMD" <<EOF
include("$MODULE")
nros_entity_inventory_seed_knobs_file("$KEEP")
EOF
cmake -P "$CMD" >/dev/null 2>&1
check
if ! nros_grep_q "NROS_DERIVED_EXECUTOR_MAX_CBS 19" "$KEEP"; then
    fail "H: seeding overwrote an existing answer"
fi

# ---------------------------------------------------------------------------
if [ "$FAILURES" -eq 0 ]; then
    log_success "cmake-entity-inventory: $CHECKS assertion(s) held"
    exit 0
fi
log_error "cmake-entity-inventory: $FAILURES of $CHECKS assertion(s) failed"
exit 1
