#!/bin/bash
# tests/cmake-declared-qos-header-tests.sh -- phase-403 step 2 / issue 1084
#
# Does the DECLARED QoS depth actually REACH A COMPILER?
#
# `just check cpp` proves the CHECK against a COMMITTED fixture header: it
# compiles a TU whose declared depth and passed QoS disagree and asserts the
# build is rejected. It hands the table to the compiler with a `-I` of its own,
# so it proves the MECHANISM and says nothing about the DELIVERY.
#
# This gate is the delivery half, and it is the half this campaign keeps
# getting wrong: six sizing mechanisms so far have been correct and unreachable,
# because the number never arrived where the code that reads it runs. Here the
# unreachable shape is silent by construction -- a component whose table never
# lands compiles with every declared-depth assertion disabled, and looks exactly
# like a component whose depths all agree.
#
# phase-412 made that literal. It retired `ENTITIES`, which was the ONLY thing
# that ever produced a depth table, and for a while this file drove a cmake
# function with no production caller (issue 1084). The producer is now the
# CONTRACT: a `qos: { depth: N }` on a subscriber endpoint rides the resolved
# SystemModel that `nano_ros_entry()` already passes to the entity inventory.
#
# So this drives the REAL seam in a REAL configure with the REAL `nros` CLI --
# `_nros_declared_qos_arm()` at register time, `_nros_declared_qos_record_model()`
# at entry time, and the deferred render between them -- and asserts:
#
#   A. a contract's declared depth lands at the path the include spelling needs
#      (`<dir>/nros/nros_declared_qos_generated.h`), keyed on the TOPIC a call
#      site writes and on BOTH type spellings;
#   B. `<dir>` is on the target's include path, so `__has_include` in
#      `nros/declared_qos.hpp` finds it;
#   C. the dir is PRIVATE -- a consumer that links the component must not
#      inherit a table describing somebody else's call sites;
#   D. an image with NO contract gets NO table and NO error. Absence is not zero
#      here either: that image has not opted in;
#   E. a BROKEN contract is FATAL and names the component. "You have not
#      declared" and "what you declared is wrong" license different actions,
#      and only the second may stop the build;
#   F. the NEGATIVE CONTROL, end to end: a call site that DISAGREES with the
#      depth this configure just rendered fails to compile, naming the topic and
#      both numbers -- and the agreeing call site beside it compiles clean, so a
#      table that matched nothing could not read as a pass;
#   G. the producer is WIRED. A + F both drive the seam directly, so deleting
#      the call from `nano_ros_node_register()` would leave them green and every
#      real component unchecked. That is issue 1084 exactly, and it is the one
#      thing this gate exists to make impossible twice.
#
# Needs cmake, a C and a C++ compiler, and a built `nros` (`just setup-cli`).
# The `just` recipe records a SKIP when any is missing -- which is a different
# claim from "it passed", and belongs in the check ledger rather than here.
#
# Usage: ./tests/cmake-declared-qos-header-tests.sh
# Exit:  0 all assertions held; 1 otherwise.

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# shellcheck source=lib/common.sh
source "$SCRIPT_DIR/lib/common.sh"

MODULE="$PROJECT_ROOT/cmake/NanoRosNodeRegister.cmake"
ENTRY_MODULE="$PROJECT_ROOT/cmake/NanoRosEntry.cmake"

FAILURES=0
CHECKS=0
fail() { log_error "$*"; FAILURES=$((FAILURES + 1)); }
check() { CHECKS=$((CHECKS + 1)); }

if [ ! -f "$MODULE" ]; then
    fail "module not found: $MODULE"
    exit 1
fi
if ! command -v cmake >/dev/null 2>&1; then
    fail "cmake is not on PATH -- this test cannot report a verdict without it"
    exit 1
fi

# The REAL CLI, not a stub: the CONTENT of the header is the thing under test,
# and a stub that writes a file only proves cmake can copy one.
NROS_BIN="${NROS_CLI:-$PROJECT_ROOT/packages/cli/target/release/nros}"
if [ ! -x "$NROS_BIN" ]; then
    fail "no built \`nros\` at $NROS_BIN -- run \`just setup-cli\`"
    exit 1
fi

CXX="${CXX:-c++}"
if ! command -v "$CXX" >/dev/null 2>&1; then
    fail "no C++ compiler ($CXX) -- case F is the negative control and cannot be skipped
quietly; a gate that examined nothing must not read as a pass"
    exit 1
fi

init_test_tmpdir "nros-declared-qos-header"
trap 'cleanup_test_tmpdir' EXIT

# The metadata every case starts from: one component, and NO `entities` key.
# That is what `_nros_metadata_emit()` writes today -- `ENTITIES` is retired, so
# a metadata row carries the component's identity and nothing about its wiring.
METADATA="$TEST_TMPDIR/metadata.json"
cat > "$METADATA" <<'EOF'
{"components": [{"name": "listener", "pkg": "demo", "class": "demo::Listener"}]}
EOF

# One tiny project per case. The two production seams are called directly rather
# than through `nano_ros_node_register()` / `nano_ros_entry()`, because those
# verbs need the whole `find_package(nano_ros)` surface and this gate is about
# the seam. The arguments and the ORDER are exactly what those verbs pass: the
# register call arms first, knowing no model, and the entry records the model
# afterwards. Case G asserts the verbs really do call them.
#
#   configure <case> <model-or-empty> [extra cmake lines]
configure() {
    local name="$1" model="$2" extra="${3:-}"
    local dir="$TEST_TMPDIR/$name"
    mkdir -p "$dir/build"
    echo "int nros_dq_probe(void) { return 0; }" > "$dir/x.c"
    {
        echo 'cmake_minimum_required(VERSION 3.22)'
        # The pkg in the metadata is `PROJECT_NAME`, because that is what
        # `_nros_metadata_emit()` writes -- so the project must be `demo` for
        # the `demo::listener` row to be the one narrowed to.
        echo 'project(demo LANGUAGES C)'
        echo "include(\"$MODULE\")"
        echo 'add_library(demo_listener_component STATIC x.c)'
        echo '_nros_declared_qos_arm("listener" "demo_listener_component" "CPP")'
        if [ -n "$model" ]; then
            echo "_nros_declared_qos_record_model(\"$model\")"
        fi
        echo 'get_target_property(_inc demo_listener_component INCLUDE_DIRECTORIES)'
        echo 'get_target_property(_iface demo_listener_component INTERFACE_INCLUDE_DIRECTORIES)'
        echo 'message(STATUS "PRIVATE_INCLUDES=[${_inc}]")'
        echo 'message(STATUS "IFACE_INCLUDES=[${_iface}]")'
        printf '%s\n' "$extra"
    } > "$dir/CMakeLists.txt"
    cp "$METADATA" "$dir/build/nros-metadata.json"
    (cd "$dir/build" && NROS_CLI="$NROS_BIN" cmake .. 2>&1)
}

# ---------------------------------------------------------------------------
# A + B + C. A contract's declared depth reaches a header on the target's
# PRIVATE include path.
# ---------------------------------------------------------------------------
log_info "A. a contract's declared depth reaches a header on the component's include path"

# The three cases the C++ check has to tell apart, as a resolved SystemModel:
#
#   /chatter     declared depth 1 -- a row exists; a QoS(1) agrees and a
#                                    QoS(10) must fail the BUILD (case F)
#   /undeclared  no `qos:` at all -- NO row; nothing asserts, and the
#                                    3-argument NROS_SUBSCRIBE keeps its
#                                    historical default profile
#
# Note the two spellings the contract uses: `/listener/chatter` is the ENDPOINT
# REF it addresses the subscription by, `/chatter` is the topic the code
# subscribes to. Only the second may reach the table (issue 1084).
DECLARED_MODEL="$TEST_TMPDIR/declared_model.yaml"
cat > "$DECLARED_MODEL" <<'EOF'
meta: { version: 1 }
structure:
  nodes:
    /listener:
      { scope: s.launch.xml, pkg: demo, exec: listener, node_name: listener }
  topics:
    /chatter:
      type: std_msgs/msg/Int32
      sub: [/listener/chatter]
    /undeclared:
      type: std_msgs/msg/Bool
      sub: [/listener/undeclared]
contracts:
  sub_endpoints:
    /listener/chatter:
      qos: { depth: 1 }
EOF

OUT="$(configure declared "$DECLARED_MODEL")"
HDR="$TEST_TMPDIR/declared/build/nros-declared-qos/listener/nros/nros_declared_qos_generated.h"
check
if [ ! -f "$HDR" ]; then
    fail "A: no header at $HDR -- the contract never reaches a compiler.
Everything downstream (the static_assert, the boot-time check) then does nothing
at all, silently. Configure said: $OUT"
fi
if [ -f "$HDR" ]; then
    check
    if ! grep -q 'NROS_DECLARED_QOS_ROW("std_msgs::msg::dds_::Int32_", "/chatter", 1)' "$HDR"; then
        fail "A: the header carries no row for the declared endpoint -- $(cat "$HDR")"
    fi
    check
    if ! grep -q 'NROS_DECLARED_QOS_ROW("std_msgs/msg/Int32", "/chatter", 1)' "$HDR"; then
        fail "A: the ROS spelling of the type is missing, so a message class carrying
it would look undeclared -- $(cat "$HDR")"
    fi
    check
    # The row must be keyed on the TOPIC. The endpoint ref is how the contract
    # ADDRESSES the subscription and is a string no call site ever writes; a
    # table keyed on it matches nothing and asserts nothing.
    if grep -q '"/listener/chatter"' "$HDR"; then
        fail "A: the table is keyed on the contract's endpoint ref, not on the topic the
code subscribes to. Every assertion built on it is then vacuous -- $(cat "$HDR")"
    fi
    check
    # The endpoint the contract declares NO depth for must produce no row. A row
    # at some default would be the whole defect this step exists to prevent.
    if grep -q '"/undeclared"' "$HDR"; then
        fail "A: an endpoint that declared no depth got a row -- absence became a number"
    fi
fi
check
if ! grep -q "PRIVATE_INCLUDES=\[.*nros-declared-qos/listener\]" <<<"$OUT"; then
    fail "B: the generated dir is not on the target's include path, so
\`__has_include(<nros/nros_declared_qos_generated.h>)\` finds nothing and every
call site in the component compiles unchecked -- $OUT"
fi
check
if grep -q "IFACE_INCLUDES=\[.*nros-declared-qos" <<<"$OUT"; then
    fail "C: the dir leaked onto INTERFACE_INCLUDE_DIRECTORIES. A consumer that
links this component would then have its OWN NROS_SUBSCRIBE calls checked against
this component's declaration -- $OUT"
fi

# ---------------------------------------------------------------------------
# D. No contract is not an error.
# ---------------------------------------------------------------------------
log_info "D. an image with no contract gets no table and no error"
OUT="$(configure undeclared "")"
check
if grep -q "CMake Error" <<<"$OUT"; then
    fail "D: an image with no contract broke the configure. Every image in this tree
without a contract sidecar is in exactly that state -- $OUT"
fi
check
if [ -f "$TEST_TMPDIR/undeclared/build/nros-declared-qos/listener/nros/nros_declared_qos_generated.h" ] && \
   grep -q "NROS_DECLARED_QOS_ROWS" \
        "$TEST_TMPDIR/undeclared/build/nros-declared-qos/listener/nros/nros_declared_qos_generated.h"; then
    fail "D: an image that declared nothing got a table of rows"
fi
check
if ! grep -q "PRIVATE_INCLUDES=\[.*nros-declared-qos/listener\]" <<<"$OUT"; then
    fail "D: the include dir must be claimed even with no contract -- it is claimed at
REGISTER time, before any model exists, and that split is what makes a
model-driven producer possible at all (issue 1084) -- $OUT"
fi

# ---------------------------------------------------------------------------
# E. A broken contract is FATAL and names the component.
# ---------------------------------------------------------------------------
log_info "E. a broken contract stops the build and names the component"
BAD_MODEL="$TEST_TMPDIR/bad_model.yaml"
cat > "$BAD_MODEL" <<'EOF'
meta: { version: 1 }
structure:
  nodes:
    /listener:
      { scope: s.launch.xml, pkg: demo, exec: listener, node_name: listener }
  topics:
    /chatter:
      type: std_msgs/msg/Int32
      sub: [/listener/chatter]
contracts:
  sub_endpoints:
    /listener/chatter:
      qos: { depth: 0 }
EOF
OUT="$(configure broken "$BAD_MODEL" | tr '\n' ' ' | tr -s ' ')"
check
if ! grep -q "CMake Error" <<<"$OUT"; then
    fail "E: a \`depth: 0\` contract configured cleanly. KEEP_LAST(0) holds no sample,
so it is a typo for \"I did not want to say\" -- and rendering it would fail the
build at every call site on that topic, naming a number nobody meant -- $OUT"
fi
check
if ! grep -q "demo::listener" <<<"$OUT"; then
    fail "E: the failure does not name the component the user has to fix -- $OUT"
fi
check
if ! grep -q "states nothing" <<<"$OUT"; then
    fail "E: the CLI's own reason did not reach the user -- $OUT"
fi

# ---------------------------------------------------------------------------
# F. THE NEGATIVE CONTROL, end to end.
#
# Case A proved a header with the right bytes exists. That is not the same
# claim as "a disagreeing call site is rejected": a table the preprocessor
# never picks up, a macro arm never selected and a topic spelled differently
# all produce exactly the same green. So compile against the header this
# configure JUST RENDERED -- the agreeing call site first, so a table that
# matches nothing shows up as a failure there rather than as a silent pass
# here.
# ---------------------------------------------------------------------------
log_info "F. a call site that DISAGREES with the rendered table fails to compile"
GEN_DIR="$TEST_TMPDIR/declared/build/nros-declared-qos/listener"
probe_src() {
    # `Q` stands in for `nros::QoS` -- the macro asks only for a constexpr
    # `depth()`, and pulling the whole nros-cpp surface in here would make this
    # gate depend on generated per-build sizes headers it has no business
    # building. `NROS_ASSERT_DECLARED_DEPTH` is the real macro; `NROS_SUBSCRIBE`
    # forwards to it, and `just check cpp` compiles that whole path.
    cat <<EOF
#include <nros/declared_qos.hpp>
namespace {
struct Q {
    int d;
    constexpr explicit Q(int v) : d(v) {}
    constexpr int depth() const { return d; }
};
struct Int32 {
    static constexpr const char* TYPE_NAME = "std_msgs::msg::dds_::Int32_";
};
} // namespace
void nros_dq_case_f() {
    NROS_ASSERT_DECLARED_DEPTH(Int32::TYPE_NAME, "/chatter", Q($1), "\"/chatter\"");
}
EOF
}
probe_compile() {
    "$CXX" -fsyntax-only -std=c++17 -fno-exceptions -fno-rtti \
        -I"$GEN_DIR" \
        -I"$PROJECT_ROOT/packages/api/nros-cpp/include" \
        -x c++ "$1" 2>&1
}
probe_src 1 > "$TEST_TMPDIR/agree.cpp"
probe_src 10 > "$TEST_TMPDIR/disagree.cpp"
check
if ! AGREE_OUT="$(probe_compile "$TEST_TMPDIR/agree.cpp")"; then
    fail "F: the AGREEING call site failed to compile. Something is wrong with the
probe or the include path, and the expected-failure assertion below would then
pass for the wrong reason -- $AGREE_OUT"
else
    check
    if DISAGREE_OUT="$(probe_compile "$TEST_TMPDIR/disagree.cpp")"; then
        fail "F: the DISAGREEING call site COMPILED. The contract declares depth 1 for
/chatter and the call site passes depth 10; the static_assert did not fire, so
the declared depth is not reaching the compiler and nothing is checked."
    else
        # A rejection is not enough: it must be OUR rejection, naming the topic
        # and BOTH depths. A typo or a missing header also exits non-zero, and a
        # diagnostic the reader cannot act on is the same as no diagnostic.
        for _want in '"/chatter"' 'declared_depth_agrees<1, 10>'; do
            check
            case "$DISAGREE_OUT" in
                *"$_want"*) ;;
                *) fail "F: the diagnostic does not contain: $_want
It must name the TOPIC and BOTH depths, or a reader cannot tell which
subscription is wrong or which of the two numbers to fix -- $DISAGREE_OUT" ;;
            esac
        done
    fi
fi

# ---------------------------------------------------------------------------
# G. The producer is WIRED.
#
# Every case above calls the seam directly, so all of them stay green with the
# call deleted from `nano_ros_node_register()` -- which is the exact state
# issue 1084 filed: a mechanism verified in every part except the one that
# would run. Both ends are asserted, because either one missing disables the
# whole chain silently.
# ---------------------------------------------------------------------------
log_info "G. the register call arms it and the entry feeds it a model"
check
if ! grep -q "^ *_nros_declared_qos_arm(" "$MODULE"; then
    fail "G: nothing in $MODULE calls _nros_declared_qos_arm(). No component then
claims an include dir, no table is ever rendered, and every case above still
passes -- that is issue 1084 verbatim."
fi
check
if ! grep -q "_nros_declared_qos_record_model(" "$ENTRY_MODULE"; then
    fail "G: nothing in $ENTRY_MODULE calls _nros_declared_qos_record_model(). The
deferred render then finds no model, abstains, and every component compiles with
its declared-depth checks off -- silently."
fi

echo ""
if [ "$FAILURES" -eq 0 ]; then
    log_success "cmake-declared-qos-header: $CHECKS assertion(s) held"
    exit 0
fi
log_error "cmake-declared-qos-header: $FAILURES of $CHECKS assertion(s) failed"
exit 1
