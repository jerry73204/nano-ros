#!/usr/bin/env bash
#
# A capability probe may gate a METHOD. It may never change `sizeof`.
#
# phase-417. This MEASURES the rule rather than grepping for it: it compiles a
# probe TU once per capability macro, forcing that macro on, and compares the
# reported `sizeof` against the baseline. A text scanner for "member inside an
# `#if`" was written first and thrown away — it could not tell a member from a
# local variable, and the file-level include guard made every line look
# conditional (the same false-negative shape that made an earlier scanner in
# this tree report zero).
#
# WHY IT MATTERS. Two TUs of one image disagreeing about a capability is a
# SUPPORTED state, not a misconfiguration:
#
#   * `examples/px4/cpp/bridge/src/modules/nros_uorb_bridge/CMakeLists.txt:123`
#     sets `-DNROS_CPP_STD=1` on ONE module of a larger image, deliberately.
#   * `zephyr/cmake/nros_rmw_cyclonedds.cmake` adds the `cxx-compat` include
#     dir for some targets only, so `__has_include` can answer differently for
#     two TUs of one build.
#
# It already shipped once. `rclcpp::Node` held
# `std::vector<std::shared_ptr<detail::WallTimer>> timers_` behind
# `NROS_CPP_HAS_STD_CHRONO`, reachable only through `NROS_CPP_STD`, so the px4
# bridge module compiled a 3776-byte node while every other TU compiled a
# 3752-byte one. They linked. Each wrote the object through its own layout.
# Restoring both halves reproduces `3752` vs `3776` exactly.
#
# The probes are themselves unreliable — three distinct failure modes measured
# in one day: `<type_traits>` present-but-hollow on Zephyr, `NROS_CPP_STD` set
# by nothing that ships, `<memory>` present-then-`#error` under
# `-ffreestanding` on GCC 13. So this gate does not ask whether a probe is
# right. It removes the class of bug where being wrong changes a layout.
#
# HOW THE SIZE IS READ. `-fsyntax-only` plus an intentionally incomplete
# template: `ShowSize<sizeof(T)> probe;` makes the compiler print the number in
# its own diagnostic. No link, no run, no library — so this needs nothing the
# header lane does not already have.

set -uo pipefail
cd "$(dirname "$0")/.."

TU="$(mktemp -d)/nros_capability_layout_probe.cpp"
INC=(-Itarget/nros-cpp-generated
     -Itarget/nros-c-generated
     -Ipackages/api/nros-cpp/include
     -Ipackages/api/nros-c/include
     -Ipackages/platform/nros-platform-api/include)

# Types whose layout must not move. Add a type here when it gains members.
TYPES=("rclcpp::Node" "::nros::Node" "::nros::QoS")

# Every capability macro the public headers define for themselves, plus the
# consumer-facing opt-in. Forcing one ON is exactly what px4 does.
CAPS=(NROS_CPP_STD
      NROS_CPP_HAS_SHARED_PTR
      NROS_CPP_HAS_STD_STRING
      NROS_CPP_HAS_STD_VECTOR
      NROS_CPP_HAS_STD_FUNCTION
      NROS_CPP_HAS_STD_CHRONO
      NROS_CPP_HAS_STD_SSTREAM)

size_of() { # $1 = type, rest = extra flags
    local ty="$1"; shift
    printf '#include <nros/nros.hpp>\ntemplate <int N> struct ShowSize;\nShowSize<static_cast<int>(sizeof(%s))> probe;\n' "$ty" > "$TU"
    c++ -fsyntax-only -std=c++17 -fno-exceptions -fno-rtti "$@" "${INC[@]}" "$TU" 2>&1 |
        grep -oE 'ShowSize<[0-9]+' | head -1 | sed 's/ShowSize<//'
}

# NEGATIVE CONTROL, on the normal path.
#
# The measurement can only be trusted if it is known to FAIL when a layout does
# follow a probe. That is not hypothetical here: forcing a macro that is already
# on in the baseline is a no-op, so a version of this gate could look green and
# detect nothing. This synthesises the divergence in a standalone TU -- no nros
# headers, no include path -- and asserts both directions.
selftest() {
    local tu="${TU%/*}/selftest.cpp" a b c d
    cat > "$tu" <<'EOF'
struct Conditional {
    void* always;
#ifdef NROS_SELFTEST_CAP
    double gated_member;
#endif
};
struct Invariant {
    void* always;
#ifdef NROS_SELFTEST_CAP
    void gated_method();
#endif
};
template <int N> struct ShowSize;
#ifdef NROS_SELFTEST_PICK_INVARIANT
ShowSize<static_cast<int>(sizeof(Invariant))> probe;
#else
ShowSize<static_cast<int>(sizeof(Conditional))> probe;
#endif
EOF
    _st() { c++ -fsyntax-only -std=c++17 "$@" "$tu" 2>&1 | grep -oE 'ShowSize<[0-9]+' | head -1 | sed 's/ShowSize<//'; }
    a="$(_st)"; b="$(_st -DNROS_SELFTEST_CAP=1)"
    c="$(_st -DNROS_SELFTEST_PICK_INVARIANT=1)"
    d="$(_st -DNROS_SELFTEST_PICK_INVARIANT=1 -DNROS_SELFTEST_CAP=1)"
    if [ -z "$a" ] || [ -z "$b" ] || [ -z "$c" ] || [ -z "$d" ]; then
        echo "check-cpp-capability-layout --selftest: the size probe produced no number" >&2
        exit 1
    fi
    if [ "$a" = "$b" ]; then
        echo "check-cpp-capability-layout --selftest: a CONDITIONAL MEMBER did not change sizeof ($a vs $b)" >&2
        echo "  The measurement cannot see the defect it exists to catch." >&2
        exit 1
    fi
    if [ "$c" != "$d" ]; then
        echo "check-cpp-capability-layout --selftest: a gated METHOD changed sizeof ($c vs $d)" >&2
        echo "  The gate would fail on the shape it is supposed to permit." >&2
        exit 1
    fi
    echo "check-cpp-capability-layout --selftest: 2 case(s) OK (member diverges, method does not)"
}

selftest

fail=0
for ty in "${TYPES[@]}"; do
    base="$(size_of "$ty")"
    if [ -z "$base" ]; then
        echo "check-cpp-capability-layout: could not measure sizeof($ty) in the baseline configuration" >&2
        echo "  (the probe TU did not compile; this gate would otherwise pass on absence)" >&2
        exit 1
    fi
    for cap in "${CAPS[@]}"; do
        got="$(size_of "$ty" "-D${cap}=1")"
        [ -z "$got" ] && continue   # forcing this macro makes the TU not compile; not a layout question
        if [ "$got" != "$base" ]; then
            echo "FAIL: sizeof($ty) changes with -D${cap}=1 — ${base} vs ${got}" >&2
            fail=1
        fi
    done
done

if [ "$fail" -ne 0 ]; then
    cat >&2 <<'MSG'

  A capability probe changed a LAYOUT. Two TUs of one image are allowed to
  disagree about that macro — px4's bridge sets -DNROS_CPP_STD on one module
  of a larger image on purpose — so they would link and then write the object
  through one layout and read it through the other. Silent. Issues 0135, 0460.

  Fix: hold the capability-dependent thing through an UNCONDITIONAL member.
  `rclcpp::Node` keeps wall-timer cells in
  `std::vector<std::shared_ptr<void>> owned_entities_` for exactly this reason,
  instead of the typed `timers_` that used to sit behind a `#ifdef`.
MSG
    exit 1
fi

echo "check-cpp-capability-layout: OK — ${#TYPES[@]} type(s) x ${#CAPS[@]} capability macro(s), no layout depends on a probe"
