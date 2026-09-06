#!/usr/bin/env bash
# Issue 0332 — the freestanding-header contract, enforced at the source.
#
# Issue 1023 widened the SCOPE, and the reason is the 0196 rule: this gate's
# coverage was narrower than the rule it enforces. It read `nros-cpp`'s public
# headers only, while `cmake/toolchain/riscv64-threadx.cmake:219` puts
# `-nostdinc++ -isystem <cxx-compat>` on EVERY C++ TU built for that board —
# which includes the whole Cyclone backend. The class then bit there twice in
# one file: issue 0942 (`<cstdio>`) and issue 1014 (`<memory>` + `<string>`,
# which meant the Cyclone backend had never built for that board at all).
#
# nros-cpp public headers must be includable on an embedded target with a
# MINIMAL C++ library (Zephyr's libcpp: `<cstdint>`/`<cstddef>` yes,
# `<string>`/`<vector>` no). The 0112 rule is that a hosted STL include gates on
# `NROS_CPP_STD`, never on `__STDC_HOSTED__` alone — a hosted compiler run
# `-nostdinc++` against that minimal libcpp still has no `<string>`.
#
# The `-ffreestanding` compile probe in `just check cpp` cannot see this: it runs
# against the host's full libstdc++, so an ungated `#include <string>` compiles
# clean. A `-nostdinc++` variant would need Zephyr's libcpp on the include path,
# which the probe host does not have. This gate detects the class at the source
# level instead: a hosted STL `#include` that is not inside an `#ifdef
# NROS_CPP_STD` / `#if defined(NROS_CPP_STD)` region is a violation.
set -euo pipefail
cd "$(dirname "$0")/.."

# The two trees this toolchain compiles with the shim on the include path. Not
# a universal claim: it is these two because these are what a board build
# reaches, and naming them is honest where "every C++ file" would not be.
#
# There is a THIRD location that compiles `-nostdinc++`, deliberately left out:
# `packages/api/nros-cpp/tests/compile/*.cpp`. Those TUs exist to BE compiled
# under the constraint, so a hosted include there fails the compile probe in
# `just check cpp` loudly and immediately. This gate earns its keep where no
# compile runs — a source-level check is a substitute for a build, not a
# duplicate of one. Measured 2026-09-05 by grepping `-nostdinc++` across
# `packages/` and `cmake/`; the only other hits are prose.
SCAN_DIRS="packages/api/nros-cpp/include/nros packages/rmw/cyclonedds/nros-rmw-cyclonedds/src"


# Hosted-only STL headers absent from a minimal freestanding libcpp. The
# freestanding-guaranteed set (`<cstdint>`, `<cstddef>`, `<cstdlib>`,
# `<cstring>`, `<type_traits>`, `<utility>`, `<new>`, `<initializer_list>`,
# `<limits>`, `<cstdarg>`, `<cstdio>`) is deliberately NOT listed — those are
# allowed ungated.
HOSTED='string|vector|map|unordered_map|unordered_set|set|functional|memory|chrono|sstream|iostream|fstream|ostream|istream|algorithm|deque|list|thread|mutex|future|regex'

violations=0

# TWO CONTRACTS, and conflating them would either miss violations or reject
# correct code:
#
#   strict=1  nros-cpp's public headers. The 0112 rule is specific — a hosted
#             include gates on `NROS_CPP_STD` and on nothing else, because a
#             HOSTED compiler run `-nostdinc++` against Zephyr's minimal libcpp
#             still has no `<string>`. `__STDC_HOSTED__` is the wrong question.
#   strict=0  a backend TU. It has no `NROS_CPP_STD` notion; it guards on the
#             PLATFORM, which is the right question there —
#             `nros-rmw-cyclonedds/src/internal.hpp` takes `<chrono>`/`<thread>`
#             only in the `#else` of its `NROS_PLATFORM_*` chain. Any
#             conditional counts; depth 0 does not.
for entry in "packages/api/nros-cpp/include/nros:1" \
             "packages/rmw/cyclonedds/nros-rmw-cyclonedds/src:0"; do
  dir="${entry%:*}"
  strict="${entry##*:}"
  # `.cpp` too, not only headers: the 1014 break was in a TRANSLATION UNIT, and
  # a TU that cannot compile is exactly as broken as a header that cannot be
  # included.
  for hdr in $(ls "$dir"/*.hpp "$dir"/*.cpp 2>/dev/null); do
    base="$(basename "$hdr")"

    # Walk the file tracking NROS_CPP_STD guard depth. Flag a hosted `#include`
    # that appears at guard-depth 0.
    hits="$(awk -v hosted="$HOSTED" -v strict="$strict" '
        BEGIN { depth = 0 }
        # Enter an NROS_CPP_STD region: `#ifdef NROS_CPP_STD` or
        # `#if defined(NROS_CPP_STD)`. Other #if/#ifdef push a neutral level so
        # a nested #endif does not close the NROS_CPP_STD region prematurely.
        /^[[:space:]]*#[[:space:]]*(ifdef|if)([[:space:]]|\().*NROS_CPP_STD/ { stack[++sp] = "std"; next }
        # `\b` is NOT a word boundary in POSIX ERE — awk reads it as an escape
        # with no such meaning, so these two rules MATCHED NOTHING. The "other"
        # push therefore never happened, which is the very bug the comment above
        # says it prevents: a nested `#endif` popped the NROS_CPP_STD region
        # early. Latent while every guarded include sat in a flat `#ifdef`;
        # found by issue 1023 when a backend TU with a real `#if/#elif/#else`
        # chain came into scope. `([[:space:]]|$)` is the portable spelling.
        /^[[:space:]]*#[[:space:]]*(ifdef|ifndef|if)([[:space:]]|$)/ { stack[++sp] = "other"; next }
        /^[[:space:]]*#[[:space:]]*endif([[:space:]]|$)/ { if (sp > 0) sp-- ; next }
        {
            guarded = 0
            if (strict == 1) {
                for (i = 1; i <= sp; i++) if (stack[i] == "std") guarded = 1
            } else {
                guarded = (sp > 0)
            }
            if (guarded) next
            if ($0 ~ ("^[[:space:]]*#[[:space:]]*include[[:space:]]*<(" hosted ")>")) {
                printf "%d: %s\n", NR, $0
            }
        }
    ' "$hdr")"

    if [ -n "$hits" ]; then
        echo "check-cpp-freestanding-includes: $base includes a hosted STL header outside a guard (issues 0332/1023):" >&2
        while IFS= read -r line; do echo "  $base:$line" >&2; done <<<"$hits"
        violations=1
    fi
  done
done

if [ "$violations" -ne 0 ]; then
    echo >&2
    echo "Fix: wrap the hosted section in a guard — \`#ifdef NROS_CPP_STD\` for nros-cpp" >&2
    echo "(std_compat.hpp / bridge.hpp), or a platform \`#if\` for a backend TU" >&2
    echo "(nros-rmw-cyclonedds/src/internal.hpp does the latter for <chrono>/<thread>)." >&2
    echo "If neither fits, the header is not available on that board: do without it." >&2
    exit 1
fi

count="$(for d in $SCAN_DIRS; do ls "$d"/*.hpp "$d"/*.cpp 2>/dev/null; done | wc -l)"
echo "check-cpp-freestanding-includes: OK ($count file(s) across nros-cpp headers and the Cyclone backend; no ungated hosted STL includes)"
