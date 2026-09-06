#!/usr/bin/env bash
# phase-436 W5 (issue 1196) — the UNBOUNDED condvar wait stays confined to the
# shim that bridges an upstream API which is itself unbounded.
#
# `nros_platform_condvar_wait` blocks forever by construction: every port
# implements it with that port's forever spelling (`TX_WAIT_FOREVER`,
# `portMAX_DELAY`, `K_FOREVER`). A bounded `nros_platform_condvar_wait_until`
# sits directly beside it, and the executor's own wait already uses the bounded
# `nros_platform_wake_wait_ms`.
#
# ONE exemption, and it is not a grandfathering: `platform_aliases.c` implements
# zenoh-pico's `_z_condvar_wait`, whose CONTRACT is an unbounded wait. Bridging
# an unbounded upstream primitive to an unbounded platform primitive is the
# correct mapping; narrowing it there would silently change zenoh-pico's
# semantics. The exemption is by path, so a second such call anywhere else
# still has to be argued for.
#
# The risk this guards is the next backend or port: `condvar_wait` is the
# obvious spelling, so it gets reached for and the bound is lost silently and
# without review. A marked header states the rule; only a gate keeps it.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

ALLOWED='packages/rmw/zenoh/zpico-sys/c/zpico/platform_aliases.c'

# Match the call, never the bounded `_until` sibling and never a declaration.
# `git grep` — an index lookup, not a filesystem walk (check-no-tracked-file-find).
hits="$(git grep -n -e 'nros_platform_condvar_wait[^_a-zA-Z]' -- \
    'packages/core/**/*.rs' 'packages/core/**/*.c' 'packages/core/**/*.cpp' \
    'packages/core/**/*.h' 'packages/core/**/*.hpp' \
    'packages/rmw/**/*.rs' 'packages/rmw/**/*.c' 'packages/rmw/**/*.cpp' \
    'packages/rmw/**/*.h' 'packages/rmw/**/*.hpp' \
    'packages/api/**/*.rs' 'packages/api/**/*.c' 'packages/api/**/*.cpp' \
    'packages/api/**/*.h' 'packages/api/**/*.hpp' 2>/dev/null \
    | grep -v "^${ALLOWED}:" || true)"

if [ -n "$hits" ]; then
    echo "check-no-unbounded-condvar-wait: the UNBOUNDED condvar wait is called outside the zenoh-pico alias shim:" >&2
    printf '%s\n' "$hits" | sed 's/^/  /' >&2
    echo >&2
    echo '  nros_platform_condvar_wait has no deadline. Use' >&2
    echo '  nros_platform_condvar_wait_until (absolute ms deadline), or the' >&2
    echo "  executor's nros_platform_wake_wait_ms. See issue 1196, phase-436 W5." >&2
    exit 1
fi
echo "check-no-unbounded-condvar-wait OK (confined to the zenoh-pico alias shim)."
