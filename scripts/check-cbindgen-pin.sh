#!/usr/bin/env bash
#
# Issue 0452 — the cbindgen requirement must stay EXACT, in one place.
#
# WHY AN EXACT REQUIREMENT AND NOT A PIN FILE
#
# `nros_generated.h` and `nros_cpp_ffi.h` are COMMITTED headers that `build.rs`
# regenerates IN PLACE (nros-build-helpers `generate_cbindgen_header`). cbindgen's
# output moves between patch releases: 0.29.4 narrows the C23 enum-base guard the
# committed headers carry, ~36 lines across the two files. So a graph that
# resolves a different patch release silently dirties the worktree, and
# committing that result REVERTS an upstream improvement — it had to be
# hand-reverted twice during phase-338.
#
# A caret `"0.29"` cannot prevent this. The ROOT lock pins one version, but the
# leaves that regenerate these headers have no tracked `Cargo.lock` (an example's
# lock is never committed — CLAUDE.md), so each resolves the caret freshly.
# `packages/testing/nros-bench/wake-latency-cortex-m3` was measured building
# 0.29.4 while the root lock said 0.29.3.
#
# phase-400 W2a NARROWED THIS, AND THE COUNT BELOW WILL SAY SO.
#
# cbindgen is now an OPTIONAL dependency of `nros-build-helpers` /
# `nros-zpico-build`, activated only by `nros-cbindgen-headers` (the single
# writer). So no leaf resolves cbindgen at all any more, and the "a lockless
# leaf resolves the caret freshly" hazard above is closed STRUCTURALLY rather
# than by this gate. That moved arm 3 from 19 tracked locks to 1 (the root).
#
# That is a real reduction in what this gate can observe, and it is not a
# regression: what it used to watch no longer exists. The pin still matters —
# the regenerator's output is the committed headers — so arm 1 and arm 2 are
# unchanged, and the vacuity guard at the bottom still refuses a silent zero.
#
# This is deliberately NOT the `[tool.clang-format]` / bindgen-cli treatment.
# Those are PATH binaries with no resolver, so they need a version file plus a
# provisioning recipe. cbindgen is a cargo dependency: cargo's resolver IS the
# pinning mechanism, and a separate pin file would be a second spelling of the
# same fact — the drift class this repo keeps re-learning.
#
# Three things have to hold, and each has failed somewhere in this repo before:
#   1. the workspace requirement is exact (`=x.y.z`), not a caret;
#   2. no crate spells its own cbindgen version — all inherit the workspace one;
#   3. the resolved lock entry matches the requirement (a lock that drifted from
#      the pin builds fine locally and differently everywhere else).

set -euo pipefail
cd "$(dirname "$0")/.."

status=0

# 1. The workspace requirement is exact.
req="$(sed -n 's/^cbindgen = "\(=[0-9][^"]*\)".*/\1/p' Cargo.toml | head -1)"
if [ -z "$req" ]; then
    echo "[FAIL] Cargo.toml [workspace.dependencies] has no exact cbindgen requirement." >&2
    echo "       Expected a line like: cbindgen = \"=0.29.3\"" >&2
    echo "       A caret req lets a lockless leaf resolve a different patch release" >&2
    echo "       and rewrite the committed headers (issue 0452)." >&2
    status=1
fi
pin="${req#=}"

# 2. No crate carries its own cbindgen version.
offenders="$(git grep -n '^cbindgen = ' -- '*/Cargo.toml' | grep -v 'workspace = true' || true)"
if [ -n "$offenders" ]; then
    echo "[FAIL] these manifests spell their own cbindgen version instead of" >&2
    echo "       inheriting the workspace pin (\`cbindgen = { workspace = true }\`):" >&2
    echo "$offenders" | sed 's/^/         /' >&2
    status=1
fi

# 3. EVERY tracked lock that carries cbindgen resolves to exactly the pin.
#
# This arm originally read the root `Cargo.lock` alone, and that was too narrow
# by 16 files. Introducing the exact requirement made 14 leaf locks stale
# against it — they predate the pin — `check-leaf-lockfiles` went red and
# ci-matrix was blocked until they were moved by hand. The sweep that missed
# them looked for locks containing `nros-build-helpers` (3); the invariant is
# about locks containing CBINDGEN (17). A gate whose coverage is narrower than
# the rule it enforces is issue 0196's shape, and this is the fix for it here.
#
# Since phase-400 W2a only the root lock carries cbindgen (see the header note),
# so this arm reads one file. Keep it derived from a `git grep` rather than
# hardcoding "the root lock": if anything ever activates the
# `cbindgen-drift-check` feature again, the arm widens on its own.
locks_checked=0
if [ -n "$pin" ]; then
    off_pin=""
    while IFS= read -r lock; do
        [ -n "$lock" ] || continue
        locks_checked=$((locks_checked + 1))
        locked="$(awk '/^name = "cbindgen"$/{getline; sub(/^version = "/,""); sub(/"$/,""); print; exit}' "$lock")"
        if [ "$locked" != "$pin" ]; then
            off_pin="$off_pin\n         $lock (has $locked)"
        fi
    done <<EOF
$(git grep -l 'name = "cbindgen"' -- '*Cargo.lock')
EOF
    if [ -n "$off_pin" ]; then
        echo "[FAIL] these tracked locks disagree with the cbindgen pin ($pin):" >&2
        # shellcheck disable=SC2059
        printf "$off_pin\n" >&2
        echo "       Move each with \`just lock-update cbindgen $pin <dir>\` — the" >&2
        echo "       sanctioned path — and regenerate the committed headers in the" >&2
        echo "       same commit if the output moved." >&2
        status=1
    fi
fi

if [ "$status" -eq 0 ]; then
    if [ "$locks_checked" -eq 0 ]; then
        echo "[FAIL] no tracked lock carries cbindgen — this gate just passed vacuously." >&2
        exit 1
    fi
    echo "check-cbindgen-pin: OK (cbindgen pinned =$pin, inherited everywhere, $locks_checked tracked lock(s) agree)"
fi
exit "$status"
