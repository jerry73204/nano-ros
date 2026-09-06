#!/usr/bin/env bash
# Run the `check-fast` gates concurrently instead of as serial just dependencies.
#
# Issue 0726. Measured on a 32-core host: `just check fast` is 90 s warm and
# several hundred cold, the whole time at 1-2 runnable cores — roughly 5% of the
# machine, sitting in front of every fixture build. The 112 gates sum to 56 s
# individually with the slowest at 8.3 s and a mean of 501 ms, so no outlier
# owns the time and there is nothing to optimise gate-by-gate. Fanning out
# bounds the phase at about the slowest gate.
#
# Two properties of the serial version are deliberately NOT preserved:
#
#   * Fail-fast. Serial `check-fast` stops at the first red, so a run tells you
#     about one gate and hides the rest. This reports EVERY failure, the same
#     choice `check-tier-preconditions` makes and for the same reason: one
#     failure per attempt is the shape that makes people stop running it.
#   * Output interleaving. Each gate's output is captured and printed as a
#     block when it finishes, so a red is readable rather than shredded across
#     31 other gates.
#
# NOT YET SAFE AS THE DEFAULT. On the first full run,
# `check-rmw-force-link-anchor` failed here and passed standalone, claiming a
# zephyr example declares `rmw-xrce` without a `force_link_backend!` anchor;
# two immediate re-runs were green. An INTERMITTENT failure means at least one
# gate pair is not independent — something transiently rewrites what that gate
# reads (a generated tree, or a leaf config a sync-ish gate touches). A flaky
# gate is worse than a slow one, so `check-fast` still runs serially and this
# is opt-in until the conflicting pair is identified. Issue 0726.
#
# Ordering that DOES matter is preserved: `_check-skip-reset` truncates the
# shared skip log and must complete before any gate appends to it. The appends
# themselves are safe — `nros_check_skip` does one short `printf >>`, and a
# single write under O_APPEND below PIPE_BUF is atomic, so concurrent gates
# cannot interleave a line.
set -uo pipefail
cd "$(dirname "$0")/../.."

jobs="${NROS_GATE_JOBS:-$(nproc)}"

# One CPU budget for the whole fan-out — the thing this runner did not have.
#
# Each gate is an independent `just` recipe, and the ones that run tests start
# their own `cargo nextest`, which reads `test-threads` from
# `.config/nextest.toml` and knows nothing about its siblings. So the demand
# this script creates is `jobs` x that number, and the number is 25: on a
# 12-core host, up to 300 test threads for 12 cores. The header above notes this
# runner was measured on a 32-core machine, where the ratio hurt less and the
# problem stayed invisible.
#
# What it costs is not throughput. It is that a test asserting a wall-clock RATE
# stops measuring the code: `periodic_timer_fires_repeatedly` (5 ms period,
# `count >= 4`) reported `got 2` twice in three runs at 2x oversubscription and
# passed 3 of 3 idle, and `check-required-features-tests` went red on it inside
# `check-build`. Those assertions are now deadline-based and no longer care, but
# a budget nobody sets is a budget every future timing test rediscovers.
#
# `nros_nextest_cpu_budget_args` (scripts/build/cargo.sh) turns this into
# `--test-threads`, clamped by the config's own value so it can only ever LOWER
# it — that number is the Cyclone DOMAIN partition's slot count (issue 0838), a
# correctness bound, not a performance one. Exported HERE and nowhere else, so
# `just test-unit` and a bare `cargo nextest` are untouched: they own the
# machine and the config's value is right for them.
#
# Build jobs are deliberately NOT throttled. Concurrent cargos sharing a
# target-dir already serialize on cargo's build lock, and a compile is not
# wall-clock-sensitive, so capping `-j` would trade a real slowdown for nothing:
# `check-build` is ~587 s serial and the fan-out is what makes it affordable.
_cpus="$(nproc 2>/dev/null || echo 4)"
_share=$(( _cpus / jobs ))
[ "$_share" -lt 1 ] && _share=1
export NROS_GATE_CPU_SHARE="${NROS_GATE_CPU_SHARE:-$_share}"

# The gates share one `.git/index`, and most reach it. Read-only git commands
# still refresh the index opportunistically, which takes `.git/index.lock`.
# GIT_OPTIONAL_LOCKS=0 tells git to skip anything that would take a lock, which
# is what a read-only gate wants regardless.
#
# This is hygiene, NOT the fix for the known flake below — I proposed the
# partial-`git ls-files` mechanism, then disproved it: 200 probes of the exact
# pathspec under fan-out load never came back short, and the flake survived
# this setting. Kept because it is correct on its own terms; do not read it as
# the resolution.
export GIT_OPTIONAL_LOCKS=0

# `--serial` runs the same set one gate at a time, fail-fast, output
# unshredded — what `just check fast-serial` is for. A bare positional is a
# pre-built list file, which is how a caller pins the set it wants.
serial=0
list=""
for arg in "$@"; do
    case "$arg" in
        --serial) serial=1 ;;
        -*) echo "$0: unknown flag: $arg" >&2; exit 2 ;;
        *) list="$arg" ;;
    esac
done

# `NROS_GATE_LANE` picks WHICH lane to run — issue 0993. `build` gained a
# parallel runner when it moved onto the pull-request path: serial it is
# ~587 s, and its slowest single gate is ~148 s, so the fan-out is the
# difference between a tolerable PR and an intolerable one. Defaulting to
# `fast-serial` keeps every existing caller unchanged.
lane="${NROS_GATE_LANE:-fast-serial}"

# The gate list is DERIVED, never kept beside the recipes: a second copy
# silently drifts the moment someone adds a gate, and this runner would then
# report OK over a set that is missing it.
#
# It used to be awk over `fast-serial:`'s dependency line. That line is GONE
# (issue 1072) — it was a 218-name list every gate-adding pull request appended
# to, and two authors adding alphabetically adjacent names insert at the same
# base line and conflict with certainty. No merge driver can fix that, because
# GitHub rebases queue entries server-side (issue 0884), so the list had to
# stop existing. A recipe in `just/check.just` is now a fast gate unless it is
# in `build-serial:`, exempt in `.config/gate-lane-exempt.txt`, or takes
# parameters.
#
# `check-gate-lists.py --list` is the ONE place that derivation lives, shared
# with the gate that verifies it, and it exits non-zero rather than emitting a
# short list — which the emptiness check below backstops.
if [ -z "$list" ]; then
    list="$(mktemp "${TMPDIR:-/tmp}/nros-gate-list.XXXXXX")"
    if ! python3 scripts/check/check-gate-lists.py --list "$lane" > "$list"; then
        echo "$0: could not derive the \`$lane\` gate list — refusing to run" >&2
        exit 2
    fi
fi
[ -s "$list" ] || {
    echo "$0: derived an EMPTY gate list — refusing to report OK over nothing" >&2
    exit 2
}

# The lane this run is reporting on. It used to be the literal string
# "check-fast", which became a lie the moment `build` got a parallel runner
# too (issue 0993) — a summary naming the wrong lane is how a green build
# line gets read as a green fast line.
label="${lane%-serial}"
out_dir="$(mktemp -d "${TMPDIR:-/tmp}/nros-gates.XXXXXX")"
trap 'rm -rf "$out_dir"' EXIT

# The skip log is shared, so reset it once, before the fan-out.
just _check-skip-reset >/dev/null 2>&1 || true

# The skip ledger's reader. Sourced here rather than at the bottom because
# BOTH paths close on it now. Derive the path, do NOT spell it (RFC-0070):
# `nros_check_skip` writes through `nros_build_dir`, which honours
# `NROS_BUILD_ROOT` and resolves against the REPO rather than `$PWD` — so a
# literal `$(pwd)/build/...` reads a different file in a git worktree or with
# the cache root moved, and every skip is invisible exactly where it was
# recorded.
# shellcheck source=scripts/build/check-skip.sh
. "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/check-skip.sh"

# `just check fast-serial`'s path: in list order, one at a time, output
# streaming, STOP at the first red. That fail-fast ordering is the whole reason
# the serial spelling exists — the parallel path deliberately reports every
# failure instead, which is the right default and the wrong thing when you are
# bisecting one.
#
# It runs through this script rather than through `just` dependencies so that
# the two spellings answer from ONE derived list. Under dependencies the serial
# lane had its own copy of the gate names, which is exactly the drift the
# derivation exists to remove.
if [ "$serial" = 1 ]; then
    ran=0
    while IFS= read -r gate; do
        case "$gate" in ''|'#'*) continue ;; esac
        ran=$((ran + 1))
        if ! just check "$gate"; then
            printf '\ncheck-%s (serial): FAILED at `%s` (%d gate(s) in)\n' \
                "$label" "$gate" "$ran" >&2
            exit 1
        fi
    done <"$list"
    nros_check_skip_report "check-$label (serial): $ran gate(s) OK"
    exit 0
fi

run_one() {
    local gate="$1" dir="$2" start end rc
    start=$(date +%s%N)
    just check "$gate" >"$dir/$gate.out" 2>&1
    rc=$?
    end=$(date +%s%N)
    printf '%s\t%s\t%s\n' "$gate" "$rc" "$(( (end - start) / 1000000 ))" \
        >>"$dir/results.tsv"
    return 0
}
export -f run_one

grep -vE '^\s*(#|$)' "$list" \
    | xargs -P "$jobs" -I{} bash -c 'run_one "$@"' _ {} "$out_dir"

failed=0
while IFS=$'\t' read -r gate rc ms; do
    if [ "$rc" != "0" ]; then
        failed=$((failed + 1))
        printf '\n===== FAIL (%s, rc=%s, %sms) =====\n' "$gate" "$rc" "$ms"
        cat "$out_dir/$gate.out"
    fi
done <"$out_dir/results.tsv"

total=$(wc -l <"$out_dir/results.tsv")
slowest=$(sort -k3 -rn "$out_dir/results.tsv" | head -1)
if [ "$failed" -gt 0 ]; then
    printf '\ncheck-%s (parallel): %d of %d gate(s) FAILED\n' "$label" "$failed" "$total"
    exit 1
fi
# QUALIFY the success line by the gates that did not run.
#
# The serial path closes with `nros_check_skip_report`, which refuses to say
# "All checks passed!" over a skipped gate — that is issue 0650's whole point.
# The parallel path RESET the shared ledger (above) and then never read it, so
# `check-fast` printed "N gate(s) OK" while gates skipped, in the lane that runs
# on EVERY PUSH.
#
# Measured when this was written: four skips reported as an unqualified OK —
# three from one stale CLI, and `check-abi-bindings`, which had never run on
# that host at all because `bindgen-cli` was not installed. A person reading
# "138 gate(s) OK" had no way to know the ABI bindings were unchecked.
#
# Still exit 0: these are missing tools and build products, not failures, and
# `check-fast` must stay green on a bare worktree. What changes is only that
# the sentence stops overstating what happened.
#
# `check-skip.sh` is sourced ABOVE, before the serial path, which closes on the
# same ledger. Found while adding `action-client-arena-budget`, which skipped
# whenever nothing was built and therefore skipped in most CI runs — that gate
# has since left this lane and fails closed instead (phase-413 W7, issue 1001),
# but the reporting it exposed is what the remaining skippers need.
skips="$(nros_build_dir "$NROS_KIND_CHECK_SKIPS")/checks.skipped"
skipped=0
if [ -s "$skips" ]; then
    skipped=$(wc -l <"$skips")
fi

if [ "$skipped" -gt 0 ]; then
    printf 'check-%s (parallel): %d gate(s) ran at -P%s (%s test-thread(s)/gate), %d SKIPPED; slowest %s\n' "$label" \
        "$((total - skipped))" "$jobs" "$NROS_GATE_CPU_SHARE" "$skipped" \
        "$(printf '%s' "$slowest" | awk '{print $1" "$3"ms"}')"
    while IFS=$'\t' read -r gate reason; do
        printf '  [SKIPPED] %s: %s\n' "$gate" "$reason"
    done <"$skips"
    exit 0
fi
printf 'check-%s (parallel): %d gate(s) OK at -P%s (%s test-thread(s)/gate); slowest %s\n' "$label" \
    "$total" "$jobs" "$NROS_GATE_CPU_SHARE" "$(printf '%s' "$slowest" | awk '{print $1" "$3"ms"}')"
