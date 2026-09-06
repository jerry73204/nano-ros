#!/usr/bin/env bash
#
# A submodule pin may only move FORWARD.
#
# Every submodule here lives on `main` (or a dedicated branch) with linear
# history, so advancing a pin means "fast-forward to a descendant". Anything
# else — an older commit, or a commit on a different line — is either a mistake
# or a decision that needs saying out loud.
#
# WHY THIS EXISTS. On 2026-08-15 `f003d0cb1` bumped zenoh-pico to d3f0d268 with
# a message naming the fix ("Zephyr declares `socklen_t` as..."). Ninety minutes
# later `e56354410` — a 24-file commit about renumbering ISSUE IDS, whose
# message never mentions the submodule — moved the same pin back to 43ddb0ec.
# The Zephyr build fix was silently unshipped for seven hours, and nothing
# noticed until a rebase conflict surfaced it. That is the `git add -A` hazard
# CLAUDE.md already warns about, one layer down: the pointer is a FILE, a
# blanket add scoops it up, and a pointer diff looks like noise in a large
# commit.
#
# A backward move cannot be caught by reading a diff — `-Subproject commit
# d3f0d26 / +Subproject commit 43ddb0e` is two hex strings, and which one is
# newer is not visible without asking the submodule. So ask it.
#
# Usage:
#   scripts/ci/submodule-pins-check.sh [<baseline-ref> [<local-ref>]]
#     baseline defaults to origin/main, local to HEAD.
#   The pre-push hook passes the REMOTE's actual sha as the baseline, which is
#   more precise than origin/main (that ref can be stale).
#
# Cost is proportional to pins that MOVED: an unchanged pin needs no submodule
# and no network.
#
# "CANNOT EVALUATE" IS NOT A VERDICT (issue 1043)
#
# Reading a pin's history needs that submodule's OBJECT STORE, and no lane
# checks out all twenty. `gate.yml`'s `check` job initialises exactly
# `packages/cli/third-party/play_launch` plus the `-sys` sources `nros setup`
# provisions. So a bump to any OTHER submodule used to arrive here with nothing
# to read and was reported as `CANNOT VERIFY ... fail`, with a remedy
# (`git submodule update --init <path>`) addressed to a developer who has a
# checkout. In CI nobody could act on it: the failure was UNCONDITIONAL, so the
# nuttx pin bump of issue 1034 passed `just check fast` locally and could not
# pass the required lane at all, however many times it was rebased or re-pushed.
#
# A gate that fails when it cannot evaluate is indistinguishable from a gate
# that found a defect, which is the same shape as a gate that reports OK when it
# compared nothing — the failure this script's baseline arm already fixes, in
# the other direction. So the three outcomes are now distinct:
#
#   FAIL        the ancestry was MEASURED and the move is not a fast-forward.
#   NOT VERIFIED  the objects to measure with are absent here. Reported per
#               path AND in the verdict line, so `OK` never overstates what was
#               checked. Never silent.
#   OK          measured, fast-forward.
#
# Escalate a skip to a failure with NROS_SUBMODULE_PINS_STRICT=1 — for a lane
# that CLAIMS to provide every submodule, where "not checked out" really is a
# defect in the lane. Do not set it on a lane that checks out a subset: that
# reinstates the unconditional failure this note exists about.
#
# Bypass for a deliberate rollback: NROS_ALLOW_SUBMODULE_REWIND=1, and say why
# in the commit message.

set -uo pipefail

baseline="${1:-${NROS_SUBMODULE_PIN_BASELINE:-origin/main}}"
exceptions_file="${NROS_SUBMODULE_PIN_EXCEPTIONS:-docs/reference/submodule-pin-exceptions.txt}"
allowed=0
local_ref="${2:-HEAD}"

# --- selftest, on the NORMAL path (phase-395) -------------------------------
#
# This gate spent its life able to report OK without comparing anything, and a
# negative control nobody runs would not have caught that. So it runs here, and
# it exercises the exact decision that was wrong: what happens when the baseline
# cannot be resolved. It re-invokes this same script, so the selftest cannot
# drift from the shipped logic.
#
# NROS_SUBMODULE_PINS_REENTRY is a RECURSION guard, not an opt-in flag — the
# inner runs must not selftest themselves. Named for what it does, because an
# earlier spelling with SELFTEST in the name read to both check-gate-selftests
# and to a human as "this only runs when asked", which is the thing being
# forbidden.
nros_submodule_pins_selftest() {
    [ -n "${NROS_SUBMODULE_PINS_REENTRY:-}" ] && return 0
    local missing="refs/heads/__nros_selftest_absent__" rc

    NROS_SUBMODULE_PINS_REENTRY=1 GITHUB_ACTIONS=true \
        bash "$0" "$missing" >/dev/null 2>&1
    rc=$?
    if [ "$rc" -ne 1 ]; then
        echo "submodule-pins SELFTEST FAILED (got rc=$rc, want 1): an unresolvable" >&2
        echo "  baseline must be FATAL in CI. That it was not is how this gate" >&2
        echo "  passed a real pin rewind at PR stage while comparing nothing." >&2
        exit 1
    fi

    NROS_SUBMODULE_PINS_REENTRY=1 GITHUB_ACTIONS= CI= \
        bash "$0" "$missing" >/dev/null 2>&1
    rc=$?
    if [ "$rc" -ne 0 ]; then
        echo "submodule-pins SELFTEST FAILED (got rc=$rc, want 0): a fresh LOCAL" >&2
        echo "  clone with no origin/main must still be able to run the fast tier." >&2
        exit 1
    fi

    nros_submodule_pins_mutation_selftest
}

# The two directions of issue 1043, on a CONSTRUCTED repo.
#
# Making "cannot evaluate" a skip is a coverage LOSS if it also swallows a real
# rewind, and no gate in this repo has ever been able to demonstrate its own red
# on the pin-ancestry path — the 2026-08-15 zenoh-pico rewind was found by a
# human, and issue 1043 was found by a pin bump. So both mutations run here, on
# the normal path, for the reason `check-gate-selftests` exists: a negative
# control nobody runs decays into a comment.
#
# The fixture is a throwaway superproject whose gitlink is written with
# `update-index --cacheinfo` — no clone, no network, no `git submodule add`.
# Measured cost: ~120 ms for both mutations.
nros_submodule_pins_mutation_selftest() {
    local tmp super sub self out rc
    command -v mktemp >/dev/null 2>&1 || return 0
    # The mutations run with cwd INSIDE the fixture, so `$0` (relative, as
    # `just` and the pre-push hook spell it) would not resolve there. Absolute
    # it once. Getting this wrong is not silent — the selftest reported
    # `rc=127 ... No such file or directory` rather than passing.
    self="$(cd "$(dirname "$0")" && pwd)/$(basename "$0")"
    tmp="$(mktemp -d 2>/dev/null)" || return 0
    super="$tmp/super"
    sub="$super/sub"

    (
        set -e
        export GIT_CONFIG_GLOBAL=/dev/null GIT_CONFIG_SYSTEM=/dev/null
        export GIT_AUTHOR_NAME=nros GIT_AUTHOR_EMAIL=nros@invalid
        export GIT_COMMITTER_NAME=nros GIT_COMMITTER_EMAIL=nros@invalid
        git init -q "$super"
        git init -q "$sub"
        git -C "$sub" commit -q --allow-empty -m "sub: older"
        git -C "$sub" rev-parse HEAD > "$tmp/old"
        git -C "$sub" commit -q --allow-empty -m "sub: newer"
        git -C "$sub" rev-parse HEAD > "$tmp/new"
        # Baseline commit pins the NEWER sha, head commit pins the OLDER one:
        # a rewind, the exact move this gate exists to refuse.
        git -C "$super" update-index --add --cacheinfo "160000,$(cat "$tmp/new"),sub"
        git -C "$super" commit -q -m "pin newer"
        git -C "$super" update-index --add --cacheinfo "160000,$(cat "$tmp/old"),sub"
        git -C "$super" commit -q -m "pin older (rewind)"
    ) >"$tmp/build.log" 2>&1 || {
        # LOUD, not `return 0`. A selftest that skips itself when its fixture
        # fails to build is the silent-skip shape this whole gate is about; it
        # would report OK for a mutation it never ran.
        echo "submodule-pins SELFTEST FAILED: could not build the fixture repo" >&2
        sed 's/^/    /' "$tmp/build.log" >&2
        rm -rf "$tmp"
        exit 1
    }

    # Mutation 1 — the submodule IS readable and the pin moved BACKWARD.
    # Must still FAIL, and say REWIND.
    out="$(cd "$super" && NROS_SUBMODULE_PINS_REENTRY=1 NROS_ALLOW_SUBMODULE_REWIND=0 \
           NROS_SUBMODULE_PINS_STRICT= NROS_SUBMODULE_PIN_EXCEPTIONS=/dev/null \
           bash "$self" HEAD~1 HEAD 2>&1)"
    rc=$?
    case "$rc:$out" in
        1:*REWIND*) ;;
        *)
            echo "submodule-pins SELFTEST FAILED: a readable REWIND must fail" >&2
            echo "  (got rc=$rc, want 1 with 'REWIND'). Making 'cannot evaluate'" >&2
            echo "  a skip must not also swallow a measured backward move." >&2
            echo "$out" | sed 's/^/    /' >&2
            rm -rf "$tmp"
            exit 1
            ;;
    esac

    # Mutation 2 — the SAME rewind with the submodule not checked out. Must
    # SKIP (rc=0) and SAY it skipped: this is issue 1043's case, and a silent
    # pass here would be the defect one level down.
    mv "$sub/.git" "$tmp/sub-git-away"
    out="$(cd "$super" && NROS_SUBMODULE_PINS_REENTRY=1 NROS_SUBMODULE_PINS_STRICT= \
           NROS_SUBMODULE_PIN_EXCEPTIONS=/dev/null \
           bash "$self" HEAD~1 HEAD 2>&1)"
    rc=$?
    case "$rc:$out" in
        0:*"NOT VERIFIED"*) ;;
        *)
            echo "submodule-pins SELFTEST FAILED: an ABSENT submodule must skip" >&2
            echo "  and report it (got rc=$rc, want 0 with 'NOT VERIFIED'). A gate" >&2
            echo "  that fails when it cannot evaluate is issue 1043; one that" >&2
            echo "  passes silently is worse." >&2
            echo "$out" | sed 's/^/    /' >&2
            rm -rf "$tmp"
            exit 1
            ;;
    esac

    rm -rf "$tmp"
}
nros_submodule_pins_selftest

repo_root="$(git rev-parse --show-toplevel)"
cd "$repo_root" || exit 2

if ! git rev-parse --verify --quiet "$baseline^{commit}" >/dev/null; then
    # A gate that cannot check must SAY SO, not report OK.
    #
    # `actions/checkout` is shallow by default, so on a pull request
    # `origin/main` usually does not exist — and this branch used to `exit 0`.
    # The gate therefore ran on every PR and compared NOTHING, for as long as it
    # had existed. Measured: a real play_launch pin rewind reached a pushed
    # branch and was caught only by a LOCAL run.
    #
    # gate.yml already documents this exact trap for a sibling check:
    # "the base ref is usually absent and the diff would fail — which the
    # fail-safe would turn into code=true forever, i.e. safe but never actually
    # firing". Same trap, different gate.
    #
    # Locally the skip is still right: a fresh clone genuinely has nothing to
    # compare against, and refusing to run the fast tier there helps nobody.
    if [ -n "${GITHUB_ACTIONS:-}${CI:-}" ]; then
        echo "submodule-pins: FAILED — baseline '$baseline' does not resolve, in CI." >&2
        echo "" >&2
        echo "  This gate is NETWORK-FREE by the check-fast contract, so it cannot" >&2
        echo "  fetch the base itself. The workflow must provide it before" >&2
        echo "  \`just check fast\`, e.g.:" >&2
        echo "" >&2
        echo "    git fetch --no-tags --depth=1 origin \\" >&2
        echo "      +refs/heads/\${GITHUB_BASE_REF:-main}:refs/remotes/origin/\${GITHUB_BASE_REF:-main}" >&2
        echo "" >&2
        echo "  or pass one via \$NROS_SUBMODULE_PIN_BASELINE. Failing instead of" >&2
        echo "  skipping is deliberate: a silent skip here is indistinguishable" >&2
        echo "  from a green check." >&2
        exit 1
    fi
    echo "submodule-pins: NOT CHECKED — baseline '$baseline' does not resolve." >&2
    echo "  (a fresh LOCAL clone with no origin/main yet; nothing to compare" >&2
    echo "   against. In CI this is a FAILURE, not a skip.)" >&2
    exit 0
fi

# `<mode> <type> <sha>\t<path>` for every gitlink, at one commit.
pins_at() {
    git ls-tree -r "$1" 2>/dev/null | awk '$2 == "commit" { print $4 "\t" $3 }'
}

baseline_pins="$(pins_at "$baseline")"
local_pins="$(pins_at "$local_ref")"

# The object store to ask about a submodule's history, or nothing.
#
# `$path/.git` is the usual answer and NOT the only one: `git submodule deinit`
# — and an `--init` that half-finished — removes the WORKTREE and leaves the
# objects at `.git/modules/<name>`. Asking only about the worktree calls a
# perfectly readable history absent, which turns a verdict this gate COULD give
# into a skip. So resolve the store, not the checkout.
#
# `--git-common-dir`, not `--git-dir`: in a linked WORKTREE the latter is
# `.git/worktrees/<wt>`, which has no `modules/`. Agents here work in worktrees,
# so the wrong spelling would be wrong exactly where it is used most.
submodule_git_dir() {
    local path="$1" name common modules
    if [ -e "$path/.git" ]; then
        git -C "$path" rev-parse --absolute-git-dir 2>/dev/null && return 0
    fi
    name="$(git config -f .gitmodules --get-regexp '^submodule\..*\.path$' 2>/dev/null \
            | awk -v p="$path" '$2 == p { print $1 }' \
            | sed -e 's/^submodule\.//' -e 's/\.path$//' | head -1)"
    [ -n "$name" ] || name="$path"
    common="$(git rev-parse --path-format=absolute --git-common-dir 2>/dev/null)"
    modules="$common/modules/$name"
    if [ -d "$modules" ]; then
        printf '%s\n' "$modules"
        return 0
    fi
    return 1
}

# A narrowing has to reach the LANE's summary, not just this gate's stdout.
#
# `run-gates-parallel.sh` discards the output of every gate that exits 0 and
# prints only failures, so a `NOT VERIFIED` line on the happy path is invisible
# in the lane that runs on every push — which would make this fix a silent skip,
# the exact defect issue 0650 removed from the other six gates. The ledger is
# the SHARED spelling for "this green is narrower than it looks" (check-skip.sh,
# read by both the serial and the parallel closing lines), so use it rather than
# inventing a second one.
#
# Not during the selftest: those runs are re-entrant and their cwd is a
# throwaway fixture, so their narrowing is not this checkout's.
record_narrowing() {
    local reason="$1" lib="$repo_root/scripts/build/check-skip.sh"
    [ -z "${NROS_SUBMODULE_PINS_REENTRY:-}" ] || return 0
    [ -f "$lib" ] || return 0
    # shellcheck source=scripts/build/check-skip.sh
    . "$lib" 2>/dev/null || return 0
    command -v nros_check_skip >/dev/null 2>&1 || return 0
    nros_check_skip submodule-pins "$reason" || true
}

# A pin move this checkout has no way to evaluate. NOT a verdict — see the
# header. Loud per path, counted, and repeated in the summary line.
unverifiable() {
    local path="$1" old="$2" new="$3"
    unverified=$((unverified + 1))
    unverified_paths="${unverified_paths}${unverified_paths:+ }${path}"
    echo "submodule-pins: NOT VERIFIED $path" >&2
    echo "    the pin moved ${old:0:12} -> ${new:0:12}; this checkout cannot read" >&2
    echo "    that submodule's history, so the move was NOT evaluated either way." >&2
}

fail=0
moved=0
verified=0
unverified=0
unverified_paths=""

while IFS=$'\t' read -r path new_sha; do
    [ -n "${path:-}" ] || continue
    old_sha="$(printf '%s\n' "$baseline_pins" | awk -F'\t' -v p="$path" '$1 == p { print $2 }')"

    # New submodule, or unchanged: nothing to prove.
    [ -z "$old_sha" ] && continue
    [ "$old_sha" = "$new_sha" ] && continue

    moved=$((moved + 1))

    # A rebased fork patch line is deliberately not a descendant of the old pin,
    # and the only bypass used to be an environment variable NOTHING IN CI CAN
    # SET -- so a correct pin passed locally and could never pass the merge
    # queue. An allowlist row names the exact path and both shas, so it expires
    # as soon as either moves; the variable stays for the interactive case.
    #
    # This runs BEFORE the object-availability checks below, deliberately. After
    # a force-push the OLD commit is no longer reachable from the branch, so a
    # CI clone cannot fetch it however deep it digs, and the comparison that
    # needs it can never run -- `CANNOT VERIFY`, not `DIVERGED`. A row is a
    # human asserting the relationship, which is exactly the evidence that the
    # unreachable object would have provided, so requiring both is requiring the
    # impossible.
    if [ -f "$exceptions_file" ] \
       && grep -qE "^[[:space:]]*${path}[[:space:]]+${old_sha}[[:space:]]+${new_sha}[[:space:]]" \
                "$exceptions_file"; then
        reason="$(grep -E "^[[:space:]]*${path}[[:space:]]+${old_sha}[[:space:]]+${new_sha}[[:space:]]" \
                       "$exceptions_file" | head -1 | cut -d' ' -f4-)"
        echo "submodule-pins: $path — non-fast-forward ALLOWED by" >&2
        echo "  ${exceptions_file}: ${reason}" >&2
        allowed=$((allowed + 1))
        continue
    fi

    # NOT initialised here. This is the issue-1043 case and it is NOT a verdict
    # about the pin: no lane checks out every submodule, so the old `fail=1`
    # was unconditional for anything outside the small set gate.yml initialises.
    if ! store="$(submodule_git_dir "$path")"; then
        unverifiable "$path" "$old_sha" "$new_sha"
        if [ -n "${GITHUB_ACTIONS:-}${CI:-}" ]; then
            echo "    THIS LANE does not check out $path. gate.yml's" >&2
            echo "    'Init submodules whose pin moved (commits only)' step exists to" >&2
            echo "    provide it; if you are reading this in CI, that step did not run" >&2
            echo "    or could not fetch. Nothing you can push fixes it from the" >&2
            echo "    branch — the LANE has to hand the objects over." >&2
        else
            echo "    Run \`git submodule update --init $path\` to turn this skip into" >&2
            echo "    a verdict; the pin is unchecked until you do." >&2
        fi
        continue
    fi

    # Both commits must be present locally to compare them. A pin that moved
    # forward normally has them; fetch once if not (the sha may live only on the
    # remote when someone else advanced it).
    for sha in "$old_sha" "$new_sha"; do
        if ! git --git-dir="$store" cat-file -e "${sha}^{commit}" 2>/dev/null; then
            git --git-dir="$store" fetch --quiet --all 2>/dev/null || true
            break
        fi
    done

    for sha in "$old_sha" "$new_sha"; do
        if ! git --git-dir="$store" cat-file -e "${sha}^{commit}" 2>/dev/null; then
            # Same class as the arm above: the objects are absent, so the
            # ancestry was not measured. Both causes below are OTHER gates' or
            # the LANE's to answer, and neither is evidence about this move.
            unverifiable "$path" "$old_sha" "$new_sha"
            echo "    commit ${sha:0:12} is not in the submodule's object store, even" >&2
            echo "    after a fetch. Two causes, neither established here:" >&2
            echo "" >&2
            echo "    1. The commit was never pushed. A pin nobody can resolve" >&2
            echo "       clones as a broken tree; push it FIRST, then bump the" >&2
            echo "       pointer. \`check submodule-commits-reachable\` is the gate" >&2
            echo "       that asks the remote, and it FAILS on this — which is why" >&2
            echo "       this arm does not have to." >&2
            echo "" >&2
            echo "    2. The submodule is a SHALLOW clone, so the baseline commit" >&2
            echo "       (the one before the tip) is simply absent. This is the" >&2
            echo "       usual case in CI, and it is not your pin's fault. The" >&2
            echo "       fetch above cannot help: \`git fetch --all\` does not" >&2
            echo "       deepen a shallow clone. check-fast is network-free by" >&2
            echo "       contract, so the WORKFLOW must provide the objects --" >&2
            echo "       see the 'Fetch submodule history' step in gate.yml." >&2
            continue 2
        fi
    done

    if git --git-dir="$store" merge-base --is-ancestor "$old_sha" "$new_sha" 2>/dev/null; then
        verified=$((verified + 1))
        continue  # fast-forward: the sanctioned move
    fi


    # Not an ancestor. Say WHICH kind of wrong it is — a rewind and a fork need
    # different fixes, and the diff looks identical for both.
    if git --git-dir="$store" merge-base --is-ancestor "$new_sha" "$old_sha" 2>/dev/null; then
        kind="REWIND — the new pin is an ANCESTOR of the old one"
        remedy="If you meant to keep the newer commit, restore it:
        git -C $path checkout $old_sha && git add $path"
    else
        kind="DIVERGED — neither pin contains the other"
        remedy="Rebase the submodule work onto its branch so the move is a
        fast-forward, then re-add the pointer. Merges are not used here."
    fi

    subject="$(git --git-dir="$store" log -1 --format='%s' "$old_sha" 2>/dev/null)"
    echo "submodule-pins: $path" >&2
    echo "    $kind" >&2
    echo "      was: ${old_sha:0:12}  $subject" >&2
    echo "      now: ${new_sha:0:12}  $(git --git-dir="$store" log -1 --format='%s' "$new_sha" 2>/dev/null)" >&2
    echo "    $remedy" >&2
    fail=1
done <<< "$local_pins"

# A lane that CLAIMS to provide every submodule can ask for the old behaviour —
# there, "not checked out" really is the lane's defect. Opt-in, because the
# default must not be the unconditional failure of issue 1043.
if [ "$unverified" -ne 0 ] && [ "${NROS_SUBMODULE_PINS_STRICT:-0}" = "1" ]; then
    echo "" >&2
    echo "submodule-pins: FAILED — NROS_SUBMODULE_PINS_STRICT=1 and $unverified moved" >&2
    echo "  pin(s) could not be evaluated here: $unverified_paths" >&2
    echo "  Initialise them, or drop the strict flag on a lane that checks out a" >&2
    echo "  SUBSET of the submodules." >&2
    exit 1
fi

if [ "$fail" -ne 0 ]; then
    if [ "${NROS_ALLOW_SUBMODULE_REWIND:-0}" = "1" ]; then
        echo "" >&2
        echo "submodule-pins: OVERRIDDEN by NROS_ALLOW_SUBMODULE_REWIND=1 — say why in" >&2
        echo "  the commit message, or the next reader will assume it was an accident." >&2
        exit 0
    fi
    echo "" >&2
    echo "  A pin moving backward silently unships whatever the skipped commits fixed." >&2
    echo "  Deliberate rollback: NROS_ALLOW_SUBMODULE_REWIND=1 (and say why)." >&2
    exit 1
fi

# The verdict line states the NARROWING, not just the outcome. `OK (2 pins
# moved)` after skipping one of them claims a check that did not happen — the
# same overstatement as the silent baseline skip this gate already fixed, so it
# is spelled out here rather than left to whoever reads the log.
summary="$verified verified fast-forward"
[ "$allowed" -ne 0 ] && summary="$summary; $allowed allowed by $exceptions_file"
if [ "$unverified" -ne 0 ]; then
    echo "" >&2
    echo "submodule-pins: NARROWED — $unverified of $moved moved pin(s) were NOT" >&2
    echo "  evaluated in this checkout: $unverified_paths" >&2
    echo "  This verdict is about the rest of them. Set NROS_SUBMODULE_PINS_STRICT=1" >&2
    echo "  on a lane that provides every submodule to make this a failure." >&2
    summary="$summary; $unverified NOT VERIFIED ($unverified_paths)"
    record_narrowing "PARTIAL — $unverified of $moved moved pin(s) NOT VERIFIED (not checked out here): $unverified_paths"
fi
echo "submodule-pins: OK ($moved pin(s) moved — $summary)"
exit 0
