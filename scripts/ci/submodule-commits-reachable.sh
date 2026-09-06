#!/usr/bin/env bash
#
# Every recorded submodule pin must be OBTAINABLE FROM ITS REMOTE.
#
# The sibling of `submodule-pins-check.sh`: that one asks whether a pin moved
# the right DIRECTION, this one asks whether the commit it names exists
# anywhere a clone can reach.
#
# WHY THIS EXISTS. AGENTS.md's first submodule rule is "push the submodule
# commit BEFORE the superproject pins it", and the reason is that nothing
# downstream can recover from getting it wrong: a pin naming an unpushed commit
# is a repository that only builds on the machine that made it. Every clone and
# every CI run dies at `git submodule update` with `reference is not a tree`,
# and the superproject commit has to be rewritten or reverted, because a pin
# cannot point at nothing.
#
# The rule was written down and nothing enforced it. `git push` is happy to
# publish a superproject commit whose gitlink names a commit that exists only
# in someone's working tree.
#
# WHAT "REACHABLE" MEANS HERE, precisely. Not "present in my clone" — that is
# exactly the state a bad pin produces on the machine that made it, so a local
# `cat-file -e` would pass on the one host where the bug is invisible. The
# question is whether a FRESH clone can get it, so it is asked of the REMOTE,
# with the same operation `git submodule update` performs: fetch the sha from
# the URL.
#
# Note this also passes for a commit on no branch but reachable from one, and
# fails for a commit on no ref at all — which is correct, because that is
# precisely what a clone can and cannot obtain.
#
# A FALSE ALARM THIS AVOIDS. A submodule clone's fetch refspec is often
# NARROWED (ours carry `+refs/heads/main:...` plus one branch), so a plain
# `git fetch` in the submodule does not see commits on other branches and a
# local lookup reports "missing" for a commit that is perfectly well published.
# That misreading happened while writing this, and cost a wrong claim that main
# pinned an unpushed commit. Asking the remote for the sha directly is immune:
# it does not depend on which refs the local clone chose to track.
#
# Usage:
#   scripts/ci/submodule-commits-reachable.sh [--changed <baseline-ref>]
#
#   With no argument every pin is checked. `--changed <ref>` checks only pins
#   that DIFFER from that ref, which is what the pre-push hook wants: the cost
#   is then proportional to what the push actually moves, and an unchanged pin
#   was already published or the repo would not build today.
#
# Exit: 0 all reachable (or nothing to check), 1 a pin is unreachable,
#       2 the network is unavailable and the check could not be performed.
set -uo pipefail

# This script builds throwaway repositories, so an inherited repository-local
# git environment (GIT_DIR and friends) silently redirects that work into the
# CALLER'S repository — `git init <tmp>` rewrites the caller's config,
# `git -C <tmp> update-index --add` stages into the caller's index. Observed as
# a pre-push hook that corrupted the repo it was guarding (issue 0986). Cleared
# here because every git invocation below names its target explicitly.
# shellcheck source=scripts/lib/git-hook-env.sh
. "$(cd "$(dirname "${BASH_SOURCE[0]}")/../lib" && pwd)/git-hook-env.sh"
nros_clear_inherited_git_env


# Prove the two verdicts on every run, against a LOCAL remote.
#
# `check-gate-selftests` requires this and is right to: the whole value here is
# a negative control, and one that only ever ran by hand during development
# decays into a comment. It is also the half most likely to rot silently — a
# refactor that made every pin "reachable" would look like a green gate.
#
# `file://` rather than the network, so the selftest costs milliseconds, works
# offline, and does not depend on any host being up. The code path exercised is
# the real one: same `git fetch --depth=1 <url> <sha>`.
selftest() {
    local tmp good bad out rc
    tmp="$(mktemp -d)" || return 1
    (
        set -e
        export GIT_AUTHOR_NAME=t GIT_AUTHOR_EMAIL=t@t
        export GIT_COMMITTER_NAME=t GIT_COMMITTER_EMAIL=t@t
        git init -q "$tmp/dep.git" --bare
        git init -q "$tmp/work"
        git -C "$tmp/work" commit -q --allow-empty -m one
        git -C "$tmp/work" push -q "$tmp/dep.git" HEAD:refs/heads/main
    ) >/dev/null 2>&1 || { rm -rf "$tmp"; return 1; }
    good="$(git -C "$tmp/work" rev-parse HEAD)"
    bad="0123456789abcdef0123456789abcdef01234567"

    _mk_super() {
        rm -rf "$tmp/super"
        (
            set -e
            export GIT_AUTHOR_NAME=t GIT_AUTHOR_EMAIL=t@t
            export GIT_COMMITTER_NAME=t GIT_COMMITTER_EMAIL=t@t
            git init -q "$tmp/super"
            printf '[submodule "dep"]\n\tpath = dep\n\turl = %s\n' "$tmp/dep.git" \
                > "$tmp/super/.gitmodules"
            git -C "$tmp/super" add .gitmodules
            git -C "$tmp/super" update-index --add --cacheinfo "160000,$1,dep"
            git -C "$tmp/super" commit -q -m pin
        ) >/dev/null 2>&1
    }

    local ok=0 fail=0
    _mk_super "$good"
    out="$(cd "$tmp/super" && NROS_SUBMODULE_REACH_SELFTEST=1 bash "$self" 2>&1)"; rc=$?
    if [ "$rc" -eq 0 ]; then ok=$((ok+1)); else
        fail=$((fail+1)); echo "  FAIL selftest: a published pin was rejected: $out" >&2
    fi

    _mk_super "$bad"
    out="$(cd "$tmp/super" && NROS_SUBMODULE_REACH_SELFTEST=1 bash "$self" 2>&1)"; rc=$?
    if [ "$rc" -eq 1 ]; then ok=$((ok+1)); else
        fail=$((fail+1))
        echo "  FAIL selftest: an UNREACHABLE pin was accepted (rc=$rc). This gate" >&2
        echo "       cannot fail, so it is reporting nothing: $out" >&2
    fi

    rm -rf "$tmp"
    if [ "$fail" -gt 0 ]; then
        echo "submodule-commits-reachable: SELFTEST FAILED ($fail of $((ok+fail)))" >&2
        return 1
    fi
    echo "submodule-commits-reachable --selftest: 2 case(s) OK (published accepted, unreachable rejected)"
    return 0
}

self="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
# The env var stops the selftest's own child invocations from recursing.
if [ -z "${NROS_SUBMODULE_REACH_SELFTEST:-}" ]; then
    selftest || exit 1
fi

repo_root="$(git rev-parse --show-toplevel)"
cd "$repo_root" || exit 1

changed_against=""
if [ "${1:-}" = "--changed" ]; then
    changed_against="${2:-}"
    if [ -z "$changed_against" ]; then
        echo "submodule-commits-reachable: --changed needs a ref" >&2
        exit 1
    fi
fi

# `git config -f .gitmodules` rather than `git submodule foreach`: this must
# work from a checkout where a submodule is UNINITIALISED, which is the normal
# state for most of them here (RFC-0060 keeps layer-3 uninitialised on purpose)
# and is also the state a reviewer's fresh clone is in.
mapfile -t paths < <(git config -f .gitmodules --get-regexp '^submodule\..*\.path$' | awk '{print $2}')
if [ "${#paths[@]}" -eq 0 ]; then
    echo "submodule-commits-reachable: no submodules declared"
    exit 0
fi

scratch="$(mktemp -d)"
trap 'rm -rf "$scratch"' EXIT
git init -q --bare "$scratch/probe" || exit 1

bad=0
checked=0
skipped_unchanged=0
network_failed=0

for path in "${paths[@]}"; do
    name="$(git config -f .gitmodules --get-regexp '^submodule\..*\.path$' \
        | awk -v p="$path" '$2 == p {print $1}' | sed 's/^submodule\.//;s/\.path$//')"
    url="$(git config -f .gitmodules --get "submodule.$name.url" 2>/dev/null)"
    [ -n "$url" ] || continue

    pin="$(git ls-tree HEAD "$path" 2>/dev/null | awk '{print $3}')"
    [ -n "$pin" ] || continue

    if [ -n "$changed_against" ]; then
        base="$(git ls-tree "$changed_against" "$path" 2>/dev/null | awk '{print $3}')"
        if [ "$base" = "$pin" ]; then
            skipped_unchanged=$((skipped_unchanged + 1))
            continue
        fi
    fi

    checked=$((checked + 1))
    # `--depth=1`: the question is existence, not history. Nothing is kept.
    if err="$(git -C "$scratch/probe" fetch --quiet --depth=1 --no-tags "$url" "$pin" 2>&1)"; then
        continue
    fi
    # Tell a network failure apart from a missing commit. Reporting "not
    # pushed" for an offline laptop would be a false and very specific
    # accusation, and the remedy it implies (push the submodule) is wrong.
    case "$err" in
        *"Could not resolve host"*|*"unable to access"*|*"Connection timed out"* \
        |*"Operation timed out"*|*"network is unreachable"*|*"Temporary failure"* \
        |*"shallow file has changed"*)
            # `shallow file has changed since we read it` belongs here and not
            # below: it is the PROBE repo failing locally, not the remote
            # answering. Reported as unreachable it becomes the exact false,
            # specific accusation the comment above exists to prevent — "push
            # the submodule commit first" for a commit that is already
            # published — and it blocks the push with no honest remedy.
            echo "submodule-commits-reachable: COULD NOT ASK while checking $path" >&2
            echo "  $err" >&2
            network_failed=1
            break
            ;;
    esac
    bad=$((bad + 1))
    echo "[FAIL] $path pins $pin, which $url cannot serve." >&2
    echo "       A clone cannot obtain it, so this commit builds only where it was made." >&2
    echo "       Push the submodule commit FIRST, then pin it (AGENTS.md, Submodules #1):" >&2
    echo "         git -C $path push <remote> <branch>" >&2
    echo "       git said: $err" >&2
done

if [ "$network_failed" -eq 1 ]; then
    echo "submodule-commits-reachable: SKIPPED — cannot ask the remotes." >&2
    exit 2
fi

if [ "$bad" -gt 0 ]; then
    echo "" >&2
    echo "submodule-commits-reachable: $bad unreachable pin(s)." >&2
    exit 1
fi

msg="submodule-commits-reachable: OK — $checked pin(s) served by their remotes"
if [ "$skipped_unchanged" -gt 0 ]; then
    msg="$msg ($skipped_unchanged unchanged, not re-checked)"
fi
echo "$msg"
exit 0
