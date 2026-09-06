#!/usr/bin/env bash
#
# Claim a unit of work — `refs/claims/<id>` — ATOMICALLY, across every parallel
# agent, on every machine. phase-395 W8.
#
# WHY A REF AND NOT A TASK LIST
#
# A shared markdown "who is doing what" file has the lost-update race that
# `scripts/reserve-issue-id.sh` was written to fix: two agents read it, two
# agents write it, and neither sees the other until one pushes. The fix is the
# same one, for the same reason.
#
# THE ATOMIC PRIMITIVE (identical to reserve-issue-id.sh — read that file first)
#
# `git push` creating a ref that already exists on the remote is REJECTED. That
# is a compare-and-swap against state every agent already shares, and it
# arbitrates at ORIGIN, so two agents on two machines are serialised by the
# server rather than by trust.
#
# The pushed object must be UNIQUE PER ATTEMPT or the CAS is unsound: pushing a
# ref that already exists pointing at the SAME object is a silent no-op success,
# and both racers would believe they won. Every object built here carries a
# nanosecond timestamp, a pid, a hostname and a random nonce for exactly that
# reason. (`--selftest` asserts the failure mode: it pushes one object twice and
# shows git reporting success both times.)
#
# WHAT A CLAIM NEEDS THAT AN ID RESERVATION DOES NOT: EXPIRY
#
# An issue id is permanent. A claim held by an agent that died must not be, or
# the work is frozen behind a ghost. So the object carries `agent`,
# `claimed_at` and `ttl`; the holder `renew`s inside the TTL; and a would-be
# stealer takes an EXPIRED claim with
#
#     git push --force-with-lease=refs/claims/<id>:<expected-oid> …
#
# so that two agents racing to steal the same dead claim cannot both win — the
# server rejects whichever one read the ref first ("stale info").
#
# TTLs ARE HOURS, NOT DAYS, AND NOT MINUTES
#
# Hours because an OPEN PR SUPERSEDES THE CLAIM: the ref only governs the window
# between "I started" and "I pushed something visible", which is short by
# design. Not minutes because CLOCK SKEW across machines makes every TTL
# approximate — `claimed_at` is written by one host's clock and judged by
# another's. Default 4h; a claim whose `claimed_at` is in the future is reported
# as skew rather than silently treated as fresh.
#
# STEALING IS LOUD ON PURPOSE
#
# An agent dying mid-task is invisible and therefore recurs. A successful steal
# prints a leftovers report — the pushed `fix/<id>` branch nobody would look for
# (silently discarding it loses real work), `git log --grep=<id>` for partially
# landed work — and tells the caller to comment on the issue so a human sees
# that an agent died there.
#
# CLAIMS ARE ADVISORY. SAY SO RATHER THAN IMPLYING MORE.
#
# A duplicate issue id is mechanically detectable, so the `pre-push` hook can
# ENFORCE `reserve-issue-id.sh`. There is no mechanical map from a diff to a
# claim, so nothing here can be enforced. This prevents accidental duplication
# between COOPERATING agents and nothing else; it does not stop a determined
# collision, and no ref scheme will. Outside contributors cannot write refs at
# all — for them the coordination mechanism is GitHub issue assignment, so an
# agent must check BOTH before starting.
#
# LOCAL SIDE EFFECTS
#
# Reading a claim's body needs the object, so claim refs are mirrored to
# `refs/nros-claims/*` (not `refs/remotes/`, not `refs/tags/`) — invisible to a
# default fetch, `git tag -l` and `git branch -a`.
#
# EXIT CODES (scriptable — the whole point is telling these apart)
#
#   0  ok
#   1  usage, bad id, nothing to do
#   2  held by a LIVE claim, or lost the race for a free one
#   3  remote unreachable / no push permission — NOT contention
#   4  held by an EXPIRED claim; use `steal`
#   5  the CAS caught a race (lease was stale) — re-read and retry

set -euo pipefail

# This script's selftest builds throwaway repositories, so an inherited
# repository-local git environment (GIT_DIR and friends) silently redirects that
# work into the CALLER'S repository. Observed as a pre-push hook that corrupted
# the repo it was guarding (issue 0986). Cleared here because every git
# invocation below names its target explicitly — including the `cd` on the next
# lines, which is how this script chooses the repo it operates on.
# shellcheck source=scripts/lib/git-hook-env.sh
. "$(cd "$(dirname "${BASH_SOURCE[0]}")/lib" && pwd)/git-hook-env.sh"
nros_clear_inherited_git_env


SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
cd "${NROS_CLAIM_REPO:-$(dirname "$SELF")/..}"

REF_NS="refs/claims"
MIRROR_NS="refs/nros-claims"
REMOTE="${NROS_CLAIM_REMOTE:-origin}"
DEFAULT_TTL_SECONDS="${NROS_CLAIM_TTL_SECONDS:-14400}"   # 4h
SKEW_TOLERANCE=900                                        # 15m

DRY_RUN=0
FORCE=0
TTL_SECONDS="$DEFAULT_TTL_SECONDS"
NOTE=""

LAST_ERR=""
PUSH_ERR=""
C_BODY=""; C_AGENT=""; C_HOST=""; C_PID=""; C_CLAIMED_AT=""; C_TTL=""

# ---------------------------------------------------------------- primitives

die() { printf '[FAIL] %s\n' "$*" >&2; exit 1; }

now_epoch() { date +%s; }

fmt_dur() {
    local s="$1" sign=""
    if [ "$s" -lt 0 ]; then sign="-"; s=$(( -s )); fi
    if [ "$s" -lt 60 ]; then
        printf '%s%ds' "$sign" "$s"
    elif [ "$s" -lt 3600 ]; then
        printf '%s%dm' "$sign" "$((s / 60))"
    else
        printf '%s%dh%02dm' "$sign" "$((s / 3600))" "$(((s % 3600) / 60))"
    fi
}

agent_id() {
    if [ -n "${NROS_CLAIM_AGENT:-}" ]; then
        printf '%s' "$NROS_CLAIM_AGENT"
        return 0
    fi
    printf '%s@%s' "${USER:-unknown}" "$(hostname 2>/dev/null || echo unknown)"
}

# An id names a work unit: `issue-0827`, `phase-392-W3a`. It becomes a ref path
# component, so anything that could escape the namespace is rejected outright.
validate_id() {
    local id="$1"
    if [ -z "$id" ]; then
        printf '[FAIL] no id given. Try: %s claim issue-0827\n' "$(basename "$SELF")" >&2
        return 1
    fi
    case "$id" in
        *[!A-Za-z0-9._-]*)
            printf '[FAIL] bad id %q — use only [A-Za-z0-9._-] (e.g. issue-0827, phase-392-W3a)\n' \
                "$id" >&2
            return 1 ;;
        .*|*..*)
            printf '[FAIL] bad id %q — must not start with a dot or contain ".."\n' "$id" >&2
            return 1 ;;
    esac
    return 0
}

# A commit object no other agent can reproduce. See the header: an identical
# object pushed to an existing ref is a no-op SUCCESS, which would make two
# racers both believe they won.
build_object() {
    local id="$1" claimed_at="$2" ttl="$3" empty_tree stamp
    empty_tree="$(git hash-object -t tree /dev/null)"
    stamp="$(date +%s%N 2>/dev/null || date +%s)"
    printf 'claim %s\n\nagent: %s\nhost: %s\npid: %s\nclaimed_at: %s\nttl: %s\nnonce: %s-%s\nnote: %s\n' \
        "$id" "$(agent_id)" "$(hostname 2>/dev/null || echo unknown)" "$$" \
        "$claimed_at" "$ttl" "$stamp" "${RANDOM}${RANDOM}" "${NOTE:-<none>}" |
        GIT_AUTHOR_NAME="nros-claim" GIT_AUTHOR_EMAIL="nros-claim@localhost" \
        GIT_COMMITTER_NAME="nros-claim" GIT_COMMITTER_EMAIL="nros-claim@localhost" \
        git commit-tree "$empty_tree"
}

# The remote's current oid for a claim, or "" when the ref does not exist.
# Returns 3 (not "") when the REMOTE could not be reached — the distinction
# between "already taken" and "no network" is the whole reason this is separate.
remote_oid() {
    local out rc=0
    out="$(git ls-remote "$REMOTE" "$REF_NS/$1" 2>&1)" || rc=$?
    if [ "$rc" -ne 0 ]; then
        LAST_ERR="$out"
        return 3
    fi
    printf '%s' "$out" | awk 'NR==1 {print $1}'
    return 0
}

fetch_mirror() {
    local out rc=0
    out="$(git fetch --quiet --prune "$REMOTE" "+$REF_NS/*:$MIRROR_NS/*" 2>&1)" || rc=$?
    if [ "$rc" -ne 0 ]; then
        LAST_ERR="$out"
        return 3
    fi
    return 0
}

claim_field() { printf '%s\n' "$C_BODY" | sed -n "s/^$1: //p" | head -1; }

# Populate C_* from a claim object. Fetches once if the object is not local yet.
load_claim() {
    local oid="$1" body rc=0
    C_BODY=""; C_AGENT=""; C_HOST=""; C_PID=""; C_CLAIMED_AT=""; C_TTL=""
    body="$(git cat-file commit "$oid" 2>/dev/null)" || rc=$?
    if [ "$rc" -ne 0 ]; then
        fetch_mirror || true
        rc=0
        body="$(git cat-file commit "$oid" 2>/dev/null)" || rc=$?
    fi
    if [ "$rc" -ne 0 ]; then
        return 1
    fi
    C_BODY="$body"
    C_AGENT="$(claim_field agent)"
    C_HOST="$(claim_field host)"
    C_PID="$(claim_field pid)"
    C_CLAIMED_AT="$(claim_field claimed_at)"
    C_TTL="$(claim_field ttl)"
    case "$C_CLAIMED_AT" in ''|*[!0-9]*) C_CLAIMED_AT="" ;; esac
    case "$C_TTL" in ''|*[!0-9]*) C_TTL="" ;; esac
    return 0
}

# Seconds until the loaded claim expires; <= 0 means expired (the lease has
# lapsed AT claimed_at+ttl, not one second after). A claim whose
# fields are unreadable is treated as LIVE and effectively un-stealable, because
# guessing "probably dead" about an object we cannot parse is the one error that
# loses another agent's work.
claim_remaining() {
    if [ -z "$C_CLAIMED_AT" ] || [ -z "$C_TTL" ]; then
        printf '%s' "$DEFAULT_TTL_SECONDS"
        return 0
    fi
    printf '%s' "$(( C_CLAIMED_AT + C_TTL - $(now_epoch) ))"
}

claim_skew() {
    if [ -z "$C_CLAIMED_AT" ]; then printf '0'; return 0; fi
    printf '%s' "$(( C_CLAIMED_AT - $(now_epoch) ))"
}

describe_holder() {
    local remaining skew
    remaining="$(claim_remaining)"
    skew="$(claim_skew)"
    printf '       holder:  %s (host %s, pid %s)\n' \
        "${C_AGENT:-<unparseable>}" "${C_HOST:-?}" "${C_PID:-?}" >&2
    if [ -n "$C_CLAIMED_AT" ]; then
        printf '       claimed: %s ago, ttl %s\n' \
            "$(fmt_dur "$(( $(now_epoch) - C_CLAIMED_AT ))")" \
            "$(fmt_dur "${C_TTL:-0}")" >&2
    else
        printf '       claimed: <unparseable object — treated as live>\n' >&2
    fi
    if [ "$remaining" -gt 0 ]; then
        printf '       expires: in %s\n' "$(fmt_dur "$remaining")" >&2
    else
        printf '       expires: %s AGO\n' "$(fmt_dur "$(( -remaining ))")" >&2
    fi
    if [ "$skew" -gt "$SKEW_TOLERANCE" ]; then
        printf '       [skew]   claimed_at is %s in the FUTURE by this clock.\n' \
            "$(fmt_dur "$skew")" >&2
        printf '                TTLs are approximate across machines; treat as live.\n' >&2
    fi
}

# git push, wrapped so --dry-run performs none and so the selftest can inject a
# concurrent update between our READ of the ref and our WRITE (the race the
# lease exists to catch).
do_push() {
    local out rc=0
    if [ "$DRY_RUN" = 1 ]; then
        printf '[dry-run] would run: git push %s\n' "$*" >&2
        PUSH_ERR=""
        return 0
    fi
    if [ -n "${NROS_CLAIM_PRE_PUSH_HOOK:-}" ]; then
        bash -c "$NROS_CLAIM_PRE_PUSH_HOOK" >&2 2>&1 || true
    fi
    out="$(git push "$@" 2>&1)" || rc=$?
    PUSH_ERR="$out"
    return "$rc"
}

# "taken" | "lease" | "remote". Anything unrecognised is an ENVIRONMENT problem,
# never contention — reporting a broken remote as "someone else has it" would
# send an agent to steal a claim that may not exist.
classify_push_error() {
    local low="${1,,}"
    case "$low" in
        *permission*|*"could not read from remote"*|*"unable to access"*|\
        *authentication*|*"connection"*|*"resolve host"*|*"timed out"*|\
        *"does not appear to be a git repository"*|*"repository not found"*)
            printf 'remote' ;;
        *"stale info"*)
            printf 'lease' ;;
        *"already exists"*|*"non-fast-forward"*|*"fetch first"*|*rejected*)
            printf 'taken' ;;
        *)
            printf 'remote' ;;
    esac
}

report_remote_failure() {
    printf '[FAIL] could not reach %s — this is NOT contention:\n' "$REMOTE" >&2
    printf '%s\n' "$1" | sed 's/^/       /' >&2
    printf '\n       No claim was taken. You are working UNCLAIMED: another agent\n' >&2
    printf '       may be on the same work and neither of you can see the other.\n' >&2
}

# ---------------------------------------------------------------- advisories

# The doc'd predecessor rule (finding 1): W3b needs W3a's code, so claiming it
# while W3a is claimed-and-unlanded is a valid claim on blocked work. This WARNS
# rather than refuses: wave order is not machine-readable anywhere in this repo,
# so a refusal would be guessing, and lexical order is only a heuristic.
predecessor_warning() {
    local id="$1" phase wave other other_wave first
    case "$id" in
        phase-*-W*) ;;
        *) return 0 ;;
    esac
    phase="${id%%-W*}"
    wave="${id##*-W}"
    fetch_mirror || return 0
    while IFS= read -r ref; do
        other="${ref##*/}"
        [ "$other" = "$id" ] && continue
        case "$other" in "$phase"-W*) ;; *) continue ;; esac
        local oid
        oid="$(git rev-parse "$ref" 2>/dev/null || true)"
        [ -z "$oid" ] && continue
        load_claim "$oid" || continue
        [ "$(claim_remaining)" -lt 0 ] && continue
        other_wave="${other##*-W}"
        first="$(printf '%s\n%s\n' "$other_wave" "$wave" | sort -V | head -1)"
        if [ "$first" = "$other_wave" ] && [ "$other_wave" != "$wave" ]; then
            printf '[warn] %s is claimed and unlanded by %s.\n' "$other" "${C_AGENT:-?}" >&2
            printf '       If %s depends on it, you will branch from a main that does\n' "$id" >&2
            printf '       not have it yet. Advisory only: wave order is not machine-\n' >&2
            printf '       readable, so this is lexical, not a dependency graph.\n' >&2
        fi
    done < <(git for-each-ref --format='%(refname)' "$MIRROR_NS/" 2>/dev/null || true)
    return 0
}

# Everything an agent's death leaves behind that nobody would look for.
leftovers_report() {
    local id="$1" token found=0 heads rc=0
    token="${id#issue-}"
    printf '\n  LEFTOVERS — an agent died here. Look BEFORE starting fresh:\n' >&2

    heads="$(git ls-remote --heads "$REMOTE" 2>/dev/null)" || rc=$?
    if [ "$rc" -eq 0 ]; then
        while IFS= read -r line; do
            local ref="${line#*$'\t'}"
            case "$ref" in
                *"$id"*|*"$token"*)
                    printf '    branch  %s  (pushed, no PR — discarding it loses real work)\n' \
                        "${ref#refs/heads/}" >&2
                    found=1 ;;
            esac
        done <<< "$heads"
    else
        printf '    branch  <could not list %s heads; run: git ls-remote --heads %s>\n' \
            "$REMOTE" "$REMOTE" >&2
    fi
    [ "$found" = 0 ] && printf '    branch  none matching %s or %s\n' "$id" "$token" >&2

    printf '    landed  git log --grep=%s --oneline   (this repo names ids in commits):\n' "$token" >&2
    if ! git log --grep="$token" --oneline -10 2>/dev/null | sed 's/^/            /' >&2; then
        printf '            <no local history to search>\n' >&2
    fi

    printf '    PR      gh pr list --state all --search %s\n' "$token" >&2
    printf '            (an OPEN PR supersedes the claim — take it over or close it,\n' >&2
    printf '             do not start again)\n' >&2
    printf '\n    >> COMMENT ON THE ISSUE that an agent died holding this claim.\n' >&2
    printf '       Otherwise the failure is invisible and recurs.\n' >&2
}

# ---------------------------------------------------------------- subcommands

cmd_claim() {
    local id="$1" oid rc=0 remaining obj kind
    validate_id "$id" || return 1

    oid="$(remote_oid "$id")" || rc=$?
    if [ "$rc" -eq 3 ]; then report_remote_failure "$LAST_ERR"; return 3; fi

    if [ -n "$oid" ]; then
        if load_claim "$oid"; then
            remaining="$(claim_remaining)"
        else
            remaining="$DEFAULT_TTL_SECONDS"
        fi
        if [ "$remaining" -gt 0 ]; then
            printf '[held] %s is claimed by a LIVE claim.\n' "$id" >&2
            describe_holder
            printf '       Pick different work, or coordinate. Do NOT work it anyway.\n' >&2
            return 2
        fi
        printf '[expired] %s is claimed but the lease lapsed %s ago.\n' \
            "$id" "$(fmt_dur "$(( -remaining ))")" >&2
        describe_holder
        printf '       Take it with:  %s steal %s\n' "$(basename "$SELF")" "$id" >&2
        printf '       (a separate verb on purpose: stealing must be deliberate and loud)\n' >&2
        return 4
    fi

    predecessor_warning "$id" || true

    obj="$(build_object "$id" "$(now_epoch)" "$TTL_SECONDS")"
    if do_push "$REMOTE" "$obj:$REF_NS/$id"; then
        if [ "$DRY_RUN" = 1 ]; then
            printf '[dry-run] %s is free; no push was made and nothing is claimed.\n' "$id" >&2
            return 0
        fi
        printf '%s\n' "$id"
        printf 'claimed %s on %s for %s (as %s)\n' \
            "$id" "$REMOTE" "$(fmt_dur "$TTL_SECONDS")" "$(agent_id)" >&2
        printf '  renew:   %s renew %s      (run it from a LIVENESS supervisor, not\n' \
            "$(basename "$SELF")" "$id" >&2
        printf '                                   between steps — a 40-minute fixture\n' >&2
        printf '                                   build must not look like death)\n' >&2
        printf '  release: %s release %s    (on completion — do NOT wait for expiry)\n' \
            "$(basename "$SELF")" "$id" >&2
        printf '  note:    an open PR supersedes this claim; push early.\n' >&2
        printf '  note:    outside contributors cannot write refs. Check the GitHub\n' >&2
        printf '           issue assignee too:  gh issue list --search %s\n' "${id#issue-}" >&2
        return 0
    fi

    kind="$(classify_push_error "$PUSH_ERR")"
    if [ "$kind" = "remote" ]; then
        report_remote_failure "$PUSH_ERR"
        return 3
    fi
    printf '[held] lost the race for %s — another agent claimed it between our\n' "$id" >&2
    printf '       read and our write. That is the CAS working, not an error.\n' >&2
    rc=0
    oid="$(remote_oid "$id")" || rc=$?
    if [ "$rc" -eq 0 ] && [ -n "$oid" ] && load_claim "$oid"; then describe_holder; fi
    return 2
}

cmd_renew() {
    local id="$1" oid rc=0 obj kind mine
    validate_id "$id" || return 1
    mine="$(agent_id)"

    oid="$(remote_oid "$id")" || rc=$?
    if [ "$rc" -eq 3 ]; then report_remote_failure "$LAST_ERR"; return 3; fi

    # Renewing something nobody holds is a claim. This is what makes `renew`
    # safe to call on a timer from the first tick onwards.
    if [ -z "$oid" ]; then
        printf '[note] %s was not held (expired ref deleted, or never claimed) — claiming it.\n' \
            "$id" >&2
        cmd_claim "$id"
        return $?
    fi

    if load_claim "$oid"; then
        if [ -n "$C_AGENT" ] && [ "$C_AGENT" != "$mine" ] && [ "$FORCE" != 1 ]; then
            printf '[held] %s is held by another agent; renewing it would be a silent steal.\n' "$id" >&2
            describe_holder
            printf '       If that agent is dead:  %s steal %s\n' "$(basename "$SELF")" "$id" >&2
            return 2
        fi
    fi

    obj="$(build_object "$id" "$(now_epoch)" "$TTL_SECONDS")"
    if do_push --force-with-lease="$REF_NS/$id:$oid" "$REMOTE" "$obj:$REF_NS/$id"; then
        if [ "$DRY_RUN" = 1 ]; then
            printf '[dry-run] would renew %s (lease %s)\n' "$id" "${oid:0:12}" >&2
            return 0
        fi
        printf 'renewed %s for %s (as %s)\n' "$id" "$(fmt_dur "$TTL_SECONDS")" "$mine" >&2
        return 0
    fi

    kind="$(classify_push_error "$PUSH_ERR")"
    case "$kind" in
        remote) report_remote_failure "$PUSH_ERR"; return 3 ;;
        *)
            printf '[race] the claim moved under us while renewing %s (lease %s was stale).\n' \
                "$id" "${oid:0:12}" >&2
            printf '       Someone stole or released it. Re-read before assuming anything:\n' >&2
            printf '         %s list\n' "$(basename "$SELF")" >&2
            return 5 ;;
    esac
}

cmd_release() {
    local id="$1" oid rc=0 kind mine
    validate_id "$id" || return 1
    mine="$(agent_id)"

    oid="$(remote_oid "$id")" || rc=$?
    if [ "$rc" -eq 3 ]; then report_remote_failure "$LAST_ERR"; return 3; fi
    if [ -z "$oid" ]; then
        printf '[ok] %s was not claimed; nothing to release.\n' "$id" >&2
        return 0
    fi

    if load_claim "$oid"; then
        if [ -n "$C_AGENT" ] && [ "$C_AGENT" != "$mine" ] && [ "$FORCE" != 1 ]; then
            printf '[held] %s is held by another agent — releasing it would hand their\n' "$id" >&2
            printf '       in-flight work to anyone. Use --force only if you know it is dead.\n' >&2
            describe_holder
            return 2
        fi
    fi

    if do_push --force-with-lease="$REF_NS/$id:$oid" "$REMOTE" ":$REF_NS/$id"; then
        if [ "$DRY_RUN" = 1 ]; then
            printf '[dry-run] would release %s (lease %s)\n' "$id" "${oid:0:12}" >&2
            return 0
        fi
        printf 'released %s\n' "$id" >&2
        return 0
    fi

    kind="$(classify_push_error "$PUSH_ERR")"
    case "$kind" in
        remote) report_remote_failure "$PUSH_ERR"; return 3 ;;
        *)
            printf '[race] %s moved while releasing (lease %s was stale) — someone else\n' \
                "$id" "${oid:0:12}" >&2
            printf '       already took it. Nothing was deleted.\n' >&2
            return 5 ;;
    esac
}

cmd_steal() {
    local id="$1" oid rc=0 remaining obj kind
    validate_id "$id" || return 1

    oid="$(remote_oid "$id")" || rc=$?
    if [ "$rc" -eq 3 ]; then report_remote_failure "$LAST_ERR"; return 3; fi
    if [ -z "$oid" ]; then
        printf '[FAIL] %s is not claimed — nothing to steal. Use: %s claim %s\n' \
            "$id" "$(basename "$SELF")" "$id" >&2
        return 1
    fi

    if load_claim "$oid"; then
        remaining="$(claim_remaining)"
    else
        remaining="$DEFAULT_TTL_SECONDS"
    fi
    if [ "$remaining" -gt 0 ] && [ "$FORCE" != 1 ]; then
        printf '[held] %s has NOT expired — %s left. Stealing a live claim is how two\n' \
            "$id" "$(fmt_dur "$remaining")" >&2
        printf '       agents end up on one task, which wastes both.\n' >&2
        describe_holder
        return 2
    fi

    obj="$(build_object "$id" "$(now_epoch)" "$TTL_SECONDS")"
    # The lease is the whole mechanism: two agents that both read $oid and both
    # push, only one wins, because the loser's expected value no longer matches.
    if do_push --force-with-lease="$REF_NS/$id:$oid" "$REMOTE" "$obj:$REF_NS/$id"; then
        if [ "$DRY_RUN" = 1 ]; then
            printf '[dry-run] would steal %s from %s (lease %s)\n' \
                "$id" "${C_AGENT:-?}" "${oid:0:12}" >&2
            leftovers_report "$id"
            return 0
        fi
        printf '%s\n' "$id"
        if [ "$remaining" -gt 0 ]; then
            printf 'STOLE %s from %s — a LIVE claim, forced with --force (%s of lease\n' \
                "$id" "${C_AGENT:-<unparseable>}" "$(fmt_dur "$remaining")" >&2
            printf '      was left). If that agent is not actually dead, you are now both\n' >&2
            printf '      on this work.\n' >&2
        else
            printf 'STOLE %s from %s (claim had expired %s ago)\n' \
                "$id" "${C_AGENT:-<unparseable>}" "$(fmt_dur "$(( -remaining ))")" >&2
        fi
        leftovers_report "$id"
        return 0
    fi

    kind="$(classify_push_error "$PUSH_ERR")"
    case "$kind" in
        remote) report_remote_failure "$PUSH_ERR"; return 3 ;;
        *)
            printf '[race] the lease on %s was STALE — the ref moved between our read\n' "$id" >&2
            printf '       (%s) and our push. Another agent stole it first, or the\n' "${oid:0:12}" >&2
            printf '       holder came back and renewed. You did NOT get the claim.\n' >&2
            printf '       This is the force-with-lease CAS doing its job.\n' >&2
            return 5 ;;
    esac
}

cmd_list() {
    local rc=0 count=0 ref id oid remaining state
    if ! fetch_mirror; then report_remote_failure "$LAST_ERR"; return 3; fi
    while IFS= read -r ref; do
        [ -z "$ref" ] && continue
        id="${ref#"$MIRROR_NS/"}"
        oid="$(git rev-parse "$ref" 2>/dev/null || true)"
        [ -z "$oid" ] && continue
        if load_claim "$oid"; then
            remaining="$(claim_remaining)"
        else
            remaining="$DEFAULT_TTL_SECONDS"
        fi
        if [ "$remaining" -gt 0 ]; then state="live"; else state="EXPIRED"; fi
        if [ "$count" -eq 0 ]; then
            printf '%-24s %-28s %-8s %-9s %s\n' ID HOLDER AGE STATE REMAINING
        fi
        printf '%-24s %-28s %-8s %-9s %s\n' \
            "$id" "${C_AGENT:-<unparseable>}" \
            "$(if [ -n "$C_CLAIMED_AT" ]; then fmt_dur "$(( $(now_epoch) - C_CLAIMED_AT ))"; else printf '?'; fi)" \
            "$state" \
            "$(if [ "$remaining" -gt 0 ]; then fmt_dur "$remaining"; else printf -- '-%s' "$(fmt_dur "$(( -remaining ))")"; fi)"
        count=$((count + 1))
    done < <(git for-each-ref --sort=refname --format='%(refname)' "$MIRROR_NS/" 2>/dev/null || true)

    if [ "$count" -eq 0 ]; then
        printf 'no claims on %s\n' "$REMOTE"
    else
        printf '\n%s claim(s). EXPIRED ones can be taken with `steal <id>`.\n' "$count" >&2
        printf 'Claims are ADVISORY and bind only agents with push access — check the\n' >&2
        printf 'GitHub issue assignees too before starting.\n' >&2
    fi
    return "$rc"
}

usage() {
    cat >&2 <<EOF
usage: $(basename "$SELF") <command> [id] [options]

  claim   <id>   take refs/claims/<id> (fails if held by a live claim)
  renew   <id>   refresh the lease; safe to call repeatedly, claims if free
  release <id>   drop it on completion — do not wait for expiry
  steal   <id>   take an EXPIRED claim, with a --force-with-lease CAS
  list           every claim, with holder, age and state
  leftovers <id> what a dead agent left behind (branch / commits / PR)

options:
  --dry-run             decide everything, push nothing
  --ttl <hours>         lease length (default $((DEFAULT_TTL_SECONDS / 3600))h; hours, not days)
  --note <text>         recorded in the claim object
  --force               renew/release/steal against another agent's live claim
  --selftest            prove the failure paths (hermetic; no network)
  -h, --help            this

env:
  NROS_CLAIM_REMOTE     remote to arbitrate on (default: origin)
  NROS_CLAIM_AGENT      agent identity (default: \$USER@\$(hostname))
  NROS_CLAIM_TTL_SECONDS  default lease in seconds (overrides the 4h default)
  NROS_CLAIM_REPO       repo whose object store is used (default: this checkout)

exit: 0 ok | 1 usage | 2 held (live) | 3 remote unreachable | 4 held (expired)
      5 the CAS caught a race
EOF
}

# ------------------------------------------------------------------ selftest

# A gate that cannot fail reads as coverage. Everything below runs against a
# THROWAWAY bare repo, so the CAS, the lease and the error classification are
# exercised with REAL git push semantics and zero network — and a failure here
# can never leave the real worktree or origin dirty.
selftest() {
    local pass=0 fail=0 origin tmp
    # Global, not local: the EXIT trap fires after this frame has popped.
    SELFTEST_TMP="$(mktemp -d "${TMPDIR:-/tmp}/nros-claim-selftest.XXXXXX")"
    trap 'rm -rf "${SELFTEST_TMP:-}"' EXIT
    tmp="$SELFTEST_TMP"
    origin="$tmp/origin.git"
    git init --quiet --bare "$origin"
    git init --quiet "$tmp/a"
    git init --quiet "$tmp/b"
    git init --quiet "$tmp/c"

    local OUT="" RC=0
    # run <agent> <repo> [args…] — invokes THIS script, captures output + status
    run() {
        local ag="$1" repo="$2"; shift 2
        RC=0
        OUT="$(env NROS_CLAIM_REPO="$tmp/$repo" NROS_CLAIM_REMOTE="$origin" \
                   NROS_CLAIM_AGENT="$ag" \
                   bash "$SELF" "$@" 2>&1)" || RC=$?
    }
    check() { # check <desc> <expected-rc>
        if [ "$RC" = "$2" ]; then
            printf '  ok    %s (rc=%s)\n' "$1" "$RC"
            pass=$((pass + 1))
        else
            printf '  FAIL  %s: expected rc=%s got rc=%s\n' "$1" "$2" "$RC"
            printf '%s\n' "$OUT" | sed 's/^/          /'
            fail=$((fail + 1))
        fi
    }
    check_says() { # check_says <desc> <substring>
        case "$OUT" in
            *"$2"*) printf '  ok    %s\n' "$1"; pass=$((pass + 1)) ;;
            *) printf '  FAIL  %s: output does not mention %q\n' "$1" "$2"
               printf '%s\n' "$OUT" | sed 's/^/          /'
               fail=$((fail + 1)) ;;
        esac
    }
    oid_of() { git ls-remote "$origin" "$REF_NS/$1" | awk 'NR==1{print $1}'; }

    printf 'the primitive this whole design rests on\n'
    # WHY the object must be unique per attempt: pushing an IDENTICAL object to
    # an existing ref is a no-op SUCCESS, so two racers would both "win".
    local tree obj probe="refs/nros-selftest/probe"
    tree="$(git -C "$tmp/a" hash-object -t tree /dev/null)"
    obj="$(printf 'identical\n' | git -C "$tmp/a" \
            -c user.name=t -c user.email=t@t commit-tree "$tree")"
    RC=0; OUT="$(git -C "$tmp/a" push "$origin" "$obj:$probe" 2>&1)" || RC=$?
    check "first push of an object creates the ref" 0
    RC=0; OUT="$(git -C "$tmp/a" push "$origin" "$obj:$probe" 2>&1)" || RC=$?
    check "SAME object to an EXISTING ref is a silent no-op SUCCESS (hence the nonce)" 0
    check_says "…and git calls it 'up-to-date', not 'rejected'" "up-to-date"
    RC=0; obj="$(printf 'different\n' | git -C "$tmp/a" \
            -c user.name=t -c user.email=t@t commit-tree "$tree")"
    OUT="$(git -C "$tmp/a" push "$origin" "$obj:$probe" 2>&1)" || RC=$?
    check "a DIFFERENT object to an existing ref is REJECTED — the CAS" 1
    git -C "$tmp/a" push --quiet "$origin" ":$probe" >/dev/null 2>&1

    printf '\nclaim\n'
    run agent-a a claim demo;            check "a free id can be claimed" 0
    [ -n "$(oid_of demo)" ] && { printf '  ok    the ref exists on the remote\n'; pass=$((pass+1)); } \
        || { printf '  FAIL  claim reported success but the ref is absent\n'; fail=$((fail+1)); }
    run agent-b b claim demo;            check "a SECOND claim of a held id FAILS" 2
    check_says "and it names the holder" "agent-a"
    run agent-a a claim "bad/id";        check "an id that could escape the ref namespace is refused" 1
    run agent-a a claim;                 check "a missing id is a usage error" 1
    run agent-a a claim demo --ttl soon; check "a non-numeric --ttl is refused" 1
    # `git ls-remote <remote> <pattern>` matches whole path components, so a
    # held `issue-0827` must not make `issue-082` read as taken. A prefix match
    # here would block real work with a confident, wrong "already claimed".
    run agent-b b claim dem;             check "a PREFIX of a held id is still free" 0
    run agent-b b release dem >/dev/null

    printf '\ndry-run makes no push\n'
    run agent-a a claim untouched --dry-run; check "dry-run claim succeeds" 0
    [ -z "$(oid_of untouched)" ] && { printf '  ok    …and pushed NOTHING\n'; pass=$((pass+1)); } \
        || { printf '  FAIL  --dry-run created refs/claims/untouched\n'; fail=$((fail+1)); }

    printf '\nrenew\n'
    local before after
    before="$(oid_of demo)"
    run agent-a a renew demo;            check "the holder can renew" 0
    after="$(oid_of demo)"
    [ "$before" != "$after" ] && { printf '  ok    …and the object CHANGED (unique per attempt)\n'; pass=$((pass+1)); } \
        || { printf '  FAIL  renew pushed an identical object — the CAS would be unsound\n'; fail=$((fail+1)); }
    run agent-a a renew demo;            check "renew is safe to call repeatedly" 0
    run agent-b b renew demo;            check "a NON-holder cannot renew (that would be a silent steal)" 2

    printf '\nsteal\n'
    run agent-b b steal demo;            check "a LIVE claim cannot be stolen" 2
    # Expire on purpose: ttl 0 means claimed_at+0 is already in the past.
    RC=0; OUT="$(env NROS_CLAIM_REPO="$tmp/a" NROS_CLAIM_REMOTE="$origin" \
                 NROS_CLAIM_AGENT=agent-a NROS_CLAIM_TTL_SECONDS=0 \
                 bash "$SELF" claim dead 2>&1)" || RC=$?
    check "a claim with a lapsed ttl can be created" 0
    run agent-b b claim dead;            check "claim on an EXPIRED id refuses and points at steal" 4
    check_says "…loudly" "steal dead"
    run agent-b b steal dead;            check "steal takes an expired claim" 0
    check_says "steal reports the leftovers a dead agent leaves" "LEFTOVERS"
    check_says "…including the branch nobody would look for" "branch"
    check_says "…and tells a human to comment on the issue" "COMMENT ON THE ISSUE"

    printf '\nsteal race: a STALE expected-oid must lose\n'
    RC=0; env NROS_CLAIM_REPO="$tmp/a" NROS_CLAIM_REMOTE="$origin" \
             NROS_CLAIM_AGENT=agent-a NROS_CLAIM_TTL_SECONDS=0 \
             bash "$SELF" claim dead2 >/dev/null 2>&1 || RC=$?
    check "a second expired claim exists to race for" 0
    local stolen_by_c
    # agent-c steals in the window between agent-b's READ of the ref and its
    # PUSH. b's --force-with-lease names an oid that no longer exists remotely.
    cat > "$tmp/hook.sh" <<EOF
unset NROS_CLAIM_PRE_PUSH_HOOK
NROS_CLAIM_REPO="$tmp/c" NROS_CLAIM_REMOTE="$origin" NROS_CLAIM_AGENT=agent-c \\
    bash "$SELF" steal dead2 >/dev/null 2>&1
EOF
    RC=0
    OUT="$(env NROS_CLAIM_REPO="$tmp/b" NROS_CLAIM_REMOTE="$origin" \
               NROS_CLAIM_AGENT=agent-b \
               NROS_CLAIM_PRE_PUSH_HOOK="bash $tmp/hook.sh" \
               bash "$SELF" steal dead2 2>&1)" || RC=$?
    check "the loser of a two-way steal is REJECTED (stale lease)" 5
    check_says "…and says so as a race, not as an environment failure" "STALE"
    stolen_by_c=0
    if load_claim_in "$tmp/b" "$(oid_of dead2)" "$origin"; then
        [ "$C_AGENT" = "agent-c" ] && stolen_by_c=1
    fi
    [ "$stolen_by_c" = 1 ] && { printf '  ok    exactly one winner, and it is the other agent\n'; pass=$((pass+1)); } \
        || { printf '  FAIL  ref holder after the race is %q, expected agent-c\n' "$C_AGENT"; fail=$((fail+1)); }

    printf '\nrelease\n'
    run agent-b b release dead2;         check "a NON-holder cannot release another agent'\''s claim" 2
    run agent-c c release dead2 --dry-run; check "dry-run release succeeds" 0
    [ -n "$(oid_of dead2)" ] && { printf '  ok    …and deleted NOTHING\n'; pass=$((pass+1)); } \
        || { printf '  FAIL  --dry-run release deleted the ref\n'; fail=$((fail+1)); }
    run agent-c c release dead2;         check "the holder can release" 0
    [ -z "$(oid_of dead2)" ] && { printf '  ok    …and the ref is gone\n'; pass=$((pass+1)); } \
        || { printf '  FAIL  release reported success but the ref remains\n'; fail=$((fail+1)); }
    run agent-c c release dead2;         check "release is idempotent" 0
    run agent-c c renew dead2;           check "renew of a released id re-claims it" 0
    [ -n "$(oid_of dead2)" ] && { printf '  ok    …so a renew timer works from the first tick\n'; pass=$((pass+1)); } \
        || { printf '  FAIL  renew of a free id left no claim\n'; fail=$((fail+1)); }
    run agent-c c release dead2 >/dev/null

    printf '\nan UNPARSEABLE claim object is treated as LIVE, never as dead\n'
    # Guessing "probably dead" about an object we cannot read is the one error
    # that silently destroys another agent's work, so both verbs must refuse.
    local junk
    junk="$(printf 'not a claim at all\n' | git -C "$tmp/a" \
             -c user.name=t -c user.email=t@t commit-tree "$tree")"
    git -C "$tmp/a" push --quiet "$origin" "$junk:$REF_NS/junk" >/dev/null 2>&1
    run agent-b b claim junk;            check "claim refuses an unreadable claim" 2
    run agent-b b steal junk;            check "steal refuses an unreadable claim" 2

    printf '\nremote failure is NOT contention\n'
    RC=0; OUT="$(env NROS_CLAIM_REPO="$tmp/a" NROS_CLAIM_REMOTE="$tmp/no-such-repo.git" \
                 NROS_CLAIM_AGENT=agent-a bash "$SELF" claim demo 2>&1)" || RC=$?
    check "an unreachable remote exits 3, not 2" 3
    check_says "…and says it is not contention" "NOT contention"
    RC=0; OUT="$(env NROS_CLAIM_REPO="$tmp/a" NROS_CLAIM_REMOTE="$tmp/no-such-repo.git" \
                 NROS_CLAIM_AGENT=agent-a bash "$SELF" list 2>&1)" || RC=$?
    check "list on an unreachable remote exits 3" 3

    printf '\nlist\n'
    run agent-a a list;                  check "list succeeds" 0
    check_says "…shows the live claim" "demo"
    check_says "…shows its holder" "agent-a"
    check_says "…and marks the lapsed one EXPIRED" "EXPIRED"

    printf '\n%d passed, %d failed\n' "$pass" "$fail"
    if [ "$fail" -ne 0 ]; then
        printf 'selftest: FAILED\n'
        return 1
    fi
    printf 'selftest: ok — a held id cannot be re-claimed, a live claim cannot be\n'
    printf '  stolen, a stale lease loses the steal race, a non-holder cannot renew or\n'
    printf '  release, --dry-run pushes nothing, and an unreachable remote is reported\n'
    printf '  as an environment failure rather than as contention.\n'
    return 0
}

# Read a claim object from a specific repo/remote — selftest helper, so the
# assertion about "who won the race" reads the REMOTE rather than trusting
# either racer's report.
load_claim_in() {
    local repo="$1" oid="$2" remote="$3" body rc=0
    C_AGENT=""
    [ -z "$oid" ] && return 1
    git -C "$repo" fetch --quiet "$remote" "+$REF_NS/*:$MIRROR_NS/*" >/dev/null 2>&1 || true
    body="$(git -C "$repo" cat-file commit "$oid" 2>/dev/null)" || rc=$?
    [ "$rc" -ne 0 ] && return 1
    C_BODY="$body"
    C_AGENT="$(claim_field agent)"
    return 0
}

# ---------------------------------------------------------------------- main

main() {
    local cmd="" id="" arg
    while [ "$#" -gt 0 ]; do
        arg="$1"
        case "$arg" in
            --dry-run) DRY_RUN=1 ;;
            --force)   FORCE=1 ;;
            --ttl)
                shift
                [ "$#" -gt 0 ] || die "--ttl needs a value in hours"
                case "$1" in ''|*[!0-9]*) die "--ttl takes whole hours, got '$1'" ;; esac
                TTL_SECONDS=$(( $1 * 3600 ))
                if [ "$1" -gt 24 ]; then
                    printf '[warn] a %sh lease is longer than this design intends. An open PR\n' "$1" >&2
                    printf '       supersedes the claim, so the TTL governs only the window\n' >&2
                    printf '       before FIRST PUSH — hours, not days.\n' >&2
                fi ;;
            --note)
                shift
                [ "$#" -gt 0 ] || die "--note needs a value"
                NOTE="$1" ;;
            --selftest) selftest; return $? ;;
            -h|--help) usage; return 0 ;;
            -*) die "unknown option $arg (try --help)" ;;
            *)
                if [ -z "$cmd" ]; then cmd="$arg"
                elif [ -z "$id" ]; then id="$arg"
                else die "unexpected argument $arg"
                fi ;;
        esac
        shift
    done

    case "$cmd" in
        claim)     cmd_claim "$id" ;;
        renew)     cmd_renew "$id" ;;
        release)   cmd_release "$id" ;;
        steal)     cmd_steal "$id" ;;
        list)      cmd_list ;;
        leftovers) validate_id "$id" || return 1; leftovers_report "$id" ;;
        ""|help)   usage; return 1 ;;
        *)         printf '[FAIL] unknown command %q\n' "$cmd" >&2; usage; return 1 ;;
    esac
}

main "$@"
