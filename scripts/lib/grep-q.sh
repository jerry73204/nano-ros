# shellcheck shell=bash
# Sourced, not executed — hence no shebang.
# nros_grep_q — `grep -q` that cannot report a tool failure as a finding.
#
# Issue 0726. `grep` exits 1 for "no match" and >=2 for an ERROR, and the
# natural spellings cannot tell them apart:
#
#     if ! printf '%s' "$text" | grep -q "$pat"; then   # error => false finding
#     if   printf '%s' "$text" | grep -q "$pat"; then   # error => check skipped
#
# Both are wrong and they fail in OPPOSITE directions. The first was a real
# defect in `check-rmw-force-link-anchor`: under a 32-way gate fan-out a forked
# grep failed to start, and the gate reported a missing force-link anchor for an
# example that has one — a confident, specific, false claim about the source
# tree. It only ever failed green->red under load, which is the direction that
# teaches people to re-run a gate rather than believe it.
#
# Usage mirrors `grep -q`, and the caller branches on the STATUS:
#
#     nros_grep_q "$pattern" "$file"          # or: … <<<"$text"
#     case $? in
#         0) : ;;                             # matched
#         1) report_the_finding ;;            # genuinely absent
#     esac                                    # >=2 never returns — it exits 2
#
# A tool failure exits the whole script with 2 rather than returning, because
# every caller of this helper is a checker: continuing past a grep that did not
# run means producing a verdict from missing evidence, and there is no useful
# way for a caller to "handle" that.
#
# FEED IT A HERE-STRING. NEVER A PIPE. Issue 1077 — this helper does not make
# `printf '%s' "$text" | nros_grep_q PAT` safe, and the pipe is a SEPARATE
# defect from the one above:
#
#   * Bash's builtin `printf` flushes per LINE, and a `-q` grep exits at the
#     FIRST match. So whenever the needle is on any line but the last, grep can
#     close the read end while the writer still has a line to send. The writer
#     takes SIGPIPE, and `set -o pipefail` makes that 141 the PIPELINE's status
#     — so `if ! …` reads a SUCCESSFUL match as a MISS. Traced:
#
#         write(1, "    demo/msg/Open (unbounded)\n", 30) = -1 EPIPE
#         --- SIGPIPE ---  +++ killed by SIGPIPE +++   PIPESTATUS=(141 0)
#
#     Measured at 60 of 3000 calls (2%) on a three-line haystack on an IDLE
#     12-core host, and at 13 of 300 runs of the gate that carried it, against
#     0 of 300 with the here-string. That was `check-fast` flaking on four
#     different gates over four runs, each flake wearing a considered assertion
#     message pointing at real code — the shape issue 0876 names, where a red
#     is not a verdict and is articulate enough to survive review.
#
#   * A pipeline element is a SUBSHELL, so the `exit 2` above ends only that
#     subshell. The fatal path cannot stop the caller, which is the whole point
#     of routing through this helper.
#
# So: `nros_grep_q PAT <<<"$text"`, or `nros_grep_q PAT "$file"`. Gated by
# `check-pipefail-sigpipe-assertions`.
#
# FLAGS. Leading `-…` arguments are passed through, so `grep -qi` / `grep -qE` /
# `grep -qF` convert without rephrasing the pattern:
#
#     nros_grep_q -E 'a|b' "$file"
#     nros_grep_q -qiE 'a|b' "$file"          # a literal q in the bundle is fine
#     nros_grep_q -F -- "$maybe_dashy" "$f"   # `--` ends the flags
#
# Only flags that take NO separate operand are accepted, and the allowlist is
# closed: `-e`, `-f`, `-m`, `-A/-B/-C` and every unknown long option are a FATAL
# error rather than a guess. Guessing is the failure this file exists to remove
# — `nros_grep_q -m 1 foo f` under a permissive parser would search for `1` in
# `foo` and report a confident answer about the wrong thing.
#
# A PATTERN that may begin with `-` must be preceded by `--`; without it the
# flag scanner sees it first. Every pattern in the tree today is a literal or a
# variable that cannot start with `-`, and the fatal path catches the rest.
_nros_grep_q_flag_ok() {
    case "$1" in
        # Long options that take no operand. Deliberately short: a long option
        # not listed here is fatal, which is the safe direction.
        --ignore-case | --extended-regexp | --fixed-strings | --basic-regexp \
            | --perl-regexp | --invert-match | --word-regexp | --line-regexp \
            | --no-messages | --text | --recursive | --dereference-recursive \
            | --null-data | --quiet | --silent)
            return 0
            ;;
        --*) return 1 ;;
    esac
    local rest="${1#-}"
    [ -n "$rest" ] || return 1
    while [ -n "$rest" ]; do
        # Short flags that take no operand. `q` is allowed so a converted site
        # may keep the `-qE` spelling it had; `-q` is passed to grep anyway.
        case "${rest:0:1}" in
            [EFGPUZabchIiLlnoqRrsUvwxyz]) ;;
            *) return 1 ;;
        esac
        rest="${rest:1}"
    done
    return 0
}

_nros_grep_q_bad_flag() {
    echo "FATAL: $1: unrecognised leading argument: $2" >&2
    echo "       Only flags taking NO separate operand are passed through." >&2
    echo "       -e/-f/-m/-A/-B/-C and unknown long options are refused, and a" >&2
    echo "       PATTERN starting with '-' must be preceded by '--' — because" >&2
    echo "       misreading one as the other is a confident wrong answer, which" >&2
    echo "       is exactly what this helper exists to prevent (issue 0726)." >&2
    exit 2
}

nros_grep_q() {
    local flags=()
    while [ "$#" -gt 0 ]; do
        case "$1" in
            --)
                shift
                break
                ;;
            -) break ;; # `-` is grep's stdin FILE, not a flag
            -*)
                _nros_grep_q_flag_ok "$1" \
                    || _nros_grep_q_bad_flag nros_grep_q "$1"
                flags+=("$1")
                shift
                ;;
            *) break ;;
        esac
    done
    local pat="${1:?nros_grep_q: pattern}"
    shift
    local rc
    grep -q ${flags[@]+"${flags[@]}"} -- "$pat" "$@"
    rc=$?
    if [ "$rc" -ge 2 ]; then
        echo "FATAL: grep failed (rc=$rc) searching for: $pat" >&2
        echo "       This is a TOOL failure, not a finding. Refusing to draw a" >&2
        echo "       conclusion from a grep that did not run (issue 0726)." >&2
        exit 2
    fi
    return "$rc"
}

# nros_git_grep_q — the same contract over `git grep`.
#
# Not a wrapper detail: `git grep` exits 1 for "no match" and 128 for a fatal
# (bad pathspec, not a repository, a broken index), so it carries the identical
# conflation, and a gate that greps the tree with it is exactly the shape issue
# 0726 was about. It cannot share `nros_grep_q` because `--` means something
# ELSE here — it separates the pattern from the PATHSPECS, not options from
# operands — so the pattern is passed as `-e` and the caller's `-- <path>…`
# survives untouched:
#
#     nros_git_grep_q -E 'pub fn run\b' -- "$dir/src"
nros_git_grep_q() {
    local flags=()
    while [ "$#" -gt 0 ]; do
        case "$1" in
            --)
                shift
                break
                ;;
            -*)
                _nros_grep_q_flag_ok "$1" \
                    || _nros_grep_q_bad_flag nros_git_grep_q "$1"
                flags+=("$1")
                shift
                ;;
            *) break ;;
        esac
    done
    local pat="${1:?nros_git_grep_q: pattern}"
    shift
    local rc
    git grep -q ${flags[@]+"${flags[@]}"} -e "$pat" "$@"
    rc=$?
    if [ "$rc" -ge 2 ]; then
        echo "FATAL: git grep failed (rc=$rc) searching for: $pat" >&2
        echo "       This is a TOOL failure, not a finding. Refusing to draw a" >&2
        echo "       conclusion from a git grep that did not run (issue 0726)." >&2
        exit 2
    fi
    return "$rc"
}

# nros_grep_count — the COUNTING sibling of `nros_grep_q`, same hazard.
#
# `grep -c` has the identical 0 / 1 / >=2 status split, and the idiom it invites
# is worse than the `grep -q` one because it corrupts the VALUE too:
#
#     n=$(grep -c "$pat" "$f" 2>/dev/null || true)
#
# On no-match, `grep -c` prints `0` and exits 1, so the `|| true` is there for a
# reason. But on an ERROR — file missing, unreadable, fork failure — it prints
# NOTHING and exits 2, and the same `|| true` swallows that too. The caller gets
# `n=""`, and `[ "$n" -gt 0 ]` then dies with "integer expression expected"
# pointing at the comparison rather than at the missing file, or, in a script
# without `set -e`, treats an absent log as "zero messages received" and reports
# a delivery failure that never happened. Issue 0726's class, one shape over.
#
# NOTE THE CALLING CONVENTION, and why it is not `n=$(nros_grep_count …)`: an
# `exit` inside a command substitution ends only the SUBSHELL. Written that way
# the fatal path cannot stop the caller — it returns the empty string and the
# script sails on, which is precisely the bug this helper exists to remove.
# (Confirmed by writing it wrong first.) So the count comes back through a
# named variable and the fatal path runs in the caller's own shell:
#
#     nros_grep_count n "$pattern" "$file"      # sets $n; exits 2 on tool failure
#     [ "$n" -gt 0 ] || report_no_delivery
nros_grep_count() {
    local __out_var="${1:?nros_grep_count: output variable name}"
    local pat="${2:?nros_grep_count: pattern}"
    shift 2
    local out rc
    out=$(grep -c -- "$pat" "$@" 2>/dev/null)
    rc=$?
    if [ "$rc" -ge 2 ]; then
        echo "FATAL: grep -c failed (rc=$rc) counting: $pat" >&2
        echo "       This is a TOOL failure, not a count of zero. Refusing to" >&2
        echo "       report a number nothing produced (issue 0726)." >&2
        exit 2
    fi
    printf -v "$__out_var" '%s' "${out:-0}"
    return 0
}
