#!/usr/bin/env bash
# Gate: the activation files must run to COMPLETION in every shell they claim
# to support, on a host whose SDK store is empty AND on one whose store holds
# only the versioned layout.
#
# Why this exists (issue 0372): `activate.sh` advertises "POSIX shell (bash /
# zsh)" and "the script never errors", but two unmatched SDK-store globs made
# zsh abort it mid-file — at line 92 before `nros setup`, at line 115 after it
# — silently dropping every export below the failure (the SDK PATH loop that
# wires the SDK store, pinned ninja/make, .env, sdk-env.sh). No lane sourced these
# files under anything but bash, so it went unnoticed. Anything that reaches
# the shell's word expansion as a glob can regress this; the check is cheap,
# so it runs in check-fast.
#
# zsh / fish are OPTIONAL: absent interpreters are skipped loudly, never
# treated as passes for a shell that was never run.

set -uo pipefail

REPO_ROOT="$(cd -P "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
# issue 0726 — every conditional below is `if ! … grep -q …`, the shape that
# reads a grep which failed to START as "the activation never delivered this".
# `nros_grep_q` exits 2 there instead. The searches are HERESTRINGS rather than
# `printf … | grep`: a pipeline puts the helper in a subshell, where its `exit`
# ends only that subshell and hands the caller back the ambiguity it just fixed.
# shellcheck source=scripts/lib/grep-q.sh
source "$REPO_ROOT/scripts/lib/grep-q.sh"
FAILURES=0
RAN=0

# Issue 0451 — running to completion is NOT the same as delivering the
# variables, and the difference was hiding a live defect in every shell: the
# SDK block completed everywhere while exporting 14 of 23 under bash, 2 under
# fish and 0 under zsh. The list is READ from the SSoT, never restated here.
SSOT_VARS="$(sed -n 's/^export \([A-Za-z_][A-Za-z0-9_]*\)[[:space:]]*:=.*/\1/p' \
    "$REPO_ROOT/just/sdk-env.just" | tr '\n' ' ')"
if [ -z "${SSOT_VARS// /}" ]; then
    echo "FAIL: no exports parsed from just/sdk-env.just — the gate would pass vacuously" >&2
    exit 1
fi

fail() {
    echo "FAIL: $*" >&2
    FAILURES=$((FAILURES + 1))
}

# A completed activation is one that (a) exits 0, (b) leaves NROS_REPO_DIR set
# to the checkout, and (c) reached the LAST statement of the file — proven by
# `_nros_root` being unset again, which only happens on the final lines.
# Sentinel output is a single line so any shell can emit it identically.
#
# The SSoT-variable check emits its own `PROBEVARS` line rather than extending
# the `PROBE` one, so the existing end-anchored `root_unset=$` match keeps
# working.
PROBE_SH='
printf "PROBE repo=%s root_unset=%s\n" "${NROS_REPO_DIR:-}" "${_nros_root:+set}"
_nros_miss=""
for _nros_v in '"$SSOT_VARS"'; do
    eval "[ -n \"\${$_nros_v+x}\" ]" || _nros_miss="$_nros_miss $_nros_v"
done
printf "PROBEVARS missing=%s\n" "$_nros_miss"
'

PROBE_FISH='
printf "PROBE repo=%s root_unset=%s\n" "$NROS_REPO_DIR" (set -q _nros_root; and echo set; or echo "")
set -l _nros_miss ""
for _nros_v in '"$SSOT_VARS"'
    if not set -q $_nros_v
        set _nros_miss "$_nros_miss $_nros_v"
    end
end
printf "PROBEVARS missing=%s\n" "$_nros_miss"
'

# Issue 0451 — each probe runs in a CLEARED environment (`env -i`, keeping only
# HOME and PATH). A maintainer host almost always has direnv active, so the
# probe would otherwise INHERIT the very variables it is checking for and pass
# no matter what activation did. Measured: 22 of 23 arrived that way.
#
# $1 shell name, $2 store dir, $3 label
run_case() {
    local shell="$1" store="$2" label="$3" out rc
    if ! command -v "$shell" >/dev/null 2>&1; then
        echo "SKIP: $shell not installed — $label not covered on this host" >&2
        return 0
    fi
    RAN=$((RAN + 1))

    case "$shell" in
        fish)
            out="$(env -i HOME="$HOME" PATH="$PATH" \
                NROS_HOME="$store" NROS_QUIET_ACTIVATE=1 "$shell" -c \
                "source '$REPO_ROOT/activate.fish'; $PROBE_FISH" 2>&1)"
            ;;
        *)
            out="$(env -i HOME="$HOME" PATH="$PATH" \
                NROS_HOME="$store" NROS_QUIET_ACTIVATE=1 "$shell" -c \
                ". '$REPO_ROOT/activate.sh'; $PROBE_SH" 2>&1)"
            ;;
    esac
    rc=$?

    local probe
    probe="$(printf '%s\n' "$out" | grep '^PROBE ' || true)"

    if [ "$rc" -ne 0 ]; then
        fail "$shell/$label: activation exited $rc"
        printf '%s\n' "$out" | sed 's/^/    /' >&2
        return 0
    fi
    if [ -z "$probe" ]; then
        fail "$shell/$label: activation never reached the end of the file"
        printf '%s\n' "$out" | sed 's/^/    /' >&2
        return 0
    fi
    if ! nros_grep_q "repo=$REPO_ROOT " <<<"$probe"; then
        fail "$shell/$label: NROS_REPO_DIR wrong or unset ($probe)"
        return 0
    fi
    if ! nros_grep_q 'root_unset=$' <<<"$probe"; then
        # `_nros_root` is unset by the file's last lines, so a shell that still
        # has it never got there — the file aborted partway (issue 0372's exact
        # symptom) or grew an early return. Note a zsh `nomatch` abort ends the
        # SOURCED file only: the outer shell keeps going and still exits 0, so
        # this sentinel — not the exit status — is what catches the regression.
        fail "$shell/$label: activation stopped before its final lines ($probe)"
        printf '%s\n' "$out" | sed 's/^/    /' >&2
        return 0
    fi
    # Issue 0451 — did activation actually DELIVER the SSoT variables? A shell
    # can reach the last line with the whole SDK block having silently done
    # nothing: zsh reported `bad substitution` on bash-only indirect expansion
    # and carried on, fish imported only names matching `NROS_*`.
    local probevars missing
    probevars="$(printf '%s\n' "$out" | grep '^PROBEVARS ' || true)"
    if [ -z "$probevars" ]; then
        fail "$shell/$label: activation never reported its SDK variables"
        return 0
    fi
    missing="${probevars#PROBEVARS missing=}"
    if [ -n "${missing// /}" ]; then
        fail "$shell/$label: activation left these just/sdk-env.just variables unset:${missing}"
        echo "    Every var exported there must survive activation in every supported" >&2
        echo "    shell; otherwise an embedded build fails much later on an unset path" >&2
        echo "    and reads like a code fault (issue 0451)." >&2
        return 0
    fi

    # An unmatched-glob abort in a shell that reports it non-fatally would still
    # show up here, so treat the message itself as a failure regardless of rc.
    if nros_grep_q -i 'no matches\|bad pattern' <<<"$out"; then
        fail "$shell/$label: glob reached word expansion"
        printf '%s\n' "$out" | sed 's/^/    /' >&2
        return 0
    fi
    echo "ok: $shell/$label"
}

tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT

# Case 1 — empty store: the fresh-machine state, before any `nros setup`.
empty_store="$tmp/empty"
mkdir -p "$empty_store"

# Case 2 — versioned store only: the state `nros setup` actually leaves behind
# (sdk/<tool>/<version>/bin), which is what made the second glob site fatal.
versioned_store="$tmp/versioned"
# `espflash`, not `zenohd`: the router was retired with phase-362 / RFC-0075 and
# is off `scripts/sdk-path-tools.txt` (issue 0653), so a fixture built on it
# would assert that a name nothing wires any more reaches PATH — a gate proving
# the opposite of the rule. The tool here must be one the list still carries.
mkdir -p "$versioned_store/sdk/espflash/4.5.0-nros1/bin"
: >"$versioned_store/sdk/espflash/4.5.0-nros1/bin/espflash"
chmod +x "$versioned_store/sdk/espflash/4.5.0-nros1/bin/espflash"
mkdir -p "$versioned_store/sdk/play_launch_parser/0.5.0/bin"
: >"$versioned_store/sdk/play_launch_parser/0.5.0/bin/play_launch_parser"
chmod +x "$versioned_store/sdk/play_launch_parser/0.5.0/bin/play_launch_parser"

for sh in bash zsh fish; do
    run_case "$sh" "$empty_store" "empty-store"
    run_case "$sh" "$versioned_store" "versioned-store"
done

# The versioned store must actually put its bin dirs on PATH — a fix that
# silences the abort by skipping the lookup would otherwise pass the checks
# above. bash is mandatory, so assert the wiring there.
path_out="$(NROS_HOME="$versioned_store" NROS_QUIET_ACTIVATE=1 bash -c \
    ". '$REPO_ROOT/activate.sh'; printf '%s' \"\$PATH\"" 2>/dev/null)"
for tool in espflash play_launch_parser; do
    if ! nros_grep_q "^$versioned_store/sdk/$tool/" <<<"${path_out//:/$'\n'}"; then
        fail "activate.sh: versioned $tool bin dir never reached PATH"
    fi
done

# phase-431 W3 — and a STORE-INSTALLED `nros` must NOT reach PATH.
#
# The store gained a `[tool.nros]` entry, so from now on a contributor's machine
# can hold a released `nros` beside the checkout's own. `activate.sh` only adds a
# store bin dir for the names in `scripts/sdk-path-tools.txt`, and `nros` is
# deliberately absent from it: that list exists for binaries something we do not
# control invokes by BARE NAME (an RTOS `make`, a cmake `find_program`), and the
# CLI is not one of those. If it were on the list, sourcing activate.sh in a
# checkout would put a foreign emitter first on PATH — which `nros build` now
# refuses (W1) and `just doctor` now fails on (W2). Better that it never
# happens than that two other checks catch it.
#
# The user reaches the released binary through `$NROS_HOME/bin` (the `front`
# link), which is a directory THEY put on PATH, not one activate.sh adds.
mkdir -p "$versioned_store/sdk/nros/0.5.0-nros1/bin"
: >"$versioned_store/sdk/nros/0.5.0-nros1/bin/nros"
chmod +x "$versioned_store/sdk/nros/0.5.0-nros1/bin/nros"
path_out="$(NROS_HOME="$versioned_store" NROS_QUIET_ACTIVATE=1 bash -c \
    ". '$REPO_ROOT/activate.sh'; printf '%s' \"\$PATH\"" 2>/dev/null)"
if nros_grep_q "^$versioned_store/sdk/nros/" <<<"${path_out//:/$'\n'}"; then
    fail "activate.sh: a STORE-installed nros reached PATH — it would shadow the checkout's CLI (phase-431 W2/W3)"
else
    echo "ok: bash/store-nros-does-not-shadow"
fi
RAN=$((RAN + 1))

if [ "$RAN" -eq 0 ]; then
    fail "no shell was exercised"
fi

if [ "$FAILURES" -ne 0 ]; then
    echo "check-activate-shells: $FAILURES failure(s)" >&2
    exit 1
fi
echo "Activation files run to completion in every available shell."
