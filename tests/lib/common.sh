#!/bin/bash
# Shared logging + cleanup helpers for nano-ros shell-based tests.
#
# Source this from a test script:
#
#     SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
#     source "$SCRIPT_DIR/lib/common.sh"   # adjust path to reach tests/lib/
#
# Provides:
#   - Color constants (RED/GREEN/YELLOW/BLUE/CYAN/NC)
#   - Logging:  log_info / log_success / log_warn / log_error / log_header
#   - PID cleanup registry: register_pid <pid>; cleanup_pids
#   - Auto-mktemp tmpdir + tmpfile helper:  init_test_tmpdir <prefix>; tmpfile <name>
#
# The script does NOT install a `trap`. Callers register their own
# cleanup handler so they can mix in extra steps (kill named binaries,
# unmount, etc.) without fighting an inherited trap.

# issue 1173 — `nros_grep_q` for every consumer, from the one place they all
# already source. `grep -q` exits on the match, so the writer of a
# `printf … | grep -q …` takes SIGPIPE, and `set -o pipefail` — which every
# script here sets — then reports that as the CONDITION. It only ever fires
# green->red, under load, in the gate fan-out: `check-reconfigure-on-change`
# claimed "the bound was hit silently" while printing the very warning it was
# grepping for. `grep`'s error/no-match status conflation (issue 0726) is the
# same helper's other half.
#
# Sourced here rather than per script because the population that matters is
# "the scripts a check-fast gate invokes", and these are all of them.
# shellcheck source=scripts/lib/grep-q.sh
source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)/scripts/lib/grep-q.sh"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

# Logging
log_info()    { echo -e "${BLUE}[INFO]${NC} $*"; }
log_success() { echo -e "${GREEN}[PASS]${NC} $*"; }
log_warn()    { echo -e "${YELLOW}[WARN]${NC} $*"; }
log_error()   { echo -e "${RED}[FAIL]${NC} $*"; }
log_header()  { echo -e "\n${CYAN}=== $* ===${NC}"; }

# PID registry — caller invokes cleanup_pids from its own trap handler.
declare -a CLEANUP_PIDS=()

register_pid() {
    CLEANUP_PIDS+=("$1")
}

cleanup_pids() {
    for pid in "${CLEANUP_PIDS[@]}"; do
        if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
        fi
    done
}

# Test temp directory. `init_test_tmpdir <prefix>` creates a unique dir
# under $TMPDIR (or /tmp) and exports its path as TEST_TMPDIR. `tmpfile
# <name>` returns a path inside it.
init_test_tmpdir() {
    local prefix="${1:-nano-ros-test}"
    TEST_TMPDIR="$(mktemp -d "${TMPDIR:-/tmp}/${prefix}.XXXXXX")"
    export TEST_TMPDIR
}

cleanup_test_tmpdir() {
    if [ -n "${TEST_TMPDIR:-}" ] && [ -d "$TEST_TMPDIR" ]; then
        rm -rf "$TEST_TMPDIR"
    fi
}

tmpfile() {
    echo "${TEST_TMPDIR}/$1"
}
