#!/bin/bash
# Test: Zephyr C Examples (native_sim)
#
# This test verifies that the nros C API examples (zephyr-c-talker)
# running on Zephyr RTOS (native_sim) can successfully publish messages
# via zenoh.
#
# Prerequisites:
#   - Zephyr workspace set up (./scripts/zephyr/setup.sh)
#   - zenohd installed (native_sim uses NSOS on host loopback — no TAP bridge)
#
# Usage:
#   ./tests/zephyr/run-c.sh
#   ./tests/zephyr/run-c.sh --verbose
#   ./tests/zephyr/run-c.sh --skip-build
#
# Note: This test requires the Zephyr workspace to be initialized.
# Run ./scripts/zephyr/setup.sh first if not done.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# issue 0660 — the router comes from ROS, not from a vendored build.
#
# phase-362 W4 deleted `build/zenohd` and the recipe that produced it (RFC-0075:
# `rmw_zenohd` ships with `rmw_zenoh_cpp` and links the same `libzenohc.so` the
# RMW does, so it cannot drift from it). This script kept pointing at the
# deleted path.
#
# issue 0654 — resolution is `nros_zenohd_bin`, not a private copy of it.
#
# This carried its own three-step resolver, written to "mirror" the Rust one.
# It had already drifted: it still globs `/opt/ros/*` and takes the newest name,
# which issue 0653 REMOVED from both real resolvers precisely because it returns
# a distro the user never chose — on a host with humble and jazzy installed it
# picks jazzy by collation, and both are genuine ROS routers, so nothing about
# the answer looks wrong. It also never learned `AMENT_PREFIX_PATH`, so a ROS
# built from source or a colcon overlay resolved nothing here while working
# everywhere else.
#
# A mirror maintained by hand is a third implementation; sourcing the shared one
# is the whole point of it existing.
# shellcheck source=../../scripts/dev/zenohd.sh
. "$(dirname "$0")/../../scripts/dev/zenohd.sh"
ZENOHD="$(nros_zenohd_bin)" || ZENOHD=""

# =============================================================================
# phase-277 W2.a — mirror of the talker/listener log-line prefixes.
#
# Shell can't import the Rust constants, so these are a manually-kept mirror
# of `TALKER_LOG_PREFIX` / `LISTENER_LOG_PREFIX` in
# packages/testing/nros-tests/src/output.rs (the source of truth). If a
# future wording flip changes those constants, update this mirror too.
# =============================================================================
TALKER_LOG_PREFIX="Publishing:"
# shellcheck disable=SC2034  # kept for mirror completeness; this script only checks the talker side
LISTENER_LOG_PREFIX="I heard:"

# =============================================================================
# Utilities (shared via tests/lib/common.sh)
# =============================================================================

# shellcheck source=../lib/common.sh
source "$PROJECT_ROOT/tests/lib/common.sh"

init_test_tmpdir "nano-ros-zephyr-test"

# Cleanup function - kills all registered processes and removes tmpdir.
cleanup() {
    log_info "Cleaning up..."
    cleanup_pids
    # Also kill by name as fallback
    pkill -x zenohd 2>/dev/null || true
    cleanup_test_tmpdir
    CLEANUP_PIDS=()
}

# Setup trap for cleanup
setup_cleanup() {
    trap cleanup EXIT INT TERM
}

# =============================================================================
# Script Configuration
# =============================================================================

# Parse arguments
VERBOSE=false
SKIP_BUILD=false
while [[ $# -gt 0 ]]; do
    case $1 in
        --verbose|-v) VERBOSE=true; shift ;;
        --skip-build) SKIP_BUILD=true; shift ;;
        *) shift ;;
    esac
done

# Configuration - workspace location (via symlink or env var)
# Priority: 1. ZEPHYR_NANO_ROS env var, 2. zephyr-workspace symlink, 3. sibling workspace
if [ -n "${ZEPHYR_NANO_ROS:-}" ]; then
    ZEPHYR_WORKSPACE="$ZEPHYR_NANO_ROS"
elif [ -L "$PROJECT_ROOT/zephyr-workspace" ]; then
    ZEPHYR_WORKSPACE="$(readlink -f "$PROJECT_ROOT/zephyr-workspace")"
else
    NANO_ROS_NAME="$(basename "$PROJECT_ROOT")"
    ZEPHYR_WORKSPACE="$(dirname "$PROJECT_ROOT")/${NANO_ROS_NAME}-workspace"
fi
TEST_TIMEOUT=15

setup_cleanup

log_header "Zephyr C Examples Test (native_sim)"

# =============================================================================
# Prerequisites Check
# =============================================================================

check_zephyr_prerequisites() {
    log_header "Checking Zephyr Prerequisites"

    local missing=0

    # Check Zephyr workspace
    if [ -d "$ZEPHYR_WORKSPACE" ]; then
        log_success "Zephyr workspace found: $ZEPHYR_WORKSPACE"

        # Check for zephyr subdirectory
        if [ -d "$ZEPHYR_WORKSPACE/zephyr" ]; then
            log_success "Zephyr SDK found"
        else
            log_error "Zephyr SDK not found in workspace"
            missing=1
        fi
    else
        log_error "Zephyr workspace not found at $ZEPHYR_WORKSPACE"
        log_info "Run: ./scripts/zephyr/setup.sh"
        missing=1
    fi

    # Check west
    if command -v west &>/dev/null; then
        log_success "west found: $(which west)"
    else
        log_error "west not found"
        missing=1
    fi

    # Check zenohd
    # NOT `--version`: `rmw_zenohd` ignores its argv and would START a router
    # (the fixture's own comment records that `--help` starts one). Test for an
    # executable file instead.
    if [ -n "$ZENOHD" ] && [ -x "$ZENOHD" ]; then
        log_success "zenohd found: $ZENOHD"
    else
        log_error "zenohd not found"
        log_info "Build with: just build-zenohd"
        missing=1
    fi

    # Check for existing build
    if [ -f "$ZEPHYR_WORKSPACE/build-c-talker/zephyr/zephyr.exe" ]; then
        log_success "Zephyr C talker executable found"
    else
        log_info "Zephyr C talker executable not found (will build)"
    fi

    return $missing
}

# =============================================================================
# Build Zephyr Examples
# =============================================================================

build_zephyr_examples() {
    log_header "Building Zephyr Examples"

    cd "$ZEPHYR_WORKSPACE"

    # Source environment
    if [ -f ".venv/bin/activate" ]; then
        # shellcheck source=/dev/null
        source .venv/bin/activate
    fi
    if [ -f "zephyr/zephyr-env.sh" ]; then
        # shellcheck source=/dev/null
        source zephyr/zephyr-env.sh
    fi
    export ZEPHYR_BASE="$ZEPHYR_WORKSPACE/zephyr"

    # Determine example path based on workspace type
    # In-tree workspace: examples at nros/examples/
    # External workspace: examples at $PROJECT_ROOT/examples/
    local example_path
    if [ -d "nros/examples/zephyr/c/talker" ]; then
        example_path="nros/examples/zephyr/c/talker"
    elif [ -d "$PROJECT_ROOT/examples/zephyr/c/talker" ]; then
        example_path="$PROJECT_ROOT/examples/zephyr/c/talker"
    else
        log_error "Could not find c-talker example"
        log_info "Expected at: nros/examples/zephyr/c/talker or $PROJECT_ROOT/examples/zephyr/c/talker"
        return 1
    fi

    # Build C talker for native_sim/native/64
    log_info "Building zephyr-c-talker for native_sim/native/64..."
    if west build -b native_sim/native/64 "$example_path" -d build-c-talker -p auto -- -DCONF_FILE="prj.conf;prj-zenoh.conf" 2>&1 | tee "$(tmpfile zephyr_build.txt)" | tail -10; then
        log_success "Talker build complete"
    else
        log_error "Talker build failed"
        [ "$VERBOSE" = true ] && cat "$(tmpfile zephyr_build.txt)"
        return 1
    fi

    return 0
}

# =============================================================================
# Test: Zephyr Talker -> Native Subscriber
# =============================================================================

test_zephyr_to_native() {
    log_header "Test: Zephyr C Talker"

    # Start zenoh router
    log_info "Starting zenoh router..."
    pkill -x zenohd 2>/dev/null || true
    sleep 1
    # Zephyr C pubsub examples use port 7556 (lang_stride = 100, C = Rust+100).
    # `rmw_zenohd` takes NO command-line configuration — it reads
    # `ZENOH_CONFIG_OVERRIDE`, `;`-separated, with `=` where the CLI used `:`.
    # Same translation `fixtures::zenohd_router::router_command` documents and
    # verified against the installed binary there:
    #   --listen tcp/127.0.0.1:7556  ->  listen/endpoints=["tcp/127.0.0.1:7556"]
    #   --no-multicast-scouting      ->  scouting/multicast/enabled=false
    ZENOH_CONFIG_OVERRIDE='scouting/multicast/enabled=false;listen/endpoints=["tcp/127.0.0.1:7556"]' \
        "$ZENOHD" > "$(tmpfile zephyr_zenohd.txt)" 2>&1 &
    local zenohd_pid=$!
    register_pid $zenohd_pid
    sleep 2

    if ! kill -0 $zenohd_pid 2>/dev/null; then
        log_error "Failed to start zenohd"
        cat "$(tmpfile zephyr_zenohd.txt)"
        return 1
    fi
    log_success "zenohd started (PID: $zenohd_pid)"

    # Start Zephyr C talker
    log_info "Starting Zephyr C talker..."
    cd "$ZEPHYR_WORKSPACE"
    timeout "$TEST_TIMEOUT" ./build-c-talker/zephyr/zephyr.exe > "$(tmpfile zephyr_talker.txt)" 2>&1 &
    local zephyr_pid=$!
    register_pid $zephyr_pid

    # Wait for messages to be published
    log_info "Waiting for Zephyr to publish messages (timeout: ${TEST_TIMEOUT}s)..."

    local elapsed=0
    while [ $elapsed -lt $TEST_TIMEOUT ]; do
        # Check if talker has published at least 3 messages
        local count
        count=$(grep -c "$TALKER_LOG_PREFIX" "$(tmpfile zephyr_talker.txt)" 2>/dev/null | head -1 || echo "0")
        count="${count:-0}"
        if [ "$count" -ge 3 ] 2>/dev/null; then
            break
        fi
        # Check for errors
        if nros_grep_q "Failed to" "$(tmpfile zephyr_talker.txt)" 2>/dev/null; then
            break
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done

    # Check results
    local pub_count
    pub_count=$(grep -c "$TALKER_LOG_PREFIX" "$(tmpfile zephyr_talker.txt)" 2>/dev/null || echo 0)

    if [ "$pub_count" -ge 3 ]; then
        log_success "Zephyr C talker published $pub_count messages successfully!"

        if [ "$VERBOSE" = true ]; then
            echo ""
            echo "=== Zephyr Talker Output ==="
            head -30 "$(tmpfile zephyr_talker.txt)"
        fi
        return 0
    else
        log_error "Zephyr C talker failed to publish messages"
        echo ""
        echo "=== Zephyr Output ==="
        cat "$(tmpfile zephyr_talker.txt)" 2>/dev/null | head -40
        echo ""
        echo "=== zenohd Output ==="
        cat "$(tmpfile zephyr_zenohd.txt)" 2>/dev/null | tail -10
        return 1
    fi
}

# =============================================================================
# Main
# =============================================================================

RESULT=0

# Check prerequisites
if ! check_zephyr_prerequisites; then
    log_error "Prerequisites not met"
    log_info ""
    log_info "To set up the Zephyr workspace:"
    log_info "  ./scripts/zephyr/setup.sh"
    exit 1
fi

# Build examples
if [ "$SKIP_BUILD" = false ]; then
    if ! build_zephyr_examples; then
        log_error "Build failed"
        exit 1
    fi
fi

# Run test
test_zephyr_to_native || RESULT=1

# Summary
log_header "Test Summary"
if [ $RESULT -eq 0 ]; then
    log_success "Zephyr C examples test passed!"
else
    log_error "Zephyr C examples test failed"
    log_info ""
    log_info "Troubleshooting:"
    log_info "  1. Check TAP interface: ip addr show $TAP_INTERFACE"
    log_info "  2. Check zenohd is accessible: ZENOH_CONFIG_OVERRIDE='listen/endpoints=["tcp/0.0.0.0:7447"];scouting/multicast/enabled=false' ros2 run rmw_zenoh_cpp rmw_zenohd"
    log_info "  3. Check Zephyr can reach host: ping $HOST_IP (from Zephyr)"
    log_info "  4. Run with --verbose for detailed output"
fi

exit $RESULT
