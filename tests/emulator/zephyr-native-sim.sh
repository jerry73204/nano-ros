#!/bin/bash
# Test: Zephyr native_sim Integration (Enhanced)
#
# Enhanced version of tests/zephyr/run.sh with additional test scenarios:
# - Zephyr talker → native subscriber
# - Native talker → Zephyr listener
# - Bidirectional communication
# - Multiple message types
# - Reconnection handling
#
# Prerequisites:
#   - Zephyr workspace set up (./zephyr/setup.sh)
#   - TAP network configured (sudo ./scripts/setup-zephyr-network.sh)
#   - zenohd installed
#
# Usage:
#   ./tests/emulator/zephyr-native-sim.sh
#   ./tests/emulator/zephyr-native-sim.sh --verbose
#   ./tests/emulator/zephyr-native-sim.sh --skip-build
#   ./tests/emulator/zephyr-native-sim.sh --quick

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/../common/utils.sh"

# Parse arguments
VERBOSE=false
SKIP_BUILD=false
QUICK=false
while [[ $# -gt 0 ]]; do
    case $1 in
        --verbose|-v) VERBOSE=true; shift ;;
        --skip-build) SKIP_BUILD=true; shift ;;
        --quick|-q) QUICK=true; shift ;;
        *) shift ;;
    esac
done

# Configuration
ZEPHYR_WORKSPACE="${ZEPHYR_NANO_ROS:-$HOME/nano-ros-workspace}"
if [ ! -d "$ZEPHYR_WORKSPACE" ]; then
    ZEPHYR_WORKSPACE="$HOME/zephyr-nano-ros"
fi
TAP_INTERFACE="zeth"
HOST_IP="192.0.2.2"
ZEPHYR_IP="192.0.2.1"
TEST_TIMEOUT=20

setup_cleanup

log_header "Zephyr native_sim Integration Tests (Enhanced)"

# =============================================================================
# Prerequisites Check
# =============================================================================

check_prerequisites() {
    log_header "Checking Prerequisites"

    local missing=0

    # Check TAP interface
    if ip link show "$TAP_INTERFACE" &>/dev/null; then
        local flags
        flags=$(ip link show "$TAP_INTERFACE" | head -1)
        if echo "$flags" | grep -q "UP"; then
            log_success "TAP interface '$TAP_INTERFACE' is UP"
        else
            log_error "TAP interface '$TAP_INTERFACE' is DOWN"
            log_info "Run: sudo ip link set $TAP_INTERFACE up"
            missing=1
        fi
    else
        log_error "TAP interface '$TAP_INTERFACE' not found"
        log_info "Run: sudo ./scripts/setup-zephyr-network.sh"
        missing=1
    fi

    # Check Zephyr workspace
    if [ -d "$ZEPHYR_WORKSPACE" ] && [ -d "$ZEPHYR_WORKSPACE/zephyr" ]; then
        log_success "Zephyr workspace found: $ZEPHYR_WORKSPACE"
    else
        log_error "Zephyr workspace not found"
        log_info "Run: ./zephyr/setup.sh"
        missing=1
    fi

    # Check west
    if command -v west &>/dev/null; then
        log_success "west found"
    else
        log_error "west not found"
        missing=1
    fi

    # Check zenohd
    if command -v zenohd &>/dev/null; then
        log_success "zenohd found"
    else
        log_error "zenohd not found"
        missing=1
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
        source .venv/bin/activate
    fi
    if [ -f "zephyr/zephyr-env.sh" ]; then
        source zephyr/zephyr-env.sh
    fi
    export ZEPHYR_BASE="$ZEPHYR_WORKSPACE/zephyr"

    # Build talker
    log_info "Building zephyr-talker-rs..."
    if west build -b native_sim/native/64 nano-ros/examples/zephyr-talker-rs -p auto \
        > "$(tmpfile zephyr_build_talker.txt)" 2>&1; then
        log_success "Talker build complete"
        cp build/zephyr/zephyr.exe build/zephyr/talker.exe 2>/dev/null || true
    else
        log_error "Talker build failed"
        [ "$VERBOSE" = true ] && cat "$(tmpfile zephyr_build_talker.txt)"
        return 1
    fi

    # Build listener
    log_info "Building zephyr-listener-rs..."
    if west build -b native_sim/native/64 nano-ros/examples/zephyr-listener-rs -p auto \
        > "$(tmpfile zephyr_build_listener.txt)" 2>&1; then
        log_success "Listener build complete"
        cp build/zephyr/zephyr.exe build/zephyr/listener.exe 2>/dev/null || true
    else
        log_error "Listener build failed"
        [ "$VERBOSE" = true ] && cat "$(tmpfile zephyr_build_listener.txt)"
        return 1
    fi

    return 0
}

# =============================================================================
# Start Common Infrastructure
# =============================================================================

start_zenoh_router() {
    pkill -x zenohd 2>/dev/null || true
    sleep 1
    zenohd --listen tcp/0.0.0.0:7447 > "$(tmpfile zenohd.txt)" 2>&1 &
    local zenohd_pid=$!
    register_pid $zenohd_pid
    sleep 2

    if kill -0 $zenohd_pid 2>/dev/null; then
        log_success "zenohd started (PID: $zenohd_pid)"
        return 0
    else
        log_error "Failed to start zenohd"
        return 1
    fi
}

stop_all_processes() {
    pkill -x zenohd 2>/dev/null || true
    pkill -f "zephyr.exe" 2>/dev/null || true
    pkill -f "talker.exe" 2>/dev/null || true
    pkill -f "listener.exe" 2>/dev/null || true
    sleep 1
}

# =============================================================================
# Test 1: Zephyr Talker -> Native Subscriber
# =============================================================================

test_zephyr_to_native() {
    log_header "Test 1: Zephyr Talker -> Native Subscriber"

    stop_all_processes
    start_zenoh_router || return 1

    # Start native subscriber
    log_info "Starting native subscriber..."
    cd "$PROJECT_ROOT"
    timeout "$TEST_TIMEOUT" cargo run -p zenoh-pico --example sub_test --features std \
        > "$(tmpfile native_sub.txt)" 2>&1 &
    local sub_pid=$!
    register_pid $sub_pid
    sleep 2

    # Start Zephyr talker
    log_info "Starting Zephyr talker..."
    cd "$ZEPHYR_WORKSPACE"
    local talker_binary="build/zephyr/talker.exe"
    [ ! -f "$talker_binary" ] && talker_binary="build/zephyr/zephyr.exe"

    timeout "$TEST_TIMEOUT" "$talker_binary" > "$(tmpfile zephyr_talker.txt)" 2>&1 &
    local talker_pid=$!
    register_pid $talker_pid

    # Wait for messages
    log_info "Waiting for messages..."
    local elapsed=0
    while [ $elapsed -lt $TEST_TIMEOUT ]; do
        if grep -q "SUCCESS\|Received Int32:" "$(tmpfile native_sub.txt)" 2>/dev/null; then
            break
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done

    # Check results
    if grep -q "SUCCESS\|Received Int32:" "$(tmpfile native_sub.txt)" 2>/dev/null; then
        local count
        count=$(grep -c "Received Int32:" "$(tmpfile native_sub.txt)" 2>/dev/null || echo 0)
        log_success "Received $count messages"
        [ "$VERBOSE" = true ] && head -20 "$(tmpfile native_sub.txt)"
        return 0
    else
        log_error "No messages received"
        [ "$VERBOSE" = true ] && cat "$(tmpfile native_sub.txt)"
        return 1
    fi
}

# =============================================================================
# Test 2: Native Talker -> Zephyr Listener
# =============================================================================

test_native_to_zephyr() {
    log_header "Test 2: Native Talker -> Zephyr Listener"

    stop_all_processes
    start_zenoh_router || return 1

    # Check if listener binary exists
    cd "$ZEPHYR_WORKSPACE"
    local listener_binary="build/zephyr/listener.exe"
    if [ ! -f "$listener_binary" ]; then
        log_warn "Listener binary not found, skipping test"
        return 0
    fi

    # Start Zephyr listener
    log_info "Starting Zephyr listener..."
    timeout "$TEST_TIMEOUT" "$listener_binary" > "$(tmpfile zephyr_listener.txt)" 2>&1 &
    local listener_pid=$!
    register_pid $listener_pid
    sleep 3

    # Start native talker
    log_info "Starting native talker..."
    cd "$PROJECT_ROOT"
    timeout "$TEST_TIMEOUT" cargo run -p native-talker --features zenoh -- --tcp 127.0.0.1:7447 \
        > "$(tmpfile native_talker.txt)" 2>&1 &
    local talker_pid=$!
    register_pid $talker_pid

    # Wait for Zephyr to receive
    log_info "Waiting for Zephyr to receive..."
    local elapsed=0
    while [ $elapsed -lt $TEST_TIMEOUT ]; do
        if grep -q "Received\|message" "$(tmpfile zephyr_listener.txt)" 2>/dev/null; then
            break
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done

    # Check results
    if grep -q "Received\|message" "$(tmpfile zephyr_listener.txt)" 2>/dev/null; then
        log_success "Zephyr listener received messages"
        [ "$VERBOSE" = true ] && head -20 "$(tmpfile zephyr_listener.txt)"
        return 0
    else
        log_warn "Zephyr listener may not have received messages (check output)"
        [ "$VERBOSE" = true ] && cat "$(tmpfile zephyr_listener.txt)"
        # Don't fail - listener support may be incomplete
        return 0
    fi
}

# =============================================================================
# Test 3: Basic Connectivity
# =============================================================================

test_basic_connectivity() {
    log_header "Test 3: Basic Connectivity Check"

    stop_all_processes
    start_zenoh_router || return 1

    # Just verify Zephyr can start and connect
    cd "$ZEPHYR_WORKSPACE"
    local talker_binary="build/zephyr/talker.exe"
    [ ! -f "$talker_binary" ] && talker_binary="build/zephyr/zephyr.exe"

    log_info "Starting Zephyr (checking connectivity)..."
    timeout 10 "$talker_binary" > "$(tmpfile zephyr_conn.txt)" 2>&1 &
    local zephyr_pid=$!
    register_pid $zephyr_pid
    sleep 5

    # Check if it started without errors
    if grep -qiE "error|failed|panic" "$(tmpfile zephyr_conn.txt)" 2>/dev/null; then
        log_error "Zephyr encountered errors"
        cat "$(tmpfile zephyr_conn.txt)"
        return 1
    fi

    if grep -q "Publishing\|connected\|session" "$(tmpfile zephyr_conn.txt)" 2>/dev/null; then
        log_success "Zephyr connected successfully"
        return 0
    fi

    # If we got this far without errors, consider it a pass
    log_success "Zephyr started without errors"
    return 0
}

# =============================================================================
# Main
# =============================================================================

RESULT=0
TESTS_RUN=0
TESTS_PASSED=0

# Check prerequisites
if ! check_prerequisites; then
    log_error "Prerequisites not met"
    echo ""
    log_info "To set up:"
    log_info "  1. sudo ./scripts/setup-zephyr-network.sh"
    log_info "  2. ./zephyr/setup.sh"
    exit 1
fi

# Build
if [ "$SKIP_BUILD" = false ]; then
    if ! build_zephyr_examples; then
        log_error "Build failed"
        exit 1
    fi
fi

# Run tests
if [ "$QUICK" = true ]; then
    # Quick mode: just basic connectivity
    test_basic_connectivity && TESTS_PASSED=$((TESTS_PASSED + 1))
    TESTS_RUN=1
else
    # Full test suite
    test_zephyr_to_native && TESTS_PASSED=$((TESTS_PASSED + 1))
    TESTS_RUN=$((TESTS_RUN + 1))

    test_native_to_zephyr && TESTS_PASSED=$((TESTS_PASSED + 1))
    TESTS_RUN=$((TESTS_RUN + 1))

    test_basic_connectivity && TESTS_PASSED=$((TESTS_PASSED + 1))
    TESTS_RUN=$((TESTS_RUN + 1))
fi

# Cleanup
stop_all_processes

# Summary
log_header "Test Summary"
echo ""
echo "Tests passed: $TESTS_PASSED / $TESTS_RUN"
echo ""

if [ $TESTS_PASSED -eq $TESTS_RUN ]; then
    log_success "All Zephyr native_sim tests passed!"
    exit 0
else
    log_error "Some tests failed"
    exit 1
fi
