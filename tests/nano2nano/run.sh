#!/bin/bash
# Test: nano-ros to nano-ros communication
#
# This test verifies that two nano-ros nodes can communicate with each other
# through a zenoh router.
#
# Usage:
#   ./tests/nano2nano/run.sh              # Run all tests
#   ./tests/nano2nano/run.sh --peer       # Test peer mode (no router)
#   ./tests/nano2nano/run.sh --verbose    # Verbose output

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/../common/utils.sh"
source "$SCRIPT_DIR/../common/prerequisites.sh"

# Parse arguments
PEER_MODE=false
VERBOSE=false
while [[ $# -gt 0 ]]; do
    case $1 in
        --peer) PEER_MODE=true; shift ;;
        --verbose|-v) VERBOSE=true; shift ;;
        *) shift ;;
    esac
done

# Setup
setup_cleanup

log_header "nano-ros to nano-ros Communication Test"

# Check prerequisites
if ! check_nano2nano_prerequisites; then
    log_error "Prerequisites not met"
    exit 1
fi

# Start zenohd (unless peer mode)
if [ "$PEER_MODE" = false ]; then
    if ! start_zenohd; then
        exit 1
    fi
    CONNECT_ARGS="--tcp 127.0.0.1:7447"
else
    log_info "Running in peer mode (no router)"
    CONNECT_ARGS=""
fi

# Test 1: Basic pub/sub communication
test_basic_pubsub() {
    log_header "Test: Basic Pub/Sub Communication"

    # Start talker
    log_info "Starting nano-ros talker..."
    RUST_LOG=info "$TALKER_BIN" $CONNECT_ARGS > "$(tmpfile nano_talker.txt)" 2>&1 &
    local talker_pid=$!
    register_pid $talker_pid
    sleep 2

    # Start listener
    log_info "Starting nano-ros listener..."
    RUST_LOG=info timeout "$TEST_TIMEOUT" "$LISTENER_BIN" $CONNECT_ARGS > "$(tmpfile nano_listener.txt)" 2>&1 &
    local listener_pid=$!
    register_pid $listener_pid

    # Wait for messages
    log_info "Waiting for messages..."
    if wait_for_pattern "$(tmpfile nano_listener.txt)" "Received:" "$TEST_TIMEOUT"; then
        local count
        count=$(count_pattern "$(tmpfile nano_listener.txt)" "Received:")
        log_success "Received $count messages"

        if [ "$VERBOSE" = true ]; then
            echo ""
            echo "=== Talker Output ==="
            cat "$(tmpfile nano_talker.txt)" | head -15
            echo ""
            echo "=== Listener Output ==="
            cat "$(tmpfile nano_listener.txt)" | head -15
        fi
        return 0
    else
        log_error "No messages received within timeout"
        echo ""
        echo "=== Talker Output ==="
        cat "$(tmpfile nano_talker.txt)" 2>/dev/null || echo "(no output)"
        echo ""
        echo "=== Listener Output ==="
        cat "$(tmpfile nano_listener.txt)" 2>/dev/null || echo "(no output)"
        return 1
    fi
}

# Test 2: Multiple messages
test_multiple_messages() {
    log_header "Test: Multiple Messages"

    # Kill previous processes
    pkill -f "/talker" 2>/dev/null || true
    pkill -f "/listener" 2>/dev/null || true
    sleep 1

    # Start listener first this time
    log_info "Starting listener..."
    RUST_LOG=info timeout 20 "$LISTENER_BIN" $CONNECT_ARGS > "$(tmpfile nano_listener2.txt)" 2>&1 &
    local listener_pid=$!
    register_pid $listener_pid
    sleep 2

    # Start talker
    log_info "Starting talker..."
    RUST_LOG=info "$TALKER_BIN" $CONNECT_ARGS > "$(tmpfile nano_talker2.txt)" 2>&1 &
    local talker_pid=$!
    register_pid $talker_pid

    # Wait for at least 5 messages
    log_info "Waiting for multiple messages..."
    sleep 8

    local count
    count=$(count_pattern "$(tmpfile nano_listener2.txt)" "Received:")

    if [ "$count" -ge 3 ]; then
        log_success "Received $count messages (target: >= 3)"
        return 0
    else
        log_error "Only received $count messages (target: >= 3)"
        return 1
    fi
}

# Run tests
RESULT=0

test_basic_pubsub || RESULT=1

sleep 2

test_multiple_messages || RESULT=1

# Summary
log_header "Test Summary"
if [ $RESULT -eq 0 ]; then
    log_success "All nano2nano tests passed!"
else
    log_error "Some tests failed"
fi

exit $RESULT
