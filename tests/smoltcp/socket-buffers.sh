#!/bin/bash
# Test: smoltcp Socket Buffers
#
# Tests the socket operations and buffer management in the smoltcp platform layer.
#
# Tests:
#   - Socket open/close lifecycle
#   - Multiple socket allocation
#   - Invalid handle error handling
#   - Connect and address storage
#   - RX buffer operations (push/read)
#   - TX buffer operations (write/pop)
#   - Connected flag management
#
# Usage:
#   ./tests/smoltcp/socket-buffers.sh
#   ./tests/smoltcp/socket-buffers.sh --verbose

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/../common/utils.sh"

VERBOSE=false
[[ "$1" == "--verbose" || "$1" == "-v" ]] && VERBOSE=true

setup_cleanup

log_header "smoltcp Socket Buffer Tests"

RESULT=0

# =============================================================================
# Test 1: Socket Open/Close
# =============================================================================

test_socket_open_close() {
    log_header "Test: Socket Open/Close"

    log_info "Running socket open/close tests..."

    if cargo test -p zenoh-pico-shim-sys --features smoltcp \
        test_socket_open_close -- --test-threads=1 --nocapture \
        > "$(tmpfile smoltcp_socket_open_close.txt)" 2>&1; then
        log_success "Socket open/close test passed"
        return 0
    else
        log_error "Socket open/close test failed"
        [ "$VERBOSE" = true ] && cat "$(tmpfile smoltcp_socket_open_close.txt)"
        return 1
    fi
}

# =============================================================================
# Test 2: Multiple Socket Open
# =============================================================================

test_socket_multiple_open() {
    log_header "Test: Multiple Socket Open"

    log_info "Running multiple socket open tests..."

    if cargo test -p zenoh-pico-shim-sys --features smoltcp \
        test_socket_multiple_open -- --test-threads=1 --nocapture \
        > "$(tmpfile smoltcp_socket_multiple.txt)" 2>&1; then
        log_success "Multiple socket open test passed"
        return 0
    else
        log_error "Multiple socket open test failed"
        [ "$VERBOSE" = true ] && cat "$(tmpfile smoltcp_socket_multiple.txt)"
        return 1
    fi
}

# =============================================================================
# Test 3: Invalid Handle
# =============================================================================

test_socket_close_invalid() {
    log_header "Test: Invalid Handle"

    log_info "Running invalid handle tests..."

    if cargo test -p zenoh-pico-shim-sys --features smoltcp \
        test_socket_close_invalid -- --test-threads=1 --nocapture \
        > "$(tmpfile smoltcp_socket_invalid.txt)" 2>&1; then
        log_success "Invalid handle test passed"
        return 0
    else
        log_error "Invalid handle test failed"
        [ "$VERBOSE" = true ] && cat "$(tmpfile smoltcp_socket_invalid.txt)"
        return 1
    fi
}

# =============================================================================
# Test 4: Socket Connect
# =============================================================================

test_socket_connect() {
    log_header "Test: Socket Connect"

    log_info "Running socket connect tests..."

    if cargo test -p zenoh-pico-shim-sys --features smoltcp \
        test_socket_connect -- --test-threads=1 --nocapture \
        > "$(tmpfile smoltcp_socket_connect.txt)" 2>&1; then
        log_success "Socket connect test passed"
        return 0
    else
        log_error "Socket connect test failed"
        [ "$VERBOSE" = true ] && cat "$(tmpfile smoltcp_socket_connect.txt)"
        return 1
    fi
}

# =============================================================================
# Test 5: RX Buffer Push/Read
# =============================================================================

test_socket_buffer_push_rx() {
    log_header "Test: RX Buffer Push/Read"

    log_info "Running RX buffer tests..."

    if cargo test -p zenoh-pico-shim-sys --features smoltcp \
        test_socket_buffer_push_rx -- --test-threads=1 --nocapture \
        > "$(tmpfile smoltcp_socket_rx.txt)" 2>&1; then
        log_success "RX buffer test passed"
        return 0
    else
        log_error "RX buffer test failed"
        [ "$VERBOSE" = true ] && cat "$(tmpfile smoltcp_socket_rx.txt)"
        return 1
    fi
}

# =============================================================================
# Test 6: TX Buffer Write/Pop
# =============================================================================

test_socket_buffer_pop_tx() {
    log_header "Test: TX Buffer Write/Pop"

    log_info "Running TX buffer tests..."

    if cargo test -p zenoh-pico-shim-sys --features smoltcp \
        test_socket_buffer_pop_tx -- --test-threads=1 --nocapture \
        > "$(tmpfile smoltcp_socket_tx.txt)" 2>&1; then
        log_success "TX buffer test passed"
        return 0
    else
        log_error "TX buffer test failed"
        [ "$VERBOSE" = true ] && cat "$(tmpfile smoltcp_socket_tx.txt)"
        return 1
    fi
}

# =============================================================================
# Test 7: Connected Flag
# =============================================================================

test_socket_connected_flag() {
    log_header "Test: Connected Flag"

    log_info "Running connected flag tests..."

    if cargo test -p zenoh-pico-shim-sys --features smoltcp \
        test_socket_connected_flag -- --test-threads=1 --nocapture \
        > "$(tmpfile smoltcp_socket_connected.txt)" 2>&1; then
        log_success "Connected flag test passed"
        return 0
    else
        log_error "Connected flag test failed"
        [ "$VERBOSE" = true ] && cat "$(tmpfile smoltcp_socket_connected.txt)"
        return 1
    fi
}

# =============================================================================
# Run Tests
# =============================================================================

test_socket_open_close || RESULT=1
test_socket_multiple_open || RESULT=1
test_socket_close_invalid || RESULT=1
test_socket_connect || RESULT=1
test_socket_buffer_push_rx || RESULT=1
test_socket_buffer_pop_tx || RESULT=1
test_socket_connected_flag || RESULT=1

# =============================================================================
# Summary
# =============================================================================

log_header "Test Summary"

echo ""
if [ $RESULT -eq 0 ]; then
    log_success "All socket buffer tests passed!"
else
    log_error "Some socket buffer tests failed"
fi

exit $RESULT
