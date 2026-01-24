#!/bin/bash
# Test: smoltcp Clock Synchronization
#
# Tests the monotonic clock implementation in the smoltcp platform layer.
#
# Tests:
#   - Initial value retrieval
#   - Set and get operations
#   - Large value handling
#   - Random number generator
#
# Usage:
#   ./tests/smoltcp/clock-sync.sh
#   ./tests/smoltcp/clock-sync.sh --verbose

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/../common/utils.sh"

VERBOSE=false
[[ "$1" == "--verbose" || "$1" == "-v" ]] && VERBOSE=true

setup_cleanup

log_header "smoltcp Clock Synchronization Tests"

RESULT=0

# =============================================================================
# Test 1: Clock Initial Value
# =============================================================================

test_clock_initial_value() {
    log_header "Test: Clock Initial Value"

    log_info "Running clock initial value tests..."

    if cargo test -p zenoh-pico-shim-sys --features smoltcp \
        test_clock_initial_value -- --test-threads=1 --nocapture \
        > "$(tmpfile smoltcp_clock_initial.txt)" 2>&1; then
        log_success "Clock initial value test passed"
        return 0
    else
        log_error "Clock initial value test failed"
        [ "$VERBOSE" = true ] && cat "$(tmpfile smoltcp_clock_initial.txt)"
        return 1
    fi
}

# =============================================================================
# Test 2: Clock Set and Get
# =============================================================================

test_clock_set_and_get() {
    log_header "Test: Clock Set and Get"

    log_info "Running clock set/get tests..."

    if cargo test -p zenoh-pico-shim-sys --features smoltcp \
        test_clock_set_and_get -- --test-threads=1 --nocapture \
        > "$(tmpfile smoltcp_clock_setget.txt)" 2>&1; then
        log_success "Clock set/get test passed"
        return 0
    else
        log_error "Clock set/get test failed"
        [ "$VERBOSE" = true ] && cat "$(tmpfile smoltcp_clock_setget.txt)"
        return 1
    fi
}

# =============================================================================
# Test 3: Clock Large Values
# =============================================================================

test_clock_large_values() {
    log_header "Test: Clock Large Values"

    log_info "Running clock large value tests..."

    if cargo test -p zenoh-pico-shim-sys --features smoltcp \
        test_clock_large_values -- --test-threads=1 --nocapture \
        > "$(tmpfile smoltcp_clock_large.txt)" 2>&1; then
        log_success "Clock large value test passed"
        return 0
    else
        log_error "Clock large value test failed"
        [ "$VERBOSE" = true ] && cat "$(tmpfile smoltcp_clock_large.txt)"
        return 1
    fi
}

# =============================================================================
# Test 4: Random Number Generator Values
# =============================================================================

test_random_returns_values() {
    log_header "Test: Random Number Generator"

    log_info "Running RNG tests..."

    if cargo test -p zenoh-pico-shim-sys --features smoltcp \
        test_random_returns_values -- --test-threads=1 --nocapture \
        > "$(tmpfile smoltcp_random.txt)" 2>&1; then
        log_success "RNG test passed"
        return 0
    else
        log_error "RNG test failed"
        [ "$VERBOSE" = true ] && cat "$(tmpfile smoltcp_random.txt)"
        return 1
    fi
}

# =============================================================================
# Test 5: Random Sequence
# =============================================================================

test_random_sequence() {
    log_header "Test: Random Sequence"

    log_info "Running random sequence tests..."

    if cargo test -p zenoh-pico-shim-sys --features smoltcp \
        test_random_sequence -- --test-threads=1 --nocapture \
        > "$(tmpfile smoltcp_random_seq.txt)" 2>&1; then
        log_success "Random sequence test passed"
        return 0
    else
        log_error "Random sequence test failed"
        [ "$VERBOSE" = true ] && cat "$(tmpfile smoltcp_random_seq.txt)"
        return 1
    fi
}

# =============================================================================
# Run Tests
# =============================================================================

test_clock_initial_value || RESULT=1
test_clock_set_and_get || RESULT=1
test_clock_large_values || RESULT=1
test_random_returns_values || RESULT=1
test_random_sequence || RESULT=1

# =============================================================================
# Summary
# =============================================================================

log_header "Test Summary"

echo ""
if [ $RESULT -eq 0 ]; then
    log_success "All clock synchronization tests passed!"
else
    log_error "Some clock synchronization tests failed"
fi

exit $RESULT
