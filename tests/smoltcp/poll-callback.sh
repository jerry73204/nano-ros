#!/bin/bash
# Test: smoltcp Poll Callback
#
# Tests the poll callback mechanism in the smoltcp platform layer.
#
# Tests:
#   - Callback registration
#   - Callback invocation on poll()
#   - No-callback safety (null function pointer)
#
# Usage:
#   ./tests/smoltcp/poll-callback.sh
#   ./tests/smoltcp/poll-callback.sh --verbose

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/../common/utils.sh"

VERBOSE=false
[[ "$1" == "--verbose" || "$1" == "-v" ]] && VERBOSE=true

setup_cleanup

log_header "smoltcp Poll Callback Tests"

RESULT=0

# =============================================================================
# Test 1: Poll Callback Registration
# =============================================================================

test_poll_callback_registration() {
    log_header "Test: Poll Callback Registration"

    log_info "Running poll callback registration tests..."

    if cargo test -p zenoh-pico-shim-sys --features smoltcp \
        test_poll_callback_registration -- --test-threads=1 --nocapture \
        > "$(tmpfile smoltcp_poll_reg.txt)" 2>&1; then
        log_success "Poll callback registration test passed"
        return 0
    else
        log_error "Poll callback registration test failed"
        [ "$VERBOSE" = true ] && cat "$(tmpfile smoltcp_poll_reg.txt)"
        return 1
    fi
}

# =============================================================================
# Test 2: Poll No Callback
# =============================================================================

test_poll_no_callback() {
    log_header "Test: Poll No Callback"

    log_info "Running poll no callback tests..."

    if cargo test -p zenoh-pico-shim-sys --features smoltcp \
        test_poll_no_callback -- --test-threads=1 --nocapture \
        > "$(tmpfile smoltcp_poll_none.txt)" 2>&1; then
        log_success "Poll no callback test passed"
        return 0
    else
        log_error "Poll no callback test failed"
        [ "$VERBOSE" = true ] && cat "$(tmpfile smoltcp_poll_none.txt)"
        return 1
    fi
}

# =============================================================================
# Run Tests
# =============================================================================

test_poll_callback_registration || RESULT=1
test_poll_no_callback || RESULT=1

# =============================================================================
# Summary
# =============================================================================

log_header "Test Summary"

echo ""
if [ $RESULT -eq 0 ]; then
    log_success "All poll callback tests passed!"
else
    log_error "Some poll callback tests failed"
fi

exit $RESULT
