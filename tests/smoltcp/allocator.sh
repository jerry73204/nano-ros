#!/bin/bash
# Test: smoltcp Allocator
#
# Tests the bump allocator implementation in the smoltcp platform layer.
#
# Tests:
#   - Basic allocation (alignment)
#   - Multiple allocations (non-overlapping)
#   - Reallocation (null input, zero size, grow)
#   - Free operation (no-op)
#
# Usage:
#   ./tests/smoltcp/allocator.sh
#   ./tests/smoltcp/allocator.sh --verbose

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/../common/utils.sh"

VERBOSE=false
[[ "$1" == "--verbose" || "$1" == "-v" ]] && VERBOSE=true

setup_cleanup

log_header "smoltcp Allocator Tests"

RESULT=0

# =============================================================================
# Test 1: Basic Allocation
# =============================================================================

test_alloc_basic() {
    log_header "Test: Basic Allocation"

    log_info "Running allocator basic tests..."

    if cargo test -p zenoh-pico-shim-sys --features smoltcp \
        test_alloc_basic -- --test-threads=1 --nocapture \
        > "$(tmpfile smoltcp_alloc_basic.txt)" 2>&1; then
        log_success "Basic allocation test passed"
        return 0
    else
        log_error "Basic allocation test failed"
        [ "$VERBOSE" = true ] && cat "$(tmpfile smoltcp_alloc_basic.txt)"
        return 1
    fi
}

# =============================================================================
# Test 2: Multiple Allocations
# =============================================================================

test_alloc_multiple() {
    log_header "Test: Multiple Allocations"

    log_info "Running multiple allocation tests..."

    if cargo test -p zenoh-pico-shim-sys --features smoltcp \
        test_alloc_multiple -- --test-threads=1 --nocapture \
        > "$(tmpfile smoltcp_alloc_multiple.txt)" 2>&1; then
        log_success "Multiple allocation test passed"
        return 0
    else
        log_error "Multiple allocation test failed"
        [ "$VERBOSE" = true ] && cat "$(tmpfile smoltcp_alloc_multiple.txt)"
        return 1
    fi
}

# =============================================================================
# Test 3: Reallocation (null input)
# =============================================================================

test_realloc_null() {
    log_header "Test: Realloc Null Input"

    log_info "Running realloc null input tests..."

    if cargo test -p zenoh-pico-shim-sys --features smoltcp \
        test_realloc_null -- --test-threads=1 --nocapture \
        > "$(tmpfile smoltcp_realloc_null.txt)" 2>&1; then
        log_success "Realloc null input test passed"
        return 0
    else
        log_error "Realloc null input test failed"
        [ "$VERBOSE" = true ] && cat "$(tmpfile smoltcp_realloc_null.txt)"
        return 1
    fi
}

# =============================================================================
# Test 4: Reallocation (zero size)
# =============================================================================

test_realloc_zero_size() {
    log_header "Test: Realloc Zero Size"

    log_info "Running realloc zero size tests..."

    if cargo test -p zenoh-pico-shim-sys --features smoltcp \
        test_realloc_zero_size -- --test-threads=1 --nocapture \
        > "$(tmpfile smoltcp_realloc_zero.txt)" 2>&1; then
        log_success "Realloc zero size test passed"
        return 0
    else
        log_error "Realloc zero size test failed"
        [ "$VERBOSE" = true ] && cat "$(tmpfile smoltcp_realloc_zero.txt)"
        return 1
    fi
}

# =============================================================================
# Test 5: Reallocation (grow)
# =============================================================================

test_realloc_grow() {
    log_header "Test: Realloc Grow"

    log_info "Running realloc grow tests..."

    if cargo test -p zenoh-pico-shim-sys --features smoltcp \
        test_realloc_grow -- --test-threads=1 --nocapture \
        > "$(tmpfile smoltcp_realloc_grow.txt)" 2>&1; then
        log_success "Realloc grow test passed"
        return 0
    else
        log_error "Realloc grow test failed"
        [ "$VERBOSE" = true ] && cat "$(tmpfile smoltcp_realloc_grow.txt)"
        return 1
    fi
}

# =============================================================================
# Test 6: Free Operation
# =============================================================================

test_free_noop() {
    log_header "Test: Free Operation"

    log_info "Running free operation tests..."

    if cargo test -p zenoh-pico-shim-sys --features smoltcp \
        test_free_noop -- --test-threads=1 --nocapture \
        > "$(tmpfile smoltcp_free_noop.txt)" 2>&1; then
        log_success "Free operation test passed"
        return 0
    else
        log_error "Free operation test failed"
        [ "$VERBOSE" = true ] && cat "$(tmpfile smoltcp_free_noop.txt)"
        return 1
    fi
}

# =============================================================================
# Run Tests
# =============================================================================

test_alloc_basic || RESULT=1
test_alloc_multiple || RESULT=1
test_realloc_null || RESULT=1
test_realloc_zero_size || RESULT=1
test_realloc_grow || RESULT=1
test_free_noop || RESULT=1

# =============================================================================
# Summary
# =============================================================================

log_header "Test Summary"

echo ""
if [ $RESULT -eq 0 ]; then
    log_success "All allocator tests passed!"
else
    log_error "Some allocator tests failed"
fi

exit $RESULT
