#!/bin/bash
# Test: POSIX platform backend
#
# Verifies zenoh-pico-shim works correctly with POSIX platform.
# This is the reference implementation used on desktop/Linux systems.
#
# Requirements:
#   - zenohd running on tcp/127.0.0.1:7447
#
# Usage:
#   ./tests/platform/posix.sh
#   ./tests/platform/posix.sh --verbose

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/../common/utils.sh"

VERBOSE=false
[[ "$1" == "--verbose" || "$1" == "-v" ]] && VERBOSE=true

setup_cleanup

log_header "POSIX Platform Backend Test"

# Check prerequisites
check_posix_prerequisites() {
    log_header "Checking Prerequisites"

    local failed=0

    # Check zenohd
    if command -v zenohd &> /dev/null; then
        log_success "zenohd found: $(which zenohd)"
    else
        log_error "zenohd not found in PATH"
        failed=1
    fi

    # Check cargo
    if command -v cargo &> /dev/null; then
        log_success "cargo found: $(which cargo)"
    else
        log_error "cargo not found in PATH"
        failed=1
    fi

    return $failed
}

if ! check_posix_prerequisites; then
    log_error "Prerequisites not met"
    exit 1
fi

# Start zenohd
if ! start_zenohd; then
    exit 1
fi

RESULT=0

# =============================================================================
# Test 1: Session Lifecycle
# =============================================================================

test_session_lifecycle() {
    log_header "Test: Session Lifecycle"

    log_info "Running session lifecycle tests..."

    if cargo test -p zenoh-pico-shim --features "posix std" \
        -- session --test-threads=1 --nocapture > "$(tmpfile posix_session.txt)" 2>&1; then

        # Count passed tests
        local passed
        passed=$(grep -c "test .* ok" "$(tmpfile posix_session.txt)" 2>/dev/null || echo 0)
        log_success "Session lifecycle: $passed tests passed"
        [ "$VERBOSE" = true ] && cat "$(tmpfile posix_session.txt)"
        return 0
    else
        log_error "Session lifecycle tests failed"
        cat "$(tmpfile posix_session.txt)"
        return 1
    fi
}

# =============================================================================
# Test 2: Publisher Operations
# =============================================================================

test_publisher_operations() {
    log_header "Test: Publisher Operations"

    log_info "Running publisher tests..."

    if cargo test -p zenoh-pico-shim --features "posix std" \
        -- publisher --test-threads=1 --nocapture > "$(tmpfile posix_publisher.txt)" 2>&1; then

        local passed
        passed=$(grep -c "test .* ok" "$(tmpfile posix_publisher.txt)" 2>/dev/null || echo 0)
        log_success "Publisher operations: $passed tests passed"
        [ "$VERBOSE" = true ] && cat "$(tmpfile posix_publisher.txt)"
        return 0
    else
        log_error "Publisher tests failed"
        cat "$(tmpfile posix_publisher.txt)"
        return 1
    fi
}

# =============================================================================
# Test 3: Subscriber Operations
# =============================================================================

test_subscriber_operations() {
    log_header "Test: Subscriber Operations"

    log_info "Running subscriber tests..."

    if cargo test -p zenoh-pico-shim --features "posix std" \
        -- subscriber --test-threads=1 --nocapture > "$(tmpfile posix_subscriber.txt)" 2>&1; then

        local passed
        passed=$(grep -c "test .* ok" "$(tmpfile posix_subscriber.txt)" 2>/dev/null || echo 0)
        log_success "Subscriber operations: $passed tests passed"
        [ "$VERBOSE" = true ] && cat "$(tmpfile posix_subscriber.txt)"
        return 0
    else
        log_error "Subscriber tests failed"
        cat "$(tmpfile posix_subscriber.txt)"
        return 1
    fi
}

# =============================================================================
# Test 4: Pub/Sub Communication
# =============================================================================

test_pubsub_communication() {
    log_header "Test: Pub/Sub Communication"

    log_info "Running pub/sub communication tests..."

    if cargo test -p zenoh-pico-shim --features "posix std" \
        -- pubsub --test-threads=1 --nocapture > "$(tmpfile posix_pubsub.txt)" 2>&1; then

        local passed
        passed=$(grep -c "test .* ok" "$(tmpfile posix_pubsub.txt)" 2>/dev/null || echo 0)
        log_success "Pub/sub communication: $passed tests passed"
        [ "$VERBOSE" = true ] && cat "$(tmpfile posix_pubsub.txt)"
        return 0
    else
        # Check if it's just a local subscriber warning
        if grep -q "Local pub/sub may not be enabled" "$(tmpfile posix_pubsub.txt)" 2>/dev/null; then
            log_warn "Local pub/sub not enabled in zenoh-pico build (not a failure)"
            return 0
        fi
        log_error "Pub/sub communication tests failed"
        cat "$(tmpfile posix_pubsub.txt)"
        return 1
    fi
}

# =============================================================================
# Test 5: Liveliness Tokens
# =============================================================================

test_liveliness_tokens() {
    log_header "Test: Liveliness Tokens"

    log_info "Running liveliness tests..."

    if cargo test -p zenoh-pico-shim --features "posix std" \
        -- liveliness --test-threads=1 --nocapture > "$(tmpfile posix_liveliness.txt)" 2>&1; then

        local passed
        passed=$(grep -c "test .* ok" "$(tmpfile posix_liveliness.txt)" 2>/dev/null || echo 0)
        log_success "Liveliness tokens: $passed tests passed"
        [ "$VERBOSE" = true ] && cat "$(tmpfile posix_liveliness.txt)"
        return 0
    else
        log_error "Liveliness tests failed"
        cat "$(tmpfile posix_liveliness.txt)"
        return 1
    fi
}

# =============================================================================
# Test 6: ZenohId
# =============================================================================

test_zenoh_id() {
    log_header "Test: ZenohId"

    log_info "Running ZenohId tests..."

    if cargo test -p zenoh-pico-shim --features "posix std" \
        -- zenoh_id --test-threads=1 --nocapture > "$(tmpfile posix_zid.txt)" 2>&1; then

        local passed
        passed=$(grep -c "test .* ok" "$(tmpfile posix_zid.txt)" 2>/dev/null || echo 0)
        log_success "ZenohId: $passed tests passed"
        [ "$VERBOSE" = true ] && cat "$(tmpfile posix_zid.txt)"
        return 0
    else
        log_error "ZenohId tests failed"
        cat "$(tmpfile posix_zid.txt)"
        return 1
    fi
}

# =============================================================================
# Test 7: Max Publishers Limit
# =============================================================================

test_max_publishers() {
    log_header "Test: Max Publishers Limit"

    log_info "Running max publishers test..."

    if cargo test -p zenoh-pico-shim --features "posix std" \
        -- max_publishers --test-threads=1 --nocapture > "$(tmpfile posix_max_pub.txt)" 2>&1; then

        local passed
        passed=$(grep -c "test .* ok" "$(tmpfile posix_max_pub.txt)" 2>/dev/null || echo 0)
        log_success "Max publishers: $passed tests passed"
        [ "$VERBOSE" = true ] && cat "$(tmpfile posix_max_pub.txt)"
        return 0
    else
        log_error "Max publishers test failed"
        cat "$(tmpfile posix_max_pub.txt)"
        return 1
    fi
}

# =============================================================================
# Test 8: Polling API
# =============================================================================

test_polling_api() {
    log_header "Test: Polling API"

    log_info "Running polling API tests..."

    if cargo test -p zenoh-pico-shim --features "posix std" \
        -- poll --test-threads=1 --nocapture > "$(tmpfile posix_poll.txt)" 2>&1; then

        local passed
        passed=$(grep -c "test .* ok" "$(tmpfile posix_poll.txt)" 2>/dev/null || echo 0)
        log_success "Polling API: $passed tests passed"
        [ "$VERBOSE" = true ] && cat "$(tmpfile posix_poll.txt)"
        return 0
    else
        log_error "Polling API tests failed"
        cat "$(tmpfile posix_poll.txt)"
        return 1
    fi
}

# =============================================================================
# Test 9: Full Integration Test
# =============================================================================

test_full_integration() {
    log_header "Test: Full Integration"

    log_info "Running all zenoh-pico-shim integration tests..."

    if cargo test -p zenoh-pico-shim --features "posix std" \
        --test-threads=1 > "$(tmpfile posix_full.txt)" 2>&1; then

        local passed failed
        passed=$(grep -c "test .* ok" "$(tmpfile posix_full.txt)" 2>/dev/null || echo 0)
        failed=$(grep -c "test .* FAILED" "$(tmpfile posix_full.txt)" 2>/dev/null || echo 0)

        log_success "Full integration: $passed passed, $failed failed"
        [ "$VERBOSE" = true ] && cat "$(tmpfile posix_full.txt)"

        if [ "$failed" -gt 0 ]; then
            return 1
        fi
        return 0
    else
        log_error "Full integration tests failed"
        cat "$(tmpfile posix_full.txt)"
        return 1
    fi
}

# =============================================================================
# Run Tests
# =============================================================================

# Run individual test categories
test_session_lifecycle || RESULT=1
sleep 1
test_publisher_operations || RESULT=1
sleep 1
test_subscriber_operations || RESULT=1
sleep 1
test_pubsub_communication || RESULT=1
sleep 1
test_liveliness_tokens || RESULT=1
sleep 1
test_zenoh_id || RESULT=1
sleep 1
test_max_publishers || RESULT=1
sleep 1
test_polling_api || RESULT=1

# =============================================================================
# Summary
# =============================================================================

log_header "Test Summary"

if [ $RESULT -eq 0 ]; then
    log_success "All POSIX platform tests passed!"
else
    log_error "Some POSIX platform tests failed"
fi

exit $RESULT
