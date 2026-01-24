#!/bin/bash
# Test: Key expression format
#
# Verifies that nano-ros uses the correct key expression format for
# ROS 2 topic compatibility.
#
# Data keyexpr format: <domain>/<topic>/<type>/<hash>
# Example: 0/chatter/std_msgs::msg::dds_::Int32_/TypeHashNotSupported
#
# Usage:
#   ./tests/rmw-detailed/keyexpr.sh
#   ./tests/rmw-detailed/keyexpr.sh --verbose

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/../common/utils.sh"
source "$SCRIPT_DIR/../common/prerequisites.sh"

VERBOSE=false
[[ "$1" == "--verbose" || "$1" == "-v" ]] && VERBOSE=true

setup_cleanup

log_header "Key Expression Format Test"

# Check prerequisites
if ! check_nano2nano_prerequisites; then
    log_error "Prerequisites not met"
    exit 1
fi

# Start zenohd
if ! start_zenohd; then
    exit 1
fi

RESULT=0

# Test 1: Data keyexpr format (from talker debug output)
test_data_keyexpr() {
    log_header "Test: Data Key Expression Format"

    # Start nano-ros talker with debug logging
    log_info "Starting nano-ros talker..."
    RUST_LOG=debug timeout 8 "$TALKER_BIN" --tcp 127.0.0.1:7447 > "$(tmpfile talker_keyexpr.txt)" 2>&1 &
    register_pid $!
    sleep 4

    # Check for data keyexpr in output
    if grep -q "Publisher data keyexpr:" "$(tmpfile talker_keyexpr.txt)" 2>/dev/null; then
        local line keyexpr
        line=$(grep "Publisher data keyexpr:" "$(tmpfile talker_keyexpr.txt)" | head -1)
        # Extract the keyexpr part after the colon
        keyexpr=$(echo "$line" | sed 's/.*Publisher data keyexpr: //')

        log_info "Captured keyexpr: $keyexpr"

        # Parse components
        local domain topic type_name type_hash
        domain=$(echo "$keyexpr" | cut -d'/' -f1)
        topic=$(echo "$keyexpr" | cut -d'/' -f2)
        type_name=$(echo "$keyexpr" | cut -d'/' -f3)
        type_hash=$(echo "$keyexpr" | cut -d'/' -f4)

        # Validate domain
        if [ "$domain" = "0" ]; then
            log_success "Domain ID: 0 (correct)"
        else
            log_error "Domain ID: $domain (expected: 0)"
            RESULT=1
        fi

        # Validate topic
        if [ "$topic" = "chatter" ]; then
            log_success "Topic name: chatter (correct)"
        else
            log_error "Topic name: $topic (expected: chatter)"
            RESULT=1
        fi

        # Validate type name (DDS mangled format)
        if echo "$type_name" | grep -q "std_msgs::msg::dds_::Int32_"; then
            log_success "Type name: $type_name (correct DDS format)"
        else
            log_error "Type name: $type_name (expected DDS mangled format)"
            RESULT=1
        fi

        # Validate type hash (Humble uses TypeHashNotSupported)
        if [ "$type_hash" = "TypeHashNotSupported" ]; then
            log_success "Type hash: TypeHashNotSupported (correct for Humble)"
        elif echo "$type_hash" | grep -q "RIHS01_"; then
            log_success "Type hash: $type_hash (RIHS01 format for Iron+)"
        else
            log_warn "Type hash: $type_hash (unexpected format)"
        fi

        if [ "$VERBOSE" = true ]; then
            echo ""
            echo "Full keyexpr: $keyexpr"
        fi
        return 0
    else
        log_error "Data keyexpr not found in output"
        [ "$VERBOSE" = true ] && cat "$(tmpfile talker_keyexpr.txt)"
        return 1
    fi
}

# Test 2: Keyexpr components validation
test_keyexpr_components() {
    log_header "Test: Key Expression Components"

    # The keyexpr should already be captured from test 1
    if grep -q "Publisher data keyexpr:" "$(tmpfile talker_keyexpr.txt)" 2>/dev/null; then
        local keyexpr
        keyexpr=$(grep "Publisher data keyexpr:" "$(tmpfile talker_keyexpr.txt)" | head -1 | sed 's/.*Publisher data keyexpr: //')

        # Count components (should be 4: domain/topic/type/hash)
        local count
        count=$(echo "$keyexpr" | tr '/' '\n' | wc -l)

        if [ "$count" -eq 4 ]; then
            log_success "Keyexpr has 4 components (domain/topic/type/hash)"
        else
            log_error "Keyexpr has $count components (expected 4)"
            return 1
        fi

        # Check no leading slash on topic
        if echo "$keyexpr" | grep -q "^0/chatter/"; then
            log_success "Topic has no leading slash"
        else
            log_warn "Topic format may be incorrect"
        fi

        return 0
    else
        log_error "No keyexpr data available"
        return 1
    fi
}

# Test 3: Wildcard subscriber compatibility
test_wildcard_subscriber() {
    log_header "Test: Wildcard Subscriber"

    # Clear previous
    pkill -f "/talker"; pkill -f "/listener" 2>/dev/null || true
    sleep 2

    # nano-ros uses wildcard subscriber: 0/chatter/std_msgs::msg::dds_::Int32_/*
    # This should match any type hash

    log_info "Testing wildcard matching..."

    # Start nano-ros listener
    RUST_LOG=info timeout 15 "$LISTENER_BIN" --tcp 127.0.0.1:7447 > "$(tmpfile wildcard_test.txt)" 2>&1 &
    register_pid $!
    sleep 2

    # Setup ROS 2 and publish
    if setup_ros2_env "humble" 2>/dev/null; then
        # ROS 2 publisher
        timeout 10 ros2 topic pub -r 1 /chatter std_msgs/msg/Int32 "{data: 555}" \
            --qos-reliability best_effort > "$(tmpfile ros2_wc.txt)" 2>&1 &
        register_pid $!
        sleep 6

        if grep -q "Received:" "$(tmpfile wildcard_test.txt)" 2>/dev/null; then
            log_success "Wildcard subscriber receives ROS 2 messages"

            if grep -q "data=555" "$(tmpfile wildcard_test.txt)" 2>/dev/null; then
                log_success "Data integrity verified (data=555)"
            fi

            [ "$VERBOSE" = true ] && head -10 "$(tmpfile wildcard_test.txt)"
            return 0
        else
            log_error "Wildcard subscriber did not receive messages"
            [ "$VERBOSE" = true ] && cat "$(tmpfile wildcard_test.txt)"
            return 1
        fi
    else
        log_warn "ROS 2 not available, testing with nano-ros only"

        # Test with nano-ros talker instead
        RUST_LOG=info timeout 10 "$TALKER_BIN" --tcp 127.0.0.1:7447 > "$(tmpfile nano_wc.txt)" 2>&1 &
        register_pid $!
        sleep 5

        if grep -q "Received:" "$(tmpfile wildcard_test.txt)" 2>/dev/null; then
            log_success "Wildcard subscriber receives nano-ros messages"
            return 0
        fi
        return 1
    fi
}

# Run tests
test_data_keyexpr || RESULT=1
test_keyexpr_components || RESULT=1
sleep 2
test_wildcard_subscriber || RESULT=1

# Summary
log_header "Test Summary"
if [ $RESULT -eq 0 ]; then
    log_success "All keyexpr tests passed!"
else
    log_error "Some tests failed"
fi

exit $RESULT
