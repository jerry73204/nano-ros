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

# Check z_sub is available
if [ ! -x "$Z_SUB" ]; then
    log_error "z_sub required for this test"
    log_info "Build zenoh-pico examples or install zenoh tools"
    exit 1
fi

# Start zenohd
if ! start_zenohd; then
    exit 1
fi

RESULT=0

# Test 1: Data keyexpr format
test_data_keyexpr() {
    log_header "Test: Data Key Expression Format"

    # Subscribe to all domain 0 topics
    log_info "Subscribing to domain 0 topics..."
    timeout 10 "$Z_SUB" -m client -e tcp/127.0.0.1:7447 -k "0/**" \
        > /tmp/keyexpr_data.txt 2>&1 &
    local zsub_pid=$!
    register_pid $zsub_pid
    sleep 1

    # Start nano-ros talker
    log_info "Starting nano-ros talker..."
    timeout 8 "$TALKER_BIN" --tcp 127.0.0.1:7447 > /tmp/nano_keyexpr.txt 2>&1 &
    register_pid $!
    sleep 4

    # Analyze captured keyexprs
    if [ ! -s /tmp/keyexpr_data.txt ]; then
        log_error "No data captured"
        return 1
    fi

    log_info "Analyzing captured key expressions..."

    # Expected format: 0/chatter/std_msgs::msg::dds_::Int32_/TypeHashNotSupported
    # Or with wildcards for subscriber: 0/chatter/std_msgs::msg::dds_::Int32_/*

    local keyexpr
    keyexpr=$(head -5 /tmp/keyexpr_data.txt | grep -o "0/[^']*" | head -1 || echo "")

    if [ -z "$keyexpr" ]; then
        log_error "Could not extract keyexpr from output"
        [ "$VERBOSE" = true ] && cat /tmp/keyexpr_data.txt
        return 1
    fi

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
        echo "=== Full Output ==="
        head -10 /tmp/keyexpr_data.txt
    fi

    return 0
}

# Test 2: Compare with ROS 2 keyexpr
test_compare_ros2_keyexpr() {
    log_header "Test: Compare with ROS 2 Key Expression"

    setup_ros2_env "humble" 2>/dev/null || {
        log_warn "ROS 2 not available, skipping comparison"
        return 0
    }

    # Clear previous
    pkill -f "target/release/talker" 2>/dev/null || true
    sleep 1

    # Capture ROS 2 keyexpr
    log_info "Subscribing to capture ROS 2 keyexpr..."
    timeout 15 "$Z_SUB" -m client -e tcp/127.0.0.1:7447 -k "0/**" \
        > /tmp/keyexpr_ros2.txt 2>&1 &
    register_pid $!
    sleep 1

    # Start ROS 2 publisher
    log_info "Starting ROS 2 publisher..."
    timeout 10 ros2 topic pub -r 1 /chatter std_msgs/msg/Int32 "{data: 123}" \
        --qos-reliability best_effort > /tmp/ros2_pub_ke.txt 2>&1 &
    register_pid $!
    sleep 5

    # Extract and compare
    local ros2_keyexpr
    ros2_keyexpr=$(head -10 /tmp/keyexpr_ros2.txt | grep -o "0/chatter[^']*" | head -1 || echo "")

    if [ -n "$ros2_keyexpr" ]; then
        log_info "ROS 2 keyexpr: $ros2_keyexpr"

        # Compare type hash component
        local ros2_hash
        ros2_hash=$(echo "$ros2_keyexpr" | cut -d'/' -f4)
        log_info "ROS 2 type hash: $ros2_hash"

        if [ "$ros2_hash" = "TypeHashNotSupported" ]; then
            log_success "ROS 2 Humble also uses TypeHashNotSupported"
        else
            log_info "ROS 2 uses hash: $ros2_hash"
        fi
    else
        log_warn "Could not capture ROS 2 keyexpr"
    fi

    return 0
}

# Test 3: Wildcard subscriber compatibility
test_wildcard_subscriber() {
    log_header "Test: Wildcard Subscriber"

    # Clear previous
    pkill -f "target/release" 2>/dev/null || true
    sleep 1

    # nano-ros uses wildcard subscriber: 0/chatter/std_msgs::msg::dds_::Int32_/*
    # This should match any type hash

    log_info "Testing wildcard matching..."

    # Start nano-ros listener (uses wildcard for type hash)
    timeout 15 "$LISTENER_BIN" --tcp 127.0.0.1:7447 > /tmp/wildcard_test.txt 2>&1 &
    register_pid $!
    sleep 2

    # Setup ROS 2 and publish
    setup_ros2_env "humble" 2>/dev/null || {
        log_warn "ROS 2 not available, testing with nano-ros only"

        # Test with nano-ros talker instead
        timeout 10 "$TALKER_BIN" --tcp 127.0.0.1:7447 > /tmp/nano_wc.txt 2>&1 &
        register_pid $!
        sleep 5

        if grep -q "Received:" /tmp/wildcard_test.txt 2>/dev/null; then
            log_success "Wildcard subscriber receives nano-ros messages"
            return 0
        fi
        return 1
    }

    # ROS 2 publisher
    timeout 10 ros2 topic pub -r 1 /chatter std_msgs/msg/Int32 "{data: 555}" \
        --qos-reliability best_effort > /tmp/ros2_wc.txt 2>&1 &
    register_pid $!
    sleep 6

    if grep -q "Received:" /tmp/wildcard_test.txt 2>/dev/null; then
        log_success "Wildcard subscriber receives ROS 2 messages"

        if grep -q "data=555" /tmp/wildcard_test.txt 2>/dev/null; then
            log_success "Data integrity verified"
        fi
        return 0
    else
        log_error "Wildcard subscriber did not receive messages"
        [ "$VERBOSE" = true ] && cat /tmp/wildcard_test.txt
        return 1
    fi
}

# Run tests
test_data_keyexpr || RESULT=1
sleep 2
test_compare_ros2_keyexpr || true  # Don't fail if ROS 2 not available
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
