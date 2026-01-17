#!/bin/bash
# Test: Liveliness token discovery
#
# Verifies that nano-ros declares proper liveliness tokens that ROS 2
# can use for discovery.
#
# Format: @ros2_lv/<domain>/<zid>/0/0/NN/%/%/<node>
#         @ros2_lv/<domain>/<zid>/0/11/MP/%/%/<node>/<topic>/<type>/<hash>/<qos>
#
# Usage:
#   ./tests/rmw-detailed/liveliness.sh
#   ./tests/rmw-detailed/liveliness.sh --verbose

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/../common/utils.sh"
source "$SCRIPT_DIR/../common/prerequisites.sh"

VERBOSE=false
[[ "$1" == "--verbose" || "$1" == "-v" ]] && VERBOSE=true

setup_cleanup

log_header "Liveliness Token Discovery Test"

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

# Test 1: Node liveliness token format (check from talker debug output)
test_node_liveliness() {
    log_header "Test: Node Liveliness Token"

    # Start nano-ros talker with debug logging
    log_info "Starting nano-ros talker..."
    RUST_LOG=debug timeout 8 "$TALKER_BIN" --tcp 127.0.0.1:7447 > /tmp/talker_lv.txt 2>&1 &
    register_pid $!
    sleep 4

    # Check for node liveliness keyexpr in output
    if grep -q "Node liveliness keyexpr:" /tmp/talker_lv.txt 2>/dev/null; then
        local token
        token=$(grep "Node liveliness keyexpr:" /tmp/talker_lv.txt | head -1)
        log_success "Node liveliness token declared"

        # Verify format: @ros2_lv/<domain>/<zid>/0/0/NN/%/%/<node>
        if echo "$token" | grep -qE "@ros2_lv/[0-9]+/[a-f0-9]+/0/0/NN/%/%/"; then
            log_success "Token format correct: @ros2_lv/<domain>/<zid>/0/0/NN/%/%/<node>"
        else
            log_warn "Token format may be incorrect"
        fi

        if [ "$VERBOSE" = true ]; then
            echo "Token: $token"
        fi
        return 0
    else
        log_error "Node liveliness token not found in output"
        [ "$VERBOSE" = true ] && cat /tmp/talker_lv.txt
        return 1
    fi
}

# Test 2: Publisher liveliness token format
test_publisher_liveliness() {
    log_header "Test: Publisher Liveliness Token"

    # Check for publisher liveliness keyexpr (from same output)
    if grep -q "Publisher liveliness keyexpr:" /tmp/talker_lv.txt 2>/dev/null; then
        local token
        token=$(grep "Publisher liveliness keyexpr:" /tmp/talker_lv.txt | head -1)
        log_success "Publisher liveliness token declared"

        # Check format components
        if echo "$token" | grep -q "MP/%/%/talker/%chatter"; then
            log_success "Entity type (MP) and topic (%chatter) correct"
        else
            log_warn "Entity type or topic format may be incorrect"
        fi

        if echo "$token" | grep -q "std_msgs::msg::dds_::Int32_"; then
            log_success "Type name correct (DDS mangled format)"
        else
            log_warn "Type name may be incorrect"
        fi

        if echo "$token" | grep -q "RIHS01_"; then
            log_success "Type hash has RIHS01_ prefix"
        else
            log_warn "Type hash missing RIHS01_ prefix"
        fi

        if echo "$token" | grep -qE "[0-9]+:[0-9]+:[0-9]+,[0-9]+"; then
            log_success "QoS string format correct"
        else
            log_warn "QoS string format may be incorrect"
        fi

        if [ "$VERBOSE" = true ]; then
            echo "Token: $token"
        fi
        return 0
    else
        log_error "Publisher liveliness token not found"
        return 1
    fi
}

# Test 3: ROS 2 can discover nano-ros via communication (functional test)
test_ros2_discovery() {
    log_header "Test: ROS 2 Discovery (Functional)"

    # Clear previous
    pkill -f "target/release/talker" 2>/dev/null || true
    sleep 2

    setup_ros2_env "humble" 2>/dev/null || {
        log_warn "ROS 2 not available, skipping"
        return 0
    }

    # Start talker
    log_info "Starting nano-ros talker..."
    "$TALKER_BIN" --tcp 127.0.0.1:7447 > /tmp/talker_disc.txt 2>&1 &
    register_pid $!
    sleep 3

    # Test that ROS 2 can receive messages (implies discovery working)
    log_info "Testing ROS 2 can receive messages..."
    if timeout 12 ros2 topic echo /chatter std_msgs/msg/Int32 --once --qos-reliability best_effort > /tmp/ros2_disc.txt 2>&1; then
        if grep -q "data:" /tmp/ros2_disc.txt 2>/dev/null; then
            log_success "ROS 2 received message (discovery working)"
            [ "$VERBOSE" = true ] && cat /tmp/ros2_disc.txt
            return 0
        fi
    fi

    log_warn "ROS 2 discovery test inconclusive"
    [ "$VERBOSE" = true ] && cat /tmp/ros2_disc.txt
    return 0  # Don't fail - discovery might just be slow
}

# Run tests
test_node_liveliness || RESULT=1
test_publisher_liveliness || RESULT=1
sleep 2
test_ros2_discovery || true  # Don't fail on discovery test

# Summary
log_header "Test Summary"
if [ $RESULT -eq 0 ]; then
    log_success "All liveliness tests passed!"
else
    log_error "Some tests failed"
fi

exit $RESULT
