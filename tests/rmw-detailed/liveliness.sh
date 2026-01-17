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

# Test 1: Node liveliness token
test_node_liveliness() {
    log_header "Test: Node Liveliness Token"

    # Start z_sub to monitor liveliness tokens
    if [ -x "$Z_SUB" ]; then
        log_info "Monitoring liveliness tokens..."
        timeout 10 "$Z_SUB" -m client -e tcp/127.0.0.1:7447 -k "@ros2_lv/**" \
            > /tmp/liveliness_tokens.txt 2>&1 &
        local zsub_pid=$!
        register_pid $zsub_pid
        sleep 1
    else
        log_warn "z_sub not available, using ROS 2 node list instead"
    fi

    # Start nano-ros talker
    log_info "Starting nano-ros talker..."
    timeout 8 "$TALKER_BIN" --tcp 127.0.0.1:7447 > /tmp/nano_talker_lv.txt 2>&1 &
    local talker_pid=$!
    register_pid $talker_pid
    sleep 3

    # Check node token format
    if [ -f /tmp/liveliness_tokens.txt ]; then
        if grep -q "@ros2_lv/0/.*NN/%/%/talker" /tmp/liveliness_tokens.txt 2>/dev/null; then
            log_success "Node liveliness token has correct format"

            if [ "$VERBOSE" = true ]; then
                echo "Token:"
                grep "NN/%/%/talker" /tmp/liveliness_tokens.txt | head -3
            fi
            return 0
        else
            log_error "Node liveliness token format incorrect"
            [ "$VERBOSE" = true ] && cat /tmp/liveliness_tokens.txt
            return 1
        fi
    fi

    # Fallback: check with ROS 2
    setup_ros2_env "humble" 2>/dev/null || return 1
    log_info "Checking with ros2 node list..."

    if timeout 5 ros2 node list 2>/dev/null | grep -q "talker"; then
        log_success "ROS 2 discovers nano-ros node"
        return 0
    else
        log_error "ROS 2 cannot discover nano-ros node"
        return 1
    fi
}

# Test 2: Publisher liveliness token
test_publisher_liveliness() {
    log_header "Test: Publisher Liveliness Token"

    # Clear previous
    pkill -f "target/release/talker" 2>/dev/null || true
    sleep 1

    if [ -x "$Z_SUB" ]; then
        timeout 10 "$Z_SUB" -m client -e tcp/127.0.0.1:7447 -k "@ros2_lv/**" \
            > /tmp/pub_liveliness.txt 2>&1 &
        register_pid $!
        sleep 1
    fi

    # Start talker
    timeout 8 "$TALKER_BIN" --tcp 127.0.0.1:7447 > /tmp/nano_talker_pub.txt 2>&1 &
    register_pid $!
    sleep 3

    # Check publisher token
    # Expected: @ros2_lv/0/<zid>/0/11/MP/%/%/talker/%chatter/std_msgs::msg::dds_::Int32_/RIHS01_<hash>/<qos>
    if [ -f /tmp/pub_liveliness.txt ]; then
        if grep -q "@ros2_lv/0/.*MP/%/%/talker/%chatter" /tmp/pub_liveliness.txt 2>/dev/null; then
            log_success "Publisher liveliness token declared"

            # Check components
            local token
            token=$(grep "MP/%/%/talker/%chatter" /tmp/pub_liveliness.txt | head -1)

            if echo "$token" | grep -q "std_msgs::msg::dds_::Int32_"; then
                log_success "Type name correct"
            else
                log_warn "Type name may be incorrect"
            fi

            if echo "$token" | grep -q "RIHS01_"; then
                log_success "Type hash has RIHS01_ prefix"
            else
                log_warn "Type hash missing RIHS01_ prefix"
            fi

            # QoS format check
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
            [ "$VERBOSE" = true ] && cat /tmp/pub_liveliness.txt
            return 1
        fi
    fi

    # Fallback: check with ROS 2 topic info
    setup_ros2_env "humble" 2>/dev/null || return 1
    log_info "Checking with ros2 topic info..."

    if timeout 5 ros2 topic info /chatter 2>/dev/null | grep -q "Publisher"; then
        log_success "ROS 2 sees nano-ros publisher"
        return 0
    else
        log_error "ROS 2 cannot see nano-ros publisher"
        return 1
    fi
}

# Test 3: Discovery visible to ROS 2
test_ros2_discovery() {
    log_header "Test: ROS 2 Discovery"

    setup_ros2_env "humble" 2>/dev/null || {
        log_warn "ROS 2 not available, skipping"
        return 0
    }

    # Clear previous
    pkill -f "target/release/talker" 2>/dev/null || true
    sleep 1

    # Start talker
    timeout 15 "$TALKER_BIN" --tcp 127.0.0.1:7447 > /tmp/nano_disc.txt 2>&1 &
    register_pid $!
    sleep 3

    # Check node discovery
    log_info "Running ros2 node list..."
    local nodes
    nodes=$(timeout 5 ros2 node list 2>/dev/null || echo "")

    if echo "$nodes" | grep -q "talker"; then
        log_success "Node discovered via ros2 node list"
    else
        log_warn "Node not visible in ros2 node list"
    fi

    # Check topic discovery
    log_info "Running ros2 topic list..."
    local topics
    topics=$(timeout 5 ros2 topic list 2>/dev/null || echo "")

    if echo "$topics" | grep -q "/chatter"; then
        log_success "Topic discovered via ros2 topic list"
    else
        log_warn "Topic not visible in ros2 topic list"
    fi

    # Check topic info
    log_info "Running ros2 topic info..."
    local info
    info=$(timeout 5 ros2 topic info /chatter -v 2>/dev/null || echo "")

    if echo "$info" | grep -q "Publisher"; then
        log_success "Publisher visible in topic info"

        if [ "$VERBOSE" = true ]; then
            echo ""
            echo "=== Topic Info ==="
            echo "$info"
        fi
        return 0
    else
        log_warn "Publisher not visible in topic info"
        return 1
    fi
}

# Run tests
test_node_liveliness || RESULT=1
sleep 2
test_publisher_liveliness || RESULT=1
sleep 2
test_ros2_discovery || RESULT=1

# Summary
log_header "Test Summary"
if [ $RESULT -eq 0 ]; then
    log_success "All liveliness tests passed!"
else
    log_error "Some tests failed"
fi

exit $RESULT
