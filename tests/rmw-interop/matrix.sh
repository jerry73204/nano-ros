#!/bin/bash
# Test: Full communication matrix
#
# Tests all 4 combinations of nano-ros and ROS 2 talker/listener:
#   1. nano-ros talker → nano-ros listener
#   2. nano-ros talker → ROS 2 listener
#   3. ROS 2 talker → nano-ros listener
#   4. ROS 2 talker → ROS 2 listener (baseline)
#
# Usage:
#   ./tests/rmw-interop/matrix.sh
#   ./tests/rmw-interop/matrix.sh --ros-distro jazzy
#   ./tests/rmw-interop/matrix.sh --verbose

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/../common/utils.sh"
source "$SCRIPT_DIR/../common/prerequisites.sh"

# Parse arguments
ROS_DISTRO="humble"
VERBOSE=false
while [[ $# -gt 0 ]]; do
    case $1 in
        --ros-distro) ROS_DISTRO="$2"; shift 2 ;;
        --verbose|-v) VERBOSE=true; shift ;;
        *) shift ;;
    esac
done

# Setup
setup_cleanup

log_header "Full Communication Matrix Test"
log_info "Testing all talker/listener combinations"
log_info "ROS 2 distro: $ROS_DISTRO"

# Check prerequisites
if ! check_rmw_prerequisites "$ROS_DISTRO"; then
    log_error "Prerequisites not met"
    exit 1
fi

# Start zenohd
if ! start_zenohd; then
    exit 1
fi

# Results tracking
declare -A RESULTS
TOTAL_TESTS=0
PASSED_TESTS=0

run_test() {
    local name="$1"
    shift  # Remove name, leaving just the test function
    local result=0

    log_header "Test: $name"
    TOTAL_TESTS=$((TOTAL_TESTS + 1))

    # Run the test function
    if "$@"; then
        result=0
        PASSED_TESTS=$((PASSED_TESTS + 1))
        log_success "$name"
    else
        result=1
        log_error "$name"
    fi

    RESULTS["$name"]=$result

    # Cleanup between tests
    pkill -f "target/release/talker" 2>/dev/null || true
    pkill -f "target/release/listener" 2>/dev/null || true
    pkill -f "ros2 topic" 2>/dev/null || true
    sleep 2

    return $result
}

# Test 1: nano-ros talker → nano-ros listener
test_nano_to_nano() {
    log_info "Starting nano-ros listener..."
    RUST_LOG=info timeout 15 "$LISTENER_BIN" --tcp 127.0.0.1:7447 > /tmp/matrix_nano_listener.txt 2>&1 &
    register_pid $!
    sleep 2

    log_info "Starting nano-ros talker..."
    RUST_LOG=info "$TALKER_BIN" --tcp 127.0.0.1:7447 > /tmp/matrix_nano_talker.txt 2>&1 &
    register_pid $!

    sleep 8

    if grep -q "Received:" /tmp/matrix_nano_listener.txt 2>/dev/null; then
        local count
        count=$(count_pattern /tmp/matrix_nano_listener.txt "Received:")
        log_info "Received $count messages"
        [ "$VERBOSE" = true ] && head -10 /tmp/matrix_nano_listener.txt
        return 0
    fi
    return 1
}

# Test 2: nano-ros talker → ROS 2 listener
test_nano_to_ros2() {
    setup_ros2_env "$ROS_DISTRO"

    log_info "Starting ROS 2 topic echo..."
    timeout 15 ros2 topic echo /chatter std_msgs/msg/Int32 --qos-reliability best_effort \
        > /tmp/matrix_ros2_listener.txt 2>&1 &
    register_pid $!
    sleep 3

    log_info "Starting nano-ros talker..."
    RUST_LOG=info "$TALKER_BIN" --tcp 127.0.0.1:7447 > /tmp/matrix_nano_talker2.txt 2>&1 &
    register_pid $!

    sleep 10

    if grep -q "data:" /tmp/matrix_ros2_listener.txt 2>/dev/null; then
        local count
        count=$(count_pattern /tmp/matrix_ros2_listener.txt "data:")
        log_info "ROS 2 received $count messages"
        [ "$VERBOSE" = true ] && head -15 /tmp/matrix_ros2_listener.txt
        return 0
    fi
    return 1
}

# Test 3: ROS 2 talker → nano-ros listener
test_ros2_to_nano() {
    setup_ros2_env "$ROS_DISTRO"

    log_info "Starting nano-ros listener..."
    RUST_LOG=info timeout 15 "$LISTENER_BIN" --tcp 127.0.0.1:7447 > /tmp/matrix_nano_listener2.txt 2>&1 &
    register_pid $!
    sleep 2

    log_info "Starting ROS 2 topic pub..."
    timeout 12 ros2 topic pub -r 1 /chatter std_msgs/msg/Int32 "{data: 99}" \
        --qos-reliability best_effort > /tmp/matrix_ros2_pub.txt 2>&1 &
    register_pid $!

    sleep 10

    if grep -q "Received:" /tmp/matrix_nano_listener2.txt 2>/dev/null; then
        local count
        count=$(count_pattern /tmp/matrix_nano_listener2.txt "Received:")
        log_info "nano-ros received $count messages"
        [ "$VERBOSE" = true ] && head -10 /tmp/matrix_nano_listener2.txt
        return 0
    fi
    return 1
}

# Test 4: ROS 2 talker → ROS 2 listener (baseline)
test_ros2_to_ros2() {
    setup_ros2_env "$ROS_DISTRO"

    log_info "Starting ROS 2 topic echo..."
    timeout 15 ros2 topic echo /chatter std_msgs/msg/Int32 --qos-reliability best_effort \
        > /tmp/matrix_ros2_listener2.txt 2>&1 &
    register_pid $!
    sleep 3

    log_info "Starting ROS 2 topic pub..."
    timeout 12 ros2 topic pub -r 1 /chatter std_msgs/msg/Int32 "{data: 77}" \
        --qos-reliability best_effort > /tmp/matrix_ros2_pub2.txt 2>&1 &
    register_pid $!

    sleep 10

    if grep -q "data:" /tmp/matrix_ros2_listener2.txt 2>/dev/null; then
        local count
        count=$(count_pattern /tmp/matrix_ros2_listener2.txt "data:")
        log_info "ROS 2 received $count messages"
        [ "$VERBOSE" = true ] && head -15 /tmp/matrix_ros2_listener2.txt
        return 0
    fi
    return 1
}

# Run all tests
RESULT=0

run_test "nano-ros → nano-ros" test_nano_to_nano || RESULT=1
run_test "nano-ros → ROS 2" test_nano_to_ros2 || RESULT=1
run_test "ROS 2 → nano-ros" test_ros2_to_nano || RESULT=1
run_test "ROS 2 → ROS 2" test_ros2_to_ros2 || RESULT=1

# Print summary matrix
log_header "Communication Matrix Results"
echo ""
echo "┌─────────────────┬───────────────┬─────────────┐"
echo "│                 │   Listener    │             │"
echo "│     Talker      ├───────┬───────┤             │"
echo "│                 │nano-ros│ ROS 2 │             │"
echo "├─────────────────┼───────┼───────┤             │"

nano_nano="${RESULTS["nano-ros → nano-ros"]}"
nano_ros2="${RESULTS["nano-ros → ROS 2"]}"
ros2_nano="${RESULTS["ROS 2 → nano-ros"]}"
ros2_ros2="${RESULTS["ROS 2 → ROS 2"]}"

[[ "$nano_nano" == "0" ]] && nn="✓" || nn="✗"
[[ "$nano_ros2" == "0" ]] && nr="✓" || nr="✗"
[[ "$ros2_nano" == "0" ]] && rn="✓" || rn="✗"
[[ "$ros2_ros2" == "0" ]] && rr="✓" || rr="✗"

echo "│ nano-ros        │   $nn   │   $nr   │             │"
echo "│ ROS 2           │   $rn   │   $rr   │             │"
echo "└─────────────────┴───────┴───────┴─────────────┘"
echo ""
echo "Tests passed: $PASSED_TESTS / $TOTAL_TESTS"

if [ $RESULT -eq 0 ]; then
    log_success "All matrix tests passed!"
else
    log_error "Some tests failed"
fi

exit $RESULT
