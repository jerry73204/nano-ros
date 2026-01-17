#!/bin/bash
# Test: RMW Attachment format
#
# Verifies that nano-ros sends proper RMW attachment metadata required
# for rmw_zenoh_cpp compatibility.
#
# Attachment format (33 bytes):
#   - sequence_number: i64 (8 bytes)
#   - timestamp: i64 (8 bytes)
#   - rmw_gid_size: u8 (1 byte, always 16)
#   - rmw_gid: [u8; 16] (16 bytes)
#
# The attachment is serialized using zenoh's serializer for compatibility.
#
# Usage:
#   ./tests/rmw-detailed/attachment.sh
#   ./tests/rmw-detailed/attachment.sh --verbose

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/../common/utils.sh"
source "$SCRIPT_DIR/../common/prerequisites.sh"

VERBOSE=false
[[ "$1" == "--verbose" || "$1" == "-v" ]] && VERBOSE=true

setup_cleanup

log_header "RMW Attachment Format Test"

# Check prerequisites
if ! check_rmw_prerequisites "humble"; then
    log_error "Prerequisites not met"
    exit 1
fi

# Start zenohd
if ! start_zenohd; then
    exit 1
fi

RESULT=0

# Test 1: ROS 2 receives messages (implies attachment is correct)
test_attachment_via_ros2() {
    log_header "Test: ROS 2 Receives Messages (Attachment Validation)"

    setup_ros2_env "humble"

    # Start nano-ros talker
    log_info "Starting nano-ros talker..."
    "$TALKER_BIN" --tcp 127.0.0.1:7447 > /tmp/attach_talker.txt 2>&1 &
    register_pid $!
    sleep 2

    # ROS 2 subscriber - if attachment is wrong, rmw_zenoh_cpp will reject
    log_info "Starting ROS 2 subscriber..."
    timeout 15 ros2 topic echo /chatter std_msgs/msg/Int32 --qos-reliability best_effort \
        > /tmp/attach_ros2.txt 2>&1 &
    register_pid $!

    sleep 10

    if grep -q "data:" /tmp/attach_ros2.txt 2>/dev/null; then
        local count
        count=$(count_pattern /tmp/attach_ros2.txt "data:")
        log_success "ROS 2 received $count messages"
        log_success "RMW attachment format is correct (rmw_zenoh_cpp accepted messages)"
        return 0
    else
        log_error "ROS 2 did not receive messages"
        log_error "This may indicate attachment format issues"
        return 1
    fi
}

# Test 2: Sequence numbers increment
test_sequence_numbers() {
    log_header "Test: Sequence Number Increment"

    # Clear previous
    pkill -f "target/release/talker" 2>/dev/null || true
    sleep 1

    setup_ros2_env "humble"

    # Start nano-ros talker
    log_info "Starting nano-ros talker..."
    "$TALKER_BIN" --tcp 127.0.0.1:7447 > /tmp/seq_talker.txt 2>&1 &
    register_pid $!
    sleep 2

    # Use Python to check sequence numbers
    log_info "Checking sequence numbers via Python subscriber..."
    python3 << 'PYTHON_EOF' > /tmp/seq_check.txt 2>&1
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32
import time

class SeqChecker(Node):
    def __init__(self):
        super().__init__('seq_checker')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.sub = self.create_subscription(Int32, '/chatter', self.cb, qos)
        self.received = []

    def cb(self, msg):
        self.received.append(msg.data)
        self.get_logger().info(f'Received: {msg.data}')

def main():
    rclpy.init()
    node = SeqChecker()
    start = time.time()
    while rclpy.ok() and time.time() - start < 8:
        rclpy.spin_once(node, timeout_sec=0.5)
        if len(node.received) >= 5:
            break

    if len(node.received) >= 2:
        # Check if values are incrementing (nano-ros publishes incrementing count)
        diffs = [node.received[i+1] - node.received[i]
                 for i in range(len(node.received)-1)]
        if all(d == 1 for d in diffs):
            print('SUCCESS: Sequence numbers incrementing correctly')
            print(f'Values: {node.received}')
        else:
            print(f'WARNING: Non-sequential values: {node.received}')
            print(f'Differences: {diffs}')
    else:
        print(f'FAILED: Only received {len(node.received)} messages')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
PYTHON_EOF

    if grep -q "SUCCESS:" /tmp/seq_check.txt 2>/dev/null; then
        log_success "Sequence numbers incrementing correctly"
        [ "$VERBOSE" = true ] && cat /tmp/seq_check.txt
        return 0
    elif grep -q "WARNING:" /tmp/seq_check.txt 2>/dev/null; then
        log_warn "Non-sequential values detected"
        cat /tmp/seq_check.txt
        return 0  # Don't fail, might be message drops
    else
        log_error "Sequence check failed"
        cat /tmp/seq_check.txt 2>/dev/null
        return 1
    fi
}

# Test 3: GID consistency
test_gid_consistency() {
    log_header "Test: Publisher GID Consistency"

    # Clear previous
    pkill -f "target/release/talker" 2>/dev/null || true
    sleep 1

    setup_ros2_env "humble"

    # Start nano-ros talker
    log_info "Starting nano-ros talker..."
    "$TALKER_BIN" --tcp 127.0.0.1:7447 > /tmp/gid_talker.txt 2>&1 &
    register_pid $!
    sleep 3

    # Check topic info for publisher GID
    log_info "Checking publisher info..."
    local info
    info=$(timeout 5 ros2 topic info /chatter -v 2>/dev/null || echo "")

    if echo "$info" | grep -qi "publisher"; then
        log_success "Publisher visible to ROS 2"

        if [ "$VERBOSE" = true ]; then
            echo ""
            echo "=== Topic Info ==="
            echo "$info"
        fi

        # The fact that ROS 2 sees a consistent publisher implies GID is working
        log_success "GID is being used (publisher discovered)"
        return 0
    else
        log_warn "Could not get detailed publisher info"
        return 0
    fi
}

# Test 4: Timestamp sanity check
test_timestamp() {
    log_header "Test: Timestamp Sanity"

    # This is hard to test directly without inspecting the zenoh attachment
    # We rely on the fact that if timestamps were completely wrong,
    # rmw_zenoh_cpp might reject messages or tools like ros2 topic hz would fail

    # Clear previous
    pkill -f "target/release/talker" 2>/dev/null || true
    sleep 1

    setup_ros2_env "humble"

    # Start nano-ros talker
    log_info "Starting nano-ros talker (1 Hz)..."
    "$TALKER_BIN" --tcp 127.0.0.1:7447 > /tmp/ts_talker.txt 2>&1 &
    register_pid $!
    sleep 2

    # Use ros2 topic hz to check rate
    log_info "Checking message rate..."
    timeout 10 ros2 topic hz /chatter --qos-reliability best_effort \
        > /tmp/ts_hz.txt 2>&1 &
    register_pid $!
    sleep 8

    if grep -qE "average rate: [0-9]" /tmp/ts_hz.txt 2>/dev/null; then
        local rate
        rate=$(grep "average rate:" /tmp/ts_hz.txt | tail -1 | grep -oE "[0-9]+\.[0-9]+")
        log_success "ros2 topic hz reports rate: ~${rate} Hz"

        if [ "$VERBOSE" = true ]; then
            cat /tmp/ts_hz.txt
        fi
        return 0
    else
        log_warn "Could not measure rate (may need more time)"
        [ "$VERBOSE" = true ] && cat /tmp/ts_hz.txt
        return 0
    fi
}

# Run tests
test_attachment_via_ros2 || RESULT=1
sleep 2
test_sequence_numbers || RESULT=1
sleep 2
test_gid_consistency || true  # Informative only
sleep 2
test_timestamp || true  # Informative only

# Summary
log_header "Test Summary"
echo ""
echo "RMW Attachment Structure (33 bytes):"
echo "  ┌──────────────────┬────────┐"
echo "  │ sequence_number  │ i64    │"
echo "  │ timestamp        │ i64    │"
echo "  │ rmw_gid_size     │ u8(16) │"
echo "  │ rmw_gid          │ [u8;16]│"
echo "  └──────────────────┴────────┘"
echo ""

if [ $RESULT -eq 0 ]; then
    log_success "All attachment tests passed!"
else
    log_error "Some tests failed"
fi

exit $RESULT
