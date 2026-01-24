#!/bin/bash
# Test: nano-ros talker → ROS 2 listener
#
# Verifies that ROS 2 nodes using rmw_zenoh_cpp can receive messages
# from nano-ros publishers.
#
# Usage:
#   ./tests/rmw-interop/nano2ros.sh
#   ./tests/rmw-interop/nano2ros.sh --ros-distro jazzy
#   ./tests/rmw-interop/nano2ros.sh --verbose

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

log_header "nano-ros Talker → ROS 2 Listener Test"
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

# Test with ros2 topic echo
test_with_topic_echo() {
    log_header "Test: Using ros2 topic echo"

    # Setup ROS 2 environment
    setup_ros2_env "$ROS_DISTRO"

    # Start ROS 2 listener (ros2 topic echo)
    log_info "Starting ROS 2 topic echo..."
    timeout 20 ros2 topic echo /chatter std_msgs/msg/Int32 --qos-reliability best_effort \
        > /tmp/ros2_listener.txt 2>&1 &
    local ros2_pid=$!
    register_pid $ros2_pid
    sleep 3

    # Start nano-ros talker
    log_info "Starting nano-ros talker..."
    RUST_LOG=info "$TALKER_BIN" --tcp 127.0.0.1:7447 > /tmp/nano_talker.txt 2>&1 &
    local talker_pid=$!
    register_pid $talker_pid

    # Wait for messages
    log_info "Waiting for ROS 2 to receive messages..."
    if wait_for_pattern /tmp/ros2_listener.txt "data:" "$TEST_TIMEOUT"; then
        local count
        count=$(count_pattern /tmp/ros2_listener.txt "data:")
        log_success "ROS 2 received $count messages from nano-ros"

        if [ "$VERBOSE" = true ]; then
            echo ""
            echo "=== nano-ros Talker Output ==="
            cat /tmp/nano_talker.txt | head -10
            echo ""
            echo "=== ROS 2 Listener Output ==="
            cat /tmp/ros2_listener.txt | head -20
        fi
        return 0
    else
        log_error "ROS 2 did not receive messages"
        echo ""
        echo "=== nano-ros Talker Output ==="
        cat /tmp/nano_talker.txt 2>/dev/null || echo "(no output)"
        echo ""
        echo "=== ROS 2 Listener Output ==="
        cat /tmp/ros2_listener.txt 2>/dev/null || echo "(no output)"
        return 1
    fi
}

# Test with Python subscriber (more control over QoS)
test_with_python_subscriber() {
    log_header "Test: Using Python Subscriber"

    # Kill previous processes
    pkill -f "/talker" 2>/dev/null || true
    pkill -f "ros2 topic echo" 2>/dev/null || true
    sleep 2

    # Setup ROS 2 environment
    setup_ros2_env "$ROS_DISTRO"

    # Start nano-ros talker
    log_info "Starting nano-ros talker..."
    RUST_LOG=info "$TALKER_BIN" --tcp 127.0.0.1:7447 > /tmp/nano_talker2.txt 2>&1 &
    local talker_pid=$!
    register_pid $talker_pid
    sleep 2

    # Run Python subscriber
    log_info "Running Python subscriber..."
    python3 << 'PYTHON_EOF' > /tmp/ros2_python.txt 2>&1
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Int32
import time

class TestSubscriber(Node):
    def __init__(self):
        super().__init__('nano_ros_test_subscriber')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            Int32, '/chatter', self.callback, qos)
        self.received = []
        self.get_logger().info('Subscriber ready')

    def callback(self, msg):
        self.received.append(msg.data)
        self.get_logger().info(f'Received: {msg.data}')

def main():
    rclpy.init()
    node = TestSubscriber()

    start = time.time()
    while rclpy.ok() and time.time() - start < 12:
        rclpy.spin_once(node, timeout_sec=0.5)
        if len(node.received) >= 3:
            break

    if node.received:
        print(f'SUCCESS: Received {len(node.received)} messages: {node.received[:5]}')
    else:
        print('FAILED: No messages received')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
PYTHON_EOF

    if grep -q "SUCCESS:" /tmp/ros2_python.txt 2>/dev/null; then
        log_success "Python subscriber received messages"
        if [ "$VERBOSE" = true ]; then
            cat /tmp/ros2_python.txt
        fi
        return 0
    else
        log_error "Python subscriber failed"
        cat /tmp/ros2_python.txt 2>/dev/null
        return 1
    fi
}

# Run tests
RESULT=0

test_with_topic_echo || RESULT=1

sleep 2

test_with_python_subscriber || RESULT=1

# Summary
log_header "Test Summary"
if [ $RESULT -eq 0 ]; then
    log_success "nano-ros → ROS 2 communication works!"
else
    log_error "Some tests failed"
fi

exit $RESULT
