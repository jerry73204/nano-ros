#!/bin/bash
# Test: ROS 2 talker → nano-ros listener
#
# Verifies that nano-ros can receive messages from ROS 2 nodes
# using rmw_zenoh_cpp.
#
# Usage:
#   ./tests/rmw-interop/ros2nano.sh
#   ./tests/rmw-interop/ros2nano.sh --ros-distro jazzy
#   ./tests/rmw-interop/ros2nano.sh --verbose

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

log_header "ROS 2 Talker → nano-ros Listener Test"
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

# Test with ros2 topic pub
test_with_topic_pub() {
    log_header "Test: Using ros2 topic pub"

    # Setup ROS 2 environment
    setup_ros2_env "$ROS_DISTRO"

    # Start nano-ros listener
    log_info "Starting nano-ros listener..."
    RUST_LOG=info timeout 25 "$LISTENER_BIN" --tcp 127.0.0.1:7447 > "$(tmpfile nano_listener.txt)" 2>&1 &
    local listener_pid=$!
    register_pid $listener_pid
    sleep 3

    # Start ROS 2 publisher
    log_info "Starting ROS 2 topic pub..."
    timeout 20 ros2 topic pub -r 1 /chatter std_msgs/msg/Int32 "{data: 42}" \
        --qos-reliability best_effort > "$(tmpfile ros2_pub.txt)" 2>&1 &
    local ros2_pid=$!
    register_pid $ros2_pid

    # Wait for messages
    log_info "Waiting for nano-ros to receive messages..."
    if wait_for_pattern "$(tmpfile nano_listener.txt)" "Received:" "$TEST_TIMEOUT"; then
        local count
        count=$(count_pattern "$(tmpfile nano_listener.txt)" "Received:")
        log_success "nano-ros received $count messages from ROS 2"

        # Check data integrity
        if grep -q "data=42" "$(tmpfile nano_listener.txt)" 2>/dev/null; then
            log_success "Data integrity verified (data=42)"
        fi

        if [ "$VERBOSE" = true ]; then
            echo ""
            echo "=== ROS 2 Publisher Output ==="
            cat "$(tmpfile ros2_pub.txt)" | head -10
            echo ""
            echo "=== nano-ros Listener Output ==="
            cat "$(tmpfile nano_listener.txt)" | head -15
        fi
        return 0
    else
        log_error "nano-ros did not receive messages"
        echo ""
        echo "=== ROS 2 Publisher Output ==="
        cat "$(tmpfile ros2_pub.txt)" 2>/dev/null || echo "(no output)"
        echo ""
        echo "=== nano-ros Listener Output ==="
        cat "$(tmpfile nano_listener.txt)" 2>/dev/null || echo "(no output)"
        return 1
    fi
}

# Test with Python publisher
test_with_python_publisher() {
    log_header "Test: Using Python Publisher"

    # Kill previous processes
    pkill -f "/listener" 2>/dev/null || true
    pkill -f "ros2 topic pub" 2>/dev/null || true
    sleep 2

    # Setup ROS 2 environment
    setup_ros2_env "$ROS_DISTRO"

    # Start nano-ros listener
    log_info "Starting nano-ros listener..."
    RUST_LOG=info timeout 20 "$LISTENER_BIN" --tcp 127.0.0.1:7447 > "$(tmpfile nano_listener2.txt)" 2>&1 &
    local listener_pid=$!
    register_pid $listener_pid
    sleep 2

    # Run Python publisher
    log_info "Running Python publisher..."
    python3 << 'PYTHON_EOF' > "$(tmpfile ros2_python_pub.txt)" 2>&1
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Int32
import time

class TestPublisher(Node):
    def __init__(self):
        super().__init__('nano_ros_test_publisher')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.publisher = self.create_publisher(Int32, '/chatter', qos)
        self.counter = 100  # Start at 100 to distinguish from nano-ros messages
        self.timer = self.create_timer(1.0, self.publish)
        self.get_logger().info('Publisher ready')

    def publish(self):
        msg = Int32()
        msg.data = self.counter
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {self.counter}')
        self.counter += 1

def main():
    rclpy.init()
    node = TestPublisher()

    start = time.time()
    while rclpy.ok() and time.time() - start < 10:
        rclpy.spin_once(node, timeout_sec=0.1)

    print(f'Published {node.counter - 100} messages')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
PYTHON_EOF

    # Check if nano-ros received
    sleep 2
    if grep -q "Received:" "$(tmpfile nano_listener2.txt)" 2>/dev/null; then
        local count
        count=$(count_pattern "$(tmpfile nano_listener2.txt)" "Received:")
        log_success "nano-ros received $count messages from Python publisher"

        # Check for values >= 100 (from Python publisher)
        if grep -q "data=10[0-9]" "$(tmpfile nano_listener2.txt)" 2>/dev/null; then
            log_success "Received correct data values (100+)"
        fi

        if [ "$VERBOSE" = true ]; then
            cat "$(tmpfile nano_listener2.txt)" | head -15
        fi
        return 0
    else
        log_error "nano-ros did not receive messages"
        cat "$(tmpfile nano_listener2.txt)" 2>/dev/null
        return 1
    fi
}

# Run tests
RESULT=0

test_with_topic_pub || RESULT=1

sleep 2

test_with_python_publisher || RESULT=1

# Summary
log_header "Test Summary"
if [ $RESULT -eq 0 ]; then
    log_success "ROS 2 → nano-ros communication works!"
else
    log_error "Some tests failed"
fi

exit $RESULT
