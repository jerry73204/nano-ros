#!/bin/bash
# Test: QoS compatibility
#
# Verifies that nano-ros QoS settings are compatible with ROS 2.
#
# Current nano-ros QoS: BEST_EFFORT reliability, VOLATILE durability
# QoS string format: reliability:durability:history,depth:...
# Example: 2:2:1,1:,:,:,,
#
# Usage:
#   ./tests/rmw-detailed/qos.sh
#   ./tests/rmw-detailed/qos.sh --verbose

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/../common/utils.sh"
source "$SCRIPT_DIR/../common/prerequisites.sh"

VERBOSE=false
[[ "$1" == "--verbose" || "$1" == "-v" ]] && VERBOSE=true

setup_cleanup

log_header "QoS Compatibility Test"

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

# Test 1: BEST_EFFORT publisher with BEST_EFFORT subscriber
test_best_effort_match() {
    log_header "Test: BEST_EFFORT Publisher → BEST_EFFORT Subscriber"

    setup_ros2_env "humble"

    # Start nano-ros talker (BEST_EFFORT)
    log_info "Starting nano-ros talker (BEST_EFFORT)..."
    RUST_LOG=info "$TALKER_BIN" --tcp 127.0.0.1:7447 > /tmp/qos_talker.txt 2>&1 &
    register_pid $!
    sleep 2

    # ROS 2 subscriber with matching QoS
    log_info "Starting ROS 2 subscriber (BEST_EFFORT)..."
    python3 << 'PYTHON_EOF' > /tmp/qos_sub_be.txt 2>&1
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Int32
import time

class QoSTestSubscriber(Node):
    def __init__(self):
        super().__init__('qos_test_sub')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.sub = self.create_subscription(Int32, '/chatter', self.cb, qos)
        self.count = 0

    def cb(self, msg):
        self.count += 1
        self.get_logger().info(f'Received: {msg.data}')

def main():
    rclpy.init()
    node = QoSTestSubscriber()
    start = time.time()
    while rclpy.ok() and time.time() - start < 10:
        rclpy.spin_once(node, timeout_sec=0.5)
        if node.count >= 3:
            break
    print(f'RESULT: {node.count} messages received')
    if node.count > 0:
        print('SUCCESS')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
PYTHON_EOF

    if grep -q "SUCCESS" /tmp/qos_sub_be.txt 2>/dev/null; then
        log_success "BEST_EFFORT → BEST_EFFORT communication works"
        [ "$VERBOSE" = true ] && cat /tmp/qos_sub_be.txt
        return 0
    else
        log_error "BEST_EFFORT → BEST_EFFORT communication failed"
        cat /tmp/qos_sub_be.txt 2>/dev/null
        return 1
    fi
}

# Test 2: BEST_EFFORT publisher with RELIABLE subscriber (should fail or warn)
test_best_effort_to_reliable() {
    log_header "Test: BEST_EFFORT Publisher → RELIABLE Subscriber"
    log_info "Note: This combination may not receive messages (QoS mismatch)"

    # Clear previous
    pkill -f "/talker" 2>/dev/null || true
    sleep 1

    setup_ros2_env "humble"

    # Start nano-ros talker (BEST_EFFORT)
    log_info "Starting nano-ros talker (BEST_EFFORT)..."
    RUST_LOG=info "$TALKER_BIN" --tcp 127.0.0.1:7447 > /tmp/qos_talker2.txt 2>&1 &
    register_pid $!
    sleep 2

    # ROS 2 subscriber with RELIABLE QoS (stricter)
    log_info "Starting ROS 2 subscriber (RELIABLE - mismatched)..."
    python3 << 'PYTHON_EOF' > /tmp/qos_sub_rel.txt 2>&1
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Int32
import time

class QoSTestSubscriber(Node):
    def __init__(self):
        super().__init__('qos_test_sub_rel')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  # Stricter than publisher
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.sub = self.create_subscription(Int32, '/chatter', self.cb, qos)
        self.count = 0

    def cb(self, msg):
        self.count += 1
        self.get_logger().info(f'Received: {msg.data}')

def main():
    rclpy.init()
    node = QoSTestSubscriber()
    start = time.time()
    while rclpy.ok() and time.time() - start < 8:
        rclpy.spin_once(node, timeout_sec=0.5)
        if node.count >= 1:
            break
    print(f'RESULT: {node.count} messages received')
    if node.count == 0:
        print('EXPECTED: No messages (QoS mismatch)')
    else:
        print('UNEXPECTED: Messages received despite QoS mismatch')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
PYTHON_EOF

    # This is expected behavior - log as info, not error
    if grep -q "EXPECTED:" /tmp/qos_sub_rel.txt 2>/dev/null; then
        log_success "QoS mismatch correctly prevents communication"
    elif grep -q "UNEXPECTED:" /tmp/qos_sub_rel.txt 2>/dev/null; then
        log_warn "Messages received despite QoS mismatch (rmw_zenoh may be lenient)"
    fi

    [ "$VERBOSE" = true ] && cat /tmp/qos_sub_rel.txt
    return 0
}

# Test 3: Verify QoS in liveliness token
test_qos_in_liveliness() {
    log_header "Test: QoS String in Liveliness Token"

    # Clear previous
    pkill -f "/talker" 2>/dev/null || true
    sleep 1

    # Start nano-ros talker with debug output
    log_info "Starting nano-ros talker..."
    RUST_LOG=debug "$TALKER_BIN" --tcp 127.0.0.1:7447 > /tmp/qos_talker3.txt 2>&1 &
    register_pid $!
    sleep 3

    # Check QoS format in liveliness token from debug output
    # Expected: .../RIHS01_<hash>/2:2:1,1:,:,:,,
    # 2:2:1,1 = BEST_EFFORT(2):VOLATILE(2):KEEP_LAST(1),depth(1)

    if grep -q "Publisher liveliness keyexpr:" /tmp/qos_talker3.txt 2>/dev/null; then
        local token
        token=$(grep "Publisher liveliness keyexpr:" /tmp/qos_talker3.txt | head -1)

        if echo "$token" | grep -q "2:2:1,1"; then
            log_success "QoS string found in liveliness token: 2:2:1,1..."

            if [ "$VERBOSE" = true ]; then
                echo "Token with QoS:"
                echo "$token"
            fi

            # Decode QoS values
            log_info "QoS decoded:"
            log_info "  Reliability: 2 (BEST_EFFORT)"
            log_info "  Durability: 2 (VOLATILE)"
            log_info "  History: 1 (KEEP_LAST)"
            log_info "  Depth: 1"

            return 0
        else
            log_warn "QoS string not found in token"
            [ "$VERBOSE" = true ] && echo "Token: $token"
            return 1
        fi
    else
        log_warn "Publisher liveliness token not found in output"
        [ "$VERBOSE" = true ] && cat /tmp/qos_talker3.txt
        return 1
    fi
}

# Test 4: Different QoS combinations (Python publisher test)
test_qos_combinations() {
    log_header "Test: ROS 2 Publisher QoS → nano-ros"

    # Clear previous
    pkill -f "/talker"; pkill -f "/listener" 2>/dev/null || true
    sleep 1

    setup_ros2_env "humble"

    # Start nano-ros listener
    log_info "Starting nano-ros listener..."
    RUST_LOG=info timeout 15 "$LISTENER_BIN" --tcp 127.0.0.1:7447 > /tmp/qos_nano_listener.txt 2>&1 &
    register_pid $!
    sleep 2

    # ROS 2 publisher with BEST_EFFORT
    log_info "Testing ROS 2 BEST_EFFORT publisher..."
    python3 << 'PYTHON_EOF' > /tmp/qos_ros2_pub.txt 2>&1
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Int32
import time

class QoSTestPublisher(Node):
    def __init__(self):
        super().__init__('qos_test_pub')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.pub = self.create_publisher(Int32, '/chatter', qos)
        self.timer = self.create_timer(1.0, self.publish)
        self.counter = 200

    def publish(self):
        msg = Int32()
        msg.data = self.counter
        self.pub.publish(msg)
        self.get_logger().info(f'Published: {self.counter}')
        self.counter += 1

def main():
    rclpy.init()
    node = QoSTestPublisher()
    start = time.time()
    while rclpy.ok() and time.time() - start < 8:
        rclpy.spin_once(node, timeout_sec=0.1)
    print(f'Published {node.counter - 200} messages')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
PYTHON_EOF

    sleep 2

    if grep -q "Received:" /tmp/qos_nano_listener.txt 2>/dev/null; then
        local count
        count=$(count_pattern /tmp/qos_nano_listener.txt "Received:")
        log_success "nano-ros received $count messages from ROS 2 BEST_EFFORT publisher"

        if grep -q "data=20[0-9]" /tmp/qos_nano_listener.txt 2>/dev/null; then
            log_success "Data values verified (200+)"
        fi
        return 0
    else
        log_error "nano-ros did not receive messages"
        cat /tmp/qos_nano_listener.txt 2>/dev/null
        return 1
    fi
}

# Run tests
test_best_effort_match || RESULT=1
sleep 2
test_best_effort_to_reliable || true  # Don't fail, just informative
sleep 2
test_qos_in_liveliness || RESULT=1
sleep 2
test_qos_combinations || RESULT=1

# Summary
log_header "Test Summary"
if [ $RESULT -eq 0 ]; then
    log_success "All QoS tests passed!"
else
    log_error "Some tests failed"
fi

exit $RESULT
