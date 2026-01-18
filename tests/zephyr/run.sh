#!/bin/bash
# Test: Zephyr QEMU Integration
#
# This test verifies that nano-ros running on Zephyr RTOS (in QEMU)
# can communicate with ROS 2 nodes on the host via zenoh.
#
# Prerequisites:
#   - Zephyr workspace set up (./zephyr/setup.sh)
#   - QEMU network configured (sudo ./scripts/qemu/setup-qemu-network.sh)
#   - ROS 2 Humble + rmw_zenoh installed
#   - zenohd installed
#
# Usage:
#   ./tests/zephyr/run.sh
#   ./tests/zephyr/run.sh --verbose
#   ./tests/zephyr/run.sh --skip-build
#
# Note: This test requires the Zephyr workspace to be initialized.
# Run ./zephyr/setup.sh first if not done.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/../common/utils.sh"

# Parse arguments
VERBOSE=false
SKIP_BUILD=false
while [[ $# -gt 0 ]]; do
    case $1 in
        --verbose|-v) VERBOSE=true; shift ;;
        --skip-build) SKIP_BUILD=true; shift ;;
        *) shift ;;
    esac
done

# Configuration
ZEPHYR_WORKSPACE="${ZEPHYR_NANO_ROS:-$HOME/zephyr-nano-ros}"
QEMU_TIMEOUT=30
BRIDGE="br-nano"

setup_cleanup

log_header "Zephyr QEMU Integration Test"

# =============================================================================
# Prerequisites Check
# =============================================================================

check_zephyr_prerequisites() {
    log_header "Checking Zephyr Prerequisites"

    local missing=0

    # Check Zephyr workspace
    if [ -d "$ZEPHYR_WORKSPACE" ] && [ -f "$ZEPHYR_WORKSPACE/zephyr/zephyr-env.sh" ]; then
        log_success "Zephyr workspace found: $ZEPHYR_WORKSPACE"
    else
        log_error "Zephyr workspace not found at $ZEPHYR_WORKSPACE"
        log_info "Run: ./zephyr/setup.sh"
        missing=1
    fi

    # Check west
    if command -v west &>/dev/null; then
        log_success "west found: $(which west)"
    else
        log_error "west not found"
        missing=1
    fi

    # Check QEMU
    if command -v qemu-system-x86_64 &>/dev/null || command -v qemu-system-arm &>/dev/null; then
        log_success "QEMU found"
    else
        log_error "QEMU not found"
        log_info "Install: sudo apt install qemu-system-x86 qemu-system-arm"
        missing=1
    fi

    # Check network bridge
    if ip link show "$BRIDGE" &>/dev/null; then
        log_success "Network bridge $BRIDGE configured"
    else
        log_warn "Network bridge $BRIDGE not found"
        log_info "Run: sudo ./scripts/qemu/setup-qemu-network.sh"
        # Don't fail - might work with user-mode networking
    fi

    # Check zenohd
    if command -v zenohd &>/dev/null; then
        log_success "zenohd found: $(which zenohd)"
    else
        log_error "zenohd not found"
        missing=1
    fi

    # Check ROS 2
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        log_success "ROS 2 Humble found"
    else
        log_warn "ROS 2 Humble not found at /opt/ros/humble"
    fi

    return $missing
}

# =============================================================================
# Build Zephyr Examples
# =============================================================================

build_zephyr_examples() {
    log_header "Building Zephyr Examples"

    cd "$ZEPHYR_WORKSPACE"
    source zephyr/zephyr-env.sh

    # Build talker
    log_info "Building zephyr-talker-rs for qemu_x86..."
    if west build -b qemu_x86 nano-ros/examples/zephyr-talker-rs -d build-talker 2>&1 | tee /tmp/zephyr_build_talker.txt; then
        log_success "Talker build complete"
    else
        log_error "Talker build failed"
        [ "$VERBOSE" = true ] && cat /tmp/zephyr_build_talker.txt
        return 1
    fi

    # Build listener
    log_info "Building zephyr-listener-rs for qemu_x86..."
    if west build -b qemu_x86 nano-ros/examples/zephyr-listener-rs -d build-listener 2>&1 | tee /tmp/zephyr_build_listener.txt; then
        log_success "Listener build complete"
    else
        log_error "Listener build failed"
        [ "$VERBOSE" = true ] && cat /tmp/zephyr_build_listener.txt
        return 1
    fi

    return 0
}

# =============================================================================
# Test: Zephyr Talker → ROS 2 Listener
# =============================================================================

test_zephyr_to_ros2() {
    log_header "Test: Zephyr Talker → ROS 2 Listener"

    cd "$ZEPHYR_WORKSPACE"
    source zephyr/zephyr-env.sh

    # Start zenoh router
    log_info "Starting zenoh router..."
    zenohd --listen tcp/0.0.0.0:7447 > /tmp/zephyr_zenohd.txt 2>&1 &
    local zenohd_pid=$!
    register_pid $zenohd_pid
    sleep 2

    # Setup ROS 2
    source /opt/ros/humble/setup.bash
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/127.0.0.1:7447"]'

    # Start ROS 2 listener
    log_info "Starting ROS 2 listener..."
    timeout 25 ros2 topic echo /chatter std_msgs/msg/Int32 --qos-reliability best_effort \
        > /tmp/zephyr_ros2_listener.txt 2>&1 &
    local ros2_pid=$!
    register_pid $ros2_pid
    sleep 3

    # Start Zephyr talker in QEMU
    log_info "Starting Zephyr talker in QEMU..."
    cd build-talker
    timeout "$QEMU_TIMEOUT" west build -t run > /tmp/zephyr_qemu_talker.txt 2>&1 &
    local qemu_pid=$!
    register_pid $qemu_pid

    # Wait for communication
    log_info "Waiting for messages..."
    sleep 15

    # Check results
    if grep -q "data:" /tmp/zephyr_ros2_listener.txt 2>/dev/null; then
        local count
        count=$(grep -c "data:" /tmp/zephyr_ros2_listener.txt 2>/dev/null || echo 0)
        log_success "ROS 2 received $count messages from Zephyr!"

        if [ "$VERBOSE" = true ]; then
            echo ""
            echo "=== ROS 2 Output ==="
            head -20 /tmp/zephyr_ros2_listener.txt
            echo ""
            echo "=== QEMU Output ==="
            head -30 /tmp/zephyr_qemu_talker.txt
        fi
        return 0
    else
        log_error "ROS 2 did not receive messages from Zephyr"
        echo ""
        echo "=== QEMU Output ==="
        cat /tmp/zephyr_qemu_talker.txt 2>/dev/null | head -50
        echo ""
        echo "=== ROS 2 Output ==="
        cat /tmp/zephyr_ros2_listener.txt 2>/dev/null
        return 1
    fi
}

# =============================================================================
# Test: ROS 2 Talker → Zephyr Listener
# =============================================================================

test_ros2_to_zephyr() {
    log_header "Test: ROS 2 Talker → Zephyr Listener"

    # Cleanup previous
    pkill -f "west build" 2>/dev/null || true
    pkill -f "qemu" 2>/dev/null || true
    sleep 2

    cd "$ZEPHYR_WORKSPACE"
    source zephyr/zephyr-env.sh

    # Start zenoh router (if not running)
    if ! pgrep -x zenohd > /dev/null; then
        log_info "Starting zenoh router..."
        zenohd --listen tcp/0.0.0.0:7447 > /tmp/zephyr_zenohd2.txt 2>&1 &
        register_pid $!
        sleep 2
    fi

    # Start Zephyr listener in QEMU
    log_info "Starting Zephyr listener in QEMU..."
    cd build-listener
    timeout "$QEMU_TIMEOUT" west build -t run > /tmp/zephyr_qemu_listener.txt 2>&1 &
    local qemu_pid=$!
    register_pid $qemu_pid
    sleep 5

    # Setup ROS 2 and publish
    source /opt/ros/humble/setup.bash
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/127.0.0.1:7447"]'

    log_info "Starting ROS 2 publisher..."
    timeout 15 ros2 topic pub -r 1 /chatter std_msgs/msg/Int32 "{data: 999}" \
        --qos-reliability best_effort > /tmp/zephyr_ros2_pub.txt 2>&1 &
    register_pid $!

    # Wait for communication
    sleep 12

    # Check results
    if grep -q "Received:" /tmp/zephyr_qemu_listener.txt 2>/dev/null; then
        local count
        count=$(grep -c "Received:" /tmp/zephyr_qemu_listener.txt 2>/dev/null || echo 0)
        log_success "Zephyr received $count messages from ROS 2!"

        if grep -q "data=999" /tmp/zephyr_qemu_listener.txt 2>/dev/null; then
            log_success "Data integrity verified (data=999)"
        fi

        if [ "$VERBOSE" = true ]; then
            echo ""
            echo "=== QEMU Output ==="
            cat /tmp/zephyr_qemu_listener.txt
        fi
        return 0
    else
        log_error "Zephyr did not receive messages from ROS 2"
        echo ""
        echo "=== QEMU Output ==="
        cat /tmp/zephyr_qemu_listener.txt 2>/dev/null
        return 1
    fi
}

# =============================================================================
# Main
# =============================================================================

RESULT=0

# Check prerequisites
if ! check_zephyr_prerequisites; then
    log_error "Prerequisites not met"
    log_info ""
    log_info "To set up the Zephyr workspace:"
    log_info "  ./zephyr/setup.sh"
    log_info ""
    log_info "To configure QEMU networking:"
    log_info "  sudo ./scripts/qemu/setup-qemu-network.sh"
    exit 1
fi

# Build examples
if [ "$SKIP_BUILD" = false ]; then
    if ! build_zephyr_examples; then
        log_error "Build failed"
        exit 1
    fi
fi

# Run tests
test_zephyr_to_ros2 || RESULT=1
sleep 3
test_ros2_to_zephyr || RESULT=1

# Summary
log_header "Test Summary"
if [ $RESULT -eq 0 ]; then
    log_success "All Zephyr QEMU tests passed!"
else
    log_error "Some tests failed"
fi

exit $RESULT
