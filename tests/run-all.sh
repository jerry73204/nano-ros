#!/bin/bash
# Main test runner for nano-ros integration tests
#
# Usage:
#   ./tests/run-all.sh              # Run all tests
#   ./tests/run-all.sh nano2nano    # Run only nano2nano tests
#   ./tests/run-all.sh rmw-interop  # Run only RMW interop tests
#   ./tests/run-all.sh rmw-detailed # Run only detailed RMW tests
#   ./tests/run-all.sh platform     # Run only platform backend tests
#   ./tests/run-all.sh --quick      # Run quick subset of tests
#   ./tests/run-all.sh --verbose    # Verbose output

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common/utils.sh"

# Parse arguments
TEST_SUITE=""
VERBOSE=""
QUICK=false

while [[ $# -gt 0 ]]; do
    case $1 in
        nano2nano|rmw-interop|rmw-detailed|platform|smoltcp|emulator|zephyr)
            TEST_SUITE="$1"
            shift
            ;;
        --quick|-q)
            QUICK=true
            shift
            ;;
        --verbose|-v)
            VERBOSE="--verbose"
            shift
            ;;
        --help|-h)
            echo "Usage: $0 [TEST_SUITE] [OPTIONS]"
            echo ""
            echo "Test suites:"
            echo "  nano2nano     - nano-ros to nano-ros tests"
            echo "  rmw-interop   - RMW interop tests (nano-ros <-> ROS 2)"
            echo "  rmw-detailed  - Detailed RMW protocol tests"
            echo "  platform      - Platform backend tests (posix, smoltcp, generic)"
            echo "  smoltcp       - smoltcp integration tests (allocator, sockets, clock, poll)"
            echo "  emulator      - Emulator tests (QEMU Cortex-M3, Zephyr native_sim/QEMU)"
            echo "  zephyr        - Zephyr QEMU integration tests (requires setup)"
            echo ""
            echo "Options:"
            echo "  --quick, -q   - Run quick subset of tests"
            echo "  --verbose, -v - Verbose output"
            echo "  --help, -h    - Show this help"
            exit 0
            ;;
        *)
            log_error "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Results tracking
TOTAL_SUITES=0
PASSED_SUITES=0
FAILED_SUITES=()

run_suite() {
    local name="$1"
    local script="$2"

    # Cleanup before starting each suite
    pkill -x zenohd 2>/dev/null || true
    pkill -f "/talker" 2>/dev/null || true
    pkill -f "/listener" 2>/dev/null || true
    pkill -f "ros2 topic" 2>/dev/null || true
    sleep 2

    log_header "Running: $name"
    TOTAL_SUITES=$((TOTAL_SUITES + 1))

    if [ ! -x "$script" ]; then
        chmod +x "$script"
    fi

    if "$script" $VERBOSE; then
        PASSED_SUITES=$((PASSED_SUITES + 1))
        log_success "$name completed"
    else
        FAILED_SUITES+=("$name")
        log_error "$name failed"
    fi

    # Give time for cleanup between suites
    sleep 2
}

# Main
log_header "nano-ros Integration Test Runner"
echo ""
log_info "Test directory: $SCRIPT_DIR"
log_info "Project root: $PROJECT_ROOT"
echo ""

# Ensure we start clean
pkill -x zenohd 2>/dev/null || true
pkill -f "/talker" 2>/dev/null || true
pkill -f "/listener" 2>/dev/null || true
sleep 1

# Build first
log_info "Building nano-ros..."
if ! build_nano_ros; then
    log_error "Build failed, cannot run tests"
    exit 1
fi
echo ""

# Run selected tests
case "$TEST_SUITE" in
    nano2nano)
        run_suite "nano2nano" "$SCRIPT_DIR/nano2nano/run.sh"
        ;;
    rmw-interop)
        if [ "$QUICK" = true ]; then
            run_suite "rmw-interop/matrix" "$SCRIPT_DIR/rmw-interop/matrix.sh"
        else
            run_suite "rmw-interop/nano2ros" "$SCRIPT_DIR/rmw-interop/nano2ros.sh"
            run_suite "rmw-interop/ros2nano" "$SCRIPT_DIR/rmw-interop/ros2nano.sh"
            run_suite "rmw-interop/matrix" "$SCRIPT_DIR/rmw-interop/matrix.sh"
        fi
        ;;
    rmw-detailed)
        if [ "$QUICK" = true ]; then
            run_suite "rmw-detailed/qos" "$SCRIPT_DIR/rmw-detailed/qos.sh"
        else
            run_suite "rmw-detailed/liveliness" "$SCRIPT_DIR/rmw-detailed/liveliness.sh"
            run_suite "rmw-detailed/keyexpr" "$SCRIPT_DIR/rmw-detailed/keyexpr.sh"
            run_suite "rmw-detailed/qos" "$SCRIPT_DIR/rmw-detailed/qos.sh"
            run_suite "rmw-detailed/attachment" "$SCRIPT_DIR/rmw-detailed/attachment.sh"
        fi
        ;;
    platform)
        if [ "$QUICK" = true ]; then
            # Quick mode: skip posix (requires zenohd)
            run_suite "platform/generic" "$SCRIPT_DIR/platform/generic.sh"
            run_suite "platform/smoltcp" "$SCRIPT_DIR/platform/smoltcp-sim.sh"
        else
            run_suite "platform/generic" "$SCRIPT_DIR/platform/generic.sh"
            run_suite "platform/smoltcp" "$SCRIPT_DIR/platform/smoltcp-sim.sh"
            run_suite "platform/posix" "$SCRIPT_DIR/platform/posix.sh"
        fi
        ;;
    smoltcp)
        if [ "$QUICK" = true ]; then
            # Quick mode: allocator + clock only
            run_suite "smoltcp/allocator" "$SCRIPT_DIR/smoltcp/allocator.sh"
            run_suite "smoltcp/clock-sync" "$SCRIPT_DIR/smoltcp/clock-sync.sh"
        else
            run_suite "smoltcp/allocator" "$SCRIPT_DIR/smoltcp/allocator.sh"
            run_suite "smoltcp/socket-buffers" "$SCRIPT_DIR/smoltcp/socket-buffers.sh"
            run_suite "smoltcp/clock-sync" "$SCRIPT_DIR/smoltcp/clock-sync.sh"
            run_suite "smoltcp/poll-callback" "$SCRIPT_DIR/smoltcp/poll-callback.sh"
        fi
        ;;
    emulator)
        if [ "$QUICK" = true ]; then
            # Quick mode: QEMU Cortex-M3 only (fastest)
            run_suite "emulator/qemu-cortex-m3" "$SCRIPT_DIR/emulator/qemu-cortex-m3.sh"
        else
            run_suite "emulator/qemu-cortex-m3" "$SCRIPT_DIR/emulator/qemu-cortex-m3.sh"
            run_suite "emulator/zephyr-native-sim" "$SCRIPT_DIR/emulator/zephyr-native-sim.sh"
            run_suite "emulator/zephyr-qemu-arm" "$SCRIPT_DIR/emulator/zephyr-qemu-arm.sh"
        fi
        ;;
    zephyr)
        # Zephyr tests require separate setup
        log_info "Running Zephyr QEMU tests..."
        log_info "Note: Requires Zephyr workspace setup (./zephyr/setup.sh)"
        run_suite "zephyr" "$SCRIPT_DIR/zephyr/run.sh"
        ;;
    "")
        # Run all tests
        if [ "$QUICK" = true ]; then
            log_info "Running quick test suite..."
            run_suite "platform/generic" "$SCRIPT_DIR/platform/generic.sh"
            run_suite "nano2nano" "$SCRIPT_DIR/nano2nano/run.sh"
            run_suite "rmw-interop/matrix" "$SCRIPT_DIR/rmw-interop/matrix.sh"
        else
            log_info "Running full test suite..."

            # platform (run first - no external dependencies for generic/smoltcp)
            run_suite "platform/generic" "$SCRIPT_DIR/platform/generic.sh"
            run_suite "platform/smoltcp" "$SCRIPT_DIR/platform/smoltcp-sim.sh"
            run_suite "platform/posix" "$SCRIPT_DIR/platform/posix.sh"

            # smoltcp integration tests (no zenohd required)
            run_suite "smoltcp/allocator" "$SCRIPT_DIR/smoltcp/allocator.sh"
            run_suite "smoltcp/socket-buffers" "$SCRIPT_DIR/smoltcp/socket-buffers.sh"
            run_suite "smoltcp/clock-sync" "$SCRIPT_DIR/smoltcp/clock-sync.sh"
            run_suite "smoltcp/poll-callback" "$SCRIPT_DIR/smoltcp/poll-callback.sh"

            # nano2nano
            run_suite "nano2nano" "$SCRIPT_DIR/nano2nano/run.sh"

            # rmw-interop
            run_suite "rmw-interop/nano2ros" "$SCRIPT_DIR/rmw-interop/nano2ros.sh"
            run_suite "rmw-interop/ros2nano" "$SCRIPT_DIR/rmw-interop/ros2nano.sh"
            run_suite "rmw-interop/matrix" "$SCRIPT_DIR/rmw-interop/matrix.sh"

            # rmw-detailed
            run_suite "rmw-detailed/liveliness" "$SCRIPT_DIR/rmw-detailed/liveliness.sh"
            run_suite "rmw-detailed/keyexpr" "$SCRIPT_DIR/rmw-detailed/keyexpr.sh"
            run_suite "rmw-detailed/qos" "$SCRIPT_DIR/rmw-detailed/qos.sh"
            run_suite "rmw-detailed/attachment" "$SCRIPT_DIR/rmw-detailed/attachment.sh"

            # emulator tests (QEMU, Zephyr native_sim)
            run_suite "emulator/qemu-cortex-m3" "$SCRIPT_DIR/emulator/qemu-cortex-m3.sh"
            # Note: Zephyr emulator tests are optional - require Zephyr setup
            if [ -d "${ZEPHYR_NANO_ROS:-$HOME/nano-ros-workspace}/zephyr" ]; then
                run_suite "emulator/zephyr-native-sim" "$SCRIPT_DIR/emulator/zephyr-native-sim.sh"
            fi
        fi
        ;;
esac

# Final cleanup
pkill -x zenohd 2>/dev/null || true
pkill -f "/talker" 2>/dev/null || true
pkill -f "/listener" 2>/dev/null || true

# Summary
log_header "Test Summary"
echo ""
echo "Passed: $PASSED_SUITES / $TOTAL_SUITES"

if [ ${#FAILED_SUITES[@]} -gt 0 ]; then
    echo ""
    echo "Failed suites:"
    for suite in "${FAILED_SUITES[@]}"; do
        echo "  - $suite"
    done
fi

echo ""
if [ $PASSED_SUITES -eq $TOTAL_SUITES ]; then
    log_success "All test suites passed!"
    exit 0
else
    log_error "Some test suites failed"
    exit 1
fi
