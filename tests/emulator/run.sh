#!/bin/bash
# Emulator Test Runner
#
# Runs all emulator tests for nano-ros:
# - QEMU Cortex-M3 bare-metal tests
# - Zephyr native_sim tests
# - Zephyr QEMU ARM tests
#
# Usage:
#   ./tests/emulator/run.sh              # Run all tests
#   ./tests/emulator/run.sh --quick      # Quick tests only
#   ./tests/emulator/run.sh --verbose    # Verbose output
#   ./tests/emulator/run.sh qemu         # QEMU Cortex-M3 only
#   ./tests/emulator/run.sh native-sim   # Zephyr native_sim only
#   ./tests/emulator/run.sh qemu-arm     # Zephyr QEMU ARM only

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/../common/utils.sh"

# Parse arguments
TEST_SUITE=""
VERBOSE=""
QUICK=""

while [[ $# -gt 0 ]]; do
    case $1 in
        qemu|native-sim|qemu-arm)
            TEST_SUITE="$1"
            shift
            ;;
        --quick|-q)
            QUICK="--quick"
            shift
            ;;
        --verbose|-v)
            VERBOSE="--verbose"
            shift
            ;;
        --skip-build)
            SKIP_BUILD="--skip-build"
            shift
            ;;
        --help|-h)
            echo "Usage: $0 [TEST_SUITE] [OPTIONS]"
            echo ""
            echo "Test suites:"
            echo "  qemu        - QEMU Cortex-M3 bare-metal tests"
            echo "  native-sim  - Zephyr native_sim tests"
            echo "  qemu-arm    - Zephyr QEMU ARM tests"
            echo ""
            echo "Options:"
            echo "  --quick, -q     - Run quick tests only"
            echo "  --verbose, -v   - Verbose output"
            echo "  --skip-build    - Skip build step"
            echo "  --help, -h      - Show this help"
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
SKIPPED_SUITES=()

run_suite() {
    local name="$1"
    local script="$2"
    local required="${3:-true}"

    log_header "Running: $name"
    TOTAL_SUITES=$((TOTAL_SUITES + 1))

    if [ ! -x "$script" ]; then
        chmod +x "$script"
    fi

    # Check prerequisites without running full test
    if ! "$script" --help >/dev/null 2>&1; then
        chmod +x "$script"
    fi

    if "$script" $VERBOSE $QUICK $SKIP_BUILD; then
        PASSED_SUITES=$((PASSED_SUITES + 1))
        log_success "$name completed"
    else
        local exit_code=$?
        if [ "$required" = false ]; then
            SKIPPED_SUITES+=("$name")
            log_warn "$name skipped (optional)"
        else
            FAILED_SUITES+=("$name")
            log_error "$name failed"
        fi
    fi

    # Cleanup between suites
    pkill -x zenohd 2>/dev/null || true
    pkill -f "zephyr.exe" 2>/dev/null || true
    pkill -f "qemu-system" 2>/dev/null || true
    sleep 1
}

# Main
log_header "Emulator Test Runner"
echo ""
log_info "Test directory: $SCRIPT_DIR"
log_info "Project root: $PROJECT_ROOT"
echo ""

# Run selected tests
case "$TEST_SUITE" in
    qemu)
        run_suite "qemu-cortex-m3" "$SCRIPT_DIR/qemu-cortex-m3.sh"
        ;;
    native-sim)
        run_suite "zephyr-native-sim" "$SCRIPT_DIR/zephyr-native-sim.sh"
        ;;
    qemu-arm)
        run_suite "zephyr-qemu-arm" "$SCRIPT_DIR/zephyr-qemu-arm.sh" false
        ;;
    "")
        if [ -n "$QUICK" ]; then
            # Quick mode: QEMU only (fastest)
            log_info "Running quick emulator tests..."
            run_suite "qemu-cortex-m3" "$SCRIPT_DIR/qemu-cortex-m3.sh"
        else
            # Full mode: all emulator tests
            log_info "Running all emulator tests..."

            # QEMU Cortex-M3 (no OS, fast)
            run_suite "qemu-cortex-m3" "$SCRIPT_DIR/qemu-cortex-m3.sh"

            # Zephyr native_sim (requires TAP setup)
            run_suite "zephyr-native-sim" "$SCRIPT_DIR/zephyr-native-sim.sh"

            # Zephyr QEMU ARM (optional - may have limitations)
            run_suite "zephyr-qemu-arm" "$SCRIPT_DIR/zephyr-qemu-arm.sh" false
        fi
        ;;
esac

# Final cleanup
pkill -x zenohd 2>/dev/null || true
pkill -f "zephyr.exe" 2>/dev/null || true
pkill -f "qemu-system" 2>/dev/null || true

# Summary
log_header "Test Summary"
echo ""
echo "Passed: $PASSED_SUITES / $TOTAL_SUITES"

if [ ${#SKIPPED_SUITES[@]} -gt 0 ]; then
    echo ""
    echo "Skipped (optional):"
    for suite in "${SKIPPED_SUITES[@]}"; do
        echo "  - $suite"
    done
fi

if [ ${#FAILED_SUITES[@]} -gt 0 ]; then
    echo ""
    echo "Failed:"
    for suite in "${FAILED_SUITES[@]}"; do
        echo "  - $suite"
    done
fi

echo ""
if [ ${#FAILED_SUITES[@]} -eq 0 ]; then
    log_success "All required emulator tests passed!"
    exit 0
else
    log_error "Some emulator tests failed"
    exit 1
fi
