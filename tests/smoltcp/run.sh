#!/bin/bash
# smoltcp Integration Test Runner
#
# Runs all smoltcp platform layer tests for zenoh-pico-shim-sys.
#
# Usage:
#   ./tests/smoltcp/run.sh              # Run all tests
#   ./tests/smoltcp/run.sh --verbose    # Verbose output
#   ./tests/smoltcp/run.sh --quick      # Quick tests (allocator + clock only)
#   ./tests/smoltcp/run.sh allocator    # Run only allocator tests
#   ./tests/smoltcp/run.sh sockets      # Run only socket tests
#   ./tests/smoltcp/run.sh clock        # Run only clock tests
#   ./tests/smoltcp/run.sh poll         # Run only poll tests

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/../common/utils.sh"

# Parse arguments
TEST_SUITE=""
VERBOSE=""
QUICK=false

while [[ $# -gt 0 ]]; do
    case $1 in
        allocator|sockets|clock|poll)
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
            echo "  allocator  - Bump allocator tests"
            echo "  sockets    - Socket buffer tests"
            echo "  clock      - Clock synchronization tests"
            echo "  poll       - Poll callback tests"
            echo ""
            echo "Options:"
            echo "  --quick, -q   - Run quick tests only (allocator + clock)"
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
}

# Main
log_header "smoltcp Integration Test Runner"
echo ""
log_info "Test directory: $SCRIPT_DIR"
log_info "Project root: $PROJECT_ROOT"
echo ""

# Run selected tests
case "$TEST_SUITE" in
    allocator)
        run_suite "smoltcp/allocator" "$SCRIPT_DIR/allocator.sh"
        ;;
    sockets)
        run_suite "smoltcp/socket-buffers" "$SCRIPT_DIR/socket-buffers.sh"
        ;;
    clock)
        run_suite "smoltcp/clock-sync" "$SCRIPT_DIR/clock-sync.sh"
        ;;
    poll)
        run_suite "smoltcp/poll-callback" "$SCRIPT_DIR/poll-callback.sh"
        ;;
    "")
        if [ "$QUICK" = true ]; then
            # Quick mode: allocator + clock only
            log_info "Running quick smoltcp tests..."
            run_suite "smoltcp/allocator" "$SCRIPT_DIR/allocator.sh"
            run_suite "smoltcp/clock-sync" "$SCRIPT_DIR/clock-sync.sh"
        else
            # Full mode: run all tests
            log_info "Running all smoltcp tests..."
            run_suite "smoltcp/allocator" "$SCRIPT_DIR/allocator.sh"
            run_suite "smoltcp/socket-buffers" "$SCRIPT_DIR/socket-buffers.sh"
            run_suite "smoltcp/clock-sync" "$SCRIPT_DIR/clock-sync.sh"
            run_suite "smoltcp/poll-callback" "$SCRIPT_DIR/poll-callback.sh"
        fi
        ;;
esac

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
    log_success "All smoltcp tests passed!"
    exit 0
else
    log_error "Some smoltcp tests failed"
    exit 1
fi
