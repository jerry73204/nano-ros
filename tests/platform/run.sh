#!/bin/bash
# Platform backend test runner
#
# Runs all platform backend tests for zenoh-pico-shim.
#
# Usage:
#   ./tests/platform/run.sh              # Run all tests
#   ./tests/platform/run.sh --verbose    # Verbose output
#   ./tests/platform/run.sh --quick      # Quick tests (generic + smoltcp only)
#   ./tests/platform/run.sh posix        # Run only posix tests
#   ./tests/platform/run.sh smoltcp      # Run only smoltcp tests
#   ./tests/platform/run.sh generic      # Run only generic tests

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/../common/utils.sh"

# Parse arguments
TEST_SUITE=""
VERBOSE=""
QUICK=false

while [[ $# -gt 0 ]]; do
    case $1 in
        posix|smoltcp|generic)
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
            echo "  posix    - POSIX platform tests (requires zenohd)"
            echo "  smoltcp  - smoltcp platform tests (compile + simulation)"
            echo "  generic  - Generic compile tests (no platform backend)"
            echo ""
            echo "Options:"
            echo "  --quick, -q   - Run quick tests only (generic + smoltcp)"
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

    # Cleanup between suites
    pkill -x zenohd 2>/dev/null || true
    sleep 1
}

# Main
log_header "Platform Backend Test Runner"
echo ""
log_info "Test directory: $SCRIPT_DIR"
log_info "Project root: $PROJECT_ROOT"
echo ""

# Run selected tests
case "$TEST_SUITE" in
    posix)
        run_suite "platform/posix" "$SCRIPT_DIR/posix.sh"
        ;;
    smoltcp)
        run_suite "platform/smoltcp" "$SCRIPT_DIR/smoltcp-sim.sh"
        ;;
    generic)
        run_suite "platform/generic" "$SCRIPT_DIR/generic.sh"
        ;;
    "")
        if [ "$QUICK" = true ]; then
            # Quick mode: skip posix (requires zenohd)
            log_info "Running quick platform tests (no zenohd required)..."
            run_suite "platform/generic" "$SCRIPT_DIR/generic.sh"
            run_suite "platform/smoltcp" "$SCRIPT_DIR/smoltcp-sim.sh"
        else
            # Full mode: run all tests
            log_info "Running all platform tests..."
            run_suite "platform/generic" "$SCRIPT_DIR/generic.sh"
            run_suite "platform/smoltcp" "$SCRIPT_DIR/smoltcp-sim.sh"
            run_suite "platform/posix" "$SCRIPT_DIR/posix.sh"
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
    log_success "All platform tests passed!"
    exit 0
else
    log_error "Some platform tests failed"
    exit 1
fi
