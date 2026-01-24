#!/bin/bash
# Test: QEMU Cortex-M3 Bare-Metal
#
# Runs nano-ros tests on QEMU's Cortex-M3 emulator without an OS.
# This verifies that CDR serialization and Node API work on embedded targets.
#
# Prerequisites:
#   - qemu-system-arm installed
#   - thumbv7m-none-eabi target installed
#
# Usage:
#   ./tests/emulator/qemu-cortex-m3.sh
#   ./tests/emulator/qemu-cortex-m3.sh --verbose
#   ./tests/emulator/qemu-cortex-m3.sh --skip-build

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/../common/utils.sh"
source "$SCRIPT_DIR/common/qemu-utils.sh"

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
QEMU_TEST_DIR="$PROJECT_ROOT/examples/qemu-test"
QEMU_TIMEOUT=30

setup_cleanup

log_header "QEMU Cortex-M3 Bare-Metal Tests"

# =============================================================================
# Prerequisites Check
# =============================================================================

check_qemu_prerequisites() {
    log_header "Checking Prerequisites"

    local missing=0

    # Check qemu-system-arm
    if check_qemu_arm; then
        log_success "qemu-system-arm found: $(which qemu-system-arm)"
    else
        log_error "qemu-system-arm not found"
        log_info "Install with: sudo apt install qemu-system-arm"
        missing=1
    fi

    # Check ARM toolchain
    if check_arm_toolchain; then
        log_success "thumbv7m-none-eabi target installed"
    else
        log_warn "thumbv7m-none-eabi target not installed"
        log_info "Installing..."
        install_arm_toolchain
        if check_arm_toolchain; then
            log_success "thumbv7m-none-eabi target installed"
        else
            log_error "Failed to install thumbv7m-none-eabi"
            missing=1
        fi
    fi

    # Check qemu-test example exists
    if [ -d "$QEMU_TEST_DIR" ]; then
        log_success "qemu-test example found"
    else
        log_error "qemu-test example not found at $QEMU_TEST_DIR"
        missing=1
    fi

    return $missing
}

# =============================================================================
# Build qemu-test
# =============================================================================

build_qemu_test() {
    log_header "Building qemu-test"

    log_info "Building for thumbv7m-none-eabi (release)..."

    if (cd "$QEMU_TEST_DIR" && cargo build --target thumbv7m-none-eabi --release) \
        > "$(tmpfile qemu_build.txt)" 2>&1; then
        log_success "Build complete"

        local binary_path="$QEMU_TEST_DIR/target/thumbv7m-none-eabi/release/qemu-test"
        if [ -f "$binary_path" ]; then
            local size
            size=$(stat -c%s "$binary_path")
            log_info "Binary size: $size bytes"
        fi
        return 0
    else
        log_error "Build failed"
        [ "$VERBOSE" = true ] && cat "$(tmpfile qemu_build.txt)"
        return 1
    fi
}

# =============================================================================
# Run QEMU Tests
# =============================================================================

run_qemu_tests() {
    log_header "Running QEMU Tests"

    local binary_path="$QEMU_TEST_DIR/target/thumbv7m-none-eabi/release/qemu-test"
    local output_file
    output_file="$(tmpfile qemu_output.txt)"

    if [ ! -f "$binary_path" ]; then
        log_error "Binary not found: $binary_path"
        return 1
    fi

    log_info "Starting QEMU Cortex-M3 (timeout: ${QEMU_TIMEOUT}s)..."
    log_info "Binary: $binary_path"

    # Run QEMU
    if qemu_run_cortex_m3 "$binary_path" "$output_file" "$QEMU_TIMEOUT"; then
        log_success "QEMU execution completed"
    else
        local exit_code=$?
        if [ $exit_code -eq 1 ]; then
            log_warn "QEMU timed out after ${QEMU_TIMEOUT}s"
        else
            log_error "QEMU execution failed (exit code: $exit_code)"
        fi
    fi

    # Parse results
    local results
    results=$(qemu_parse_test_results "$output_file")
    local passed
    local failed
    read -r passed failed <<< "$results"

    echo ""
    log_info "Test Results: $passed passed, $failed failed"

    # Show output
    if [ "$VERBOSE" = true ] || [ "$failed" -gt 0 ]; then
        echo ""
        echo "=== QEMU Output ==="
        cat "$output_file"
        echo "==================="
    fi

    # Check success
    if qemu_check_success "$output_file"; then
        log_success "All QEMU tests passed!"
        return 0
    else
        log_error "Some QEMU tests failed"
        return 1
    fi
}

# =============================================================================
# Individual Test Verification
# =============================================================================

verify_test_categories() {
    log_header "Verifying Test Categories"

    local output_file
    output_file="$(tmpfile qemu_output.txt)"

    # Check serialization tests
    local ser_tests=0
    if grep -q '\[PASS\] Int32 roundtrip' "$output_file" 2>/dev/null; then
        ser_tests=$((ser_tests + 1))
    fi
    if grep -q '\[PASS\] Float64 roundtrip' "$output_file" 2>/dev/null; then
        ser_tests=$((ser_tests + 1))
    fi
    if grep -q '\[PASS\] Time roundtrip' "$output_file" 2>/dev/null; then
        ser_tests=$((ser_tests + 1))
    fi
    if grep -q '\[PASS\] CDR header' "$output_file" 2>/dev/null; then
        ser_tests=$((ser_tests + 1))
    fi
    log_info "Serialization tests: $ser_tests/4"

    # Check Node API tests
    local node_tests=0
    if grep -q '\[PASS\] Node creation' "$output_file" 2>/dev/null; then
        node_tests=$((node_tests + 1))
    fi
    if grep -q '\[PASS\] Node publisher' "$output_file" 2>/dev/null; then
        node_tests=$((node_tests + 1))
    fi
    if grep -q '\[PASS\] Node subscriber' "$output_file" 2>/dev/null; then
        node_tests=$((node_tests + 1))
    fi
    if grep -q '\[PASS\] Node serialize' "$output_file" 2>/dev/null; then
        node_tests=$((node_tests + 1))
    fi
    log_info "Node API tests: $node_tests/4"

    # Check type metadata
    if grep -q '\[PASS\] Type names' "$output_file" 2>/dev/null; then
        log_info "Type metadata: OK"
    else
        log_warn "Type metadata: MISSING"
    fi
}

# =============================================================================
# Main
# =============================================================================

RESULT=0

# Check prerequisites
if ! check_qemu_prerequisites; then
    log_error "Prerequisites not met"
    exit 1
fi

# Build
if [ "$SKIP_BUILD" = false ]; then
    if ! build_qemu_test; then
        log_error "Build failed"
        exit 1
    fi
fi

# Run tests
if ! run_qemu_tests; then
    RESULT=1
fi

# Verify categories
verify_test_categories

# Summary
log_header "Test Summary"
echo ""
if [ $RESULT -eq 0 ]; then
    log_success "QEMU Cortex-M3 tests passed!"
else
    log_error "QEMU Cortex-M3 tests failed"
fi

exit $RESULT
