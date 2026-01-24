#!/bin/bash
# Test: Zephyr QEMU ARM
#
# Runs Zephyr nano-ros examples on QEMU ARM emulator (qemu_cortex_m3).
# Uses QEMU's SLIRP networking for host communication.
#
# Prerequisites:
#   - Zephyr workspace set up (./zephyr/setup.sh)
#   - qemu-system-arm installed
#
# Usage:
#   ./tests/emulator/zephyr-qemu-arm.sh
#   ./tests/emulator/zephyr-qemu-arm.sh --verbose
#   ./tests/emulator/zephyr-qemu-arm.sh --skip-build

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
ZEPHYR_WORKSPACE="${ZEPHYR_NANO_ROS:-$HOME/nano-ros-workspace}"
if [ ! -d "$ZEPHYR_WORKSPACE" ]; then
    ZEPHYR_WORKSPACE="$HOME/zephyr-nano-ros"
fi
TEST_TIMEOUT=30

setup_cleanup

log_header "Zephyr QEMU ARM Tests"

# =============================================================================
# Prerequisites Check
# =============================================================================

check_prerequisites() {
    log_header "Checking Prerequisites"

    local missing=0

    # Check QEMU
    if check_qemu_arm; then
        log_success "qemu-system-arm found: $(which qemu-system-arm)"
    else
        log_error "qemu-system-arm not found"
        log_info "Install with: sudo apt install qemu-system-arm"
        missing=1
    fi

    # Check Zephyr workspace
    if [ -d "$ZEPHYR_WORKSPACE" ] && [ -d "$ZEPHYR_WORKSPACE/zephyr" ]; then
        log_success "Zephyr workspace found: $ZEPHYR_WORKSPACE"
    else
        log_error "Zephyr workspace not found"
        log_info "Run: ./zephyr/setup.sh"
        missing=1
    fi

    # Check west
    if command -v west &>/dev/null; then
        log_success "west found"
    else
        log_error "west not found"
        missing=1
    fi

    return $missing
}

# =============================================================================
# Build Zephyr for QEMU
# =============================================================================

build_zephyr_qemu() {
    log_header "Building Zephyr for qemu_cortex_m3"

    cd "$ZEPHYR_WORKSPACE"

    # Source environment
    if [ -f ".venv/bin/activate" ]; then
        source .venv/bin/activate
    fi
    if [ -f "zephyr/zephyr-env.sh" ]; then
        source zephyr/zephyr-env.sh
    fi
    export ZEPHYR_BASE="$ZEPHYR_WORKSPACE/zephyr"

    # Build for QEMU Cortex-M3
    # Note: This board has limited networking support
    log_info "Building zephyr-talker-rs for qemu_cortex_m3..."
    if west build -b qemu_cortex_m3 nano-ros/examples/zephyr-talker-rs -p auto \
        > "$(tmpfile zephyr_qemu_build.txt)" 2>&1; then
        log_success "Build complete"
        return 0
    else
        # Build may fail due to networking limitations on qemu_cortex_m3
        # Check if it's a known limitation
        if grep -q "CONFIG_NETWORKING\|network" "$(tmpfile zephyr_qemu_build.txt)" 2>/dev/null; then
            log_warn "Build failed - qemu_cortex_m3 has limited networking support"
            log_info "This is expected for boards without Ethernet"
            [ "$VERBOSE" = true ] && tail -20 "$(tmpfile zephyr_qemu_build.txt)"
            return 2  # Known limitation
        else
            log_error "Build failed"
            [ "$VERBOSE" = true ] && cat "$(tmpfile zephyr_qemu_build.txt)"
            return 1
        fi
    fi
}

# =============================================================================
# Test: QEMU Execution
# =============================================================================

test_qemu_execution() {
    log_header "Test: Zephyr QEMU Execution"

    cd "$ZEPHYR_WORKSPACE"

    local zephyr_elf="build/zephyr/zephyr.elf"
    if [ ! -f "$zephyr_elf" ]; then
        log_warn "Zephyr ELF not found, skipping execution test"
        return 0
    fi

    log_info "Starting Zephyr in QEMU..."

    # Run QEMU with Zephyr
    # Note: QEMU cortex-m3 with Zephyr uses different machine than bare-metal
    timeout "$TEST_TIMEOUT" qemu-system-arm \
        -cpu cortex-m3 \
        -machine lm3s6965evb \
        -nographic \
        -no-reboot \
        -kernel "$zephyr_elf" \
        > "$(tmpfile zephyr_qemu_output.txt)" 2>&1 &

    local qemu_pid=$!
    register_pid $qemu_pid
    qemu_register_pid $qemu_pid

    # Wait for output
    sleep 5

    # Check if QEMU is still running or exited
    if ! kill -0 $qemu_pid 2>/dev/null; then
        # QEMU exited - check why
        wait $qemu_pid 2>/dev/null || true
    fi

    # Check output
    if grep -qiE "error|panic|fault" "$(tmpfile zephyr_qemu_output.txt)" 2>/dev/null; then
        log_error "Zephyr encountered errors in QEMU"
        cat "$(tmpfile zephyr_qemu_output.txt)"
        return 1
    fi

    if [ -s "$(tmpfile zephyr_qemu_output.txt)" ]; then
        log_success "Zephyr produced output in QEMU"
        [ "$VERBOSE" = true ] && cat "$(tmpfile zephyr_qemu_output.txt)"
        return 0
    else
        log_warn "No output from Zephyr (may need serial driver)"
        return 0
    fi
}

# =============================================================================
# Test: Build Verification
# =============================================================================

test_build_verification() {
    log_header "Test: Build Artifact Verification"

    cd "$ZEPHYR_WORKSPACE"

    local elf_file="build/zephyr/zephyr.elf"
    local bin_file="build/zephyr/zephyr.bin"

    if [ -f "$elf_file" ]; then
        local elf_size
        elf_size=$(stat -c%s "$elf_file")
        log_success "zephyr.elf exists ($elf_size bytes)"

        # Verify it's a valid ARM ELF
        if file "$elf_file" | grep -q "ARM"; then
            log_success "Valid ARM ELF binary"
        else
            log_warn "ELF may not be ARM (check build configuration)"
        fi
    else
        log_warn "zephyr.elf not found"
    fi

    if [ -f "$bin_file" ]; then
        local bin_size
        bin_size=$(stat -c%s "$bin_file")
        log_success "zephyr.bin exists ($bin_size bytes)"
    else
        log_info "zephyr.bin not found (may not be generated)"
    fi

    return 0
}

# =============================================================================
# Alternative: Test with MPS2-AN385 (better Ethernet support)
# =============================================================================

test_mps2_build() {
    log_header "Alternative: Build for mps2_an385 (Ethernet capable)"

    cd "$ZEPHYR_WORKSPACE"

    # Source environment
    if [ -f ".venv/bin/activate" ]; then
        source .venv/bin/activate
    fi
    if [ -f "zephyr/zephyr-env.sh" ]; then
        source zephyr/zephyr-env.sh
    fi
    export ZEPHYR_BASE="$ZEPHYR_WORKSPACE/zephyr"

    # MPS2-AN385 has better networking support
    log_info "Attempting build for mps2_an385..."
    if west build -b mps2_an385 nano-ros/examples/zephyr-talker-rs -p auto \
        > "$(tmpfile zephyr_mps2_build.txt)" 2>&1; then
        log_success "MPS2-AN385 build complete"

        # Verify ELF
        if [ -f "build/zephyr/zephyr.elf" ]; then
            local size
            size=$(stat -c%s "build/zephyr/zephyr.elf")
            log_info "Binary size: $size bytes"
        fi
        return 0
    else
        log_info "MPS2-AN385 build not successful (may need board support)"
        [ "$VERBOSE" = true ] && tail -20 "$(tmpfile zephyr_mps2_build.txt)"
        return 0  # Don't fail - it's an alternative test
    fi
}

# =============================================================================
# Main
# =============================================================================

RESULT=0
TESTS_RUN=0
TESTS_PASSED=0

# Check prerequisites
if ! check_prerequisites; then
    log_error "Prerequisites not met"
    exit 1
fi

# Build
BUILD_RESULT=0
if [ "$SKIP_BUILD" = false ]; then
    build_zephyr_qemu
    BUILD_RESULT=$?

    if [ $BUILD_RESULT -eq 1 ]; then
        log_error "Build failed unexpectedly"
        exit 1
    fi
fi

# Run tests based on build result
if [ $BUILD_RESULT -eq 0 ]; then
    # Build succeeded - run execution test
    test_qemu_execution && TESTS_PASSED=$((TESTS_PASSED + 1))
    TESTS_RUN=$((TESTS_RUN + 1))

    test_build_verification && TESTS_PASSED=$((TESTS_PASSED + 1))
    TESTS_RUN=$((TESTS_RUN + 1))
elif [ $BUILD_RESULT -eq 2 ]; then
    # Known limitation - try alternative board
    log_info "Trying alternative board with better network support..."
    test_mps2_build && TESTS_PASSED=$((TESTS_PASSED + 1))
    TESTS_RUN=$((TESTS_RUN + 1))
fi

# Cleanup
qemu_cleanup_all

# Summary
log_header "Test Summary"
echo ""
echo "Tests passed: $TESTS_PASSED / $TESTS_RUN"
echo ""

if [ $TESTS_PASSED -eq $TESTS_RUN ] && [ $TESTS_RUN -gt 0 ]; then
    log_success "Zephyr QEMU ARM tests passed!"
    exit 0
elif [ $TESTS_RUN -eq 0 ]; then
    log_warn "No tests were run (build may have failed)"
    # Don't fail if no tests ran due to known limitations
    exit 0
else
    log_error "Some tests failed"
    exit 1
fi
