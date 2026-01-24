#!/bin/bash
# QEMU management utilities for emulator tests
#
# This file provides common functions for managing QEMU processes,
# capturing semihosting output, and handling timeouts.
#
# Usage:
#   source "$(dirname "$0")/common/qemu-utils.sh"

# =============================================================================
# QEMU Process Management
# =============================================================================

# Array to track QEMU processes for cleanup
QEMU_PIDS=()

# Register a QEMU process for cleanup
qemu_register_pid() {
    QEMU_PIDS+=("$1")
}

# Stop all registered QEMU processes
qemu_cleanup_all() {
    for pid in "${QEMU_PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -TERM "$pid" 2>/dev/null || true
            sleep 0.5
            kill -9 "$pid" 2>/dev/null || true
        fi
    done
    QEMU_PIDS=()
}

# Wait for QEMU process to complete or timeout
# Args: pid timeout_seconds
# Returns: 0 on success, 1 on timeout, 2 on failure
qemu_wait_for_completion() {
    local pid="$1"
    local timeout="${2:-30}"
    local elapsed=0

    while [ $elapsed -lt $timeout ]; do
        if ! kill -0 "$pid" 2>/dev/null; then
            # Process exited
            wait "$pid"
            return $?
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done

    # Timeout - kill the process
    kill -TERM "$pid" 2>/dev/null || true
    sleep 0.5
    kill -9 "$pid" 2>/dev/null || true
    return 1
}

# =============================================================================
# QEMU Execution Helpers
# =============================================================================

# Run QEMU Cortex-M3 with semihosting
# Args: elf_file output_file [timeout_seconds]
# Returns: 0 on success, non-zero on failure
qemu_run_cortex_m3() {
    local elf_file="$1"
    local output_file="$2"
    local timeout="${3:-30}"

    if [ ! -f "$elf_file" ]; then
        echo "Error: ELF file not found: $elf_file" >&2
        return 1
    fi

    # Check for QEMU
    if ! command -v qemu-system-arm &>/dev/null; then
        echo "Error: qemu-system-arm not found" >&2
        echo "Install with: sudo apt install qemu-system-arm" >&2
        return 1
    fi

    # Run QEMU with semihosting
    # - LM3S6965 is a common Cortex-M3 board
    # - Semihosting enabled for printf output
    # - No graphics, just serial/semihosting
    timeout "$timeout" qemu-system-arm \
        -cpu cortex-m3 \
        -machine lm3s6965evb \
        -nographic \
        -semihosting-config enable=on,target=native \
        -kernel "$elf_file" \
        > "$output_file" 2>&1 &

    local qemu_pid=$!
    qemu_register_pid $qemu_pid

    qemu_wait_for_completion $qemu_pid $timeout
    local result=$?

    return $result
}

# Run QEMU ARM with network support (SLIRP)
# Args: kernel_file output_file [timeout_seconds] [extra_args...]
qemu_run_cortex_m3_network() {
    local kernel_file="$1"
    local output_file="$2"
    local timeout="${3:-60}"
    shift 3
    local extra_args=("$@")

    if [ ! -f "$kernel_file" ]; then
        echo "Error: Kernel file not found: $kernel_file" >&2
        return 1
    fi

    if ! command -v qemu-system-arm &>/dev/null; then
        echo "Error: qemu-system-arm not found" >&2
        return 1
    fi

    # Run QEMU with SLIRP networking
    # Port forwarding: host:17447 -> guest:7447 (zenoh)
    timeout "$timeout" qemu-system-arm \
        -cpu cortex-m3 \
        -machine lm3s6965evb \
        -nographic \
        -semihosting-config enable=on,target=native \
        -netdev user,id=net0,hostfwd=tcp::17447-:7447 \
        -net nic,netdev=net0 \
        -kernel "$kernel_file" \
        "${extra_args[@]}" \
        > "$output_file" 2>&1 &

    local qemu_pid=$!
    qemu_register_pid $qemu_pid

    return 0
}

# =============================================================================
# Semihosting Output Parsing
# =============================================================================

# Parse test results from semihosting output
# Args: output_file
# Returns: 0 if all tests passed, 1 otherwise
# Outputs: "passed failed" counts
qemu_parse_test_results() {
    local output_file="$1"

    if [ ! -f "$output_file" ]; then
        echo "0 0"
        return 1
    fi

    local passed=0
    local failed=0

    # Count PASS and FAIL lines
    # Use tr to remove any whitespace/newlines that might affect the integer comparison
    passed=$(grep -c '\[PASS\]' "$output_file" 2>/dev/null | tr -d '[:space:]')
    failed=$(grep -c '\[FAIL\]' "$output_file" 2>/dev/null | tr -d '[:space:]')

    # Default to 0 if empty
    passed=${passed:-0}
    failed=${failed:-0}

    echo "$passed $failed"

    if [ "$failed" -gt 0 ]; then
        return 1
    fi
    return 0
}

# Check if QEMU output indicates success
# Args: output_file
qemu_check_success() {
    local output_file="$1"

    if grep -q "All tests passed" "$output_file" 2>/dev/null; then
        return 0
    fi
    return 1
}

# Wait for specific string in QEMU output
# Args: output_file pattern timeout_seconds
qemu_wait_for_output() {
    local output_file="$1"
    local pattern="$2"
    local timeout="${3:-30}"
    local elapsed=0

    while [ $elapsed -lt $timeout ]; do
        if grep -q "$pattern" "$output_file" 2>/dev/null; then
            return 0
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done

    return 1
}

# =============================================================================
# Build Helpers
# =============================================================================

# Build embedded binary for Cortex-M3
# Args: project_dir [release]
qemu_build_cortex_m3() {
    local project_dir="$1"
    local release="${2:-true}"

    if [ ! -d "$project_dir" ]; then
        echo "Error: Project directory not found: $project_dir" >&2
        return 1
    fi

    local release_flag=""
    if [ "$release" = true ]; then
        release_flag="--release"
    fi

    # Build for thumbv7m-none-eabi
    (cd "$project_dir" && cargo build --target thumbv7m-none-eabi $release_flag)
}

# Get path to built binary
# Args: project_dir binary_name [release]
qemu_get_binary_path() {
    local project_dir="$1"
    local binary_name="$2"
    local release="${3:-true}"

    local profile="debug"
    if [ "$release" = true ]; then
        profile="release"
    fi

    echo "$project_dir/target/thumbv7m-none-eabi/$profile/$binary_name"
}

# =============================================================================
# Prerequisite Checks
# =============================================================================

# Check QEMU ARM is available
check_qemu_arm() {
    if command -v qemu-system-arm &>/dev/null; then
        return 0
    fi
    return 1
}

# Check ARM toolchain is available
check_arm_toolchain() {
    if rustup target list --installed | grep -q thumbv7m-none-eabi; then
        return 0
    fi
    return 1
}

# Install ARM toolchain if missing
install_arm_toolchain() {
    rustup target add thumbv7m-none-eabi
}

# =============================================================================
# Cleanup Hook
# =============================================================================

# Register cleanup on script exit
trap qemu_cleanup_all EXIT
