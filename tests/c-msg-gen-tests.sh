#!/bin/bash
# Integration test for C message generation
#
# This script tests the full pipeline:
# 1. Check the canonical nros CLI is installed
# 2. Build nros-c library
# 3. Run CMake on native-c-custom-msg example
# 4. Build and run the test executable
#
# Usage: ./tests/c-msg-gen-tests.sh
#
# Exit codes:
#   0 - All tests passed
#   1 - Test failure or error

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# shellcheck source=lib/common.sh
source "$SCRIPT_DIR/lib/common.sh"
# shellcheck source=../scripts/build/cargo.sh
source "$PROJECT_ROOT/scripts/build/cargo.sh"

# Clean up on exit
cleanup() {
    if [ -n "${BUILD_DIR:-}" ] && [ -d "$BUILD_DIR" ]; then
        rm -rf "$BUILD_DIR"
    fi
}

trap cleanup EXIT

# ============================================================================
# Step 1: Check nros CLI
# ============================================================================

NROS_CLI_BIN="$(nros_cli_bin)"
log_info "Using nros CLI: $NROS_CLI_BIN"

# ============================================================================
# Step 2: Configure native-c-custom-msg example
# ============================================================================
#
# Phase 140 — the example consumes nano-ros via
# `add_subdirectory(<repo-root>)`; no pre-installed prefix needed.

log_info "Configuring native-c-custom-msg example..."

EXAMPLE_DIR="$PROJECT_ROOT/examples/native/c/custom-msg"
BUILD_DIR="$EXAMPLE_DIR/build"

# Clean any existing build
rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# Configure with CMake — the example's CMakeLists.txt drives add_subdirectory.
cmake -DNANO_ROS_PLATFORM=posix -DNANO_ROS_RMW=zenoh -DCMAKE_BUILD_TYPE=Release ..

log_info "CMake configuration successful"

# ============================================================================
# Step 3: Build the example
# ============================================================================

log_info "Building native-c-custom-msg example..."

cmake --build . --parallel

log_info "Build successful"

# ============================================================================
# Step 4: Run the test executable
# ============================================================================

log_info "Running test executable..."

TEST_EXEC="$BUILD_DIR/test_messages"
if [ ! -f "$TEST_EXEC" ]; then
    log_error "Test executable not found at: $TEST_EXEC"
    exit 1
fi

# Run the test
OUTPUT=$("$TEST_EXEC" 2>&1)
RESULT=$?

echo "$OUTPUT"

if [ $RESULT -ne 0 ]; then
    log_error "Test executable failed with exit code: $RESULT"
    exit 1
fi

# Check for expected output
if nros_grep_q "All tests passed" <<<"$OUTPUT"; then
    log_info "Test executable reported success"
else
    # Check for individual test results
    if nros_grep_q "Temperature" <<<"$OUTPUT" && nros_grep_q "SensorData" <<<"$OUTPUT"; then
        log_info "Found expected message types in output"
    else
        log_warn "Test output may not contain expected content"
    fi
fi

# ============================================================================
# Step 5: Verify generated files
# ============================================================================

log_info "Verifying generated files..."

GEN_DIR="$BUILD_DIR/nano_ros_c/native_c_custom_msg"

# Check for expected generated files
EXPECTED_FILES=(
    "msg/native_c_custom_msg_msg_temperature.h"
    "msg/native_c_custom_msg_msg_temperature.c"
    "msg/native_c_custom_msg_msg_sensor_data.h"
    "msg/native_c_custom_msg_msg_sensor_data.c"
    "native_c_custom_msg.h"
)

for FILE in "${EXPECTED_FILES[@]}"; do
    if [ ! -f "$GEN_DIR/$FILE" ]; then
        log_error "Expected file not found: $GEN_DIR/$FILE"
        exit 1
    fi
done

log_info "All expected files generated"

# ============================================================================
# Summary
# ============================================================================

echo ""
echo "=============================================="
echo -e "${GREEN}All C message generation tests passed!${NC}"
echo "=============================================="
echo ""
echo "Generated files:"
find "$GEN_DIR" -type f -name "*.h" -o -name "*.c" | sort | while read f; do
    echo "  - ${f#$GEN_DIR/}"
done
echo ""

exit 0
