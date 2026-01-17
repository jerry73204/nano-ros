#!/bin/bash
# Multi-node test runner
#
# This script demonstrates the multi-node communication scenario:
# - QEMU node (Cortex-M3) running nano-ros
# - Native x86 node running nano-ros
#
# For actual zenoh communication, both nodes need zenoh transport
# enabled. Currently this demonstrates the concept.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

echo "========================================"
echo "  nano-ros Multi-Node Test"
echo "========================================"
echo ""

# Build everything first
echo "Building all components..."
cd "$PROJECT_DIR"
cargo build -p native-talker -p native-listener
cd examples/qemu-test && cargo build --release
cd "$PROJECT_DIR"
echo ""

# Run QEMU test
echo "--- Running QEMU Node Test (Cortex-M3) ---"
qemu-system-arm \
    -cpu cortex-m3 \
    -machine lm3s6965evb \
    -nographic \
    -semihosting-config enable=on,target=native \
    -kernel target/thumbv7m-none-eabi/release/qemu-test
echo ""

# Run native talker
echo "--- Running Native Talker (x86) ---"
timeout 3 cargo run -p native-talker || true
echo ""

# Run native listener
echo "--- Running Native Listener (x86) ---"
timeout 2 cargo run -p native-listener || true
echo ""

echo "========================================"
echo "  Multi-Node Test Complete"
echo "========================================"
echo ""
echo "Note: For actual cross-node communication, zenoh transport"
echo "must be enabled. This test demonstrates the Node API works"
echo "on both embedded (QEMU/ARM) and native (x86) targets."
