#!/bin/bash
# Debug script to see what keyexpr nano-ros is publishing on
# Uses GNU Parallel to run talker and z_sub concurrently

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Paths
TALKER="$PROJECT_ROOT/target/release/talker"
Z_SUB="$PROJECT_ROOT/crates/zenoh-pico-sys/zenoh-pico/build/examples/z_sub"
LOCATOR="tcp/127.0.0.1:7447"

# Check dependencies
if ! pgrep -x zenohd > /dev/null; then
    echo "ERROR: zenohd not running. Start it with: zenohd --listen tcp/127.0.0.1:7447"
    exit 1
fi

if [[ ! -x "$TALKER" ]]; then
    echo "Building talker..."
    cargo build --release -p native-talker --features zenoh
fi

if [[ ! -x "$Z_SUB" ]]; then
    echo "ERROR: z_sub not found at $Z_SUB"
    exit 1
fi

echo "=== Debug: Monitoring keyexprs for published messages ==="
echo ""

# Create temp dir for outputs
TMPDIR=$(mktemp -d)
trap "rm -rf $TMPDIR" EXIT

# Function to run talker
run_talker() {
    echo "[talker] Starting nano-ros talker..."
    timeout 6 "$TALKER" --tcp 127.0.0.1:7447 2>&1 || true
}

# Function to run z_sub
run_zsub() {
    sleep 1  # Wait for talker to start
    echo "[z_sub] Subscribing to '0/**' to see all ROS 2 domain 0 keyexprs..."
    timeout 5 "$Z_SUB" -m client -e "$LOCATOR" -k "0/**" 2>&1 || true
}

export -f run_talker run_zsub
export TALKER Z_SUB LOCATOR

# Run in parallel
parallel --halt now,done=1 ::: run_talker run_zsub

echo ""
echo "=== Done ==="
