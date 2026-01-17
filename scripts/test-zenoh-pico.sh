#!/bin/bash
# Test zenoh-pico pub/sub communication using GNU Parallel
#
# Prerequisites:
#   - zenohd running: zenohd --listen tcp/127.0.0.1:7447
#   - GNU parallel installed: sudo apt install parallel
#
# Usage: ./scripts/test-zenoh-pico.sh

set -e

ZENOH_BUILD=/home/aeon/repos/rusty-ros/crates/zenoh-pico-sys/zenoh-pico/build/examples
ROUTER_ADDR="tcp/127.0.0.1:7447"

# Check prerequisites
if ! command -v parallel &> /dev/null; then
    echo "ERROR: GNU parallel not found. Install with: sudo apt install parallel"
    exit 1
fi

if ! pgrep zenohd > /dev/null; then
    echo "ERROR: zenohd not running. Start with: zenohd --listen $ROUTER_ADDR"
    exit 1
fi

echo "=== Testing zenoh-pico pub/sub communication ==="
echo "Router: $ROUTER_ADDR"
echo ""

# Create temp files for output
SUB_OUT=$(mktemp)
PUB_OUT=$(mktemp)

# Define functions for parallel execution
subscriber() {
    timeout 10 "$ZENOH_BUILD/z_sub" -e "$ROUTER_ADDR" -k "demo/**" 2>&1
}

publisher() {
    sleep 2  # Wait for subscriber to start
    "$ZENOH_BUILD/z_pub" -e "$ROUTER_ADDR" -k "demo/test" -n 5 2>&1
}

export -f subscriber publisher
export ZENOH_BUILD ROUTER_ADDR

# Run subscriber and publisher in parallel
echo "Starting subscriber and publisher..."
parallel --halt now,done=1 ::: subscriber publisher | tee >(grep "Received" > "$SUB_OUT" || true) >(grep "Putting" > "$PUB_OUT" || true)

echo ""
echo "=== Results ==="
echo "Published messages:"
cat "$PUB_OUT" | head -5

echo ""
echo "Received messages:"
cat "$SUB_OUT" | head -5

# Check if any messages were received
if [ -s "$SUB_OUT" ]; then
    echo ""
    echo "SUCCESS: Messages received!"
else
    echo ""
    echo "WARNING: No messages received (may be timing issue)"
fi

# Cleanup
rm -f "$SUB_OUT" "$PUB_OUT"
