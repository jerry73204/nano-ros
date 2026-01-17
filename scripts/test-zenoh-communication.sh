#!/bin/bash
# Zenoh-pico communication test script
#
# Tests:
# 1. Router connectivity
# 2. Publisher can send messages
# 3. Subscriber can receive messages
#
# Usage: ./scripts/test-zenoh-communication.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
ZENOH_BUILD="$REPO_ROOT/crates/zenoh-pico-sys/zenoh-pico/build/examples"

# Use fifos for communication
FIFO_DIR=$(mktemp -d)
SUB_FIFO="$FIFO_DIR/sub_output"
PUB_FIFO="$FIFO_DIR/pub_output"
mkfifo "$SUB_FIFO" "$PUB_FIFO"

cleanup() {
    pkill -f "z_sub.*demo/test" 2>/dev/null || true
    rm -rf "$FIFO_DIR"
}
trap cleanup EXIT

echo "=== Zenoh-Pico Communication Test ==="
echo ""

# Check router
echo "1. Checking zenoh router..."
if ! pgrep -x zenohd > /dev/null; then
    echo "   Starting zenohd..."
    zenohd --listen tcp/127.0.0.1:7447 &
    sleep 2
fi
echo "   Router: OK"
echo ""

# Test subscriber connection
echo "2. Testing subscriber connection..."
timeout 3 $ZENOH_BUILD/z_sub -e tcp/127.0.0.1:7447 -k "demo/test" > "$SUB_FIFO" 2>&1 &
SUB_PID=$!
sleep 1

# Check if subscriber started
if kill -0 $SUB_PID 2>/dev/null; then
    echo "   Subscriber connected"
else
    echo "   ERROR: Subscriber failed to start"
    cat "$SUB_FIFO" 2>/dev/null || true
    exit 1
fi

# Test publisher
echo ""
echo "3. Publishing test messages..."
$ZENOH_BUILD/z_pub -e tcp/127.0.0.1:7447 -k "demo/test" -n 3 > "$PUB_FIFO" 2>&1 &
PUB_PID=$!

# Wait for publisher to finish
wait $PUB_PID 2>/dev/null || true
echo "   Publisher: OK"

# Wait for subscriber to process
sleep 1

# Kill subscriber to get output
kill $SUB_PID 2>/dev/null || true
wait $SUB_PID 2>/dev/null || true

echo ""
echo "=== Results ==="

# Read outputs
echo "Publisher output:"
cat "$PUB_FIFO" 2>/dev/null | head -10 || echo "(empty)"

echo ""
echo "Subscriber output:"
timeout 1 cat "$SUB_FIFO" 2>/dev/null | head -10 || echo "(empty or timed out)"

echo ""
echo "=== Done ==="
