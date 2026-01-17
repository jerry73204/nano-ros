#!/bin/bash
# Simple zenoh-pico test script
#
# Runs publisher and subscriber to test communication

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ZENOH_BUILD="$REPO_ROOT/crates/zenoh-pico-sys/zenoh-pico/build/examples"
ROUTER="tcp/127.0.0.1:7447"
TOPIC="test/hello"

# Temp files
SUB_LOG="/tmp/zenoh_sub_$$.log"
PUB_LOG="/tmp/zenoh_pub_$$.log"

cleanup() {
    echo "Cleaning up..."
    jobs -p | xargs -r kill 2>/dev/null
    rm -f "$SUB_LOG" "$PUB_LOG"
}
trap cleanup EXIT

echo "=== Zenoh-Pico Simple Test ==="
echo "Router: $ROUTER"
echo "Topic:  $TOPIC"
echo ""

# Ensure router is running
if ! pgrep zenohd >/dev/null; then
    echo "Starting router..."
    zenohd --listen "$ROUTER" >/dev/null 2>&1 &
    sleep 2
fi

# Start subscriber in background
echo "Starting subscriber..."
$ZENOH_BUILD/z_sub -e "$ROUTER" -k "$TOPIC" >"$SUB_LOG" 2>&1 &
SUB_PID=$!
sleep 2

# Verify subscriber is running
if ! kill -0 $SUB_PID 2>/dev/null; then
    echo "ERROR: Subscriber failed"
    cat "$SUB_LOG"
    exit 1
fi

# Run publisher (blocking)
echo "Running publisher..."
$ZENOH_BUILD/z_pub -e "$ROUTER" -k "$TOPIC" -n 3 >"$PUB_LOG" 2>&1

# Give time for messages to arrive
sleep 2

# Stop subscriber
kill $SUB_PID 2>/dev/null
wait $SUB_PID 2>/dev/null || true

echo ""
echo "=== Publisher Output ==="
cat "$PUB_LOG"

echo ""
echo "=== Subscriber Output ==="
cat "$SUB_LOG"

echo ""
# Check for success
if grep -q "Received" "$SUB_LOG" 2>/dev/null; then
    echo "SUCCESS: Messages received!"
else
    echo "WARNING: No messages in subscriber log"
    echo "Check if subscriber printed to stdout"
fi
