#!/bin/bash
# Detailed zenoh-pico test with timing output
# Uses stdbuf to ensure unbuffered output from subscriber

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ZENOH_BUILD="$REPO_ROOT/crates/zenoh-pico-sys/zenoh-pico/build/examples"
ROUTER="tcp/127.0.0.1:7447"
SUB_LOG="/tmp/zenoh_sub_$$.log"

cleanup() {
    echo "$(date +%H:%M:%S) Cleanup"
    pkill -f "z_sub.*test/msg" 2>/dev/null || true
    rm -f "$SUB_LOG"
}
trap cleanup EXIT

echo "$(date +%H:%M:%S) === Zenoh Detailed Test ==="

# Ensure router
if ! pgrep zenohd >/dev/null; then
    echo "$(date +%H:%M:%S) Starting router..."
    zenohd --listen "$ROUTER" &>/dev/null &
    sleep 2
fi

# Start subscriber with unbuffered output
echo "$(date +%H:%M:%S) Starting subscriber in background..."
stdbuf -oL -eL $ZENOH_BUILD/z_sub -e "$ROUTER" -k "test/msg" > "$SUB_LOG" 2>&1 &
SUB_PID=$!
echo "$(date +%H:%M:%S) Subscriber PID: $SUB_PID"

# Wait for subscriber to fully initialize
sleep 3

# Check if alive
if ! kill -0 $SUB_PID 2>/dev/null; then
    echo "$(date +%H:%M:%S) ERROR: Subscriber died"
    cat "$SUB_LOG"
    exit 1
fi
echo "$(date +%H:%M:%S) Subscriber running"
cat "$SUB_LOG"

# Publish
echo ""
echo "$(date +%H:%M:%S) Publishing messages..."
$ZENOH_BUILD/z_pub -e "$ROUTER" -k "test/msg" -n 5

echo "$(date +%H:%M:%S) Waiting for propagation..."
sleep 3

echo "$(date +%H:%M:%S) Stopping subscriber..."
kill $SUB_PID 2>/dev/null
wait $SUB_PID 2>/dev/null || true

echo ""
echo "=== Subscriber Output ==="
cat "$SUB_LOG"

echo ""
if grep -q "Received" "$SUB_LOG"; then
    echo "SUCCESS: Messages received!"
else
    echo "FAILED: No messages received"
    exit 1
fi
