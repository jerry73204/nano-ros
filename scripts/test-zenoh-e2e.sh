#!/bin/bash
# End-to-end zenoh-pico communication test
#
# This script:
# 1. Starts a zenoh router (zenohd)
# 2. Starts a subscriber
# 3. Starts a publisher
# 4. Verifies messages are received
# 5. Cleans up all processes
#
# Usage: ./scripts/test-zenoh-e2e.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
ZENOH_BUILD="$REPO_ROOT/crates/zenoh-pico-sys/zenoh-pico/build/examples"
ROUTER_ADDR="tcp/127.0.0.1:7447"
TOPIC_KEY="demo/test"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# PIDs to track
ROUTER_PID=""
SUB_PID=""
PUB_PID=""

# Cleanup function
cleanup() {
    echo -e "\n${YELLOW}Cleaning up...${NC}"
    [ -n "$PUB_PID" ] && kill $PUB_PID 2>/dev/null || true
    [ -n "$SUB_PID" ] && kill $SUB_PID 2>/dev/null || true
    [ -n "$ROUTER_PID" ] && kill $ROUTER_PID 2>/dev/null || true
    wait 2>/dev/null || true
    echo "Cleanup complete"
}
trap cleanup EXIT

# Check if zenoh-pico examples exist
if [ ! -f "$ZENOH_BUILD/z_pub" ] || [ ! -f "$ZENOH_BUILD/z_sub" ]; then
    echo -e "${RED}ERROR: zenoh-pico examples not found at $ZENOH_BUILD${NC}"
    echo "Build with: just build-zenoh-pico"
    exit 1
fi

echo "========================================"
echo "  Zenoh-Pico End-to-End Test"
echo "========================================"
echo ""

# Step 1: Start or use existing router
echo -e "${YELLOW}Step 1: Checking zenoh router...${NC}"
if pgrep -x zenohd > /dev/null; then
    echo "  Using existing zenohd process"
    ROUTER_PID=""
else
    echo "  Starting zenohd..."
    zenohd --listen $ROUTER_ADDR > /tmp/zenohd.log 2>&1 &
    ROUTER_PID=$!
    sleep 2

    if ! kill -0 $ROUTER_PID 2>/dev/null; then
        echo -e "${RED}  ERROR: Failed to start zenohd${NC}"
        cat /tmp/zenohd.log
        exit 1
    fi
    echo "  zenohd started (PID: $ROUTER_PID)"
fi
echo ""

# Step 2: Start subscriber
echo -e "${YELLOW}Step 2: Starting subscriber...${NC}"
SUB_LOG=/tmp/zenoh_sub.log
$ZENOH_BUILD/z_sub -e $ROUTER_ADDR -k "$TOPIC_KEY" > $SUB_LOG 2>&1 &
SUB_PID=$!
sleep 2

if ! kill -0 $SUB_PID 2>/dev/null; then
    echo -e "${RED}  ERROR: Subscriber failed to start${NC}"
    cat $SUB_LOG
    exit 1
fi
echo "  Subscriber started (PID: $SUB_PID)"
echo ""

# Step 3: Start publisher
echo -e "${YELLOW}Step 3: Starting publisher...${NC}"
PUB_LOG=/tmp/zenoh_pub.log
$ZENOH_BUILD/z_pub -e $ROUTER_ADDR -k "$TOPIC_KEY" -n 5 > $PUB_LOG 2>&1 &
PUB_PID=$!

# Wait for publisher to finish
echo "  Publishing 5 messages..."
wait $PUB_PID 2>/dev/null || true
PUB_PID=""
echo "  Publisher finished"
echo ""

# Step 4: Wait a bit for messages to propagate
echo -e "${YELLOW}Step 4: Waiting for messages to propagate...${NC}"
sleep 2
echo ""

# Step 5: Check results
echo -e "${YELLOW}Step 5: Checking results...${NC}"
echo ""
echo "--- Publisher Log ---"
cat $PUB_LOG
echo ""

echo "--- Subscriber Log ---"
cat $SUB_LOG
echo ""

# Count received messages
RECEIVED=$(grep -c "Received" $SUB_LOG 2>/dev/null || echo "0")

echo "========================================"
if [ "$RECEIVED" -gt 0 ]; then
    echo -e "${GREEN}SUCCESS: $RECEIVED messages received!${NC}"
    exit 0
else
    echo -e "${RED}FAILED: No messages received${NC}"
    echo ""
    echo "Debugging tips:"
    echo "  1. Check zenohd log: cat /tmp/zenohd.log"
    echo "  2. Verify router is listening: netstat -tlnp | grep 7447"
    echo "  3. Check firewall rules"
    exit 1
fi
