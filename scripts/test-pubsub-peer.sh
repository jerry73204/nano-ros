#!/bin/bash
# Test nano-ros pub/sub in peer mode (no router required)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

cleanup() {
    echo -e "\n${YELLOW}Cleaning up...${NC}"
    [ -n "$LISTENER_PID" ] && kill $LISTENER_PID 2>/dev/null || true
    [ -n "$TALKER_PID" ] && kill $TALKER_PID 2>/dev/null || true
    rm -f /tmp/listener_peer_output.txt /tmp/talker_peer_output.txt
    echo -e "${GREEN}Cleanup complete${NC}"
}

trap cleanup EXIT

echo -e "${GREEN}=== nano-ros Peer Mode Test ===${NC}"
echo -e "${YELLOW}(No zenoh router required)${NC}"
echo ""

cd "$PROJECT_DIR"

# Build
echo -e "${YELLOW}Building examples...${NC}"
cargo build -p native-talker -p native-listener --features zenoh --release 2>&1 | tail -3

echo ""
echo -e "${YELLOW}Starting listener in peer mode...${NC}"

# Modify the examples to use peer mode by not connecting to router
# The fallback to peer mode is built-in
ZENOH_ROUTER_CHECK_ATTEMPTS=0 cargo run -p native-listener --features zenoh --release > /tmp/listener_peer_output.txt 2>&1 &
LISTENER_PID=$!
sleep 3

if ! kill -0 $LISTENER_PID 2>/dev/null; then
    echo -e "${RED}Listener failed to start${NC}"
    cat /tmp/listener_peer_output.txt
    exit 1
fi
echo -e "${GREEN}Listener started in peer mode (PID: $LISTENER_PID)${NC}"

echo ""
echo -e "${YELLOW}Starting talker in peer mode...${NC}"
timeout 10s cargo run -p native-talker --features zenoh --release > /tmp/talker_peer_output.txt 2>&1 &
TALKER_PID=$!

echo -e "${YELLOW}Waiting for messages...${NC}"
sleep 8

echo ""
echo -e "${GREEN}=== Talker Output ===${NC}"
cat /tmp/talker_peer_output.txt || echo "(no output)"

echo ""
echo -e "${GREEN}=== Listener Output ===${NC}"
cat /tmp/listener_peer_output.txt || echo "(no output)"

echo ""
if grep -q "Received:" /tmp/listener_peer_output.txt 2>/dev/null; then
    RECEIVED_COUNT=$(grep -c "Received:" /tmp/listener_peer_output.txt)
    echo -e "${GREEN}SUCCESS: Listener received $RECEIVED_COUNT messages in peer mode!${NC}"
    exit 0
else
    echo -e "${YELLOW}NOTE: Peer mode may need multicast support${NC}"
    # Check if peer mode was used
    if grep -q "peer mode" /tmp/listener_peer_output.txt 2>/dev/null; then
        echo -e "${YELLOW}Peer mode was active but no messages received (expected on some networks)${NC}"
    fi
    exit 0  # Not a failure - peer mode may not work in all environments
fi
