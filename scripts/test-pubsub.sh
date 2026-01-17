#!/bin/bash
# Test nano-ros pub/sub communication via zenoh
#
# This script tests communication between native-talker and native-listener
# using the zenoh transport.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Cleanup function
cleanup() {
    echo -e "\n${YELLOW}Cleaning up...${NC}"
    # Kill background processes
    if [ -n "$ZENOHD_PID" ]; then
        kill $ZENOHD_PID 2>/dev/null || true
    fi
    if [ -n "$LISTENER_PID" ]; then
        kill $LISTENER_PID 2>/dev/null || true
    fi
    if [ -n "$TALKER_PID" ]; then
        kill $TALKER_PID 2>/dev/null || true
    fi
    # Clean up temp files
    rm -f /tmp/listener_output.txt /tmp/talker_output.txt /tmp/zenohd_output.txt
    echo -e "${GREEN}Cleanup complete${NC}"
}

trap cleanup EXIT

echo -e "${GREEN}=== nano-ros Pub/Sub Test ===${NC}"
echo ""

# Build the examples with zenoh feature
echo -e "${YELLOW}Building examples with zenoh feature...${NC}"
cd "$PROJECT_DIR"
cargo build -p native-talker -p native-listener --features zenoh --release 2>&1 | tail -5

echo ""
echo -e "${YELLOW}Starting zenoh router...${NC}"
zenohd --listen tcp/127.0.0.1:7447 > /tmp/zenohd_output.txt 2>&1 &
ZENOHD_PID=$!
sleep 2

# Check if zenohd started
if ! kill -0 $ZENOHD_PID 2>/dev/null; then
    echo -e "${RED}Failed to start zenohd${NC}"
    cat /tmp/zenohd_output.txt
    exit 1
fi
echo -e "${GREEN}zenohd started (PID: $ZENOHD_PID)${NC}"

echo ""
echo -e "${YELLOW}Starting listener...${NC}"
cargo run -p native-listener --features zenoh --release > /tmp/listener_output.txt 2>&1 &
LISTENER_PID=$!
sleep 2

# Check if listener started
if ! kill -0 $LISTENER_PID 2>/dev/null; then
    echo -e "${RED}Failed to start listener${NC}"
    cat /tmp/listener_output.txt
    exit 1
fi
echo -e "${GREEN}Listener started (PID: $LISTENER_PID)${NC}"

echo ""
echo -e "${YELLOW}Starting talker (will publish 5 messages)...${NC}"

# Run talker for a limited time
timeout 8s cargo run -p native-talker --features zenoh --release > /tmp/talker_output.txt 2>&1 &
TALKER_PID=$!

# Wait for messages
echo -e "${YELLOW}Waiting for messages...${NC}"
sleep 6

echo ""
echo -e "${GREEN}=== Talker Output ===${NC}"
cat /tmp/talker_output.txt || echo "(no output)"

echo ""
echo -e "${GREEN}=== Listener Output ===${NC}"
cat /tmp/listener_output.txt || echo "(no output)"

echo ""
# Check if listener received any messages
if grep -q "Received:" /tmp/listener_output.txt 2>/dev/null; then
    RECEIVED_COUNT=$(grep -c "Received:" /tmp/listener_output.txt)
    echo -e "${GREEN}SUCCESS: Listener received $RECEIVED_COUNT messages!${NC}"
    exit 0
else
    echo -e "${RED}FAILED: Listener did not receive any messages${NC}"
    echo ""
    echo -e "${YELLOW}=== Debug: zenohd output ===${NC}"
    cat /tmp/zenohd_output.txt | tail -20 || echo "(no output)"
    exit 1
fi
