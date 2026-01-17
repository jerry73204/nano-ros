#!/bin/bash
# Debug script to capture and analyze liveliness tokens

set -e

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TALKER="$PROJECT_ROOT/target/release/talker"
Z_SUB="$PROJECT_ROOT/crates/zenoh-pico-sys/zenoh-pico/build/examples/z_sub"

echo "=== Liveliness Token Debug ==="
echo ""

# Ensure zenohd is running
if ! pgrep -x zenohd > /dev/null; then
    echo "Starting zenohd..."
    zenohd --listen tcp/127.0.0.1:7447 &
    sleep 2
fi

echo "Subscribing to liveliness tokens (@ros2_lv/**)..."
echo "Will show tokens from nano-ros and ROS 2 nodes"
echo ""
echo "In another terminal, run:"
echo "  $TALKER --tcp 127.0.0.1:7447"
echo ""
echo "Press Ctrl+C to stop"
echo ""

# Subscribe to all liveliness tokens
"$Z_SUB" -m client -e tcp/127.0.0.1:7447 -k "@ros2_lv/**" 2>&1
