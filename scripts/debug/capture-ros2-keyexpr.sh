#!/bin/bash
# Capture the actual keyexpr ROS 2 uses when publishing
# This will tell us the exact RIHS01 hash we need

set -e

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
Z_SUB="$PROJECT_ROOT/crates/zenoh-pico-sys/zenoh-pico/build/examples/z_sub"
LOCATOR="tcp/127.0.0.1:7447"

echo "=== Capture ROS 2 Topic Key Expression ==="
echo ""
echo "This script will:"
echo "1. Subscribe to zenoh keyexpr '0/**' to see all ROS 2 domain 0 topics"
echo "2. Show the exact keyexpr including RIHS01 hash"
echo ""

# Check zenohd is running
if ! pgrep -x zenohd > /dev/null; then
    echo "ERROR: zenohd not running"
    echo "Start it with: zenohd --listen tcp/127.0.0.1:7447"
    exit 1
fi

# Check z_sub exists
if [[ ! -x "$Z_SUB" ]]; then
    echo "ERROR: z_sub not found at $Z_SUB"
    exit 1
fi

echo "Instructions:"
echo "1. In another terminal, run:"
echo "   source /opt/ros/humble/setup.bash"
echo "   RMW_IMPLEMENTATION=rmw_zenoh_cpp ros2 run demo_nodes_cpp talker"
echo ""
echo "Listening for messages on '0/**'..."
echo "(Press Ctrl+C to stop)"
echo ""

"$Z_SUB" -m client -e "$LOCATOR" -k "0/**" 2>&1 | while read -r line; do
    echo "$line"
    # Extract keyexpr from the line if it contains RIHS01
    if [[ "$line" == *"RIHS01"* ]]; then
        echo ""
        echo "=== Found RIHS01 hash in keyexpr above ==="
    fi
done
