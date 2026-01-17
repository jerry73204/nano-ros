#!/bin/bash
# Compare keyexprs used by nano-ros vs ROS 2

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
TALKER="$PROJECT_ROOT/target/release/talker"
Z_SUB="$PROJECT_ROOT/crates/zenoh-pico-sys/zenoh-pico/build/examples/z_sub"

# Colors
GREEN='\033[0;32m'
NC='\033[0m'

log_info() { echo -e "${GREEN}[INFO]${NC} $*"; }

# Cleanup
cleanup() {
    pkill -x zenohd 2>/dev/null || true
    pkill -f "target/release/talker" 2>/dev/null || true
}
trap cleanup EXIT
cleanup
sleep 1

# Start zenohd
log_info "Starting zenohd..."
zenohd --listen tcp/127.0.0.1:7447 > /tmp/zenohd.log 2>&1 &
sleep 2

echo ""
echo "=== Part 1: nano-ros publisher keyexpr ==="
echo ""

# Start nano-ros talker briefly
timeout 3 "$TALKER" --tcp 127.0.0.1:7447 > /tmp/talker.log 2>&1 &
sleep 2

# Subscribe to all data keys (not liveliness) to see what keyexpr is used
log_info "Subscribing to 0/** to capture nano-ros messages..."
timeout 3 "$Z_SUB" -m client -e tcp/127.0.0.1:7447 -k '0/**' 2>&1 | head -10 || true

# Kill nano-ros talker
pkill -f "target/release/talker" 2>/dev/null || true
sleep 1

echo ""
echo "=== Part 2: ROS 2 publisher keyexpr ==="
echo ""

# Source ROS 2 and start talker
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_ROUTER_CHECK_ATTEMPTS=0
export ROS_DOMAIN_ID=0

log_info "Starting ROS 2 talker..."
timeout 5 ros2 run demo_nodes_cpp talker &
ROS2_PID=$!
sleep 2

# Subscribe to all keys to see what keyexpr ROS 2 uses
log_info "Subscribing to 0/** to capture ROS 2 messages..."
timeout 3 "$Z_SUB" -m client -e tcp/127.0.0.1:7447 -k '0/**' 2>&1 | head -10 || true

kill $ROS2_PID 2>/dev/null || true

echo ""
echo "=== Talker log ==="
cat /tmp/talker.log
