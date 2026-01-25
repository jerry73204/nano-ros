#!/bin/bash
# Setup TAP interface for Zephyr native_sim networking
#
# This script creates a TAP interface for Zephyr native_sim to use.
# The Zephyr instance will get IP 192.0.2.1, and can reach the host at 192.0.2.2.
#
# Usage:
#   sudo ./scripts/setup-zephyr-network.sh [USERNAME]
#
# Arguments:
#   USERNAME - User who will run Zephyr (default: user who invoked sudo)
#
# After running this script:
#   1. Start a zenoh router on the host:
#      zenohd --listen tcp/0.0.0.0:7447
#
#   2. Build and run the Zephyr example (as regular user, no sudo needed):
#      west build -b native_sim/native/64 nano-ros/examples/zephyr-talker-rs
#      ./build/zephyr/zephyr.exe
#
# To tear down:
#   sudo ./scripts/setup-zephyr-network.sh --down

set -e

TAP_NAME="zeth"
HOST_IP="192.0.2.2"
NETMASK="255.255.255.0"

if [ "$EUID" -ne 0 ]; then
    echo "Please run as root: sudo $0"
    exit 1
fi

if [ "$1" = "--down" ]; then
    echo "Tearing down TAP interface..."
    ip link set $TAP_NAME down 2>/dev/null || true
    ip tuntap del dev $TAP_NAME mode tap 2>/dev/null || true
    echo "Done."
    exit 0
fi

# Determine the user who will run Zephyr
if [ -n "$1" ]; then
    TAP_USER="$1"
elif [ -n "$SUDO_USER" ]; then
    TAP_USER="$SUDO_USER"
else
    TAP_USER=$(logname 2>/dev/null || echo "")
    if [ -z "$TAP_USER" ]; then
        echo "Error: Could not determine user. Please specify: sudo $0 USERNAME"
        exit 1
    fi
fi

echo "Setting up TAP interface for Zephyr networking..."
echo "  TAP owner: $TAP_USER"

# Delete existing TAP interface to ensure clean state
if ip link show $TAP_NAME &>/dev/null; then
    echo "  Removing existing $TAP_NAME..."
    ip link set $TAP_NAME down 2>/dev/null || true
    ip tuntap del dev $TAP_NAME mode tap 2>/dev/null || true
fi

# Create TAP interface with user ownership (allows non-root access)
echo "  Creating $TAP_NAME with user $TAP_USER..."
ip tuntap add dev $TAP_NAME mode tap user $TAP_USER

# Configure interface
ip addr add $HOST_IP/24 dev $TAP_NAME
ip link set $TAP_NAME up

# Enable IP forwarding (for routing to outside if needed)
echo 1 > /proc/sys/net/ipv4/ip_forward

echo ""
echo "TAP interface ready:"
echo "  Interface: $TAP_NAME"
echo "  Host IP: $HOST_IP"
echo "  Zephyr IP: 192.0.2.1 (configured in prj.conf)"
echo "  Owner: $TAP_USER (can run Zephyr without sudo)"
echo ""
echo "Next steps:"
echo "  1. Start zenoh router: zenohd --listen tcp/0.0.0.0:7447"
echo "  2. Run Zephyr example: ./build/zephyr/zephyr.exe"
