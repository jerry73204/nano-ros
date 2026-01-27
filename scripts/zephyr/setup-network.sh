#!/bin/bash
# Setup bridge network for multiple Zephyr native_sim instances
#
# This script creates a Linux bridge with multiple TAP interfaces,
# allowing multiple Zephyr native_sim instances to communicate via zenoh.
#
# Network topology:
#   Zephyr talker (192.0.2.1/zeth0) --+
#                                     |-- Bridge (zeth-br, 192.0.2.2) -- Host
#   Zephyr listener (192.0.2.3/zeth1) -+
#
# Usage:
#   sudo ./scripts/zephyr/setup-network.sh [USERNAME]
#
# Arguments:
#   USERNAME - User who will run Zephyr (default: user who invoked sudo)
#
# After running this script:
#   1. Start a zenoh router on the host:
#      zenohd --listen tcp/0.0.0.0:7447
#
#   2. Build and run Zephyr examples (as regular user, no sudo needed):
#      west build -b native_sim/native/64 nano-ros/examples/zephyr-talker
#      ./build/zephyr/zephyr.exe
#
# To tear down:
#   sudo ./scripts/zephyr/setup-network.sh --down

set -e

BRIDGE_NAME="zeth-br"
TAP_TALKER="zeth0"
TAP_LISTENER="zeth1"
HOST_IP="192.0.2.2"
NETMASK="24"

if [ "$EUID" -ne 0 ]; then
    echo "Please run as root: sudo $0"
    exit 1
fi

teardown() {
    echo "Tearing down network interfaces..."

    # Remove interfaces from bridge first
    ip link set $TAP_TALKER nomaster 2>/dev/null || true
    ip link set $TAP_LISTENER nomaster 2>/dev/null || true

    # Delete TAP interfaces
    ip link set $TAP_TALKER down 2>/dev/null || true
    ip tuntap del dev $TAP_TALKER mode tap 2>/dev/null || true

    ip link set $TAP_LISTENER down 2>/dev/null || true
    ip tuntap del dev $TAP_LISTENER mode tap 2>/dev/null || true

    # Delete bridge
    ip link set $BRIDGE_NAME down 2>/dev/null || true
    ip link delete $BRIDGE_NAME type bridge 2>/dev/null || true

    echo "Done."
}

if [ "$1" = "--down" ]; then
    teardown
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

echo "Setting up bridge network for Zephyr native_sim..."
echo "  TAP owner: $TAP_USER"

# Clean up any existing setup
teardown 2>/dev/null || true

# Create bridge
echo "  Creating bridge $BRIDGE_NAME..."
ip link add name $BRIDGE_NAME type bridge
ip addr add $HOST_IP/$NETMASK dev $BRIDGE_NAME
ip link set $BRIDGE_NAME up

# Create TAP interface for talker (zeth0)
echo "  Creating $TAP_TALKER for talker..."
ip tuntap add dev $TAP_TALKER mode tap user $TAP_USER
ip link set $TAP_TALKER master $BRIDGE_NAME
ip link set $TAP_TALKER up

# Create TAP interface for listener (zeth1)
echo "  Creating $TAP_LISTENER for listener..."
ip tuntap add dev $TAP_LISTENER mode tap user $TAP_USER
ip link set $TAP_LISTENER master $BRIDGE_NAME
ip link set $TAP_LISTENER up

# Enable IP forwarding
echo 1 > /proc/sys/net/ipv4/ip_forward

echo ""
echo "Bridge network ready:"
echo "  Bridge: $BRIDGE_NAME (Host IP: $HOST_IP)"
echo "  TAP interfaces:"
echo "    - $TAP_TALKER: For zephyr-talker (Zephyr IP: 192.0.2.1)"
echo "    - $TAP_LISTENER: For zephyr-listener (Zephyr IP: 192.0.2.3)"
echo "  Owner: $TAP_USER (can run Zephyr without sudo)"
echo ""
echo "Next steps:"
echo "  1. Start zenoh router: zenohd --listen tcp/0.0.0.0:7447"
echo "  2. Run Zephyr examples from the workspace"
echo ""
echo "To verify:"
echo "  ip link show master $BRIDGE_NAME"
