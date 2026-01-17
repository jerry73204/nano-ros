#!/bin/bash
# Setup QEMU network bridge for multi-node testing
#
# This script creates a network bridge and TAP interfaces for
# QEMU-to-host communication.
#
# Run with: sudo ./scripts/setup-qemu-network.sh

set -e

BRIDGE="br-nano"
TAP="tap-qemu"
BRIDGE_IP="192.168.100.1"
SUBNET="192.168.100.0/24"

echo "Setting up QEMU network bridge..."

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "Error: This script must be run as root (sudo)"
    exit 1
fi

# Check if bridge already exists
if ip link show "$BRIDGE" &>/dev/null; then
    echo "Bridge $BRIDGE already exists"
else
    echo "Creating bridge $BRIDGE..."
    ip link add name "$BRIDGE" type bridge
    ip addr add "$BRIDGE_IP/24" dev "$BRIDGE"
    ip link set "$BRIDGE" up
    echo "Bridge $BRIDGE created with IP $BRIDGE_IP"
fi

# Check if TAP interface already exists
if ip link show "$TAP" &>/dev/null; then
    echo "TAP interface $TAP already exists"
else
    echo "Creating TAP interface $TAP..."
    ip tuntap add dev "$TAP" mode tap
    ip link set "$TAP" master "$BRIDGE"
    ip link set "$TAP" up
    echo "TAP interface $TAP created and attached to $BRIDGE"
fi

# Enable IP forwarding
echo "Enabling IP forwarding..."
echo 1 > /proc/sys/net/ipv4/ip_forward

# Allow traffic on bridge
echo "Configuring iptables..."
iptables -A FORWARD -i "$BRIDGE" -o "$BRIDGE" -j ACCEPT 2>/dev/null || true

echo ""
echo "Network setup complete!"
echo "  Bridge:  $BRIDGE ($BRIDGE_IP)"
echo "  TAP:     $TAP"
echo "  Subnet:  $SUBNET"
echo ""
echo "QEMU nodes should use IP addresses in 192.168.100.x range"
echo "Host is reachable at $BRIDGE_IP"
