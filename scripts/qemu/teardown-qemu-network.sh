#!/bin/bash
# Teardown QEMU network bridge
#
# This script removes the network bridge and TAP interface
# created by setup-qemu-network.sh
#
# Run with: sudo ./scripts/qemu/teardown-qemu-network.sh

set -e

BRIDGE="br-nano"
TAP="tap-qemu"
SUBNET="192.0.2.0/24"

echo ""
echo "=========================================="
echo "  nano-ros QEMU Network Teardown"
echo "=========================================="
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "Error: This script must be run as root (sudo)"
    exit 1
fi

# Find the default outbound interface for NAT cleanup
DEFAULT_IF=$(ip route | grep default | awk '{print $5}' | head -1)

# Remove iptables rules
echo "Removing iptables rules..."
if [ -n "$DEFAULT_IF" ]; then
    iptables -t nat -D POSTROUTING -s "$SUBNET" -o "$DEFAULT_IF" -j MASQUERADE 2>/dev/null || true
    iptables -D FORWARD -i "$BRIDGE" -o "$DEFAULT_IF" -j ACCEPT 2>/dev/null || true
    iptables -D FORWARD -i "$DEFAULT_IF" -o "$BRIDGE" -m state --state RELATED,ESTABLISHED -j ACCEPT 2>/dev/null || true
fi
iptables -D FORWARD -i "$BRIDGE" -o "$BRIDGE" -j ACCEPT 2>/dev/null || true

# Remove TAP interface
if ip link show "$TAP" &>/dev/null; then
    echo "Removing TAP interface $TAP..."
    ip link set "$TAP" down
    ip link delete "$TAP"
else
    echo "TAP interface $TAP does not exist"
fi

# Remove bridge
if ip link show "$BRIDGE" &>/dev/null; then
    echo "Removing bridge $BRIDGE..."
    ip link set "$BRIDGE" down
    ip link delete "$BRIDGE"
else
    echo "Bridge $BRIDGE does not exist"
fi

echo ""
echo "Network teardown complete!"
echo ""
