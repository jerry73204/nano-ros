#!/bin/bash
# Teardown QEMU network bridge
#
# Run with: sudo ./scripts/teardown-qemu-network.sh

set -e

BRIDGE="br-nano"
TAP="tap-qemu"

echo "Tearing down QEMU network bridge..."

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "Error: This script must be run as root (sudo)"
    exit 1
fi

# Remove TAP interface
if ip link show "$TAP" &>/dev/null; then
    echo "Removing TAP interface $TAP..."
    ip link set "$TAP" down
    ip link delete "$TAP"
fi

# Remove bridge
if ip link show "$BRIDGE" &>/dev/null; then
    echo "Removing bridge $BRIDGE..."
    ip link set "$BRIDGE" down
    ip link delete "$BRIDGE"
fi

echo "Network teardown complete!"
