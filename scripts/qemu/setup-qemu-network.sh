#!/bin/bash
# Setup QEMU network bridge for nano-ros Zephyr testing
#
# This script creates a network bridge and TAP interface for
# QEMU-to-host communication, enabling nano-ros on Zephyr to
# communicate with ROS 2 nodes on the host.
#
# Network Configuration:
#   Bridge (host gateway): 192.0.2.2
#   Zephyr talker:        192.0.2.1
#   Zephyr listener:      192.0.2.3
#
# Run with: sudo ./scripts/qemu/setup-qemu-network.sh
#
# See: docs/roadmap/phase-2-zephyr-qemu.md

set -e

BRIDGE="br-nano"
TAP="tap-qemu"
BRIDGE_IP="192.0.2.2"
SUBNET="192.0.2.0/24"

echo ""
echo "=========================================="
echo "  nano-ros QEMU Network Setup"
echo "=========================================="
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "Error: This script must be run as root (sudo)"
    exit 1
fi

# Cleanup function
cleanup_existing() {
    echo "Cleaning up existing interfaces..."
    ip link set "$TAP" down 2>/dev/null || true
    ip link delete "$TAP" 2>/dev/null || true
    ip link set "$BRIDGE" down 2>/dev/null || true
    ip link delete "$BRIDGE" 2>/dev/null || true
}

# Check if bridge already exists
if ip link show "$BRIDGE" &>/dev/null; then
    echo "Bridge $BRIDGE already exists"
    read -p "Recreate? [y/N] " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        cleanup_existing
    else
        echo "Using existing configuration"
        exit 0
    fi
fi

# Create bridge
echo "Creating bridge $BRIDGE with IP $BRIDGE_IP..."
ip link add name "$BRIDGE" type bridge
ip addr add "$BRIDGE_IP/24" dev "$BRIDGE"
ip link set "$BRIDGE" up

# Create TAP interface
echo "Creating TAP interface $TAP..."
ip tuntap add dev "$TAP" mode tap user "$(logname 2>/dev/null || echo $SUDO_USER)"
ip link set "$TAP" master "$BRIDGE"
ip link set "$TAP" up

# Enable IP forwarding
echo "Enabling IP forwarding..."
sysctl -w net.ipv4.ip_forward=1 > /dev/null

# Configure iptables for NAT (optional, for internet access from QEMU)
echo "Configuring NAT..."

# Find the default outbound interface
DEFAULT_IF=$(ip route | grep default | awk '{print $5}' | head -1)

if [ -n "$DEFAULT_IF" ]; then
    # Enable masquerading for internet access
    iptables -t nat -A POSTROUTING -s "$SUBNET" -o "$DEFAULT_IF" -j MASQUERADE 2>/dev/null || true
    iptables -A FORWARD -i "$BRIDGE" -o "$DEFAULT_IF" -j ACCEPT 2>/dev/null || true
    iptables -A FORWARD -i "$DEFAULT_IF" -o "$BRIDGE" -m state --state RELATED,ESTABLISHED -j ACCEPT 2>/dev/null || true
    echo "NAT configured via $DEFAULT_IF"
else
    echo "Warning: Could not determine default interface for NAT"
fi

# Allow bridge traffic
iptables -A FORWARD -i "$BRIDGE" -o "$BRIDGE" -j ACCEPT 2>/dev/null || true

echo ""
echo "=========================================="
echo "  Network Setup Complete"
echo "=========================================="
echo ""
echo "Configuration:"
echo "  Bridge:     $BRIDGE"
echo "  Bridge IP:  $BRIDGE_IP (host gateway)"
echo "  TAP:        $TAP"
echo "  Subnet:     $SUBNET"
echo ""
echo "Zephyr node IPs:"
echo "  Talker:     192.0.2.1"
echo "  Listener:   192.0.2.3"
echo ""
echo "Usage:"
echo "  1. Start zenoh router on host:"
echo "     zenohd --listen tcp/0.0.0.0:7447"
echo ""
echo "  2. Build and run Zephyr example:"
echo "     west build -b qemu_x86 nano-ros/examples/zephyr-talker-rs"
echo "     west build -t run"
echo ""
echo "  3. Test with ROS 2:"
echo "     ros2 topic echo /chatter std_msgs/msg/Int32"
echo ""
