# Building nano-ros Zephyr Examples

This guide explains how to build and run the nano-ros Zephyr examples
with zenoh-pico transport.

## Prerequisites

### 1. Install Zephyr SDK

Follow the official Zephyr getting started guide:
https://docs.zephyrproject.org/latest/develop/getting_started/index.html

```bash
# Install west
pip install west

# Initialize Zephyr workspace
west init ~/zephyrproject
cd ~/zephyrproject
west update

# Install Python dependencies
pip install -r zephyr/scripts/requirements.txt

# Install Zephyr SDK
wget https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.16.5/zephyr-sdk-0.16.5_linux-x86_64.tar.xz
tar xf zephyr-sdk-0.16.5_linux-x86_64.tar.xz
cd zephyr-sdk-0.16.5
./setup.sh
```

### 2. Build zenoh-pico for Zephyr

zenoh-pico needs to be built as a Zephyr module:

```bash
# Clone zenoh-pico (or use the submodule in this repo)
cd ~/zephyrproject
git clone https://github.com/eclipse-zenoh/zenoh-pico.git

# Add to west manifest (or add as external project in CMakeLists.txt)
```

See: https://github.com/eclipse-zenoh/zenoh-pico#zephyr

## Building the Examples

### For QEMU Cortex-M3

```bash
# Set up environment
source ~/zephyrproject/zephyr/zephyr-env.sh

# Build talker
cd /path/to/rusty-ros
west build -b qemu_cortex_m3 examples/zephyr-talker

# Build listener
west build -b qemu_cortex_m3 examples/zephyr-listener
```

### For Real Hardware (e.g., STM32)

```bash
# Build for STM32F4 Discovery
west build -b stm32f4_disco examples/zephyr-talker

# Flash
west flash
```

## Running in QEMU

### Single Node Test

```bash
west build -t run
```

### Multi-Node Test with Networking

This requires setting up QEMU networking. See the `scripts/` directory
for network setup scripts.

```bash
# Terminal 1: Run zenoh router (on host)
zenohd

# Terminal 2: Run talker in QEMU
west build -b qemu_cortex_m3 examples/zephyr-talker
west build -t run

# Terminal 3: Run listener in QEMU
west build -b qemu_cortex_m3 examples/zephyr-listener -- -DOVERLAY_CONFIG=overlay-net.conf
west build -t run
```

## Network Configuration

The examples use static IP addresses for simplicity:

- Talker: 192.0.2.1
- Listener: 192.0.2.3
- Gateway: 192.0.2.2

For QEMU with user-mode networking, the host acts as the gateway.

## Troubleshooting

### No network connectivity

Ensure the Zephyr networking stack is properly configured. Check:
- `CONFIG_NETWORKING=y`
- `CONFIG_NET_IPV4=y`
- `CONFIG_NET_SOCKETS=y`

### zenoh-pico build errors

Ensure you're using a compatible zenoh-pico version (1.1.0+) and that
the Zephyr module is properly configured.

### CDR serialization issues

The examples use little-endian CDR format. Ensure the message format
matches ROS 2 expectations:
- 4-byte encapsulation header: `00 01 00 00`
- Payload follows CDR alignment rules

## Integration with ROS 2

These examples are compatible with ROS 2 nodes using rmw_zenoh:

```bash
# On ROS 2 machine
source /opt/ros/jazzy/setup.bash
ros2 topic echo /chatter std_msgs/msg/Int32
```
