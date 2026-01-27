# Embedded Integration Guide

This document describes how to integrate nano-ros with embedded real-time systems.

## Architecture Overview

nano-ros provides a unified architecture for both desktop and embedded platforms using the `zenoh-pico-shim` crate:

```
┌─────────────────────────────────────────────────────────────────────┐
│ Application Layer                                                    │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  ┌──────────────────────────┐       ┌────────────────────────────┐  │
│  │  Native/std              │       │  Embedded (RTIC/Zephyr)    │  │
│  │  (shim-posix feature)    │       │  (shim-smoltcp/zephyr)     │  │
│  └────────────┬─────────────┘       └───────────────┬────────────┘  │
│               │                                     │                │
│               └─────────────────┬───────────────────┘                │
│                                 │                                    │
│                                 ▼                                    │
│               ┌────────────────────────────────────────┐             │
│               │  zenoh-pico-shim (High-level Rust API) │             │
│               │  ├── Session, Publisher, Subscriber    │             │
│               │  ├── LivelinessToken, ZenohId          │             │
│               │  └── Queryable (for ROS 2 services)    │             │
│               └────────────────────┬───────────────────┘             │
│                                    │                                 │
│                                    ▼                                 │
│               ┌────────────────────────────────────────┐             │
│               │  zenoh-pico-shim-sys (FFI + C code)    │             │
│               │  ├── c/shim/zenoh_shim.c (C API)       │             │
│               │  ├── c/platform_smoltcp/*.c (smoltcp)  │             │
│               │  └── zenoh-pico/ (submodule)           │             │
│               └────────────────────────────────────────┘             │
└─────────────────────────────────────────────────────────────────────┘
```

**Platform selection via feature flags:**
- `shim-posix` - Desktop Linux/macOS with POSIX threads
- `shim-zephyr` - Zephyr RTOS (built by west/CMake)
- `shim-smoltcp` - Bare-metal with smoltcp network stack

## Memory Allocation Requirements

### Zenoh-Pico Heap Requirements

Zenoh-pico **requires heap allocation** for its core operations. After analyzing the C source code:

**Memory allocation functions** (platform-abstracted):
- `z_malloc(size_t size)` - Allocates memory
- `z_realloc(void *ptr, size_t size)` - Reallocates memory
- `z_free(void *ptr)` - Frees memory

**Components requiring heap:**
- Session creation (`_z_session_t`)
- Publishers and subscribers (state objects)
- I/O buffers for network packets
- Collections (vectors, lists, hash maps)
- Message samples and internal state

**Platform implementations:**
| Platform | Allocator | Notes |
|----------|-----------|-------|
| Unix/Linux | Standard malloc/free | Full heap support |
| FreeRTOS | pvPortMalloc/vPortFree | realloc not available |
| Zephyr | k_malloc/k_free | Requires heap pool |
| smoltcp (bare-metal) | embedded-alloc | 16KB static heap |

### Implications for Rust Integration

Because zenoh-pico requires heap allocation:

1. The Rust `zenoh` feature implies `alloc`
2. Pure `no_std` without allocator cannot use zenoh directly
3. Embedded systems need an RTOS with heap support (FreeRTOS, Zephyr, etc.)

## Integration Patterns

### Pattern 1: zenoh-pico-shim (Unified API)

The `zenoh-pico-shim` crate provides a safe Rust API that works across all platforms:

```rust
use zenoh_pico_shim::{ShimContext, ShimError};

// Open session
let ctx = ShimContext::new(b"tcp/192.168.1.1:7447\0")?;

// Create publisher
let publisher = ctx.declare_publisher(b"demo/chatter\0")?;

// Publish data
publisher.publish(b"Hello, world!")?;

// Optional: Publish with RMW attachment (for ROS 2 compatibility)
let attachment = [0u8; 33];  // RMW GID format
publisher.publish_with_attachment(b"data", Some(&attachment))?;
```

**Features:**
- `no_std` compatible (requires `alloc`)
- Safe Rust API with RAII resource management
- Platform-specific backends selected via features
- ROS 2 discovery via liveliness tokens
- RMW attachment support for rmw_zenoh compatibility

### Pattern 2: C Shim Direct (for Zephyr)

For Zephyr RTOS, the C shim is compiled by west/CMake rather than Cargo:

```c
// zenoh_shim.c (provided by zenoh-pico-shim-sys)
int zenoh_shim_init_config(const char *locator);
int zenoh_shim_open_session(void);
int zenoh_shim_declare_publisher(const char *keyexpr);
int zenoh_shim_publish(int handle, const uint8_t *data, size_t len);
void zenoh_shim_close(void);
```

Rust FFI declarations:
```rust
extern "C" {
    fn zenoh_shim_init_config(locator: *const c_char) -> i32;
    fn zenoh_shim_open_session() -> i32;
    fn zenoh_shim_publish(handle: i32, data: *const u8, len: usize) -> i32;
}
```

**Benefits:**
- Avoids FFI struct layout issues
- Zenoh allocates on RTOS heap (managed by C)
- Rust code stays simple and no_std compatible
- Used by Zephyr examples (`examples/zephyr-rs-talker/`)

### Pattern 3: High-level Node API (Desktop/Linux)

For systems with full std support, use the high-level nano-ros API:

```rust
use nano_ros_node::{ConnectedNode, NodeConfig};

let config = NodeConfig::new("my_node", "/demo");
let node = ConnectedNode::connect(config, "tcp/127.0.0.1:7447")?;
let publisher = node.create_publisher::<Int32>("/counter")?;

publisher.publish(&Int32 { data: 42 })?;
```

This provides ROS 2 compatible topic naming, liveliness tokens, and RMW attachments automatically.

## smoltcp Integration (Bare-Metal Ethernet)

For bare-metal systems with Ethernet (STM32, etc.), use the smoltcp platform layer.

### Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│  Application (RTIC / polling loop)                                   │
│  ├── Poll smoltcp interface periodically                            │
│  └── Call zenoh_shim_poll() to process zenoh protocol               │
├─────────────────────────────────────────────────────────────────────┤
│  nano-ros-node + nano-ros-transport                                  │
│  ├── ShimExecutor, ShimNode API                                     │
│  └── ShimPublisher, ShimSubscriber with CDR serialization           │
├─────────────────────────────────────────────────────────────────────┤
│  zenoh-pico-shim (shim-smoltcp feature)                              │
│  ├── Rust FFI for smoltcp_* functions                               │
│  └── Bridges smoltcp sockets to zenoh-pico                          │
├─────────────────────────────────────────────────────────────────────┤
│  smoltcp + stm32-eth                                                 │
│  └── TCP/IP stack with Ethernet DMA                                  │
└─────────────────────────────────────────────────────────────────────┘
```

### Integration Steps

1. **Initialize Ethernet and smoltcp**:
   ```rust
   // Configure Ethernet peripheral
   let (eth_dma, eth_mac) = stm32_eth::new(...)?;

   // Create smoltcp interface
   let config = Config::new(HardwareAddress::Ethernet(mac_addr));
   let mut iface = Interface::new(config, &mut eth_dma, Instant::ZERO);
   iface.update_ip_addrs(|addrs| {
       addrs.push(IpCidr::new(IpAddress::v4(192, 168, 1, 10), 24)).unwrap();
   });
   ```

2. **Register poll callback**:
   ```rust
   use zenoh_pico_shim::platform_smoltcp;

   // Set callback that smoltcp platform layer will call
   platform_smoltcp::smoltcp_set_poll_callback(my_poll_callback);
   ```

3. **Poll in main loop or RTIC task**:
   ```rust
   loop {
       // Update clock for zenoh-pico timing
       platform_smoltcp::smoltcp_set_clock_ms(now_ms);

       // Poll smoltcp interface
       iface.poll(timestamp, &mut eth_dma, &mut sockets);

       // Poll zenoh (processes incoming messages)
       zenoh_shim_poll(10);  // 10ms timeout

       delay(10.millis());
   }
   ```

### Examples

- `examples/stm32f4-rs-rtic/` - RTIC 2.x with smoltcp + stm32-eth
- `examples/stm32f4-rs-polling/` - Bare-metal polling loop
- `examples/stm32f4-rs-smoltcp/` - smoltcp validation without zenoh

## RTOS Integration

### RTIC (Real-Time Interrupt-driven Concurrency)

RTIC provides priority-based preemptive scheduling for Cortex-M:

```rust
#[app(device = stm32f4xx_hal::pac, peripherals = true)]
mod app {
    #[task(priority = 2)]  // Higher priority for time-critical
    async fn zenoh_poll(_cx: zenoh_poll::Context) {
        loop {
            // Poll zenoh for incoming messages
            Mono::delay(POLL_INTERVAL_MS.millis()).await;
        }
    }

    #[task(priority = 1)]  // Lower priority for publishing
    async fn publisher_task(cx: publisher_task::Context) {
        loop {
            // Publish messages
            Mono::delay(100.millis()).await;
        }
    }
}
```

See `examples/stm32f4-rs-rtic/` for complete example.

### Embassy

Embassy provides async/await for embedded:

```rust
#[embassy_executor::task]
async fn zenoh_poll_task() {
    loop {
        // Poll zenoh
        Timer::after(Duration::from_millis(POLL_INTERVAL_MS)).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    spawner.spawn(zenoh_poll_task()).unwrap();
    spawner.spawn(publisher_task()).unwrap();
}
```

See `examples/stm32f4-rs-embassy/` for complete example.

### Bare-Metal Polling

For simple systems without an executor:

```rust
loop {
    if poll_timer.elapsed_ms(POLL_INTERVAL_MS) {
        // Poll zenoh
    }
    if keepalive_timer.elapsed_ms(KEEPALIVE_INTERVAL_MS) {
        // Send keepalive
    }
    if publish_timer.elapsed_ms(100) {
        // Publish message
    }
}
```

See `examples/stm32f4-rs-polling/` for complete example.

### Zephyr RTOS

Zephyr uses the C shim pattern with west build system. The C shim is provided by
`zenoh-pico-shim-sys` and compiled by Zephyr's CMake build system.

```
examples/zephyr-rs-talker/
├── CMakeLists.txt      # Zephyr build config (includes zenoh_shim.c)
├── prj.conf            # Kconfig options
├── src/
│   └── lib.rs          # Rust application using zenoh-pico-shim
└── Cargo.toml          # Depends on zenoh-pico-shim with zephyr feature
```

The Rust code uses the same `zenoh-pico-shim` API but with the `zephyr` feature:

```rust
use zenoh_pico_shim::{ShimContext, ShimPublisher};

// FFI declarations for Zephyr (C shim compiled by Zephyr)
extern "C" {
    fn zenoh_shim_init_config(locator: *const c_char) -> i32;
    fn zenoh_shim_open_session() -> i32;
    // ...
}
```

See `examples/zephyr-rs-talker/` and `examples/zephyr-rs-listener/`.

## Timing Constants

For manual polling (without zenoh background threads):

| Constant | Value | Purpose |
|----------|-------|---------|
| `POLL_INTERVAL_MS` | 10ms | How often to poll for incoming messages |
| `KEEPALIVE_INTERVAL_MS` | 1000ms | How often to send keepalive to router |

These are available via `nano_ros_node::rtic` when both `zenoh` and `rtic` features are enabled, or can be defined locally in your application.

## Memory Budget

### Static Allocation Sizes

| Component | Size Formula |
|-----------|--------------|
| ConnectedNode | ~200 bytes + MAX_TOKENS × 48 |
| ConnectedSubscriber | ~64 bytes + RX_BUF |
| ConnectedPublisher | ~128 bytes + TX_BUF |
| ConnectedServiceServer | ~96 bytes + REQ_BUF + REPLY_BUF |

### Typical Configurations

**Minimal (sensor node):**
- 1 publisher, 256-byte messages
- ~500 bytes static + zenoh heap

**Standard (robot node):**
- 4 publishers, 4 subscribers, 1KB messages
- ~8KB static + zenoh heap

**Large (complex node):**
- 16 publishers/subscribers, 4KB messages
- ~80KB static + zenoh heap

See `docs/memory-requirements.md` for detailed calculations.

## Hardware Requirements

### Minimum Requirements

| Resource | Minimum | Recommended |
|----------|---------|-------------|
| Flash | 128KB | 256KB+ |
| RAM | 32KB | 64KB+ |
| Clock | 48MHz | 168MHz+ |

### Tested Hardware

- **NUCLEO-F429ZI** (STM32F429, 2MB Flash, 256KB RAM, Ethernet)
- **Zephyr native_sim** (Linux simulation target)

### Network Options

- Ethernet (recommended for reliability)
- UART/Serial (for simple point-to-point)
- WiFi (with appropriate driver support)

## Build Configuration

### Cargo.toml for Embedded

```toml
[dependencies]
# Core crates (no_std compatible)
nano-ros-core = { version = "0.1", default-features = false }
nano-ros-serdes = { version = "0.1", default-features = false }

# With zenoh (requires alloc)
nano-ros-node = { version = "0.1", default-features = false, features = ["zenoh", "rtic"] }
nano-ros-transport = { version = "0.1", default-features = false, features = ["zenoh", "rtic", "sync-critical-section"] }
```

### Feature Flags

| Feature | Description |
|---------|-------------|
| `rtic` | Enable RTIC support (static buffers, timing constants) |
| `sync-critical-section` | Use critical sections for sync (RTIC/Embassy compatible) |
| `zenoh` | Enable zenoh transport (implies `alloc`) |
| `alloc` | Enable heap allocation |
| `std` | Enable standard library |

### Profile Settings

```toml
[profile.release]
codegen-units = 1
debug = 2
lto = "fat"
opt-level = 3
overflow-checks = false
```

## Troubleshooting

### "zenoh feature requires alloc"

This compile error means you enabled `zenoh` without `alloc`. Either:
1. Enable the `alloc` feature
2. Use the C shim pattern instead of direct FFI

### Session connection failures

- Check network connectivity to zenoh router
- Verify locator address format: `tcp/192.168.1.1:7447`
- Ensure router is running: `zenohd --listen tcp/0.0.0.0:7447`

### Stack overflow on embedded

- Increase task stack size in RTIC/Embassy config
- Reduce buffer sizes via const generics
- Check for recursive calls in callbacks

## ROS 2 Interoperability

nano-ros is designed for interoperability with ROS 2 via `rmw_zenoh_cpp`.

### Tested Configurations

| Direction | Status | Notes |
|-----------|--------|-------|
| nano-ros ↔ nano-ros | ✅ Working | Full pub/sub communication |
| ROS 2 → nano-ros | ✅ Working | Uses wildcard subscriber for type hash matching |
| nano-ros → ROS 2 | ✅ Working | Tested with ROS 2 Humble + rmw_zenoh_cpp |

### Testing with ROS 2

```bash
# Terminal 1: Start zenoh router
zenohd --listen tcp/127.0.0.1:7447

# Terminal 2: Run nano-ros talker
cargo run -p native-rs-talker --features zenoh -- --tcp 127.0.0.1:7447

# Terminal 3: Run ROS 2 listener (requires rmw_zenoh_cpp)
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/127.0.0.1:7447"]'
ros2 topic echo /chatter std_msgs/msg/Int32 --qos-reliability best_effort
```

### Key Implementation Details

- **Data key expression format** (Humble): `<domain_id>/<topic>/<type>/TypeHashNotSupported`
- **Liveliness tokens**: Use `RIHS01_` prefix, `%` for topic names, LSB-first ZenohId
- **QoS string**: Explicit values like `2:2:1,1:,:,:,,` for BEST_EFFORT/VOLATILE
- **RMW Attachment**: 33 bytes (sequence_number + timestamp + gid_size + gid)

See `docs/rmw_zenoh_interop.md` for detailed protocol documentation.

### Test Commands

```bash
# Run all integration tests
just test-integration

# Quick smoke test
just test-integration-quick

# Specific test suites
just test-nano2nano      # nano-ros ↔ nano-ros
just test-rmw-interop    # ROS 2 interop (requires rmw_zenoh_cpp)
just test-rmw-detailed   # Protocol-level tests
```

See `tests/README.md` for detailed test documentation.

## References

- [RTIC Book](https://rtic.rs/)
- [Embassy Documentation](https://embassy.dev/)
- [Zephyr Project](https://zephyrproject.org/)
- [zenoh-pico](https://github.com/eclipse-zenoh/zenoh-pico)
- [rmw_zenoh](https://github.com/ros2/rmw_zenoh)
