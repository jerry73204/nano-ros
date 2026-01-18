# Embedded Integration Guide

This document describes how to integrate nano-ros with embedded real-time systems.

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

### Implications for Rust Integration

Because zenoh-pico requires heap allocation:

1. The Rust `zenoh` feature implies `alloc`
2. Pure `no_std` without allocator cannot use zenoh directly
3. Embedded systems need an RTOS with heap support (FreeRTOS, Zephyr, etc.)

## Integration Patterns

### Pattern 1: C Shim (Recommended for Embedded)

The C shim pattern hides zenoh-pico's complexity behind simple C functions:

```c
// zenoh_shim.c
static z_owned_session_t g_session;
static z_owned_publisher_t g_publisher;

int zenoh_init(const char *locator);
int zenoh_publish(const uint8_t *data, size_t len);
void zenoh_close(void);
```

Rust calls the shim:
```rust
extern "C" {
    fn zenoh_init(locator: *const c_char) -> i32;
    fn zenoh_publish(data: *const u8, len: usize) -> i32;
}
```

**Benefits:**
- Avoids FFI struct layout issues
- Zenoh allocates on RTOS heap (managed by C)
- Rust code stays simple and no_std compatible
- Used by Zephyr examples (`examples/zephyr-talker-rs/`)

### Pattern 2: Direct FFI (Desktop/Linux)

For systems with full std support, use the Rust FFI bindings directly:

```rust
use nano_ros_node::{ConnectedNode, NodeConfig};

let config = NodeConfig::new("my_node", "/demo");
let node = ConnectedNode::connect(config, "tcp/127.0.0.1:7447")?;
let publisher = node.create_publisher::<Int32>("/counter")?;
```

This requires `std` or `alloc` feature.

### Pattern 3: Transport Abstraction (Future)

A future pattern could provide static buffer transport:

```rust
// Hypothetical static transport (not yet implemented)
let transport = StaticTransport::<1024>::new();
let node = Node::with_transport(config, transport);
```

This would require significant zenoh-pico modifications.

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

See `examples/rtic-stm32f4/` for complete example.

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

See `examples/embassy-stm32f4/` for complete example.

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

See `examples/polling-stm32f4/` for complete example.

### Zephyr RTOS

Zephyr uses the C shim pattern with west build system:

```
examples/zephyr-talker-rs/
├── CMakeLists.txt      # Zephyr build config
├── prj.conf            # Kconfig options
├── src/
│   ├── lib.rs          # Rust application
│   └── zenoh_shim.c    # C shim for zenoh-pico
└── Cargo.toml
```

See `examples/zephyr-talker-rs/` and `examples/zephyr-listener-rs/`.

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

## References

- [RTIC Book](https://rtic.rs/)
- [Embassy Documentation](https://embassy.dev/)
- [Zephyr Project](https://zephyrproject.org/)
- [zenoh-pico](https://github.com/eclipse-zenoh/zenoh-pico)
