# zenoh-pico-shim

High-level C shim for zenoh-pico with platform backends.

This crate provides a safe Rust wrapper around a C shim layer for zenoh-pico, enabling embedded applications to use zenoh for communication without direct FFI struct layout issues.

## Architecture

```
┌─────────────────────────────────────────────┐
│             Rust API (lib.rs)               │
│  ShimContext, ShimPublisher, ShimSubscriber │
└─────────────────┬───────────────────────────┘
                  │ FFI calls
┌─────────────────▼───────────────────────────┐
│             C Shim (zenoh_shim.c)           │
│  Platform-agnostic session/pub/sub mgmt    │
└─────────────────┬───────────────────────────┘
                  │ Platform interface
┌─────────────────▼───────────────────────────┐
│          Platform Backend                   │
│  - backend_posix.c (threaded)               │
│  - backend_zephyr.c (threaded)              │
│  - backend_smoltcp.c (polling)              │
└─────────────────┬───────────────────────────┘
                  │
┌─────────────────▼───────────────────────────┐
│            zenoh-pico C library             │
└─────────────────────────────────────────────┘
```

## Platform Backends

Select one backend via feature flags:

| Feature   | Description                      | Execution Model |
|-----------|----------------------------------|-----------------|
| `posix`   | POSIX threads, for desktop       | Threaded        |
| `zephyr`  | Zephyr RTOS threads              | Threaded        |
| `smoltcp` | Polling with smoltcp network     | Polling         |

## Features

- `posix` - POSIX backend (requires zenoh-pico-sys)
- `zephyr` - Zephyr RTOS backend (requires zenoh-pico-sys)
- `smoltcp` - smoltcp polling backend (requires zenoh-pico-sys)
- `std` - Standard library support

## Usage

```rust
use zenoh_pico_shim::{ShimContext, ShimPublisher};

// Create context with locator (null-terminated)
let ctx = ShimContext::new(b"tcp/127.0.0.1:7447\0")?;

// Declare a publisher
let publisher = ctx.declare_publisher(b"demo/topic\0")?;

// Publish data
publisher.publish(b"Hello, World!")?;

// For polling backends, call poll() regularly
if ctx.uses_polling() {
    ctx.poll(10)?;  // 10ms timeout
}
```

## Why a C Shim?

Direct Rust FFI to zenoh-pico causes struct size mismatches because:

1. Rust's placeholder types (zero-sized) don't match actual C struct sizes
2. zenoh-pico uses complex union types that are hard to represent in Rust
3. Different platforms have different struct layouts

The C shim layer handles zenoh-pico types internally, exposing only simple integer handles and byte slices to Rust.

## Building

```bash
# Desktop testing with POSIX backend
cargo build --features "posix,std"

# Run tests
cargo test --features "posix,std"

# No backend (Rust wrapper only)
cargo build
```

## Integration

For embedded targets (Zephyr, bare-metal), use this crate with the appropriate backend feature. The `nano-ros-transport` crate provides higher-level abstractions built on top of this shim.
