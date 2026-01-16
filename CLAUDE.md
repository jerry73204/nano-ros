# nano-ros

A lightweight ROS 2 client for embedded real-time systems (Zephyr, NuttX). Rewrite of Pico-ROS with `no_std` support.

## Project Overview

Pico-ROS is a lightweight ROS client for resource-constrained embedded systems. This project rewrites it in Rust with:
- `no_std` support for bare-metal and RTOS targets
- Zero-copy serialization where possible
- Compile-time type safety via traits and proc macros
- Async-ready design for non-blocking I/O

## Original Pico-ROS Architecture

### Core Components

| Component | File | Purpose |
|-----------|------|---------|
| picoros | `picoros.h/c` | Core ROS client (nodes, pub/sub, services) |
| picoserdes | `picoserdes.h/c` | CDR serialization via X-macros |
| picoparams | `picoparams.h/c` | ROS 2 parameter server |

### Dependencies
- **zenoh-pico**: Lightweight pub/sub middleware
- **Micro-CDR**: eProsima's CDR serialization library

## Workspace Structure

```
nano-ros/
├── Cargo.toml                 # Workspace root
├── CLAUDE.md
├── crates/
│   ├── nano-ros-core/         # Core types, traits, node abstraction
│   │   ├── Cargo.toml
│   │   └── src/
│   │       ├── lib.rs
│   │       ├── node.rs        # Node, Publisher, Subscriber
│   │       ├── service.rs     # Service server/client
│   │       ├── error.rs       # Error types
│   │       └── types.rs       # Core ROS type traits
│   │
│   ├── nano-ros-serdes/       # CDR serialization/deserialization
│   │   ├── Cargo.toml
│   │   └── src/
│   │       ├── lib.rs
│   │       ├── cdr.rs         # CDR encoder/decoder
│   │       ├── traits.rs      # Serialize/Deserialize traits
│   │       └── primitives.rs  # Base type implementations
│   │
│   ├── nano-ros-macros/       # Proc macros for message types
│   │   ├── Cargo.toml
│   │   └── src/
│   │       └── lib.rs         # #[derive(RosMessage)], etc.
│   │
│   ├── nano-ros-types/        # Standard ROS message types
│   │   ├── Cargo.toml
│   │   └── src/
│   │       ├── lib.rs
│   │       ├── std_msgs.rs    # std_msgs::msg::*
│   │       ├── geometry_msgs.rs
│   │       └── sensor_msgs.rs
│   │
│   ├── nano-ros-params/       # Parameter server (optional)
│   │   ├── Cargo.toml
│   │   └── src/
│   │       ├── lib.rs
│   │       ├── server.rs
│   │       └── types.rs
│   │
│   └── nano-ros-transport/    # Transport abstraction (zenoh backend)
│       ├── Cargo.toml
│       └── src/
│           ├── lib.rs
│           ├── traits.rs      # Transport trait
│           └── zenoh.rs       # Zenoh implementation
│
└── examples/
    ├── talker.rs
    ├── listener.rs
    └── service.rs
```

## Key Design Decisions

### 1. `no_std` Support

All core crates must support `#![no_std]` with optional `std` feature:

```rust
#![no_std]

#[cfg(feature = "std")]
extern crate std;

#[cfg(feature = "alloc")]
extern crate alloc;
```

**Feature flags:**
- `std` - Full standard library (desktop/Linux)
- `alloc` - Dynamic allocation only (embedded with heap)
- (default) - No heap, static buffers only

### 2. Type System (Replacing X-Macros)

Use Rust traits + proc macros instead of C X-macros:

```rust
// In nano-ros-core/src/types.rs
pub trait RosMessage: Sized {
    const TYPE_NAME: &'static str;
    const RIHS_HASH: &'static str;

    fn serialize<W: CdrWriter>(&self, writer: &mut W) -> Result<(), SerError>;
    fn deserialize<R: CdrReader>(reader: &mut R) -> Result<Self, DeserError>;
}

// Generated via proc macro
#[derive(RosMessage)]
#[ros(type_name = "std_msgs::msg::dds_::String_")]
pub struct StringMsg {
    pub data: heapless::String<256>,  // no_std compatible
}
```

### 3. Memory Management

**Static buffers for `no_std`:**
```rust
use heapless::{String, Vec};

pub struct Publisher<const BUF_SIZE: usize> {
    buffer: [u8; BUF_SIZE],
    // ...
}
```

**Dynamic allocation with `alloc` feature:**
```rust
#[cfg(feature = "alloc")]
use alloc::{string::String, vec::Vec};
```

### 4. Async Support

Design for async but don't require it:

```rust
// Blocking API (always available)
pub fn publish<M: RosMessage>(&mut self, msg: &M) -> Result<(), Error>;

// Async API (with feature flag)
#[cfg(feature = "async")]
pub async fn publish_async<M: RosMessage>(&mut self, msg: &M) -> Result<(), Error>;
```

### 5. Transport Abstraction

Abstract over zenoh-pico for portability:

```rust
pub trait Transport {
    type Publisher: TransportPublisher;
    type Subscriber: TransportSubscriber;
    type Error;

    fn create_publisher(&mut self, topic: &TopicInfo) -> Result<Self::Publisher, Self::Error>;
    fn create_subscriber<F>(&mut self, topic: &TopicInfo, callback: F) -> Result<Self::Subscriber, Self::Error>
    where F: FnMut(&[u8]) + Send;
}
```

## Original Pico-ROS Key Structures

### Node
```c
typedef struct {
    char* name;
    uint32_t domain_id;
    char guid[17];
    z_owned_session_t* session;
    zc_owned_liveliness_token_t node_token;
} picoros_node_t;
```

### Publisher
```c
typedef struct {
    z_owned_publisher_t publisher;
    char* topic_name;
    char* topic_type;
    char* rihs_hash;
    rmw_attachment_t attachment;
    z_publisher_put_options_t options;
    zc_owned_liveliness_token_t pub_token;
} picoros_publisher_t;
```

### Subscriber
```c
typedef struct {
    z_owned_subscriber_t subscriber;
    char* topic_name;
    char* topic_type;
    char* rihs_hash;
    picoros_sub_cb_t callback;
    void* userdata;
    zc_owned_liveliness_token_t sub_token;
} picoros_subscriber_t;
```

### RMW Attachment (for rmw_zenoh compatibility)
```c
typedef struct __attribute__((__packed__)) {
    int64_t sequence_number;
    int64_t time;
    uint8_t rmw_gid_size;
    uint8_t rmw_gid[16];
} rmw_attachment_t;
```

## CDR Serialization Format

- 4-byte header: `0x00 0x01 0x00 0x00` (little-endian CDR)
- Alignment: primitives aligned to their size
- Strings: 4-byte length prefix + data + null terminator
- Sequences: 4-byte count + elements

## Topic Naming Convention (rmw_zenoh compatible)

```
<domain_id>/<topic_name>/<message_type>_/RIHS01_<hash>
```

Example:
```
0/chatter/std_msgs::msg::dds_::String_/RIHS01_abc123...
```

## Service Naming Convention

```
<domain_id>/<service_name>/<service_type>_/RIHS01_<hash>
```

## Liveliness Token Format

```
@ros2_lv/<domain_id>/<zid>/<entity_type>/%%/<node_name>_<guid>/%<topic_name>/<type>_/RIHS01_<hash>/:,:,:,,
```

## Build Commands

```bash
# Desktop (full std)
cargo build --features std

# Embedded with heap
cargo build --no-default-features --features alloc --target thumbv7em-none-eabihf

# Bare metal (static only)
cargo build --no-default-features --target thumbv7em-none-eabihf
```

## Target Platform Notes

### Zephyr RTOS
- Use `zenoh-pico` Zephyr port
- Configure via Kconfig/devicetree
- Consider `embassy` for async

### NuttX RTOS
- POSIX-like interface simplifies porting
- Can use `std` with NuttX's libc
- `zenoh-pico` has NuttX support

## Error Handling

Use `Result<T, E>` throughout:

```rust
#[derive(Debug)]
pub enum Error {
    Serialization(SerError),
    Transport(TransportError),
    Timeout,
    InvalidMessage,
    BufferTooSmall,
}
```

## Testing Strategy

- Unit tests with `#[cfg(test)]` (requires `std`)
- Integration tests against real zenoh router
- Mock transport for isolated testing
- Consider `defmt` for embedded debugging

## Documentation

Development documentation lives in `docs/`:

```
docs/
├── roadmap/           # Development phases and milestones
│   └── phase-N-*.md   # Individual phase documents
├── architecture/      # Design documents and ADRs
└── api/               # API documentation (if not rustdoc)
```

## References

- Original Pico-ROS: `external/Pico-ROS-software/`
- zenoh-pico: `external/Pico-ROS-software/thirdparty/zenoh-pico/`
- Micro-CDR: `external/Pico-ROS-software/thirdparty/Micro-CDR/`
- ROS 2 CDR spec: OMG DDS-XTypes
- rmw_zenoh: https://github.com/ros2/rmw_zenoh
