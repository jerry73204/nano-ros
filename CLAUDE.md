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
│   ├── nano-ros/              # Unified API crate (like rclcpp/rclpy)
│   │   ├── Cargo.toml         # Re-exports all sub-crates
│   │   └── src/
│   │       └── lib.rs         # Main entry point with prelude
│   │
│   ├── nano-ros-core/         # Core types, traits, node abstraction
│   │   ├── Cargo.toml
│   │   └── src/
│   │       ├── lib.rs
│   │       ├── node.rs        # Node, Publisher, Subscriber
│   │       ├── service.rs     # Service traits (RosService, Request/Response)
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
│   ├── nano-ros-params/       # Parameter server (ROS 2 compatible)
│   │   ├── Cargo.toml
│   │   └── src/
│   │       ├── lib.rs
│   │       ├── server.rs      # ParameterServer with static storage
│   │       └── types.rs       # ParameterValue, ParameterDescriptor, etc.
│   │
│   ├── nano-ros-transport/    # Transport abstraction (zenoh backend)
│   │   ├── Cargo.toml
│   │   └── src/
│   │       ├── lib.rs
│   │       ├── traits.rs      # Transport trait
│   │       └── shim.rs        # zenoh-pico-shim implementation
│   │
│   ├── zenoh-pico-shim/       # High-level safe Rust API for zenoh-pico
│   │   ├── Cargo.toml
│   │   ├── src/
│   │   │   └── lib.rs         # ShimContext, ShimPublisher, ShimSubscriber, etc.
│   │   └── tests/
│   │       └── integration.rs # Integration tests (requires zenohd)
│   │
│   └── zenoh-pico-shim-sys/   # FFI bindings + all C code
│       ├── Cargo.toml
│       ├── build.rs           # Compiles zenoh-pico + C shim
│       ├── src/
│       │   └── lib.rs         # FFI declarations
│       ├── zenoh-pico/        # Git submodule (upstream C library)
│       └── c/
│           ├── shim/
│           │   ├── zenoh_shim.h   # Public C API (auto-generated)
│           │   └── zenoh_shim.c   # Core shim implementation
│           └── platform_smoltcp/  # smoltcp platform layer (optional)
│               ├── system.c       # Memory, random, clock
│               └── network.c      # TCP socket operations
│
├── colcon-nano-ros/           # Message binding generator
│   └── packages/
│       ├── cargo-nano-ros/    # cargo nano-ros subcommand
│       ├── rosidl-bindgen/    # Binding generator library
│       ├── rosidl-codegen/    # Code generator with templates
│       └── rosidl-parser/     # ROS IDL parser
│
└── examples/                  # Standalone packages (excluded from workspace)
    ├── native-talker/         # Pub/sub example with std_msgs
    │   ├── package.xml        # Declares std_msgs dependency
    │   ├── Cargo.toml
    │   ├── generated/         # Generated bindings (cargo nano-ros generate)
    │   └── .cargo/config.toml # Patches for generated crates
    ├── native-listener/
    ├── native-service-server/ # Service example with example_interfaces
    └── native-service-client/
```

**Note**: Message types are generated per-project using `cargo nano-ros generate` rather than
bundled in a `nano-ros-types` crate. This allows each project to generate only the types it
needs and supports any ROS interface package available in the ament index.

## Zenoh-Pico Bindings

The unified `zenoh-pico-shim` architecture provides Rust bindings for zenoh-pico
across all platforms (desktop, Zephyr, bare-metal with smoltcp).

### Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│  Application (nano-ros-node or direct use)                           │
└─────────────────────────────────────────────────────────────────────┘
                                 │
                                 ▼
┌─────────────────────────────────────────────────────────────────────┐
│  zenoh-pico-shim (High-level Rust API)                               │
│  ├── ShimContext - Session management                                │
│  ├── ShimPublisher - Publish with optional RMW attachment           │
│  ├── ShimSubscriber - Subscribe with callback                       │
│  ├── ShimLivelinessToken - ROS 2 discovery                          │
│  ├── ShimQueryable - ROS 2 service servers                          │
│  └── ShimZenohId - Session identifier                               │
└─────────────────────────────────────────────────────────────────────┘
                                 │
                                 ▼
┌─────────────────────────────────────────────────────────────────────┐
│  zenoh-pico-shim-sys (FFI + all C code)                              │
│  ├── c/shim/zenoh_shim.c - Core C shim implementation               │
│  ├── c/platform_smoltcp/ - smoltcp platform layer (optional)        │
│  └── zenoh-pico/ - Git submodule (upstream C library)               │
└─────────────────────────────────────────────────────────────────────┘
```

### Platform Selection

Platform-specific backends are selected via feature flags:

| Feature | Platform | Use Case |
|---------|----------|----------|
| `posix` | Linux/macOS | Desktop, native examples |
| `zephyr` | Zephyr RTOS | Embedded with Zephyr |
| `smoltcp` | Bare-metal | RTIC, Embassy, polling |

### Usage Example

```rust
use zenoh_pico_shim::{ShimContext, ShimError};

// Open session (locator must be null-terminated)
let ctx = ShimContext::new(b"tcp/127.0.0.1:7447\0")?;

// Check session is open
assert!(ctx.is_open());

// Create publisher
let publisher = ctx.declare_publisher(b"demo/chatter\0")?;

// Publish data
publisher.publish(b"Hello, world!")?;

// Publish with RMW attachment (for ROS 2 compatibility)
let attachment = [0u8; 33];  // RMW GID format
publisher.publish_with_attachment(b"data", Some(&attachment))?;

// Create subscriber with callback
extern "C" fn callback(data: *const u8, len: usize, _ctx: *mut c_void) {
    // Handle received data
}
let subscriber = unsafe {
    ctx.declare_subscriber_raw(b"demo/chatter\0", callback, std::ptr::null_mut())
}?;

// Liveliness token for ROS 2 discovery
let token = ctx.declare_liveliness(b"@ros2_lv/0/abc123/...\0")?;

// Get session ZenohId
let zid = ctx.zid()?;
let mut hex = [0u8; 32];
zid.to_hex_bytes(&mut hex);
```

**Design Notes:**
- `#![no_std]` compatible (requires `alloc` for some features)
- All resources use RAII for automatic cleanup on drop
- C shim uses integer handles (not pointers) for resource management
- Platform backends handle threading/polling differences

**Requirements:**
- Tests require a running `zenohd` router on `tcp/127.0.0.1:7447`
- Run integration tests with: `cargo test -p zenoh-pico-shim --features "posix std" -- --test-threads=1`

### C Shim API

The C shim (`zenoh_shim.c`) provides a simplified API for zenoh-pico that works
across all platforms. Key functions:

```c
// Session lifecycle
int32_t zenoh_shim_init_config(const char *locator);
int32_t zenoh_shim_open_session(void);
bool zenoh_shim_is_open(void);
void zenoh_shim_close(void);

// Publishers (returns handle >= 0 on success)
int32_t zenoh_shim_declare_publisher(const char *keyexpr);
int32_t zenoh_shim_publish(int32_t handle, const uint8_t *data, size_t len);
int32_t zenoh_shim_publish_with_attachment(int32_t handle, const uint8_t *data,
                                            size_t len, const uint8_t *attachment,
                                            size_t attachment_len);
void zenoh_shim_undeclare_publisher(int32_t handle);

// Subscribers (callback-based)
int32_t zenoh_shim_declare_subscriber(const char *keyexpr,
                                       zenoh_shim_sub_callback_t callback,
                                       void *context);
void zenoh_shim_undeclare_subscriber(int32_t handle);

// Liveliness (ROS 2 discovery)
int32_t zenoh_shim_declare_liveliness(const char *keyexpr);
void zenoh_shim_undeclare_liveliness(int32_t handle);

// Queryables (ROS 2 services)
int32_t zenoh_shim_declare_queryable(const char *keyexpr,
                                      zenoh_shim_queryable_callback_t callback,
                                      void *context);
void zenoh_shim_undeclare_queryable(int32_t handle);

// Session info
int32_t zenoh_shim_get_zid(uint8_t *out);  // 16 bytes

// Polling (for smoltcp backend)
bool zenoh_shim_uses_polling(void);
int32_t zenoh_shim_poll(uint32_t timeout_ms);
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

The RMW attachment is a 33-byte metadata blob attached to each published message.
It is required for ROS 2 nodes using rmw_zenoh_cpp to receive messages correctly.

```c
typedef struct __attribute__((__packed__)) {
    int64_t sequence_number;  // Incremented per publish (starts at 1)
    int64_t time;             // Timestamp in nanoseconds
    uint8_t rmw_gid_size;     // Always 16 (VLE encoded as single byte)
    uint8_t rmw_gid[16];      // Random GID, generated once per publisher
} rmw_attachment_t;
```

**Implementation notes:**
- Sequence number must increment for each message (use `AtomicI64` for interior mutability)
- GID is generated once when the publisher is created and remains constant
- Timestamp can be monotonic counter if real-time clock is unavailable
- Total size: 8 + 8 + 1 + 16 = 33 bytes

## CDR Serialization Format

- 4-byte header: `0x00 0x01 0x00 0x00` (little-endian CDR)
- Alignment: primitives aligned to their size
- Strings: 4-byte length prefix + data + null terminator
- Sequences: 4-byte count + elements

## Topic Naming Convention (rmw_zenoh compatible)

**Data key expression format (for ROS 2 Humble):**
```
<domain_id>/<topic_name>/<message_type>/<type_hash>
```

Example:
```
0/chatter/std_msgs::msg::dds_::Int32_/TypeHashNotSupported
```

**IMPORTANT - Humble vs Iron+ differences:**
- **Data keyexprs (Humble)**: Use `TypeHashNotSupported` as the type hash (no `RIHS01_` prefix)
- **Data keyexprs (Iron+)**: Use `RIHS01_<sha256>` format
- **Liveliness keyexprs (ALL versions)**: Always use `RIHS01_<hash>` prefix (even if hash is all zeros)

This distinction is critical for ROS 2 interoperability. The data keyexpr format determines
whether rmw_zenoh_cpp can receive messages from nano-ros.

## Service Naming Convention

```
<domain_id>/<service_name>/<service_type>_/RIHS01_<hash>
```

## Liveliness Token Format

**Node:**
```
@ros2_lv/<domain_id>/<zid>/0/0/NN/%/%/<node_name>
```

**Publisher:**
```
@ros2_lv/<domain_id>/<zid>/0/11/MP/%/%/<node_name>/%<topic>/<type>/RIHS01_<hash>/<qos>
```

**Subscriber:**
```
@ros2_lv/<domain_id>/<zid>/0/11/MS/%/%/<node_name>/%<topic>/<type>/RIHS01_<hash>/<qos>
```

Key points:
- ZenohId must be LSB-first hex format
- Topic names use `%` prefix (e.g., `%chatter`)
- Liveliness tokens **do** use `RIHS01_` prefix (unlike data keyexprs)
- QoS format: `reliability:durability:history,depth:...` (e.g., `2:2:1,1:,:,:,,` for BEST_EFFORT)

## Build Commands

Use `just` for common tasks (default recipes use `no_std`):

```bash
just setup          # Install toolchains and tools
just build          # Build with no_std (default)
just build-embedded # Build for thumbv7em-none-eabihf
just check          # Format + clippy with no_std
just test           # Run all tests (unit, miri, QEMU, integration)
just test-miri      # Run Miri for undefined behavior detection
just format         # Format code (nightly)
just quality        # Run check + test
just doc            # Generate and open docs
just clean          # Clean build artifacts

# Embedded examples (STM32F4)
just build-examples-embedded  # Build RTIC, Embassy, polling, smoltcp examples
just check-examples-embedded  # Check without full build
just size-examples-embedded   # Show binary sizes

# Integration tests (tests/ directory)
just test-integration        # Run all integration tests (requires zenohd)
just test-integration-quick  # Quick smoke test
just test-nano2nano          # nano-ros ↔ nano-ros tests
just test-rmw-interop        # ROS 2 interop (requires rmw_zenoh_cpp)
just test-rmw-detailed       # Protocol-level RMW tests
just test-zephyr             # Zephyr QEMU tests (requires west)

# Real-time static analysis
just check-realtime    # Clippy with RT safety lints
just check-miri        # Miri UB detection
just analyze-stack     # Stack usage analysis (nightly)
just static-analysis   # Run all static analysis

# zenoh-pico shim tests
just test-zenoh-shim   # Test zenoh-pico-shim (requires zenohd)
```

**Toolchain**: `rust-toolchain.toml` configures stable + `thumbv7em-none-eabihf` target.

## Target Platform Notes

### Zephyr RTOS
- Use `zenoh-pico` Zephyr port
- Configure via Kconfig/devicetree
- Consider `embassy` for async

### NuttX RTOS
- POSIX-like interface simplifies porting
- Can use `std` with NuttX's libc
- `zenoh-pico` has NuttX support

## RTIC and Embedded Support

nano-ros supports RTIC (Real-Time Interrupt-driven Concurrency) and other embedded
real-time frameworks through the `rtic` feature flag.

### Feature Flags

```toml
# For RTIC/Embassy applications
[dependencies]
nano-ros-node = { path = "...", default-features = false, features = ["rtic"] }
nano-ros-transport = { path = "...", default-features = false, features = ["rtic", "sync-critical-section"] }
```

### Static Buffer Allocation

The `rtic` feature enables static buffer allocation via const generics:

```rust
use nano_ros_node::{ConnectedNode, ConnectedSubscriber, DEFAULT_MAX_TOKENS};

// Node with custom token limit
let node: ConnectedNode<8> = ...;

// Subscriber with custom buffer size
let subscriber: ConnectedSubscriber<MyMsg, 512> =
    node.create_subscriber_sized("/topic")?;

// Service with custom request/reply buffers
let server: ConnectedServiceServer<MyService, 256, 256> =
    node.create_service_sized("/service")?;
```

### Default Buffer Sizes

| Constant | Default | Purpose |
|----------|---------|---------|
| `DEFAULT_MAX_TOKENS` | 16 | Max liveliness tokens (publishers + subscribers) |
| `DEFAULT_RX_BUFFER_SIZE` | 1024 | Subscriber receive buffer |
| `DEFAULT_REQ_BUFFER_SIZE` | 1024 | Service request buffer |
| `DEFAULT_REPLY_BUFFER_SIZE` | 1024 | Service reply buffer |

### Timing Constants

For manual polling (without background threads):

```rust
use nano_ros_node::rtic::{POLL_INTERVAL_MS, KEEPALIVE_INTERVAL_MS};

// POLL_INTERVAL_MS = 10ms - How often to poll for incoming messages
// KEEPALIVE_INTERVAL_MS = 1000ms - How often to send keepalive
```

### Example Patterns

**RTIC Example** (`examples/rtic-stm32f4/`):
```rust
#[task(priority = 2)]
async fn zenoh_poll(_cx: zenoh_poll::Context) {
    loop {
        // Poll for incoming messages
        Mono::delay(POLL_INTERVAL_MS.millis()).await;
    }
}

#[task(priority = 1)]
async fn zenoh_keepalive(_cx: zenoh_keepalive::Context) {
    loop {
        // Send keepalive
        Mono::delay(KEEPALIVE_INTERVAL_MS.millis()).await;
    }
}
```

**Embassy Example** (`examples/embassy-stm32f4/`):
```rust
#[embassy_executor::task]
async fn zenoh_poll_task() {
    loop {
        Timer::after(Duration::from_millis(POLL_INTERVAL_MS as u64)).await;
    }
}
```

**Polling Example** (`examples/polling-stm32f4/`):
```rust
// Simple main loop without executor
loop {
    if poll_timer.elapsed_ms(POLL_INTERVAL_MS) {
        // Poll zenoh
    }
    if keepalive_timer.elapsed_ms(KEEPALIVE_INTERVAL_MS) {
        // Send keepalive
    }
}
```

### Memory Requirements

See `docs/memory-requirements.md` for detailed memory calculations.

### WCET Analysis

See `docs/wcet-analysis.md` for Worst-Case Execution Time analysis workflow including:
- RTIC-Scope integration for hardware tracing
- DWT cycle counter measurement
- Task-specific WCET characteristics

### Schedulability Analysis

See `docs/schedulability-analysis.md` for Rate-Monotonic Analysis (RMA) and response time analysis:
- Utilization bound tests
- Response time calculation with worked examples
- Priority Ceiling Protocol blocking analysis
- Python script for automated analysis

### Real-Time Static Analysis

See `docs/realtime-lint-guide.md` for detecting anti-patterns that violate real-time guarantees:

| Anti-Pattern | Detection Method |
|--------------|------------------|
| Unbounded loops | Clippy: `infinite_iter`, `while_immutable_condition` |
| Recursion | Clippy: `unconditional_recursion` + cargo-call-stack |
| Large stack arrays | Clippy: `large_stack_arrays` |
| Heap allocation | `no_std` + `heapless` crate |
| Missing timeouts | Custom Dylint lints |

**Quick check:**
```bash
just check-realtime     # Clippy with RT safety lints
just static-analysis    # Full analysis suite (includes Miri)
```

### Zenoh-Pico Heap Requirements

**Important:** Zenoh-pico requires heap allocation via `z_malloc`/`z_free` for:
- Session creation and management
- Publishers and subscribers (state objects)
- I/O buffers for network packets
- Message samples and internal collections

This means:
- The Rust `zenoh` feature implies `alloc`
- Pure `no_std` without allocator cannot use zenoh directly
- Embedded systems need RTOS heap support (FreeRTOS, Zephyr)

**Recommended pattern for embedded:** Use the C shim approach (see `examples/zephyr-talker-rs/`).
The C shim manages zenoh-pico's heap allocations while keeping Rust code simple.

### Embedded Integration Guide

See `docs/embedded-integration.md` for comprehensive embedded integration documentation including:
- C shim pattern for Zephyr/FreeRTOS
- RTIC, Embassy, and polling examples
- Memory budgeting
- Hardware requirements

### Build Verification

```bash
# Verify no-alloc build compiles
just check-no-alloc
```

This checks that the RTIC features work without heap allocation.

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

### Test Locations

| Location | Type | Description |
|----------|------|-------------|
| `crates/*/tests/` | Unit/Integration | Crate-specific tests (e.g., `zenoh-pico-shim/tests/`) |
| `tests/` | Integration | Multi-component tests requiring multiple processes |
| `examples/qemu-test/` | QEMU | Cortex-M embedded tests |

### Test Categories

**Unit Tests** (`cargo nextest run`):
- CDR serialization correctness
- Message type derive macros
- Parameter server logic
- Run with: `just test`

**Miri Tests** (undefined behavior detection):
- Memory safety in `no_std` crates
- Run with: `just test-miri`

**Integration Tests** (`tests/` directory):
- `nano2nano/` - nano-ros ↔ nano-ros pub/sub and services
- `rmw-interop/` - ROS 2 ↔ nano-ros communication (requires rmw_zenoh_cpp)
- `rmw-detailed/` - Protocol-level RMW compatibility
- `zephyr/` - Zephyr QEMU tests (requires west workspace)
- `simple-workspace/` - Standalone build verification

### Running Tests

```bash
# All tests (unit + miri + QEMU + integration)
just test              # Requires zenohd for integration tests

# Individual test suites
just test-workspace    # Unit tests only (cargo nextest)
just test-miri         # Miri undefined behavior detection
just qemu-test         # QEMU Cortex-M3 test
just test-integration  # Integration tests (requires zenohd)

# Specific integration tests
just test-nano2nano    # nano-ros ↔ nano-ros only
just test-rmw-interop  # ROS 2 interop (requires rmw_zenoh_cpp)
just test-zephyr       # Zephyr QEMU tests (requires west)

# Full quality check
just quality           # format + clippy + all tests
```

### Prerequisites

**For integration tests:**
- `zenohd` router installed and in PATH
- For ROS 2 interop: ROS 2 Humble + rmw_zenoh_cpp

**For Zephyr tests:**
- West workspace with nano-ros as a module
- See `zephyr/setup.sh` for workspace setup

## Development Practices

### System Package Installation

**Never install system packages directly.** When a system package is needed:
1. Stop and inform the user what package is required
2. Provide the exact installation command for them to run
3. Continue after user confirms installation

Example:
```
QEMU ARM emulator is required but not installed.
Please run: sudo apt install qemu-system-arm
```

This ensures the user maintains control over their system and is aware of all dependencies being installed.

### Privileged Commands (sudo)

**Never execute sudo commands directly.** When root privileges are needed:
1. Explain what needs to be done and why it requires root
2. Provide the exact command(s) for the user to run
3. Wait for user confirmation before continuing

Example:
```
TAP interface setup requires root privileges.
Please run: sudo ./scripts/setup-zephyr-network.sh

This creates a TAP device owned by your user, allowing
Zephyr to run without sudo afterward.
```

This ensures users maintain control over privileged operations.

### Quality Checks

**Always run `just quality` after completing a task.** This runs:
1. Format check (`cargo +nightly fmt --check`)
2. Clippy with no default features (`-D warnings`)
3. All tests via nextest

Fix any issues before considering a task complete.

### Handling Unused Variable Warnings

When encountering unused variable warnings, follow this practice:

1. **Rename to `_name`** to suppress the warning
2. **Always add a comment** explaining why the variable is unused
3. **Use `#[allow(dead_code)]`** for struct fields in test code that exist for structural completeness

**Common patterns:**

```rust
// Trait method signature requires parameter, but implementation doesn't use it
fn serialize(&self, _writer: &mut CdrWriter) -> Result<(), SerError> {
    // Empty message - no fields to serialize
    Ok(())
}

// Test mock with fields for structural completeness
#[derive(Debug, Clone)]
struct MockRequest {
    // Fields exist for structural completeness but are not read
    // since the mock serialize/deserialize implementations are no-ops
    #[allow(dead_code)]
    pub a: i32,
    #[allow(dead_code)]
    pub b: i32,
}
```

**When to use TODO comments:**

If the unused variable represents incomplete functionality or future work, add a TODO:

```rust
fn process(&self, _data: &[u8]) -> Result<()> {
    // TODO: Implement data processing when the protocol layer is complete
    Ok(())
}
```

## Documentation

Development documentation lives in `docs/`:

```
docs/
├── rmw_zenoh_interop.md           # ROS 2 rmw_zenoh protocol documentation
├── embedded-integration.md        # Embedded integration guide (RTIC, Embassy, Zephyr)
├── wcet-analysis.md               # WCET measurement and analysis workflow
├── schedulability-analysis.md     # RMA and response time analysis
├── realtime-lint-guide.md         # Static analysis for RT anti-patterns
├── memory-requirements.md         # Memory budgeting for embedded
├── roadmap/                       # Development phases and milestones
│   ├── phase-1-foundation.md      # CDR, types, macros (COMPLETE)
│   ├── phase-2-zephyr-qemu.md     # Zephyr, QEMU, transport (COMPLETE)
│   └── phase-3-services-params.md # Services, parameters, hardware (PLANNING)
├── architecture/                  # Design documents and ADRs
└── api/                           # API documentation (if not rustdoc)
```

## Development Phases

| Phase | Focus | Status |
|-------|-------|--------|
| Phase 1 | CDR serialization, types, proc macros | **Complete** |
| Phase 2A | ROS 2 Interoperability (native) | **Complete** |
| Phase 2B | Zephyr integration + QEMU testing | **Complete** |
| Phase 3 | Services, parameters, unified API | **In Progress** |
| Phase 4 | Message generation (cargo nano-ros) | **Complete** |
| Phase 5 | RTIC integration | **Complete** |
| Phase 6 | ROS 2 Actions | **Planning** |

See `docs/roadmap/` for detailed work items.

**Deployment Model:**
- ROS 2 nodes run on Linux host using `rmw_zenoh_cpp`
- nano-ros nodes run on Zephyr RTOS (embedded targets)
- Communication via zenoh router on host

### Phase 2A Progress (ROS 2 Interoperability) - COMPLETE
- [x] `zenoh-pico-sys` - FFI bindings with static linking
- [x] `zenoh-pico` - Safe wrapper (Config, Session, Publisher, Subscriber, Liveliness)
- [x] `nano-ros-transport` - ZenohTransport backend with RMW Attachment support
- [x] `nano-ros-node` - ConnectedNode with transport integration
- [x] RMW Attachment support - Required metadata for rmw_zenoh compatibility
- [x] Liveliness tokens - Node/publisher/subscriber discovery
- [x] Native examples updated - talker/listener use real zenoh transport
- [x] nano-ros ↔ nano-ros communication tested and working
- [x] ROS 2 → nano-ros communication working (via wildcard subscriber)
- [x] nano-ros → ROS 2 communication working (Humble)

### Phase 2B Progress (Zephyr Integration) - COMPLETE
- [x] All core crates no_std compatible
- [x] West manifest workflow (`west.yml`, `zephyr/module.yml`)
- [x] Zephyr workspace setup script (`zephyr/setup.sh`)
- [x] zenoh-pico added to west manifest (v1.5.1)
- [x] Zephyr talker with zenoh-pico transport (`examples/zephyr-talker-rs/`)
- [x] Zephyr listener with zenoh-pico transport (`examples/zephyr-listener-rs/`)
- [x] C shim for zenoh-pico FFI (avoids struct size issues)
- [x] Network setup script (`scripts/setup-zephyr-network.sh`)
- [x] End-to-end test: Zephyr native_sim → zenoh router → native subscriber
- [ ] Hardware validation (deferred to Phase 3)

### Phase 3 Progress (Services, Parameters, Unified API, Hardware) - IN PROGRESS

**3.1 ROS 2 Services - Infrastructure Complete:**
- [x] Service traits (`RosService` with Request/Response types)
- [x] zenoh-pico queryable support (for service servers)
- [x] Transport layer service client/server (`ServiceInfo`, transport traits)
- [x] Node API: `create_service()`, `create_client()` (in ConnectedNode)
- [ ] `#[derive(RosService)]` proc macro
- [ ] Service examples and ROS 2 interop tests
- [ ] `rcl_interfaces` service types for parameters

**3.2 ROS 2 Parameters - Core Complete:**
- [x] Complete `nano-ros-params` implementation
- [x] Parameter types (`ParameterValue`, `ParameterType`, `ParameterDescriptor`)
- [x] Parameter server with static storage (`ParameterServer`, MAX_PARAMETERS=32)
- [x] Constraint validation (read-only, type checking, range constraints)
- [x] Fluent API via `ParameterBuilder`
- [ ] `rcl_interfaces` message/service types
- [ ] Parameter service handlers (`~/get_parameters`, `~/set_parameters`, etc.)
- [ ] ROS 2 CLI interop (`ros2 param list/get/set`)

**3.3 Unified `nano-ros` Crate - Complete:**
- [x] Create `crates/nano-ros/` as main entry point
- [x] Re-export all sub-crate types (`nano_ros::prelude::*`)
- [x] Feature flags: `std`, `alloc`, `zenoh`
- [x] `no_std` compatible
- [x] Parameter types integrated
- [x] Service types integrated
- [ ] Update all examples to use unified API

**3.4 Hardware Validation:**
- [ ] NUCLEO-F429ZI (STM32F429, Ethernet)
- [ ] Performance benchmarks (latency, throughput, memory)
- [ ] Reliability testing

**Next Steps:** Service examples, update examples to unified API, hardware validation.

### Phase 4 Progress (Message Generation) - COMPLETE

The `cargo nano-ros generate` command is now available for generating `no_std` compatible
message bindings from ROS 2 interface packages.

**Implementation**:
- [x] `rosidl-parser` - Parse .msg, .srv, .action files
- [x] `rosidl-codegen` - Code generation with Askama templates
- [x] `rosidl-bindgen` - Binding generator library
- [x] `cargo-nano-ros` - Standalone cargo subcommand
- [x] Dependency resolution via ament index
- [x] `.cargo/config.toml` generation with patches
- [x] `no_std` compatible output using `heapless` types
- [x] Native examples updated to use generated bindings

**Usage**:
```bash
cargo nano-ros generate                    # Generate from package.xml
cargo nano-ros generate --force            # Regenerate existing
cargo nano-ros generate --output bindings  # Custom output directory
```

See "Message Binding Generation" section above for detailed documentation.

## Message Binding Generation

nano-ros uses generated Rust bindings for ROS 2 message types. The `cargo nano-ros generate` command generates `no_std` compatible bindings from `package.xml` dependencies.

### Overview

The binding generator lives in `colcon-nano-ros/packages/cargo-nano-ros/` and provides:
- Standalone `cargo nano-ros` subcommand for generating bindings
- Pure Rust, `no_std` compatible output using `heapless` types
- Automatic dependency resolution via ament index
- `.cargo/config.toml` generation for crate patches

### Prerequisites

1. **ROS 2 environment sourced** - Required for ament index access
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. **package.xml in project root** - Declares ROS interface dependencies
   ```xml
   <?xml version="1.0"?>
   <package format="3">
     <name>my_package</name>
     <version>0.1.0</version>
     <description>My nano-ros package</description>
     <maintainer email="dev@example.com">Developer</maintainer>
     <license>Apache-2.0</license>
     <depend>std_msgs</depend>
     <depend>geometry_msgs</depend>
     <export>
       <build_type>ament_cargo</build_type>
     </export>
   </package>
   ```

3. **cargo-nano-ros installed**
   ```bash
   cd colcon-nano-ros && just install
   # Or: cargo install --path colcon-nano-ros/packages/cargo-nano-ros
   ```

### Workflow

**Step 1: Create package.xml**

Declare your ROS interface dependencies in `<depend>` tags:
```xml
<depend>std_msgs</depend>      <!-- For std_msgs::msg::Int32, String, etc. -->
<depend>example_interfaces</depend>  <!-- For service types -->
```

**Step 2: Generate bindings**

```bash
cd my_project
cargo nano-ros generate
```

This will:
1. Parse `package.xml` to find dependencies
2. Resolve transitive dependencies via ament index
3. Filter to interface packages (those with msg/srv/action)
4. Generate bindings to `generated/` directory
5. Create `.cargo/config.toml` with `[patch.crates-io]` entries

**Step 3: Add dependencies to Cargo.toml**

Reference the generated crates using crates.io version specifiers:
```toml
[dependencies]
std_msgs = { version = "*", default-features = false }
example_interfaces = { version = "*", default-features = false }
```

The `.cargo/config.toml` patches redirect these to local paths.

**Step 4: Use in code**

```rust
use std_msgs::msg::Int32;
use example_interfaces::srv::{AddTwoInts, AddTwoIntsRequest, AddTwoIntsResponse};

let msg = Int32 { data: 42 };
```

### Command Options

```bash
cargo nano-ros generate [OPTIONS]

Options:
  -m, --manifest <PATH>     Path to package.xml [default: package.xml]
  -o, --output <DIR>        Output directory [default: generated]
  -n, --nano-ros <PATH>     Path to nano-ros crates (for config patches)
  -f, --force               Overwrite existing bindings
  -v, --verbose             Enable verbose output
      --no-config           Skip .cargo/config.toml generation
```

### Generated Output Structure

```
my_project/
├── package.xml              # Your dependency declarations
├── Cargo.toml               # Your package manifest
├── src/
│   └── main.rs              # Your code using generated types
├── generated/               # Generated bindings (do not edit)
│   ├── std_msgs/
│   │   ├── Cargo.toml
│   │   └── src/
│   │       ├── lib.rs       # #![no_std]
│   │       └── msg/
│   │           ├── mod.rs
│   │           └── int32.rs
│   └── builtin_interfaces/  # Transitive dependency
│       └── ...
└── .cargo/
    └── config.toml          # [patch.crates-io] entries
```

### Generated Code Features

**no_std by default:**
```rust
#![no_std]

pub mod msg;
```

**std feature for optional std support:**
```toml
[features]
default = []
std = ["nano-ros-core/std", "nano-ros-serdes/std"]
```

**heapless types for embedded:**
```rust
pub struct String {
    pub data: heapless::String<256>,
}

pub struct Arrays {
    pub data: heapless::Vec<i32, 64>,
}
```

**Service types with Request/Response:**
```rust
pub struct AddTwoInts;
pub struct AddTwoIntsRequest { pub a: i64, pub b: i64 }
pub struct AddTwoIntsResponse { pub sum: i64 }

impl RosService for AddTwoInts {
    type Request = AddTwoIntsRequest;
    type Reply = AddTwoIntsResponse;
}
```

### Example: Native Examples

The `examples/native-*` directories demonstrate this workflow:

```bash
# Generate bindings for native-talker
cd examples/native-talker
source /opt/ros/humble/setup.bash
cargo nano-ros generate

# Build and run
cargo build
cargo run -- --tcp 127.0.0.1:7447
```

Each example has:
- `package.xml` declaring `std_msgs` or `example_interfaces`
- `Cargo.toml` with message crate dependencies
- Generated bindings in `generated/`
- `.cargo/config.toml` with patches

### Standalone Package Mode

Examples are configured as standalone packages (excluded from workspace) because each has its own `.cargo/config.toml` patches. Building from workspace root would conflict.

```toml
# Root Cargo.toml
[workspace]
exclude = [
    "examples/native-talker",
    "examples/native-listener",
    "examples/native-service-server",
    "examples/native-service-client",
]
```

Build each example from its own directory:
```bash
cd examples/native-talker && cargo build
cd examples/native-service-client && cargo build
```

### Regenerating Bindings

To regenerate after ROS package updates or dependency changes:
```bash
cargo nano-ros generate --force
```

### Troubleshooting

**"Failed to load ament index"**
- Ensure ROS 2 is sourced: `source /opt/ros/humble/setup.bash`

**"Package 'X' not found in ament index"**
- Check package is installed: `ros2 pkg list | grep X`
- Install if missing: `sudo apt install ros-humble-X`

**Build errors with generated code**
- Regenerate with `--force` flag
- Check nano-ros crate compatibility

## ROS 2 rmw_zenoh Interoperability

See `docs/rmw_zenoh_interop.md` for detailed protocol documentation.

### Current Status - WORKING

| Direction | Status | Notes |
|-----------|--------|-------|
| nano-ros ↔ nano-ros | ✅ Working | Full pub/sub communication |
| ROS 2 → nano-ros | ✅ Working | Uses wildcard subscriber to match any type hash |
| nano-ros → ROS 2 | ✅ Working | Tested with ROS 2 Humble + rmw_zenoh_cpp |

### Key Implementation Details

**Key Expression Formats (Critical for Interop):**

| Component | Humble Format | Iron+ Format |
|-----------|--------------|--------------|
| Data keyexpr | `<domain>/<topic>/<type>/TypeHashNotSupported` | `<domain>/<topic>/<type>/RIHS01_<hash>` |
| Liveliness keyexpr | `@ros2_lv/.../<type>/RIHS01_<hash>/<qos>` | `@ros2_lv/.../<type>/RIHS01_<hash>/<qos>` |

**Common Mistakes (that break interop):**
1. Using `RIHS01_` prefix in data keyexpr for Humble → ROS 2 won't receive messages
2. Double `RIHS01_` prefix in liveliness (e.g., `RIHS01_RIHS01_...`) → Discovery fails
3. Non-incrementing sequence numbers → Some ROS 2 tools may ignore messages

**RMW Attachment (33 bytes):**
- Sequence number: `i64`, must increment per publish (use `AtomicI64`)
- Timestamp: `i64`, nanoseconds (monotonic counter OK if no RTC)
- GID size: `u8`, always 16 (VLE encoded)
- GID: `[u8; 16]`, random, constant per publisher

**Other Details:**
- QoS string: `2:2:1,1:,:,:,,` (BEST_EFFORT/VOLATILE/KEEP_LAST,1)
- ZenohId: LSB-first hex format in liveliness tokens
- Topic names: Use `%` prefix in liveliness (e.g., `%chatter`)

### Test Commands

**Using integration test suite:**
```bash
# Run all ROS 2 interop tests
just test-rmw-interop

# Run protocol-level tests
just test-rmw-detailed

# Run nano-ros ↔ nano-ros tests
just test-nano2nano
```

**Manual testing:**
```bash
# Terminal 1: Start zenoh router
zenohd --listen tcp/127.0.0.1:7447

# Terminal 2: Run nano-ros talker
cd examples/native-talker && cargo run -- --tcp 127.0.0.1:7447

# Terminal 3: Run ROS 2 listener
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/127.0.0.1:7447"]'
ros2 topic echo /chatter std_msgs/msg/Int32 --qos-reliability best_effort
```

See `tests/README.md` for detailed test documentation.

### Zephyr E2E Test (native_sim)

```bash
# One-time setup: Create TAP interface for Zephyr networking
sudo ./scripts/setup-zephyr-network.sh

# Terminal 1: Start zenoh router
zenohd --listen tcp/0.0.0.0:7447

# Terminal 2: Run native subscriber
cargo run -p zenoh-pico --example sub_test --features std

# Terminal 3: Build and run Zephyr talker (from zephyr workspace)
cd ~/nano-ros-workspace
west build -b native_sim/native/64 nano-ros/examples/zephyr-talker-rs
./build/zephyr/zephyr.exe
```

The Zephyr talker connects to the host at 192.0.2.2:7447 and publishes
Int32 messages to `demo/chatter`. The native subscriber receives and
decodes the messages.

## References

- Original Pico-ROS: `external/Pico-ROS-software/`
- zenoh-pico: `external/Pico-ROS-software/thirdparty/zenoh-pico/`
- Micro-CDR: `external/Pico-ROS-software/thirdparty/Micro-CDR/`
- ROS 2 CDR spec: OMG DDS-XTypes
- rmw_zenoh: https://github.com/ros2/rmw_zenoh
- Zephyr Rust: https://github.com/zephyrproject-rtos/zephyr-lang-rust
- Embassy: https://embassy.dev/
- Ferrocene (deferred): https://ferrocene.dev/
