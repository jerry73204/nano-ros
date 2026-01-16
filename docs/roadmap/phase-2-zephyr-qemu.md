# Phase 2: Zephyr Integration & QEMU Testing

**Status: PLANNING**

Implement transport layer, Zephyr integration, and multi-node QEMU testing infrastructure.

## Goals

- Add QEMU testing support for Cortex-M targets
- Implement Zenoh-pico transport backend
- Create working examples for Zephyr RTOS
- Demonstrate two-node communication (Zephyr QEMU + x86 native)

## Prerequisites

- Phase 1 complete (CDR serialization, message types, proc macros)
- Zephyr SDK installed
- QEMU with ARM support (`qemu-system-arm`)

## Zenoh-Pico Rust Bindings Research

### Current State (as of January 2025)

| Option                    | Status             | Notes                                       |
|---------------------------|--------------------|---------------------------------------------|
| `zenoh-pico-sys` crate    | **Does not exist** | No Rust FFI bindings on crates.io           |
| Pure Rust `no_std` zenoh  | **Does not exist** | Main zenoh crate requires `std`             |
| zenoh-pico C library      | **Available**      | Mature, tested on Zephyr/ESP32              |
| zenoh-c (Rust→C bindings) | Available          | Opposite direction (wraps Rust zenoh for C) |

### Recommended Approach: Create `zenoh-pico-sys` Crate

Since no Rust bindings exist, we need to create our own using `bindgen`:

```rust
// build.rs for zenoh-pico-sys
use std::env;
use std::path::PathBuf;

fn main() {
    // Link zenoh-pico C library
    println!("cargo:rustc-link-lib=zenohpico");

    let bindings = bindgen::Builder::default()
        .header("zenoh-pico/include/zenoh-pico.h")
        .use_core()                    // no_std compatible
        .ctypes_prefix("cty")          // Use cty crate for C types
        .parse_callbacks(Box::new(bindgen::CargoCallbacks))
        .generate()
        .expect("Unable to generate bindings");

    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
    bindings
        .write_to_file(out_path.join("bindings.rs"))
        .expect("Couldn't write bindings!");
}
```

### Alternative Approaches

1. **Minimal Zenoh Protocol Implementation** (Future)
   - Implement subset of Zenoh protocol in pure Rust
   - Pros: No C dependency, full `no_std` support
   - Cons: Significant effort, protocol compatibility risk

2. **Use Main Zenoh Crate for x86** (Hybrid)
   - Use `zenoh` crate on Linux/x86 (native)
   - Use `zenoh-pico` FFI on Cortex-M (embedded)
   - Pros: Best of both worlds
   - Cons: Two code paths to maintain

### References

- [zenoh-pico GitHub](https://github.com/eclipse-zenoh/zenoh-pico) - C library for constrained devices
- [zenoh-c GitHub](https://github.com/eclipse-zenoh/zenoh-c) - C bindings to Rust zenoh
- [Zenoh for Microcontrollers](https://github.com/eclipse-zenoh/zenoh/wiki/Zenoh--For-Microcontrollers) - Official guide
- [bindgen no_std](https://rust-lang.github.io/rust-bindgen/nocopy.html) - FFI generation

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                        Application Layer                         │
│   (nano-ros Node, Publisher, Subscriber, Service Client/Server) │
└─────────────────────────────────────────────────────────────────┘
                                │
┌─────────────────────────────────────────────────────────────────┐
│                      nano-ros-transport                          │
│         ┌─────────────┐              ┌─────────────┐            │
│         │ zenoh-pico  │              │   (future)  │            │
│         │   backend   │              │   backends  │            │
│         └─────────────┘              └─────────────┘            │
└─────────────────────────────────────────────────────────────────┘
                                │
┌─────────────────────────────────────────────────────────────────┐
│                         Platform Layer                           │
│   ┌──────────────┐    ┌──────────────┐    ┌──────────────┐     │
│   │    Zephyr    │    │    Linux     │    │   Bare-metal │     │
│   │  (Cortex-M)  │    │    (x86)     │    │   (Cortex-M) │     │
│   └──────────────┘    └──────────────┘    └──────────────┘     │
└─────────────────────────────────────────────────────────────────┘
```

## Work Items

### 1. QEMU Testing Infrastructure

#### 1.1 Add QEMU Test Dependencies
- [ ] Add `cortex-m-rt` for Cortex-M runtime
- [ ] Add `cortex-m-semihosting` for debug output
- [ ] Add `qemu-exit` for test exit codes
- [ ] Add `panic-semihosting` with exit feature
- [ ] Add `defmt` for embedded logging

#### 1.2 Create QEMU Test Harness
- [ ] Create `examples/qemu-test/` directory structure
- [ ] Implement minimal Cortex-M3 binary that runs tests
- [ ] Add semihosting-based test output
- [ ] Create `scripts/run-qemu.sh` helper script
- [ ] Add `just qemu-test` recipe

#### 1.3 QEMU Configuration
- [ ] Target: `thumbv7m-none-eabi` (Cortex-M3)
- [ ] Machine: `lm3s6965evb` (well-supported in QEMU)
- [ ] Semihosting enabled for stdio and exit
- [ ] Memory layout configuration (`.cargo/config.toml`)

**Example QEMU command:**
```bash
qemu-system-arm \
  -cpu cortex-m3 \
  -machine lm3s6965evb \
  -nographic \
  -semihosting-config enable=on,target=native \
  -kernel target/thumbv7m-none-eabi/release/qemu-test
```

### 2. Transport Layer Implementation

#### 2.1 Transport Trait Design
- [ ] Define `Transport` trait in `nano-ros-transport`
- [ ] Define `Publisher<T>` and `Subscriber<T>` traits
- [ ] Define `Session` trait for connection management
- [ ] Support both sync and async APIs

```rust
// Target API design
pub trait Transport {
    type Error;
    type Session: Session;

    fn connect(config: &Config) -> Result<Self::Session, Self::Error>;
}

pub trait Publisher<T: RosMessage> {
    fn publish(&self, msg: &T) -> Result<(), Error>;
}

pub trait Subscriber<T: RosMessage> {
    fn take(&mut self) -> Option<T>;
    // async version for Embassy
    async fn recv(&mut self) -> T;
}
```

#### 2.2 Zenoh-Pico FFI Bindings (zenoh-pico-sys)

**Note:** No existing Rust bindings for zenoh-pico exist. We must create our own.

##### 2.2.1 Create `crates/zenoh-pico-sys/` Crate
- [ ] Set up crate structure with `build.rs`
- [ ] Add `bindgen` as build dependency
- [ ] Configure `no_std` compatible bindings generation
- [ ] Add `cty` crate for C type compatibility
- [ ] Vendor zenoh-pico as git submodule or use system library

**Crate structure:**
```
crates/zenoh-pico-sys/
├── Cargo.toml
├── build.rs           # bindgen configuration
├── src/
│   └── lib.rs         # Re-export generated bindings
└── wrapper.h          # Include zenoh-pico headers
```

**Cargo.toml:**
```toml
[package]
name = "zenoh-pico-sys"
version = "0.1.0"
edition = "2024"
links = "zenohpico"

[lib]
name = "zenoh_pico_sys"

[dependencies]
cty = "0.2"            # C types for no_std

[build-dependencies]
bindgen = "0.71"
cc = "1.0"             # Compile zenoh-pico from source

[features]
default = []
std = []
```

##### 2.2.2 Generate FFI Bindings
- [ ] Create `wrapper.h` with zenoh-pico includes
- [ ] Configure bindgen for `no_std` (`use_core`, `ctypes_prefix`)
- [ ] Allowlist only required functions/types
- [ ] Handle platform-specific definitions

**Key zenoh-pico functions to wrap:**
```c
// Session management
int8_t z_open(z_owned_session_t *zs, z_moved_config_t *config);
void z_close(z_moved_session_t *zs);

// Publisher
int8_t z_declare_publisher(z_owned_publisher_t *pub,
                           z_loaned_session_t *zs,
                           z_loaned_keyexpr_t *keyexpr,
                           const z_publisher_options_t *options);
int8_t z_publisher_put(z_loaned_publisher_t *pub,
                       z_moved_bytes_t *payload,
                       z_publisher_put_options_t *options);
void z_undeclare_publisher(z_moved_publisher_t *pub);

// Subscriber
int8_t z_declare_subscriber(z_owned_subscriber_t *sub,
                            z_loaned_session_t *zs,
                            z_loaned_keyexpr_t *keyexpr,
                            z_moved_closure_sample_t *callback,
                            z_subscriber_options_t *options);
void z_undeclare_subscriber(z_moved_subscriber_t *sub);
```

##### 2.2.3 Create Safe Rust Wrapper
- [ ] Create `nano-ros-transport/src/zenoh_pico/` module
- [ ] Wrap unsafe FFI calls in safe abstractions
- [ ] Implement `Drop` for automatic resource cleanup
- [ ] Handle static buffer allocation for `no_std`
- [ ] Implement error conversion (C error codes → Rust Result)

**Safe wrapper example:**
```rust
// In nano-ros-transport/src/zenoh_pico/session.rs
use zenoh_pico_sys as ffi;

pub struct Session {
    inner: ffi::z_owned_session_t,
}

impl Session {
    pub fn open(config: Config) -> Result<Self, Error> {
        let mut session = ffi::z_owned_session_t::default();
        let mut cfg = config.into_ffi();

        // SAFETY: FFI call with valid pointers
        let rc = unsafe { ffi::z_open(&mut session, &mut cfg) };

        if rc == 0 {
            Ok(Self { inner: session })
        } else {
            Err(Error::SessionOpen(rc))
        }
    }
}

impl Drop for Session {
    fn drop(&mut self) {
        // SAFETY: Session was successfully opened
        unsafe { ffi::z_close(&mut self.inner) };
    }
}
```

##### 2.2.4 Platform Considerations
- [ ] Zephyr: Link against Zephyr's zenoh-pico module
- [ ] Linux: Build zenoh-pico from source or use system library
- [ ] QEMU: Same as Linux (emulated)

#### 2.3 Topic Naming (rmw_zenoh compatible)
- [ ] Implement topic key generation matching Pico-ROS format
- [ ] Format: `<domain_id>/<topic>/<type>_/RIHS01_<hash>`
- [ ] Implement liveliness token generation
- [ ] Support RMW attachment metadata

### 3. Node & Pub/Sub Implementation

#### 3.1 Node Structure
- [ ] Create `nano-ros-node` crate (or add to core)
- [ ] Implement `Node` struct with name, namespace, domain_id
- [ ] Implement publisher/subscriber registry
- [ ] Support static allocation (no heap)

```rust
// Target API
pub struct Node<const MAX_PUBS: usize = 8, const MAX_SUBS: usize = 8> {
    name: &'static str,
    namespace: &'static str,
    domain_id: u32,
    publishers: heapless::Vec<PublisherHandle, MAX_PUBS>,
    subscribers: heapless::Vec<SubscriberHandle, MAX_SUBS>,
}
```

#### 3.2 Publisher Implementation
- [ ] Implement `Publisher<T>` for message types
- [ ] Serialize message using nano-ros-serdes
- [ ] Send via transport backend
- [ ] Handle QoS settings (best-effort for v1)

#### 3.3 Subscriber Implementation
- [ ] Implement `Subscriber<T>` for message types
- [ ] Receive from transport backend
- [ ] Deserialize message using nano-ros-serdes
- [ ] Support callback or polling API

### 4. Zephyr Integration

#### 4.1 Zephyr Project Structure
- [ ] Create `examples/zephyr-talker/` application
- [ ] Create `examples/zephyr-listener/` application
- [ ] Add `prj.conf` Kconfig files
- [ ] Add `CMakeLists.txt` for Zephyr build integration

**Directory structure:**
```
examples/
├── zephyr-talker/
│   ├── Cargo.toml
│   ├── src/main.rs
│   ├── prj.conf
│   ├── CMakeLists.txt
│   └── boards/
│       └── qemu_cortex_m3.conf
└── zephyr-listener/
    └── (same structure)
```

#### 4.2 Zephyr Configuration
- [ ] Enable Rust: `CONFIG_RUST=y`
- [ ] Enable networking: `CONFIG_NETWORKING=y`
- [ ] Enable Zenoh-Pico: `CONFIG_ZENOH_PICO=y`
- [ ] Configure static IP for QEMU testing

**Example prj.conf:**
```
# Rust support
CONFIG_RUST=y

# Networking
CONFIG_NETWORKING=y
CONFIG_NET_L2_ETHERNET=y
CONFIG_NET_IPV4=y
CONFIG_NET_TCP=y
CONFIG_NET_UDP=y
CONFIG_NET_SOCKETS=y
CONFIG_NET_CONFIG_SETTINGS=y
CONFIG_NET_CONFIG_MY_IPV4_ADDR="192.168.1.10"
CONFIG_NET_CONFIG_MY_IPV4_NETMASK="255.255.255.0"

# Zenoh
CONFIG_ZENOH_PICO=y
CONFIG_ZENOH_PICO_TRANSPORT_TCP=y

# Memory
CONFIG_HEAP_MEM_POOL_SIZE=32768
CONFIG_MAIN_STACK_SIZE=4096
```

#### 4.3 Zephyr Build Integration
- [ ] Configure west manifest for zephyr-lang-rust module
- [ ] Set up Cargo integration with Zephyr build system
- [ ] Handle cross-compilation for Cortex-M targets

### 5. Multi-Node QEMU Testing

#### 5.1 Network Bridge Setup
- [ ] Create `scripts/setup-qemu-network.sh`
- [ ] Create Linux bridge (`br0`)
- [ ] Create TAP interfaces (`tap0`, `tap1`)
- [ ] Configure IP routing

**Network setup script:**
```bash
#!/bin/bash
# Create bridge
sudo ip link add br0 type bridge
sudo ip addr add 192.168.1.1/24 dev br0
sudo ip link set br0 up

# Create TAP for Zephyr QEMU
sudo ip tuntap add dev tap0 mode tap
sudo ip link set tap0 master br0
sudo ip link set tap0 up

# Create TAP for second node (or use host networking)
sudo ip tuntap add dev tap1 mode tap
sudo ip link set tap1 master br0
sudo ip link set tap1 up
```

#### 5.2 Two-Node Communication Test
- [ ] Node 1: Zephyr in QEMU (Cortex-M3) - Publisher
- [ ] Node 2: Native x86 Linux - Subscriber
- [ ] Communicate via Zenoh over bridge network
- [ ] Verify message roundtrip

**Test scenario:**
```
┌─────────────────┐     ┌─────────────────┐
│  QEMU (ARM)     │     │  Native (x86)   │
│  ┌───────────┐  │     │  ┌───────────┐  │
│  │  Talker   │  │     │  │ Listener  │  │
│  │  nano-ros │  │     │  │ nano-ros  │  │
│  └─────┬─────┘  │     │  └─────┬─────┘  │
│        │        │     │        │        │
│  ┌─────┴─────┐  │     │  ┌─────┴─────┐  │
│  │zenoh-pico │  │     │  │  zenoh    │  │
│  └─────┬─────┘  │     │  └─────┬─────┘  │
└────────┼────────┘     └────────┼────────┘
         │    tap0          eth0 │
         └──────────┬────────────┘
                    │
              ┌─────┴─────┐
              │   br0     │
              │ (bridge)  │
              └───────────┘
```

#### 5.3 Integration Test Suite
- [ ] Create `tests/integration/multi_node.rs`
- [ ] Spawn QEMU process programmatically
- [ ] Send test messages from x86 subscriber
- [ ] Verify reception on QEMU talker
- [ ] Clean up resources on test completion

### 6. Example Applications

#### 6.1 Minimal QEMU Test (`examples/qemu-test/`)
```rust
#![no_std]
#![no_main]

use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use panic_semihosting as _;

#[entry]
fn main() -> ! {
    hprintln!("nano-ros QEMU test starting...");

    // Test serialization
    let mut buf = [0u8; 64];
    let msg = nano_ros_types::std_msgs::Int32 { data: 42 };
    // ... serialize and verify

    hprintln!("All tests passed!");
    qemu_exit::exit_success();
}
```

#### 6.2 Zephyr Talker (`examples/zephyr-talker/`)
```rust
#![no_std]
#![no_main]

use nano_ros::{Node, Publisher};
use nano_ros_types::std_msgs::String as StringMsg;

#[zephyr::main]
async fn main() {
    let node = Node::new("talker", "/nano_ros");
    let publisher = node.create_publisher::<StringMsg>("chatter");

    let mut count = 0u32;
    loop {
        let msg = StringMsg::new(&format!("Hello #{}", count));
        publisher.publish(&msg).unwrap();
        count += 1;
        zephyr::time::sleep(Duration::from_secs(1)).await;
    }
}
```

#### 6.3 Native Listener (`examples/native-listener/`)
```rust
use nano_ros::{Node, Subscriber};
use nano_ros_types::std_msgs::String as StringMsg;

fn main() {
    let node = Node::new("listener", "/nano_ros");
    let mut subscriber = node.create_subscriber::<StringMsg>("chatter");

    loop {
        if let Some(msg) = subscriber.take() {
            println!("Received: {}", msg.data);
        }
        std::thread::sleep(Duration::from_millis(100));
    }
}
```

## Dependencies to Add

### Workspace Cargo.toml
```toml
[workspace.dependencies]
# Embedded runtime
cortex-m = "0.7"
cortex-m-rt = "0.7"
cortex-m-semihosting = "0.5"
panic-semihosting = { version = "0.6", features = ["exit"] }
qemu-exit = "3.0"
defmt = "0.3"
defmt-rtt = "0.4"

# Async (for Embassy integration)
embassy-executor = { version = "0.7", features = ["arch-cortex-m"] }
embassy-time = "0.4"

# FFI / C interop
cty = "0.2"           # C types for no_std FFI
cc = "1.0"            # For building zenoh-pico from source

# Zenoh (native x86 only)
zenoh = { version = "1.0", optional = true }
```

### New Crate: zenoh-pico-sys (FFI Bindings)
```toml
[package]
name = "zenoh-pico-sys"
version = "0.1.0"
edition = "2024"
links = "zenohpico"
description = "Unsafe FFI bindings to zenoh-pico C library"

[lib]
name = "zenoh_pico_sys"

[dependencies]
cty = "0.2"

[build-dependencies]
bindgen = "0.71"
cc = "1.0"

[features]
default = []
std = []
```

### New Crate: nano-ros-node
```toml
[package]
name = "nano-ros-node"
version = "0.1.0"
edition = "2024"

[dependencies]
nano-ros-core = { path = "../nano-ros-core", default-features = false }
nano-ros-serdes = { path = "../nano-ros-serdes", default-features = false }
nano-ros-transport = { path = "../nano-ros-transport", default-features = false }
heapless = "0.8"

[features]
default = ["std"]
std = ["nano-ros-core/std", "nano-ros-serdes/std", "nano-ros-transport/std"]
alloc = ["nano-ros-core/alloc", "nano-ros-serdes/alloc"]
```

## Testing Plan

### Unit Tests
- [ ] Transport trait implementations
- [ ] Topic name generation
- [ ] Publisher/Subscriber lifecycle
- [ ] Message serialization through full stack

### QEMU Tests
- [ ] Basic Cortex-M3 boot and exit
- [ ] Semihosting output
- [ ] Serialization on embedded target
- [ ] Timer/delay functionality

### Integration Tests
- [ ] Single-node pub/sub (loopback)
- [ ] Two-node QEMU-to-QEMU communication
- [ ] QEMU-to-native communication
- [ ] Message type verification across nodes

### CI Configuration
```yaml
# .github/workflows/qemu.yml
name: QEMU Tests

on: [push, pull_request]

jobs:
  qemu-test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - name: Install QEMU
        run: sudo apt-get install -y qemu-system-arm
      - name: Install ARM target
        run: rustup target add thumbv7m-none-eabi
      - name: Run QEMU tests
        run: just qemu-test
```

## Justfile Additions

```just
# Install Zephyr SDK and dependencies
setup-zephyr:
    pip install west
    west init -m https://github.com/zephyrproject-rtos/zephyr --mr v4.1.0 ~/zephyrproject
    cd ~/zephyrproject && west update
    west config manifest.project-filter +zephyr-lang-rust

# Build for QEMU Cortex-M3
build-qemu:
    cargo build --release --target thumbv7m-none-eabi -p qemu-test

# Run QEMU test
qemu-test: build-qemu
    qemu-system-arm \
        -cpu cortex-m3 \
        -machine lm3s6965evb \
        -nographic \
        -semihosting-config enable=on,target=native \
        -kernel target/thumbv7m-none-eabi/release/qemu-test

# Setup QEMU network bridge (requires sudo)
setup-qemu-network:
    sudo ./scripts/setup-qemu-network.sh

# Run multi-node test
test-multi-node: setup-qemu-network
    ./scripts/run-multi-node-test.sh

# Build Zephyr example
build-zephyr-talker:
    cd examples/zephyr-talker && west build -b qemu_cortex_m3

# Run Zephyr talker in QEMU
run-zephyr-talker: build-zephyr-talker
    cd examples/zephyr-talker && west build -t run
```

## Acceptance Criteria

1. [ ] QEMU test binary boots and exits with success
2. [ ] nano-ros serialization works on Cortex-M3 target
3. [ ] Zephyr talker example compiles and runs in QEMU
4. [ ] Native listener receives messages from QEMU talker
5. [ ] CI runs QEMU tests on every push
6. [ ] Documentation covers full setup process

## Research Items (To Investigate)

- [ ] Best approach for zenoh-pico FFI (bindgen vs manual)
- [ ] Embassy integration with Zephyr kernel
- [ ] Memory requirements for zenoh-pico on Cortex-M
- [ ] QoS mapping between ROS 2 and Zenoh

## Deferred to Phase 3

- Ferrocene toolchain integration
- Service client/server implementation
- Parameter server
- Actions support
- Safety certification documentation

## References

### Zephyr + Rust
- [Zephyr Rust Documentation](https://docs.zephyrproject.org/latest/develop/languages/rust/index.html)
- [zephyr-lang-rust GitHub](https://github.com/zephyrproject-rtos/zephyr-lang-rust)
- [Zephyr 4.1 Release Notes](https://zephyrproject.org/zephyr-rtos-4-1-is-available/)

### QEMU + Embedded Rust
- [Embedded Rust Book - QEMU](https://docs.rust-embedded.org/book/start/qemu.html)
- [Embedded Rust Book - Semihosting](https://docs.rust-embedded.org/book/start/semihosting.html)
- [QEMU ARM Cortex-M3 - Zephyr](https://docs.zephyrproject.org/latest/boards/qemu/cortex_m3/doc/index.html)

### Zenoh + Embedded
- [zenoh-pico Zephyr Docs](https://docs.zephyrproject.org/latest/develop/manifest/external/zenoh-pico.html)
- [ROS 2 + Zenoh-pico Blog](https://zenoh.io/blog/2021-11-09-ros2-zenoh-pico/)
- [Pico-ROS Implementation](../external/Pico-ROS-software/)

### QEMU Networking
- [QEMU Networking Documentation](https://wiki.qemu.org/Documentation/Networking)
- [QEMU Bridge Networking](https://apiraino.github.io/qemu-bridge-networking/)
