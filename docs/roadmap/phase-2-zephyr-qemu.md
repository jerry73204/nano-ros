# Phase 2: Transport, Interoperability & Zephyr Integration

**Status: IN PROGRESS** (Phase 2A nearly complete, Phase 2B pending)

## Executive Summary

Phase 2 focuses on three main areas:
1. **Transport Layer** - zenoh-pico integration (COMPLETE)
2. **ROS 2 Interoperability** - rmw_zenoh compatibility (MOSTLY COMPLETE)
3. **Zephyr Integration** - Embedded testing (PENDING)

## Current Progress

### Completed ✅

| Component | Status | Tests |
|-----------|--------|-------|
| `zenoh-pico-sys` | FFI bindings with static linking | 1 |
| `zenoh-pico` | Safe Rust wrapper (Config, Session, Publisher, Subscriber, Liveliness) | 38 |
| `nano-ros-transport` | Transport traits + ZenohTransport backend + RMW Attachment | 7 |
| `nano-ros-node` | Node API + ConnectedNode with transport integration | 5 |
| `nano-ros-serdes` | CDR serialization with header | 30+ |
| QEMU test harness | Cortex-M3 semihosting tests | 9 |
| Native examples | talker/listener with zenoh transport | - |

**Total: 92 tests passing**

### Phase 2A: ROS 2 Interoperability ✅ (MOSTLY COMPLETE)

| Task | Status | Notes |
|------|--------|-------|
| RMW Attachment support | ✅ Complete | `put_with_attachment()` in zenoh-pico |
| Node↔Transport integration | ✅ Complete | `ConnectedNode` with publishers/subscribers |
| Liveliness tokens | ✅ Complete | Node, publisher, subscriber tokens |
| Native examples updated | ✅ Complete | talker/listener use real transport |
| Type hash computation | ⚠️ Hardcoded | Future improvement |
| Testing with ROS 2 | 🔄 In Progress | Manual testing needed |

See [docs/rmw_zenoh_interop.md](../rmw_zenoh_interop.md) for detailed analysis.

### Pending: Phase 2B - Zephyr Integration

| Task | Status |
|------|--------|
| Zephyr Rust examples | C stubs only |
| QEMU networking | Scripts exist, untested |
| Multi-node testing | Planned |

---

## Phase 2A Details (IMPLEMENTED)

### RMW Attachment Support ✅

rmw_zenoh requires metadata with each published message. Implemented in:
- `crates/zenoh-pico/src/publisher.rs` - `put_with_attachment()` method
- `crates/nano-ros-transport/src/zenoh.rs` - `RmwAttachment` struct

```rust
#[repr(C, packed)]
pub struct RmwAttachment {
    pub sequence_number: i64,  // Incremented per publish
    pub timestamp: i64,        // nanoseconds since epoch
    pub rmw_gid_size: u8,      // Always 16
    pub rmw_gid: [u8; 16],     // Random GID per publisher
}
```

### Node↔Transport Integration ✅

Created `ConnectedNode` type that wraps zenoh transport:
- `crates/nano-ros-node/src/connected.rs` - ConnectedNode, ConnectedPublisher, ConnectedSubscriber
- Automatic serialization/deserialization with CDR header
- QoS settings support

```rust
// Example usage
let config = NodeConfig::new("talker", "/demo");
let mut node = ConnectedNode::connect(config, "tcp/127.0.0.1:7447")?;
let publisher = node.create_publisher::<Int32>("/chatter")?;
publisher.publish(&Int32 { data: 42 })?;
```

### Liveliness Token Support ✅

ROS 2 discovery via liveliness tokens:
- `crates/zenoh-pico/src/liveliness.rs` - LivelinessToken, ZenohId
- `crates/nano-ros-transport/src/zenoh.rs` - Ros2Liveliness key expression builder

Token formats:
```
# Node
@ros2_lv/<domain>/<zid>/0/0/NN/%%/%%/<node_name>

# Publisher
@ros2_lv/<domain>/<zid>/0/11/MP/%%/%%/<node_name>/%<topic>/<type>_/RIHS01_<hash>/:,:,:,,

# Subscriber
@ros2_lv/<domain>/<zid>/0/11/MS/%%/%%/<node_name>/%<topic>/<type>_/RIHS01_<hash>/:,:,:,,
```

### Updated Examples ✅

Both native examples now support real zenoh transport:

**With zenoh (real transport):**
```bash
# Start zenoh router
zenohd --listen tcp/127.0.0.1:7447

# Run talker
cargo run -p native-talker --features zenoh

# Run listener
cargo run -p native-listener --features zenoh
```

**Without zenoh (simulation mode):**
```bash
cargo run -p native-talker
cargo run -p native-listener
```

---

## Phase 2A Testing (IN PROGRESS)

### Test with ROS 2 rmw_zenoh

```bash
# Terminal 1: Zenoh router
zenohd --listen tcp/127.0.0.1:7447

# Terminal 2: ROS 2 listener (requires ROS 2 with rmw_zenoh)
RMW_IMPLEMENTATION=rmw_zenoh_cpp ros2 run demo_nodes_cpp listener

# Terminal 3: nano-ros talker
cargo run -p native-talker --features zenoh
```

### Test nano-ros to nano-ros

```bash
# Terminal 1: Zenoh router
zenohd --listen tcp/127.0.0.1:7447

# Terminal 2: Listener
cargo run -p native-listener --features zenoh

# Terminal 3: Talker
cargo run -p native-talker --features zenoh
```

### Acceptance Criteria

1. [x] RMW attachment included with published messages
2. [x] Liveliness tokens declared for nodes/publishers/subscribers
3. [x] Native examples use real transport
4. [ ] nano-ros talker → ROS 2 listener works
5. [ ] ROS 2 talker → nano-ros listener works
6. [ ] `ros2 topic list` shows nano-ros topics

---

## Phase 2B: Zephyr & QEMU Integration (PENDING)

### Goal
Run nano-ros on Zephyr RTOS in QEMU, communicate with native x86 node.

### Architecture
```
┌─────────────────┐     ┌─────────────────┐
│  QEMU (ARM)     │     │  Native (x86)   │
│  ┌───────────┐  │     │  ┌───────────┐  │
│  │  Talker   │  │     │  │ Listener  │  │
│  │  nano-ros │  │     │  │ nano-ros  │  │
│  └─────┬─────┘  │     │  └─────┬─────┘  │
│        │        │     │        │        │
│  ┌─────┴─────┐  │     │  ┌─────┴─────┐  │
│  │zenoh-pico │  │     │  │zenoh-pico │  │
│  └─────┬─────┘  │     │  └─────┴─────┘  │
└────────┼────────┘     └────────┼────────┘
         │    tap0          eth0 │
         └──────────┬────────────┘
                    │
              ┌─────┴─────┐
              │   br0     │
              │ (bridge)  │
              └───────────┘
```

### Work Items

#### 2B.1 Zephyr Rust Examples
- [ ] Convert C stubs to Rust
- [ ] Integrate with zephyr-lang-rust
- [ ] Build with west

#### 2B.2 QEMU Networking
- [ ] Test network bridge setup
- [ ] Verify TAP interface configuration
- [ ] Test cross-QEMU communication

#### 2B.3 Multi-Node Testing
- [ ] Create automated test script
- [ ] Add CI pipeline for QEMU tests
- [ ] Document setup process

---

## Quick Reference

### Build Commands
```bash
just build          # Build all crates (no_std)
just build-zenoh    # Build with zenoh feature
just test           # Run all tests
just quality        # Format + clippy + tests
```

### Run Examples
```bash
# Simulation mode (no router needed)
cargo run -p native-talker
cargo run -p native-listener

# Real transport (requires zenohd)
zenohd --listen tcp/127.0.0.1:7447
cargo run -p native-talker --features zenoh
cargo run -p native-listener --features zenoh
```

---

## References

- [ROS 2 rmw_zenoh Interop Analysis](../rmw_zenoh_interop.md)
- [Original Pico-ROS](../../external/Pico-ROS-software/)
- [zenoh-pico GitHub](https://github.com/eclipse-zenoh/zenoh-pico)
- [rmw_zenoh](https://github.com/ros2/rmw_zenoh)
- [Zephyr Rust](https://github.com/zephyrproject-rtos/zephyr-lang-rust)
