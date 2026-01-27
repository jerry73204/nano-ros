# Phase 3: Services, Parameters, Message Generation & Hardware Validation

**Status: IN PROGRESS**

## Executive Summary

Phase 3 extends nano-ros with ROS 2 service support (request/response pattern),
parameter server functionality, a unified API crate, and integration with the
ROS 2 message generation toolchain via colcon-cargo-ros2.

**Goals:**
1. ~~Implement ROS 2 service client/server~~ Infrastructure complete
2. ~~Implement ROS 2 parameter server~~ Core complete
3. ~~Create unified `nano-ros` crate~~ Complete
4. Integrate with colcon-cargo-ros2 for message generation
5. Validate on real embedded hardware (STM32, nRF52840, etc.)

---

## 3.1 ROS 2 Services

### Overview

Services provide synchronous request/response communication, unlike the
asynchronous pub/sub pattern. A service client sends a request and waits
for a response from a service server.

**zenoh Implementation:**
- Service servers use zenoh **queryables** (respond to queries)
- Service clients use zenoh **get** (send query, await reply)

### Architecture

```
┌─────────────────┐         ┌─────────────────┐
│  Service Client │         │  Service Server │
│                 │         │                 │
│  send_request() │────────▶│  callback()     │
│       │         │  query  │       │         │
│       ▼         │         │       ▼         │
│  await_response │◀────────│  send_reply()   │
│                 │  reply  │                 │
└─────────────────┘         └─────────────────┘
```

### Work Items

#### 3.1.1 Service Traits and Types - COMPLETE
- [x] Create `crates/nano-ros-core/src/service.rs`
- [x] Define `RosService` trait with Request/Response associated types
- [x] Define `ServiceServer` and `ServiceClient` traits
- [x] Add service type definitions

```rust
/// Trait for ROS 2 service types
pub trait RosService: Sized {
    type Request: RosMessage;
    type Response: RosMessage;

    const SERVICE_NAME: &'static str;
    const SERVICE_TYPE: &'static str;
}
```

#### 3.1.2 Service Proc Macro
- [ ] Add `#[derive(RosService)]` macro to `nano-ros-macros`
- [ ] Generate Request/Response message types from service definition
- [ ] Generate service type name and hash

#### 3.1.3 zenoh-pico Service Support - COMPLETE
- [x] Add queryable support to `zenoh-pico` crate
- [x] Implement `Session::declare_queryable()`
- [x] Implement `Session::get()` for queries
- [ ] Add C shim functions for Zephyr (`zenoh_declare_queryable`, `zenoh_query`)

#### 3.1.4 Transport Layer Service Support - COMPLETE
- [x] Add `ServiceInfo` type to transport layer
- [x] Add service traits to transport abstraction
- [x] Implement for `ZenohTransport`

#### 3.1.5 Node Service API - COMPLETE
- [x] Add `ConnectedNode::create_service()` method
- [x] Add `ConnectedNode::create_client()` method
- [x] Service key expression format support

#### 3.1.6 Service Examples
- [ ] Create `examples/native-rs-service-server/` - AddTwoInts server
- [ ] Create `examples/native-rs-service-client/` - AddTwoInts client
- [ ] Create `examples/zephyr-service-rs/` - Zephyr service example
- [ ] Test interop with ROS 2 `ros2 service call`

#### 3.1.7 Service Integration Tests
- [ ] nano-ros server ↔ nano-ros client test
- [ ] nano-ros server ↔ ROS 2 client test
- [ ] ROS 2 server ↔ nano-ros client test

### Key Expression Format

Service key expressions follow rmw_zenoh format:
```
<domain_id>/<service_name>/<service_type>/RIHS01_<hash>
```

---

## 3.2 ROS 2 Parameters

### Overview

ROS 2 parameters provide runtime configuration for nodes. The parameter
server responds to standard ROS 2 parameter service calls.

**Standard Parameter Services:**
- `~/get_parameters` - Get parameter values
- `~/set_parameters` - Set parameter values
- `~/list_parameters` - List available parameters
- `~/describe_parameters` - Get parameter descriptors
- `~/get_parameter_types` - Get parameter types

### Work Items

#### 3.2.1 Parameter Types - COMPLETE
- [x] Implement `ParameterValue` enum with all ROS 2 types
- [x] Implement `ParameterType` enum matching rcl_interfaces
- [x] Implement `ParameterDescriptor` with constraints
- [x] Implement range validation (IntegerRange, FloatingPointRange)
- [x] `no_std` compatible using heapless types

```rust
pub enum ParameterValue {
    NotSet,
    Bool(bool),
    Integer(i64),
    Double(f64),
    String(heapless::String<256>),
    ByteArray(heapless::Vec<u8, 256>),
    BoolArray(heapless::Vec<bool, 32>),
    IntegerArray(heapless::Vec<i64, 32>),
    DoubleArray(heapless::Vec<f64, 32>),
    StringArray(heapless::Vec<heapless::String<256>, 32>),
}
```

#### 3.2.2 Parameter Server - COMPLETE
- [x] Implement `ParameterServer` with static storage (MAX_PARAMETERS=32)
- [x] Implement `declare()`, `get()`, `set()`, `remove()` methods
- [x] Implement constraint validation (read-only, type checking, ranges)
- [x] Implement `ParameterBuilder` fluent API
- [x] Implement typed accessors (`get_bool()`, `get_integer()`, etc.)

```rust
pub struct ParameterServer {
    entries: [Option<ParameterEntry>; MAX_PARAMETERS],
    count: usize,
}

impl ParameterServer {
    pub fn declare(&mut self, name: &str, value: ParameterValue) -> bool;
    pub fn get(&self, name: &str) -> Option<&ParameterValue>;
    pub fn set(&mut self, name: &str, value: ParameterValue) -> SetParameterResult;
}
```

#### 3.2.3 Parameter Service Messages
- [ ] Generate `rcl_interfaces` message types via colcon-cargo-ros2
  - `rcl_interfaces/msg/Parameter`
  - `rcl_interfaces/msg/ParameterValue`
  - `rcl_interfaces/msg/ParameterDescriptor`
  - `rcl_interfaces/msg/SetParametersResult`
- [ ] Generate `rcl_interfaces` service types
  - `rcl_interfaces/srv/GetParameters`
  - `rcl_interfaces/srv/SetParameters`
  - `rcl_interfaces/srv/ListParameters`
  - `rcl_interfaces/srv/DescribeParameters`

#### 3.2.4 Parameter Service Handlers
- [ ] Implement `~/get_parameters` service handler
- [ ] Implement `~/set_parameters` service handler
- [ ] Implement `~/list_parameters` service handler
- [ ] Implement `~/describe_parameters` service handler
- [ ] Handle parameter events topic (`/parameter_events`)

#### 3.2.5 Node Parameter Integration
- [ ] Add `Node::declare_parameter()` method
- [ ] Add `Node::get_parameter()` method
- [ ] Add `Node::set_parameter()` method
- [ ] Add parameter callbacks for change notifications
- [ ] Automatic parameter server startup

#### 3.2.6 Parameter Examples
- [ ] Create `examples/native-params/` - Parameter server demo
- [ ] Test with `ros2 param list`, `ros2 param get`, `ros2 param set`

---

## 3.3 Unified `nano-ros` Crate - COMPLETE

### Overview

A single `nano-ros` crate serves as the main entry point for users,
similar to how `rclcpp` is the unified C++ API for ROS 2.

### Status: COMPLETE

- [x] Create `crates/nano-ros/` with Cargo.toml
- [x] Re-export all sub-crate types
- [x] Define feature flags: `std`, `alloc`, `zenoh`
- [x] Create `prelude` module for common imports
- [x] Integrate parameter types
- [x] Integrate service types
- [x] `no_std` compatible

### Usage

```rust
use nano_ros::prelude::*;
use nano_ros::types::std_msgs::Int32;

let config = NodeConfig::new("my_node");
let mut node = ConnectedNode::connect(config, "tcp/127.0.0.1:7447")?;

let publisher = node.create_publisher::<Int32>("/chatter")?;
publisher.publish(&Int32 { data: 42 })?;
```

---

## 3.4 Hardware Validation

### Overview

Validate nano-ros on real embedded hardware beyond native_sim emulation.

### Target Boards

| Board | MCU | Network | Priority | Status |
|-------|-----|---------|----------|--------|
| NUCLEO-F429ZI | STM32F429 | Ethernet | High | Pending |
| nRF52840-DK | nRF52840 | BLE/Thread | Medium | Pending |
| ESP32-DevKitC | ESP32 | WiFi | Medium | Pending |

### Work Items

#### 3.4.1 Board Support
- [ ] Configure Zephyr for NUCLEO-F429ZI
- [ ] Test Ethernet connectivity to zenoh router
- [ ] Measure memory usage (RAM/Flash)
- [ ] Profile CPU usage during pub/sub

#### 3.4.2 Performance Benchmarks
- [ ] Message latency (round-trip time)
- [ ] Maximum message rate (msgs/sec)
- [ ] Memory footprint per publisher/subscriber
- [ ] Zenoh session overhead

#### 3.4.3 Reliability Testing
- [ ] Network disconnect/reconnect handling
- [ ] Message loss under load
- [ ] Long-running stability tests (24h+)

---

## 3.5 Implementation Order

Recommended implementation sequence:

```
COMPLETED:
├── Service Traits & Types (3.1.1)
├── zenoh-pico Queryable Support (3.1.3)
├── Transport Layer Services (3.1.4)
├── Node Service API (3.1.5)
├── Parameter Types (3.2.1)
├── Parameter Server (3.2.2)
└── Unified nano-ros Crate (3.3)

NEXT (Phase 3):
├── 1. Service Proc Macro (3.1.2)
│      Optional: #[derive(RosService)]
│
├── 2. Service Examples & Tests (3.1.6, 3.1.7)
│      Basic service examples with hand-written types
│
├── 3. Update Examples (pending)
│      Use unified nano-ros API
│
└── 4. Hardware Validation (3.4)
       Test on real embedded boards

PHASE 4 (Message Generation):
├── Fork colcon-cargo-ros2
├── Create nano-ros templates
├── Generate rcl_interfaces types
└── Complete parameter services (requires rcl_interfaces)
```

**Note:** Full parameter service integration (3.2.4, 3.2.5) requires `rcl_interfaces`
message types from Phase 4. Basic parameter server (3.2.1, 3.2.2) is complete.

---

## 3.6 Acceptance Criteria

### Services Complete When:
- [ ] nano-ros service server responds to ROS 2 `ros2 service call`
- [ ] nano-ros service client can call ROS 2 service servers
- [ ] Works on both native and Zephyr targets

### Parameters Complete When:
- [ ] `ros2 param list <node>` shows nano-ros parameters
- [ ] `ros2 param get/set` works with nano-ros nodes
- [ ] Parameter changes trigger callbacks

### Hardware Validation Complete When:
- [ ] At least one real board runs nano-ros
- [ ] Can communicate with ROS 2 nodes on host
- [ ] Performance metrics documented

---

---

## Related Documents

- [Phase 4: Message Generation](phase-4-message-generation.md) - colcon-cargo-ros2 integration

---

## References

- [Original Pico-ROS](external/Pico-ROS-software/) - Reference implementation
- [ROS 2 Services](https://docs.ros.org/en/humble/Concepts/Services.html)
- [ROS 2 Parameters](https://docs.ros.org/en/humble/Concepts/Parameters.html)
- [rcl_interfaces](https://github.com/ros2/rcl_interfaces) - Parameter message definitions
