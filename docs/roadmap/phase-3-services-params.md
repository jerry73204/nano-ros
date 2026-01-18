# Phase 3: Services, Parameters, Unified API & Hardware Validation

**Status: PLANNING**

## Executive Summary

Phase 3 extends nano-ros with ROS 2 service support (request/response pattern),
parameter server functionality, a unified API crate, and hardware validation.

**Goals:**
1. Implement ROS 2 service client/server
2. Implement ROS 2 parameter server
3. Create unified `nano-ros` crate (like rclcpp/rclpy)
4. Validate on real embedded hardware (STM32, nRF52840, etc.)

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

#### 3.1.1 Service Traits and Types
- [ ] Create `crates/nano-ros-core/src/service.rs`
- [ ] Define `RosService` trait with Request/Response associated types
- [ ] Define `ServiceServer<S>` struct
- [ ] Define `ServiceClient<S>` struct
- [ ] Add service type hash computation

```rust
/// Trait for ROS 2 service types
pub trait RosService: Sized {
    type Request: RosMessage;
    type Response: RosMessage;

    const SERVICE_NAME: &'static str;
    const SERVICE_TYPE: &'static str;
    const RIHS_HASH: &'static str;
}

/// Service server
pub struct ServiceServer<S: RosService, F> {
    callback: F,
    // ...
}

/// Service client
pub struct ServiceClient<S: RosService> {
    // ...
}
```

#### 3.1.2 Service Proc Macro
- [ ] Add `#[derive(RosService)]` macro to `nano-ros-macros`
- [ ] Generate Request/Response message types from service definition
- [ ] Generate service type name and hash

```rust
#[derive(RosService)]
#[ros(package = "example_interfaces", name = "AddTwoInts")]
pub struct AddTwoInts;
// Generates: AddTwoIntsRequest, AddTwoIntsResponse
```

#### 3.1.3 zenoh-pico Service Support
- [ ] Add queryable support to `zenoh-pico` crate
- [ ] Implement `Session::declare_queryable()`
- [ ] Implement `Session::get()` for queries
- [ ] Add C shim functions for Zephyr (`zenoh_declare_queryable`, `zenoh_query`)

#### 3.1.4 Transport Layer Service Support
- [ ] Add `create_service_server()` to `Transport` trait
- [ ] Add `create_service_client()` to `Transport` trait
- [ ] Implement for `ZenohTransport`
- [ ] Handle RMW attachment in service requests/replies

#### 3.1.5 Node Service API
- [ ] Add `Node::create_service()` method
- [ ] Add `Node::create_client()` method
- [ ] Add `ConnectedNode` service support
- [ ] Handle service key expression format: `<domain>/<service>/<type>/RIHS01_<hash>`

#### 3.1.6 Service Examples
- [ ] Create `examples/native-service-server/` - AddTwoInts server
- [ ] Create `examples/native-service-client/` - AddTwoInts client
- [ ] Create `examples/zephyr-service-rs/` - Zephyr service example
- [ ] Test interop with ROS 2 `ros2 service call`

#### 3.1.7 Service Integration Tests
- [ ] nano-ros server ↔ nano-ros client test
- [ ] nano-ros server ↔ ROS 2 client test
- [ ] ROS 2 server ↔ nano-ros client test
- [ ] Add to `tests/run-all.sh`

### Key Expression Format

Service key expressions follow rmw_zenoh format:
```
<domain_id>/<service_name>/<service_type>/RIHS01_<hash>
```

Example:
```
0/add_two_ints/example_interfaces::srv::dds_::AddTwoInts_/RIHS01_abc123...
```

### Reference Implementation

See original Pico-ROS: `external/Pico-ROS-software/src/picoros.h`
- `picoros_srv_server_t` - Service server using `z_owned_queryable_t`
- `picoros_srv_client_t` - Service client using zenoh get

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

### Architecture

```
┌─────────────────────────────────────────────────┐
│                 nano-ros Node                    │
│  ┌───────────────────────────────────────────┐  │
│  │           ParameterServer                  │  │
│  │  ┌─────────────┐  ┌─────────────────────┐ │  │
│  │  │ Parameters  │  │ Service Endpoints   │ │  │
│  │  │  HashMap    │  │  ~/get_parameters   │ │  │
│  │  │             │  │  ~/set_parameters   │ │  │
│  │  │ "rate": 10  │  │  ~/list_parameters  │ │  │
│  │  │ "name": ... │  │  ~/describe_params  │ │  │
│  │  └─────────────┘  └─────────────────────┘ │  │
│  └───────────────────────────────────────────┘  │
└─────────────────────────────────────────────────┘
```

### Work Items

#### 3.2.1 Parameter Types (Complete Existing Stubs)
- [ ] Complete `ParameterValue` with actual value storage
- [ ] Add `ParameterDescriptor` struct
- [ ] Add parameter validation (ranges, read-only, etc.)
- [ ] Implement `no_std` compatible storage (heapless::FnvIndexMap)

```rust
pub struct ParameterValue {
    pub param_type: ParameterType,
    pub value: ParameterData,
}

pub enum ParameterData {
    NotSet,
    Bool(bool),
    Integer(i64),
    Double(f64),
    String(heapless::String<256>),
    ByteArray(heapless::Vec<u8, 1024>),
    // ... arrays
}

pub struct ParameterDescriptor {
    pub name: heapless::String<64>,
    pub param_type: ParameterType,
    pub description: heapless::String<256>,
    pub read_only: bool,
    pub integer_range: Option<IntegerRange>,
    pub floating_point_range: Option<FloatingPointRange>,
}
```

#### 3.2.2 Parameter Storage
- [ ] Implement `ParameterStore` trait for storage backends
- [ ] Implement `StaticParameterStore<N>` using heapless::FnvIndexMap
- [ ] Optional: `DynamicParameterStore` using alloc::HashMap
- [ ] Parameter change callbacks

```rust
pub trait ParameterStore {
    fn get(&self, name: &str) -> Option<&ParameterValue>;
    fn set(&mut self, name: &str, value: ParameterValue) -> Result<(), Error>;
    fn list(&self, prefix: &str) -> impl Iterator<Item = &str>;
    fn describe(&self, name: &str) -> Option<&ParameterDescriptor>;
}
```

#### 3.2.3 Parameter Service Messages
- [ ] Add `rcl_interfaces` message types to `nano-ros-types`
  - `rcl_interfaces/msg/Parameter`
  - `rcl_interfaces/msg/ParameterValue`
  - `rcl_interfaces/msg/ParameterDescriptor`
  - `rcl_interfaces/msg/SetParametersResult`
- [ ] Add `rcl_interfaces` service types
  - `rcl_interfaces/srv/GetParameters`
  - `rcl_interfaces/srv/SetParameters`
  - `rcl_interfaces/srv/ListParameters`
  - `rcl_interfaces/srv/DescribeParameters`

#### 3.2.4 Parameter Server Implementation
- [ ] Complete `ParameterServer` in `nano-ros-params`
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

```rust
impl Node {
    pub fn declare_parameter<T: Into<ParameterValue>>(
        &mut self,
        name: &str,
        default: T,
        descriptor: ParameterDescriptor,
    ) -> Result<(), Error>;

    pub fn get_parameter<T: TryFrom<ParameterValue>>(
        &self,
        name: &str,
    ) -> Result<T, Error>;
}
```

#### 3.2.6 Parameter Examples
- [ ] Create `examples/native-params/` - Parameter server demo
- [ ] Create `examples/zephyr-params-rs/` - Zephyr with parameters
- [ ] Test with `ros2 param list`, `ros2 param get`, `ros2 param set`

#### 3.2.7 Parameter Integration Tests
- [ ] Parameter get/set from ROS 2 CLI
- [ ] Parameter persistence (optional)
- [ ] Add to `tests/run-all.sh`

### Reference Implementation

See original Pico-ROS: `external/Pico-ROS-software/src/picoparams.h`
- `pp_ParameterValue` - Parameter value container
- `pp_ParameterType` - Parameter type enum
- Uses services for parameter operations

---

## 3.3 Hardware Validation

### Overview

Validate nano-ros on real embedded hardware beyond native_sim emulation.

### Target Boards

| Board | MCU | Network | Priority | Status |
|-------|-----|---------|----------|--------|
| NUCLEO-F429ZI | STM32F429 | Ethernet | High | Pending |
| nRF52840-DK | nRF52840 | BLE/Thread | Medium | Pending |
| ESP32-DevKitC | ESP32 | WiFi | Medium | Pending |
| STM32H7 Nucleo | STM32H743 | Ethernet | Low | Pending |

### Work Items

#### 3.3.1 Board Support
- [ ] Configure Zephyr for NUCLEO-F429ZI
- [ ] Test Ethernet connectivity to zenoh router
- [ ] Measure memory usage (RAM/Flash)
- [ ] Profile CPU usage during pub/sub

#### 3.3.2 Performance Benchmarks
- [ ] Message latency (round-trip time)
- [ ] Maximum message rate (msgs/sec)
- [ ] Memory footprint per publisher/subscriber
- [ ] Zenoh session overhead

#### 3.3.3 Reliability Testing
- [ ] Network disconnect/reconnect handling
- [ ] Message loss under load
- [ ] Long-running stability tests (24h+)

---

## 3.4 Unified `nano-ros` Crate

### Overview

Create a single `nano-ros` crate that serves as the main entry point for users,
similar to how `rclcpp` is the unified C++ API and `rclpy` is the unified Python
API for ROS 2. This crate re-exports all necessary types and provides a clean,
ergonomic API for building ROS 2 nodes.

### Current vs Target Architecture

**Current (fragmented):**
```rust
use nano_ros_core::{Node, NodeConfig, RosMessage};
use nano_ros_node::ConnectedNode;
use nano_ros_serdes::{CdrReader, CdrWriter, Serialize, Deserialize};
use nano_ros_types::std_msgs::Int32;
use nano_ros_transport::ZenohTransport;
use nano_ros_params::ParameterServer;
```

**Target (unified):**
```rust
use nano_ros::prelude::*;
// or selectively:
use nano_ros::{Node, Publisher, Subscriber, Service, Client, Parameter};
use nano_ros::msg::std_msgs::Int32;
use nano_ros::srv::example_interfaces::AddTwoInts;
```

### Work Items

#### 3.4.1 Create `nano-ros` Crate
- [ ] Create `crates/nano-ros/Cargo.toml`
- [ ] Re-export core types from sub-crates
- [ ] Define feature flags for optional components
- [ ] Ensure `no_std` compatibility

```toml
[package]
name = "nano-ros"
version = "0.1.0"
edition = "2021"

[features]
default = ["std"]
std = ["nano-ros-core/std", "nano-ros-node/std", ...]
alloc = ["nano-ros-core/alloc", ...]
zenoh = ["nano-ros-transport/zenoh", "nano-ros-node/zenoh"]
params = ["nano-ros-params"]
# Target-specific
zephyr = []

[dependencies]
nano-ros-core = { path = "../nano-ros-core" }
nano-ros-serdes = { path = "../nano-ros-serdes" }
nano-ros-types = { path = "../nano-ros-types" }
nano-ros-macros = { path = "../nano-ros-macros" }
nano-ros-node = { path = "../nano-ros-node", optional = true }
nano-ros-transport = { path = "../nano-ros-transport", optional = true }
nano-ros-params = { path = "../nano-ros-params", optional = true }
```

#### 3.4.2 Define Public API Structure
- [ ] Create `src/lib.rs` with module organization
- [ ] Create `src/prelude.rs` with common imports
- [ ] Create `src/msg.rs` re-exporting message types
- [ ] Create `src/srv.rs` re-exporting service types

```rust
// crates/nano-ros/src/lib.rs
#![no_std]

#[cfg(feature = "std")]
extern crate std;

#[cfg(feature = "alloc")]
extern crate alloc;

// Re-export core types
pub use nano_ros_core::{
    Node, NodeConfig, Publisher, Subscriber,
    RosMessage, QoS, Error,
};

// Re-export serialization (usually not needed by users)
pub mod serdes {
    pub use nano_ros_serdes::*;
}

// Re-export message types
pub mod msg {
    pub use nano_ros_types::*;
}

// Re-export service types (when implemented)
#[cfg(feature = "services")]
pub mod srv {
    // pub use nano_ros_types::srv::*;
}

// Re-export connected node (requires transport)
#[cfg(feature = "zenoh")]
pub use nano_ros_node::ConnectedNode;

// Re-export parameters
#[cfg(feature = "params")]
pub mod params {
    pub use nano_ros_params::*;
}

// Re-export derive macros
pub use nano_ros_macros::{RosMessage, RosService};

// Prelude for convenient imports
pub mod prelude {
    pub use crate::{Node, NodeConfig, Publisher, Subscriber, RosMessage};
    pub use crate::msg::std_msgs::{Int32, String as RosString, Bool, Float32, Float64};

    #[cfg(feature = "zenoh")]
    pub use crate::ConnectedNode;

    #[cfg(feature = "services")]
    pub use crate::{ServiceServer, ServiceClient, RosService};

    #[cfg(feature = "params")]
    pub use crate::params::{Parameter, ParameterValue};
}
```

#### 3.4.3 Simplified Node Creation API
- [ ] Add builder pattern for node creation
- [ ] Add convenience methods for common operations
- [ ] Provide sensible defaults

```rust
// Target API
use nano_ros::prelude::*;

fn main() -> Result<(), nano_ros::Error> {
    // Simple node creation
    let node = Node::builder("talker")
        .namespace("/demo")
        .domain_id(0)
        .build()?;

    // With zenoh transport
    let node = Node::builder("talker")
        .namespace("/demo")
        .connect("tcp/127.0.0.1:7447")?;

    // Create publisher with type inference
    let publisher = node.advertise::<Int32>("/chatter")?;

    // Create subscriber with closure
    let _subscription = node.subscribe("/chatter", |msg: Int32| {
        println!("Received: {}", msg.data);
    })?;

    // Spin (process callbacks)
    node.spin()?;

    Ok(())
}
```

#### 3.4.4 Error Handling Unification
- [ ] Create unified `nano_ros::Error` type
- [ ] Implement `From` for all sub-crate errors
- [ ] Add `Result` type alias

```rust
// crates/nano-ros/src/error.rs
#[derive(Debug)]
pub enum Error {
    Core(nano_ros_core::Error),
    Serialization(nano_ros_serdes::Error),
    Transport(nano_ros_transport::Error),
    #[cfg(feature = "params")]
    Parameter(nano_ros_params::Error),
    Config(ConfigError),
}

pub type Result<T> = core::result::Result<T, Error>;
```

#### 3.4.5 Documentation and Examples
- [ ] Write crate-level documentation with examples
- [ ] Create `examples/` using unified API
- [ ] Update existing examples to use `nano-ros` crate
- [ ] Add doc tests for all public APIs

#### 3.4.6 Update Zephyr Examples
- [ ] Update `zephyr-talker-rs` to use `nano-ros` crate
- [ ] Update `zephyr-listener-rs` to use `nano-ros` crate
- [ ] Verify `no_std` compatibility

#### 3.4.7 Deprecation Path
- [ ] Add deprecation notices to direct sub-crate usage (optional)
- [ ] Document migration guide from sub-crates to unified crate

### Feature Matrix

| Feature | Description | Default |
|---------|-------------|---------|
| `std` | Standard library support | Yes |
| `alloc` | Dynamic allocation (no_std + heap) | No |
| `zenoh` | Zenoh transport + ConnectedNode | Yes |
| `params` | Parameter server support | No |
| `services` | Service client/server | No |
| `derive` | Proc macro derives | Yes |

### Comparison with ROS 2 APIs

| ROS 2 (rclcpp) | nano-ros |
|----------------|----------|
| `rclcpp::Node` | `nano_ros::Node` |
| `rclcpp::Publisher<T>` | `nano_ros::Publisher<T>` |
| `rclcpp::Subscription<T>` | `nano_ros::Subscriber<T>` |
| `rclcpp::Service<T>` | `nano_ros::ServiceServer<T>` |
| `rclcpp::Client<T>` | `nano_ros::ServiceClient<T>` |
| `rclcpp::Parameter` | `nano_ros::Parameter` |
| `std_msgs::msg::Int32` | `nano_ros::msg::std_msgs::Int32` |
| `example_interfaces::srv::AddTwoInts` | `nano_ros::srv::example_interfaces::AddTwoInts` |

---

## 3.5 Implementation Order

Recommended implementation sequence:

```
1. Service Traits & Types (3.1.1)
     ↓
2. zenoh-pico Queryable Support (3.1.3)
     ↓
3. Transport Layer Services (3.1.4)
     ↓
4. Node Service API (3.1.5)
     ↓
5. Service Examples & Tests (3.1.6, 3.1.7)
     ↓
6. Service Proc Macro (3.1.2) [optional, can use manual impl]
     ↓
7. Parameter Types (3.2.1, 3.2.2)
     ↓
8. Parameter Messages (3.2.3) [requires services]
     ↓
9. Parameter Server (3.2.4, 3.2.5)
     ↓
10. Parameter Examples & Tests (3.2.6, 3.2.7)
     ↓
11. Unified nano-ros Crate (3.4.1 - 3.4.5)
     ↓
12. Update Examples to Unified API (3.4.5, 3.4.6)
     ↓
13. Hardware Validation (3.3)
```

**Notes:**
- Services must be implemented before parameters (parameter server uses services)
- Unified crate can be started early but completed after services/params
- Hardware validation is independent and can proceed in parallel

---

## 3.6 Acceptance Criteria

### Services Complete When:
- [ ] nano-ros service server responds to ROS 2 `ros2 service call`
- [ ] nano-ros service client can call ROS 2 service servers
- [ ] Works on both native and Zephyr targets
- [ ] Integration tests pass

### Parameters Complete When:
- [ ] `ros2 param list <node>` shows nano-ros parameters
- [ ] `ros2 param get/set` works with nano-ros nodes
- [ ] Parameter changes trigger callbacks
- [ ] Works on both native and Zephyr targets

### Unified Crate Complete When:
- [ ] Single `use nano_ros::prelude::*` provides all common types
- [ ] All examples updated to use unified crate
- [ ] `no_std` builds work with `default-features = false`
- [ ] Feature flags documented and tested
- [ ] API is ergonomic and matches rclcpp/rclpy patterns
- [ ] Crate-level documentation with examples

### Hardware Validation Complete When:
- [ ] At least one real board (NUCLEO-F429ZI) runs nano-ros
- [ ] Can communicate with ROS 2 nodes on host
- [ ] Performance metrics documented

---

## References

- [Original Pico-ROS Services](../external/Pico-ROS-software/src/picoros.h)
- [Original Pico-ROS Parameters](../external/Pico-ROS-software/src/picoparams.h)
- [ROS 2 Services Concept](https://docs.ros.org/en/humble/Concepts/Services.html)
- [ROS 2 Parameters Concept](https://docs.ros.org/en/humble/Concepts/Parameters.html)
- [rcl_interfaces Package](https://github.com/ros2/rcl_interfaces)
- [zenoh Queryables](https://zenoh.io/docs/manual/abstractions/#queryable)
