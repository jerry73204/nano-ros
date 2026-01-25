# Phase 10: C++ Bindings (rclcpp-Compatible API)

**Status: IN PROGRESS**

## Executive Summary

Phase 10 implements C++ bindings for nano-ros, providing an rclcpp-compatible API built on top of the Rust implementation. This enables C++ developers to use nano-ros with familiar ROS 2 patterns while leveraging the safety and efficiency of the Rust core.

**Goals:**
1. Create C++ bindings using the `cxx` crate for safe Rust/C++ interop
2. Use Corrosion for CMake/Cargo build integration
3. Provide an API compatible with rclcpp (ROS Humble 16.0.x)
4. Support both desktop (Linux) and embedded (Zephyr) targets
5. Enable gradual migration from rclcpp to nano-ros

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                     User C++ Application                         │
│                  #include <nano_ros/node.hpp>                    │
└───────────────────────────────┬─────────────────────────────────┘
                                │
┌───────────────────────────────▼─────────────────────────────────┐
│                    nano-ros-cpp (C++ Library)                    │
│  ┌─────────────┐  ┌──────────────┐  ┌────────────────────────┐  │
│  │  node.hpp   │  │ publisher.hpp │  │  subscription.hpp     │  │
│  │  node.cpp   │  │ publisher.cpp │  │  subscription.cpp     │  │
│  └──────┬──────┘  └──────┬───────┘  └───────────┬────────────┘  │
└─────────┼────────────────┼──────────────────────┼───────────────┘
          │                │                      │
┌─────────▼────────────────▼──────────────────────▼───────────────┐
│                 cxx Bridge (Generated FFI Layer)                 │
│                      src/lib.rs - #[cxx::bridge]                 │
└───────────────────────────────┬─────────────────────────────────┘
                                │
┌───────────────────────────────▼─────────────────────────────────┐
│                    nano-ros (Rust Core Library)                  │
│         Context, Node, Publisher, Subscriber, QoS, etc.          │
└─────────────────────────────────────────────────────────────────┘
```

### Build System Integration

```
CMakeLists.txt (orchestrator)
       │
       ├── FetchContent(Corrosion)
       │         │
       │         └── corrosion_import_crate(Cargo.toml)
       │                    │
       │                    ├── cargo build → libnanoros_bridge.a
       │                    └── cxx generates bridge headers
       │
       ├── add_library(nano_ros_cpp)
       │         │
       │         └── cpp/*.cpp + include/*.hpp
       │
       └── target_link_libraries(nano_ros_cpp ← nanoros_bridge)
                 │
                 └── Final: libnanoros_cpp.so / .a
```

---

## Reference Repositories

Cloned to `external/` for API reference (ROS Humble branch):

| Repository | Path | Version | Purpose |
|------------|------|---------|---------|
| rclcpp | `external/rclcpp/` | 16.0.17 | C++ client library reference |
| rcl | `external/rcl/` | 5.3.12 | C client library patterns |
| rcutils | `external/rcutils/` | - | Utility patterns |
| rmw | `external/rmw/` | 6.1.2 | Middleware interface |
| rcl_interfaces | `external/rcl_interfaces/` | - | Standard message types |
| rosidl | `external/rosidl/` | - | Message generation reference |
| ros2_rust | `external/ros2_rust/` | 0.6.0 | Rust bindings reference |

---

## 10.1 Project Setup - COMPLETE

### Work Items

#### 10.1.1 Create Package Structure
- [x] Create `crates/nano-ros-cpp/` directory
- [x] Create `Cargo.toml` with cxx dependency
- [x] Create `CMakeLists.txt` with Corrosion integration
- [x] Create `build.rs` for cxx-build
- [x] Create directory structure:
  ```
  crates/nano-ros-cpp/
  ├── Cargo.toml
  ├── CMakeLists.txt
  ├── build.rs
  ├── src/
  │   └── lib.rs              # cxx bridge definitions
  ├── include/
  │   └── nano_ros/
  │       ├── node.hpp
  │       ├── publisher.hpp
  │       ├── subscription.hpp
  │       ├── context.hpp
  │       └── qos.hpp
  └── cpp/
      ├── node.cpp
      ├── publisher.cpp
      ├── subscription.cpp
      └── context.cpp
  ```

#### 10.1.2 Configure Cargo.toml
- [x] Add dependencies:
  ```toml
  [package]
  name = "nano-ros-cpp-bridge"
  version = "0.1.0"
  edition = "2021"

  [lib]
  crate-type = ["staticlib"]

  [dependencies]
  cxx = "1.0"
  nano-ros = { path = "../nano-ros", features = ["std"] }

  [build-dependencies]
  cxx-build = "1.0"
  ```

#### 10.1.3 Configure CMakeLists.txt
- [x] Set up Corrosion with FetchContent
- [x] Import Rust crate as CMake target
- [x] Configure C++ library compilation
- [x] Set up include directories
- [x] Configure install targets

#### 10.1.4 Verify Build System
- [x] Build compiles without errors
- [x] Rust static library is generated
- [x] C++ library links correctly
- [ ] Install target works

### Acceptance Criteria
- `cmake -B build && cmake --build build` succeeds
- Generated library can be linked by external projects
- Both static and shared library variants work

---

## 10.2 Core Types Bridge

### Work Items

#### 10.2.1 Duration and Time Types
- [ ] Define cxx bridge for `Duration`
- [ ] Define cxx bridge for `Time`
- [ ] Create C++ wrapper classes
- [ ] Match rclcpp::Duration and rclcpp::Time API

```rust
// src/lib.rs
#[cxx::bridge]
mod ffi {
    struct Duration {
        nanoseconds: i64,
    }

    struct Time {
        nanoseconds: i64,
    }

    extern "Rust" {
        fn duration_from_seconds(seconds: f64) -> Duration;
        fn duration_from_nanoseconds(nanos: i64) -> Duration;
    }
}
```

```cpp
// include/nano_ros/duration.hpp
namespace nano_ros {
class Duration {
public:
    Duration(int64_t nanoseconds);
    Duration(int32_t seconds, uint32_t nanoseconds);
    static Duration from_seconds(double seconds);

    int64_t nanoseconds() const;
    double seconds() const;
};
}
```

#### 10.2.2 QoS Types
- [ ] Define cxx bridge for QoS policies
- [ ] Create C++ QoS class with builder pattern
- [ ] Match rclcpp::QoS API

```cpp
// include/nano_ros/qos.hpp
namespace nano_ros {
class QoS {
public:
    explicit QoS(size_t depth);

    QoS& reliability(ReliabilityPolicy policy);
    QoS& durability(DurabilityPolicy policy);
    QoS& history(HistoryPolicy policy);

    // Presets
    static QoS sensor_data();
    static QoS parameters();
    static QoS services();
};
}
```

#### 10.2.3 Error Types
- [ ] Define error enum in cxx bridge
- [ ] Create C++ exception classes
- [ ] Implement Result → exception conversion

### Acceptance Criteria
- Core types can be constructed and used from C++
- API matches rclcpp patterns
- Proper error handling with exceptions

---

## 10.3 Context and Node

### Work Items

#### 10.3.1 Context Bridge
- [ ] Define opaque Rust Context in cxx bridge
- [ ] Expose `Context::new()` and `Context::from_env()`
- [ ] Create C++ Context wrapper

```rust
#[cxx::bridge]
mod ffi {
    extern "Rust" {
        type RustContext;

        fn create_context() -> Result<Box<RustContext>>;
        fn create_context_from_env() -> Result<Box<RustContext>>;
        fn context_ok(ctx: &RustContext) -> bool;
    }
}
```

```cpp
// include/nano_ros/context.hpp
namespace nano_ros {
class Context {
public:
    Context();
    static Context from_env();

    bool ok() const;
    std::shared_ptr<Node> create_node(const std::string& name);
    std::shared_ptr<Node> create_node(const std::string& name,
                                       const std::string& namespace_);

private:
    rust::Box<RustContext> impl_;
};
}
```

#### 10.3.2 Node Bridge
- [ ] Define opaque Rust Node in cxx bridge
- [ ] Expose node creation and accessor methods
- [ ] Create C++ Node wrapper with shared_ptr semantics

```cpp
// include/nano_ros/node.hpp
namespace nano_ros {
class Node : public std::enable_shared_from_this<Node> {
public:
    std::string get_name() const;
    std::string get_namespace() const;
    std::string get_fully_qualified_name() const;

    template<typename MessageT>
    std::shared_ptr<Publisher<MessageT>>
    create_publisher(const std::string& topic, const QoS& qos);

    template<typename MessageT>
    std::shared_ptr<Subscription<MessageT>>
    create_subscription(const std::string& topic, const QoS& qos,
                        std::function<void(const MessageT&)> callback);

private:
    rust::Box<RustNode> impl_;
};
}
```

#### 10.3.3 NodeOptions
- [ ] Implement NodeOptions builder class
- [ ] Support namespace, parameter overrides
- [ ] Match rclcpp::NodeOptions API

### Acceptance Criteria
- Context can be created and used to spawn nodes
- Nodes expose expected accessors
- Matches rclcpp initialization patterns

---

## 10.4 Publisher and Subscription

### Work Items

#### 10.4.1 Publisher Bridge
- [ ] Define generic publish interface in cxx bridge
- [ ] Create templated C++ Publisher class
- [ ] Support serialized message passing

```cpp
// include/nano_ros/publisher.hpp
namespace nano_ros {
template<typename MessageT>
class Publisher {
public:
    void publish(const MessageT& message);
    std::string get_topic_name() const;
    size_t get_subscription_count() const;

private:
    rust::Box<RustPublisher> impl_;
};
}
```

#### 10.4.2 Subscription Bridge
- [ ] Define callback mechanism in cxx bridge
- [ ] Create templated C++ Subscription class
- [ ] Handle message deserialization

```cpp
// include/nano_ros/subscription.hpp
namespace nano_ros {
template<typename MessageT>
class Subscription {
public:
    using CallbackType = std::function<void(const MessageT&)>;

    std::string get_topic_name() const;
    size_t get_publisher_count() const;

private:
    rust::Box<RustSubscription> impl_;
    CallbackType callback_;
};
}
```

#### 10.4.3 Message Serialization Bridge
- [ ] Implement CDR serialization for C++ messages
- [ ] Bridge serialized bytes across FFI boundary
- [ ] Support standard message types (std_msgs, etc.)

### Acceptance Criteria
- Publishers can send messages to Rust subscribers
- C++ subscriptions receive messages from Rust publishers
- Interoperates with ROS 2 nodes via zenoh

---

## 10.5 Message Type Generation

### Work Items

#### 10.5.1 C++ Message Generator
- [ ] Extend `cargo nano-ros generate` for C++ output
- [ ] Generate C++ struct definitions from .msg files
- [ ] Generate serialization code

#### 10.5.2 Standard Message Types
- [ ] Generate std_msgs (Int32, String, Header, etc.)
- [ ] Generate geometry_msgs (Point, Pose, Twist, etc.)
- [ ] Generate sensor_msgs (Image, PointCloud2, etc.)

#### 10.5.3 Message Headers
- [ ] Create include structure matching ROS 2
  ```
  include/
  └── nano_ros/
      └── msg/
          ├── std_msgs/
          │   ├── int32.hpp
          │   └── string.hpp
          └── geometry_msgs/
              └── point.hpp
  ```

### Acceptance Criteria
- Generated messages compile correctly
- Messages can be published/subscribed
- Binary compatible with ROS 2 CDR format

---

## 10.6 Executor Integration

### Work Items

#### 10.6.1 Executor Bridge
- [ ] Expose Rust executor through cxx bridge
- [ ] Create C++ Executor base class
- [ ] Implement spin(), spin_once(), spin_some()

```cpp
// include/nano_ros/executor.hpp
namespace nano_ros {
class Executor {
public:
    virtual ~Executor() = default;

    void add_node(std::shared_ptr<Node> node);
    void remove_node(std::shared_ptr<Node> node);

    void spin();
    void spin_once(Duration timeout = Duration(0));
    void spin_some(Duration max_duration = Duration(0));
    void cancel();
};

class SingleThreadedExecutor : public Executor {
public:
    SingleThreadedExecutor();
};
}
```

#### 10.6.2 Callback Handling
- [ ] Implement callback queue management
- [ ] Bridge C++ callbacks to Rust
- [ ] Handle callback exceptions safely

### Acceptance Criteria
- Executor spins and processes callbacks
- Multiple nodes can be added to executor
- Matches rclcpp executor behavior

---

## 10.7 Service Support

### Work Items

#### 10.7.1 Service Client Bridge
- [ ] Expose Rust service client through cxx
- [ ] Create templated C++ Client class
- [ ] Implement async_send_request()

#### 10.7.2 Service Server Bridge
- [ ] Expose Rust service server through cxx
- [ ] Create templated C++ Service class
- [ ] Handle request callbacks

### Acceptance Criteria
- C++ clients can call Rust/ROS 2 services
- C++ servers respond to Rust/ROS 2 clients
- Matches rclcpp service patterns

---

## 10.8 Examples and Documentation

### Work Items

#### 10.8.1 Basic Examples
- [ ] Create `examples/cpp-talker/` - Publisher example
- [ ] Create `examples/cpp-listener/` - Subscriber example
- [ ] Create `examples/cpp-service/` - Service example

#### 10.8.2 CMake Integration Example
- [ ] Example `find_package(nano_ros_cpp)` usage
- [ ] Example ament_cmake integration
- [ ] Example standalone CMake project

#### 10.8.3 Documentation
- [ ] API reference documentation
- [ ] Migration guide from rclcpp
- [ ] Build integration guide

### Acceptance Criteria
- Examples build and run correctly
- Examples interoperate with ROS 2 nodes
- Documentation is clear and complete

---

## 10.9 Zephyr Support

### Work Items

#### 10.9.1 Zephyr CMake Integration
- [ ] Configure Corrosion for Zephyr cross-compilation
- [ ] Handle no_std constraints in bridge
- [ ] Configure proper linker settings

#### 10.9.2 Zephyr Examples
- [ ] Create `examples/zephyr-cpp-talker/`
- [ ] Create `examples/zephyr-cpp-listener/`
- [ ] Test on native_sim and real hardware

### Acceptance Criteria
- Builds for Zephyr targets
- Runs on native_sim
- Memory footprint documented

---

## Implementation Order

```
Phase 10.1: Project Setup (Foundation)
    └── Package structure, CMake/Cargo integration

Phase 10.2: Core Types Bridge
    └── Duration, Time, QoS, Error types

Phase 10.3: Context and Node
    └── Context, Node, NodeOptions

Phase 10.4: Publisher and Subscription
    └── Pub/Sub with message serialization

Phase 10.5: Message Type Generation
    └── C++ message structs from .msg files

Phase 10.6: Executor Integration
    └── Spin, callbacks, multi-node

Phase 10.7: Service Support
    └── Client and Server

Phase 10.8: Examples and Documentation
    └── Usage examples, docs

Phase 10.9: Zephyr Support
    └── Embedded target integration
```

---

## Dependencies

| Dependency | Version | Purpose |
|------------|---------|---------|
| cxx | 1.0+ | Safe Rust/C++ FFI |
| cxx-build | 1.0+ | Build-time bridge generation |
| Corrosion | 0.5+ | CMake/Cargo integration |
| nano-ros | workspace | Core Rust implementation |

---

## Acceptance Criteria (Phase Complete)

### API Compatibility
- [ ] C++ API matches rclcpp patterns for core functionality
- [ ] Existing rclcpp code can migrate with minimal changes
- [ ] Full type safety at compile time

### Interoperability
- [ ] C++ nano-ros nodes communicate with Rust nano-ros nodes
- [ ] C++ nano-ros nodes communicate with ROS 2 nodes
- [ ] Works via zenoh transport

### Build System
- [ ] CMake integration via `find_package(nano_ros_cpp)`
- [ ] Optional ament_cmake integration for ROS 2 workspaces
- [ ] Cross-compilation for embedded targets

### Documentation
- [ ] API reference generated
- [ ] Migration guide from rclcpp complete
- [ ] Examples for all major use cases

---

## Related Documents

- [Phase 7: API Alignment](phase-7-api-alignment.md) - Rust API that C++ wraps
- [Phase 3: Services](phase-3-services-params.md) - Service implementation details
- [Message Generation](../message-generation.md) - Message type generation

---

## References

- [cxx crate](https://cxx.rs/) - Safe Rust/C++ interop
- [Corrosion](https://github.com/corrosion-rs/corrosion) - CMake/Cargo integration
- [rclcpp API](https://docs.ros.org/en/humble/p/rclcpp/) - Reference C++ API
- [cbindgen](https://github.com/mozilla/cbindgen) - Alternative C header generator
