# Phase 10: C++ Bindings (rclcpp-Compatible API)

**Status: COMPLETE**

## Executive Summary

Phase 10 implements C++ bindings for nano-ros, providing an rclcpp-compatible API built on top of the Rust implementation. This enables C++ developers to use nano-ros with familiar ROS 2 patterns while leveraging the safety and efficiency of the Rust core.

**Goals:**
1. Create C++ bindings using the `cxx` crate for safe Rust/C++ interop
2. Use Corrosion for CMake/Cargo build integration
3. Provide an API compatible with rclcpp (ROS Humble 16.0.x)
4. Desktop (Linux) targets only - embedded support deferred to separate C API phase
5. Enable gradual migration from rclcpp to nano-ros

**Note:** Embedded/Zephyr C++ support (10.9, 10.10) was not implemented. Zephyr has limited
C++ runtime support, and a pure C API is more appropriate for embedded targets. See the
planned Phase 11 (C API) for embedded language bindings.

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

### Message Serialization Architecture

nano-ros uses **independent C++ CDR serialization** that is wire-compatible with Rust/ROS 2:

```
┌─────────────────────────────────────────────────────────────────┐
│                      User Binary                                │
├─────────────────────────────────────────────────────────────────┤
│  main.cpp                                                       │
│    #include <nano_ros/nano_ros.hpp>                             │
│    #include <my_project/msg/my_message.hpp>  ← generated        │
│                                                                 │
│    auto pub = node->create_publisher<MyMessage>("/topic", qos); │
│    pub->publish(msg);  // msg.serialize() → raw bytes → FFI     │
└──────────────────┬──────────────────────────────────────────────┘
                   │ links
                   ▼
┌─────────────────────────────────────────────────────────────────┐
│              libnano_ros_cpp.so                                 │
├─────────────────────────────────────────────────────────────────┤
│  - Node, Context, Publisher, Subscription                       │
│  - CdrWriter, CdrReader (C++ header-only, wire-compatible)      │
│  - Passes raw bytes to Rust via FFI                             │
└──────────────────┬──────────────────────────────────────────────┘
                   │ links
                   ▼
┌─────────────────────────────────────────────────────────────────┐
│           libnano_ros_cpp_bridge.a (Rust static lib)            │
├─────────────────────────────────────────────────────────────────┤
│  - RustNode, RustPublisher, RustSubscriber                      │
│  - Zenoh-pico transport                                         │
│  - Receives/sends already-serialized CDR bytes                  │
└─────────────────────────────────────────────────────────────────┘
```

**Key Design Decisions:**

1. **Independent C++ CDR** - No FFI overhead for serialization; C++ serializes directly
2. **Wire compatibility** - Same CDR format as Rust nano-ros-serdes and ROS 2
3. **No rosidl dependency** - Message generation is self-contained, no ament required
4. **CMake-based generation** - Familiar workflow via `nano_ros_generate_interfaces()`

---

## Reference Repositories

Cloned to `external/` for API reference (ROS Humble branch):

| Repository     | Path                       | Version | Purpose                      |
|----------------|----------------------------|---------|------------------------------|
| rclcpp         | `external/rclcpp/`         | 16.0.17 | C++ client library reference |
| rcl            | `external/rcl/`            | 5.3.12  | C client library patterns    |
| rcutils        | `external/rcutils/`        | -       | Utility patterns             |
| rmw            | `external/rmw/`            | 6.1.2   | Middleware interface         |
| rcl_interfaces | `external/rcl_interfaces/` | -       | Standard message types       |
| rosidl         | `external/rosidl/`         | -       | Message generation reference |
| ros2_rust      | `external/ros2_rust/`      | 0.6.0   | Rust bindings reference      |

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

## 10.2 Core Types Bridge - COMPLETE

### Work Items

#### 10.2.1 Duration and Time Types
- [x] Define cxx bridge for `Duration`
- [x] Define cxx bridge for `Time`
- [x] Create C++ wrapper classes
- [x] Match rclcpp::Duration and rclcpp::Time API

```rust
// src/lib.rs - Implemented
#[cxx::bridge(namespace = "nano_ros::ffi")]
mod ffi {
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    struct Duration {
        sec: i32,
        nanosec: u32,
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    struct Time {
        sec: i32,
        nanosec: u32,
    }

    extern "Rust" {
        fn duration_from_seconds(seconds: f64) -> Duration;
        fn duration_from_nanoseconds(nanoseconds: i64) -> Duration;
        fn duration_from_milliseconds(milliseconds: i64) -> Duration;
        fn duration_to_nanoseconds(d: Duration) -> i64;
        fn duration_to_seconds(d: Duration) -> f64;
        fn duration_add(a: Duration, b: Duration) -> Duration;
        fn duration_sub(a: Duration, b: Duration) -> Duration;

        fn time_from_nanoseconds(nanoseconds: i64) -> Time;
        fn time_to_nanoseconds(t: Time) -> i64;
        fn time_to_seconds(t: Time) -> f64;
        fn time_add_duration(t: Time, d: Duration) -> Time;
        fn time_sub_duration(t: Time, d: Duration) -> Time;
        fn time_diff(a: Time, b: Time) -> Duration;
    }
}
```

```cpp
// include/nano_ros/duration.hpp - Implemented
namespace nano_ros {
class Duration {
public:
    Duration();
    Duration(int32_t seconds, uint32_t nanoseconds);
    explicit Duration(std::chrono::nanoseconds ns);

    static Duration from_seconds(double seconds);
    static Duration from_nanoseconds(int64_t nanoseconds);
    static Duration from_milliseconds(int64_t milliseconds);
    static Duration zero();

    int32_t sec() const;
    uint32_t nanosec() const;
    int64_t nanoseconds() const;
    double seconds() const;
    std::chrono::nanoseconds to_chrono() const;

    // Full operator support: ==, !=, <, <=, >, >=, +, -, +=, -=, unary -
};
}
```

#### 10.2.2 QoS Types
- [x] Define cxx bridge for QoS policies
- [x] Create C++ QoS class with builder pattern
- [x] Match rclcpp::QoS API

```cpp
// include/nano_ros/qos.hpp - Implemented
namespace nano_ros {
enum class HistoryPolicy { KeepLast, KeepAll };
enum class ReliabilityPolicy { BestEffort, Reliable };
enum class DurabilityPolicy { Volatile, TransientLocal };

class QoS {
public:
    explicit QoS(size_t depth);

    // Builder methods
    QoS& history(HistoryPolicy policy);
    QoS& keep_last(size_t depth);
    QoS& keep_all();
    QoS& reliability(ReliabilityPolicy policy);
    QoS& reliable();
    QoS& best_effort();
    QoS& durability(DurabilityPolicy policy);
    QoS& durability_volatile();
    QoS& transient_local();

    // Presets
    static QoS default_qos();
    static QoS sensor_data();
    static QoS services();
    static QoS parameters();

    // Accessors
    HistoryPolicy get_history() const;
    size_t get_depth() const;
    ReliabilityPolicy get_reliability() const;
    DurabilityPolicy get_durability() const;
};

// Helper functions
inline QoS KeepLast(size_t depth);
inline QoS KeepAll();
}
```

#### 10.2.3 Error Types
- [x] Define error enum in cxx bridge
- [x] Create C++ exception classes
- [x] Implement Result → exception conversion

Error handling uses `rust::Error` from cxx with conversion to `std::runtime_error`:
```cpp
try {
    auto rust_ctx = ffi::create_context();
} catch (const ::rust::Error& e) {
    throw std::runtime_error(std::string("Failed to create context: ") + e.what());
}
```

### Acceptance Criteria
- [x] Core types can be constructed and used from C++
- [x] API matches rclcpp patterns
- [x] Proper error handling with exceptions

---

## 10.3 Context and Node - COMPLETE

### Work Items

#### 10.3.1 Context Bridge
- [x] Define opaque Rust Context in cxx bridge
- [x] Expose `Context::new()` and `Context::from_env()`
- [x] Create C++ Context wrapper

```cpp
// include/nano_ros/context.hpp - Implemented
namespace nano_ros {
class Context {
public:
    static std::shared_ptr<Context> create();
    static std::shared_ptr<Context> from_env();

    bool ok() const;
    uint32_t domain_id() const;
    std::shared_ptr<Node> create_node(const std::string& name);
    std::shared_ptr<Node> create_node(const std::string& name, const std::string& ns);
    std::shared_ptr<Node> create_node(const NodeOptions& options);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}
```

#### 10.3.2 Node Bridge
- [x] Define opaque Rust Node in cxx bridge
- [x] Expose node creation and accessor methods
- [x] Create C++ Node wrapper with shared_ptr semantics
- [x] Add Clock access (get_clock, now)

```cpp
// include/nano_ros/node.hpp - Implemented
namespace nano_ros {
class Node : public std::enable_shared_from_this<Node> {
public:
    std::string get_name() const;
    std::string get_namespace() const;
    std::string get_fully_qualified_name() const;
    std::shared_ptr<Clock> get_clock() const;
    Time now() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}
```

#### 10.3.3 NodeOptions
- [x] Implement NodeOptions builder class
- [x] Support namespace configuration
- [x] Match rclcpp::NodeOptions API (subset)

```cpp
// include/nano_ros/node_options.hpp - Implemented
namespace nano_ros {
class NodeOptions {
public:
    NodeOptions();
    explicit NodeOptions(const std::string& name);

    NodeOptions& node_name(const std::string& name);
    NodeOptions& namespace_(const std::string& ns);
    NodeOptions& use_clock_type(ClockType clock_type);
    NodeOptions& use_intra_process_comms(bool enable);
    NodeOptions& start_parameter_services(bool enable);

    const std::string& get_node_name() const;
    const std::string& get_namespace() const;
    ClockType get_clock_type() const;
};
}
```

#### 10.3.4 Clock Bridge (Bonus)
- [x] Define opaque Rust Clock in cxx bridge
- [x] Create C++ Clock wrapper
- [x] Support SystemTime, SteadyTime, RosTime

```cpp
// include/nano_ros/clock.hpp - Implemented
namespace nano_ros {
enum class ClockType { SystemTime, SteadyTime, RosTime };

class Clock {
public:
    static std::shared_ptr<Clock> create_system();
    static std::shared_ptr<Clock> create_steady();
    static std::shared_ptr<Clock> create_ros();

    ClockType get_clock_type() const;
    Time now() const;
};
}
```

### Acceptance Criteria
- [x] Context can be created and used to spawn nodes
- [x] Nodes expose expected accessors
- [x] Matches rclcpp initialization patterns
- [x] Clock access available on nodes

---

## 10.4 Publisher and Subscription - COMPLETE

### Work Items

#### 10.4.1 Publisher Bridge
- [x] Define generic publish interface in cxx bridge
- [x] Create templated C++ Publisher class
- [x] Support serialized message passing (raw CDR bytes)

```rust
// src/lib.rs - Publisher bridge implemented
extern "Rust" {
    type RustPublisher;
    fn create_publisher(node: &RustNode, topic: &str, qos: &QoSProfile) -> Result<Box<RustPublisher>>;
    fn publisher_publish(pub_: &RustPublisher, data: &[u8]) -> Result<()>;
    fn publisher_topic_name(pub_: &RustPublisher) -> String;
}
```

```cpp
// include/nano_ros/publisher.hpp - Implemented
namespace nano_ros {
class Publisher {
public:
    void publish_raw(const std::vector<uint8_t>& data);
    void publish_raw(const uint8_t* data, size_t size);
    std::string get_topic_name() const;
};

template<typename MessageT>
class TypedPublisher {
public:
    void publish(const MessageT& msg);
    std::string get_topic_name() const;
};
}
```

#### 10.4.2 Subscription Bridge
- [x] Define take mechanism in cxx bridge (polling-based)
- [x] Create templated C++ Subscription class
- [x] Handle message deserialization

```rust
// src/lib.rs - Subscriber bridge implemented
extern "Rust" {
    type RustSubscriber;
    fn create_subscriber(node: &RustNode, topic: &str, qos: &QoSProfile) -> Result<Box<RustSubscriber>>;
    fn subscriber_take(sub: &RustSubscriber) -> Result<Vec<u8>>;
    fn subscriber_topic_name(sub: &RustSubscriber) -> String;
}
```

```cpp
// include/nano_ros/subscription.hpp - Implemented
namespace nano_ros {
class Subscription {
public:
    std::vector<uint8_t> take_raw();
    bool has_message();
    std::string get_topic_name() const;
};

template<typename MessageT>
class TypedSubscription {
public:
    using CallbackT = std::function<void(const MessageT&)>;
    std::optional<MessageT> take();
    size_t spin_once();
    void set_callback(CallbackT callback);
};
}
```

#### 10.4.3 Message Serialization Bridge
- [x] Bridge serialized bytes across FFI boundary via `publish_raw()`/`take_raw()`
- [ ] Typed message support deferred to Phase 10.5 (Message Type Generation)

#### 10.4.4 Node Integration
- [x] Add `Node::create_publisher()` method
- [x] Add `Node::create_subscription()` method
- [x] Support templated and non-templated creation

```cpp
// Node methods added:
std::shared_ptr<Publisher> create_publisher(const std::string& topic, const QoS& qos);
std::shared_ptr<Subscription> create_subscription(const std::string& topic, const QoS& qos);

template<typename MessageT>
std::shared_ptr<TypedPublisher<MessageT>> create_publisher(const std::string& topic, const QoS& qos);

template<typename MessageT>
std::shared_ptr<TypedSubscription<MessageT>> create_subscription(const std::string& topic, const QoS& qos);
```

### Acceptance Criteria
- [x] Publishers can send messages to Rust subscribers
- [x] C++ subscriptions receive messages from Rust publishers
- [x] Example builds and demonstrates pub/sub creation

---

## 10.5 Message Type Generation - COMPLETE

### Design Overview

Message generation follows a ROS-like CMake workflow but is self-contained (no ament/rosidl dependency):

**User's CMakeLists.txt:**
```cmake
find_package(nano_ros_cpp REQUIRED)

# Generate message bindings from .msg files
nano_ros_generate_interfaces(${PROJECT_NAME}
  "msg/MyMessage.msg"
  "msg/OtherMessage.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

add_executable(my_node src/main.cpp)
target_link_libraries(my_node
  nano_ros_cpp::nano_ros_cpp
  ${PROJECT_NAME}::msg  # Generated messages
)
```

**Generated Output Structure:**
```
${CMAKE_BINARY_DIR}/nano_ros_generated/${PROJECT_NAME}/
├── msg/
│   ├── my_message.hpp
│   └── other_message.hpp
└── ${PROJECT_NAME}_msg.cmake  # CMake target definitions
```

### Work Items

#### 10.5.1 C++ CDR Serialization Library
- [x] Create `include/nano_ros/cdr.hpp` header-only CDR implementation
- [x] Implement `CdrWriter` class:
  ```cpp
  class CdrWriter {
  public:
      void write_i8(int8_t v);
      void write_i16(int16_t v);
      void write_i32(int32_t v);
      void write_i64(int64_t v);
      void write_u8(uint8_t v);
      void write_u16(uint16_t v);
      void write_u32(uint32_t v);
      void write_u64(uint64_t v);
      void write_f32(float v);
      void write_f64(double v);
      void write_bool(bool v);
      void write_string(const std::string& s);
      template<typename T> void write_array(const std::vector<T>& arr);

      const uint8_t* data() const;
      size_t size() const;
  };
  ```
- [x] Implement `CdrReader` class (mirror of CdrWriter)
- [x] Handle CDR alignment rules (natural alignment, relative to origin)
- [x] Add CDR encapsulation header (0x00 0x01 for little-endian)
- [ ] Unit tests for wire compatibility with Rust nano-ros-serdes

#### 10.5.2 Message Generator Tool
- [x] Create Python generator script: `scripts/nano_ros_generate_cpp.py`
- [x] Parse .msg file format (field definitions, constants, comments)
- [x] Support primitive types: bool, int8-64, uint8-64, float32/64, string
- [x] Support arrays: fixed `T[N]`, bounded `T[<=N]`, unbounded `T[]`
- [x] Support nested message types
- [x] Generate struct with:
  - Default member initializers
  - `serialize(CdrWriter&)` method
  - `deserialize(CdrReader&)` method
  - `static type_name()` for topic type registration

**Generated Message Example:**
```cpp
// Generated: my_project/msg/my_message.hpp
#pragma once
#include <nano_ros/cdr.hpp>
#include <string>
#include <vector>

namespace my_project::msg {

struct MyMessage {
    int32_t value{0};
    std::string name;
    std::vector<float> data;

    void serialize(nano_ros::CdrWriter& writer) const {
        writer.write_i32(value);
        writer.write_string(name);
        writer.write_sequence(data);
    }

    void deserialize(nano_ros::CdrReader& reader) {
        value = reader.read_i32();
        name = reader.read_string();
        data = reader.read_sequence<float>();
    }

    static constexpr const char* type_name() {
        return "my_project/msg/MyMessage";
    }
};

}  // namespace my_project::msg
```

#### 10.5.3 CMake Integration
- [x] Create `cmake/nano_ros_generate_interfaces.cmake` macro
- [x] Auto-detect Python interpreter
- [x] Configure include paths for generated headers
- [x] Create CMake INTERFACE library target for generated messages
- [x] Handle DEPENDENCIES for nested message types
- [x] Support regeneration on .msg file changes

**CMake Macro Implementation:**
```cmake
function(nano_ros_generate_interfaces TARGET_NAME)
  cmake_parse_arguments(ARG "" "" "DEPENDENCIES" ${ARGN})

  # Collect .msg files from remaining arguments
  set(MSG_FILES ${ARG_UNPARSED_ARGUMENTS})

  # Output directory
  set(OUTPUT_DIR ${CMAKE_BINARY_DIR}/nano_ros_generated/${TARGET_NAME})

  # Run generator for each .msg file
  foreach(MSG_FILE ${MSG_FILES})
    # ... invoke Python generator
  endforeach()

  # Create INTERFACE library
  add_library(${TARGET_NAME}_msg INTERFACE)
  target_include_directories(${TARGET_NAME}_msg INTERFACE ${OUTPUT_DIR}/..)
  add_library(${TARGET_NAME}::msg ALIAS ${TARGET_NAME}_msg)
endfunction()
```

#### 10.5.4 Standard Message Types
- [x] Pre-generate common std_msgs: Bool, Int8-64, UInt8-64, Float32/64, String, Header
- [ ] Pre-generate common geometry_msgs: Point, Vector3, Quaternion, Pose, Twist
- [x] Pre-generate builtin_interfaces: Time, Duration
- [ ] Package as `nano_ros_std_msgs` CMake component
- [ ] Install to `share/nano_ros_cpp/msg/`

#### 10.5.5 TypedPublisher/TypedSubscription Integration
- [x] Update `TypedPublisher<T>::publish()` to use `CdrWriter`:
  ```cpp
  void publish(const MessageT& msg) {
      nano_ros::CdrWriter writer;
      writer.write_encapsulation();  // CDR header
      msg.serialize(writer);
      inner_->publish_raw(writer.data(), writer.size());
  }
  ```
- [x] Update `TypedSubscription<T>::take()` to use `CdrReader`:
  ```cpp
  std::optional<MessageT> take() {
      auto raw = inner_->take_raw();
      if (raw.empty()) return std::nullopt;
      nano_ros::CdrReader reader(raw.data(), raw.size());
      reader.read_encapsulation();  // Skip CDR header
      MessageT msg;
      msg.deserialize(reader);
      return msg;
  }
  ```

### Acceptance Criteria
- [x] `nano_ros_generate_interfaces()` CMake macro works
- [x] Generated messages compile without errors
- [ ] CDR serialization is wire-compatible with Rust nano-ros-serdes
- [ ] Messages can be published from C++ and received by Rust (and vice versa)
- [ ] Messages interoperate with ROS 2 nodes via zenoh

---

## 10.6 Executor Integration - COMPLETE

### rclcpp API Reference

The rclcpp executor API (Humble 16.0.x) provides:

| Method                    | Description                                       |
|---------------------------|---------------------------------------------------|
| `spin()`                  | Blocking spin loop until cancelled                |
| `spin_once(timeout)`      | Execute one work item, can block waiting for work |
| `spin_some(max_duration)` | Execute all immediately available work            |
| `spin_all(max_duration)`  | Execute work repeatedly until duration exceeded   |
| `add_node(node)`          | Add node to executor (weak reference)             |
| `remove_node(node)`       | Remove node from executor                         |
| `cancel()`                | Request spin to stop (thread-safe)                |
| `is_spinning()`           | Check if currently spinning                       |

See [rclcpp Executor documentation](https://docs.ros.org/en/humble/p/rclcpp/generated/classrclcpp_1_1Executor.html).

### Embedded Compatibility

The Rust side provides two executor types:
- **BasicExecutor** (std): Full-featured with blocking `spin()`, uses threads
- **PollingExecutor** (no_std): Manual `spin_once(delta_ms)`, no threads/blocking

The C++ API mirrors this with two executor classes for different platforms:

| Feature | SingleThreadedExecutor | PollingExecutor |
|---------|----------------------|-----------------|
| Platform | Desktop (std) | Embedded (Zephyr, NuttX) |
| `spin()` | Blocking loop | Not available |
| `spin_once()` | With timeout (blocking) | Non-blocking only |
| Threading | Uses `std::this_thread::sleep_for` | No threading |
| Allocation | Uses `std::function` | Function pointers only |
| rclcpp compatible | Yes | Partial (spin_once only) |

**Embedded Constraints:**
- No `std::thread` or `std::this_thread::sleep_for`
- Avoid heap allocation in hot paths (callbacks, message processing)
- Timer handling via `delta_ms` parameter (user provides elapsed time)
- Function pointers instead of `std::function` for callbacks (optional)

### Design Decisions

1. **Node Ownership**: Unlike rclcpp where nodes are independent, nano-ros nodes are
   conceptually owned by the executor context. The C++ API provides `add_node()`/`remove_node()`
   for API compatibility, but nodes must be created via `Context::create_node()` first.

2. **spin_once Semantics**:
   - **SingleThreadedExecutor** (std): rclcpp-compatible with timeout
     - timeout < 0: Block indefinitely (busy-poll with short sleeps)
     - timeout == 0: Non-blocking, process available work only
     - timeout > 0: Poll with sleep until timeout
   - **PollingExecutor** (embedded): Non-blocking only
     - `spin_once(delta_ms)` - user provides elapsed time for timers
     - Returns immediately after processing available work

3. **Callback Storage**:
   - **SingleThreadedExecutor**: Uses `std::function` (heap allocation OK)
   - **PollingExecutor**: Works with either `std::function` or function pointers

4. **Deferred Features**: Callback groups and `spin_until_future_complete()` are not
   implemented in the initial version but can be added later.

### Work Items

#### 10.6.1 Executor Bridge
- [ ] Define executor FFI in cxx bridge
- [ ] Create C++ Executor base class
- [ ] Implement SingleThreadedExecutor (std platforms)
- [ ] Implement PollingExecutor (embedded platforms)

```cpp
// include/nano_ros/executor.hpp
namespace nano_ros {

/// Executor options
struct ExecutorOptions {
    std::shared_ptr<Context> context;

    ExecutorOptions();
    explicit ExecutorOptions(std::shared_ptr<Context> context);
};

/// Base executor class (common interface)
class Executor {
public:
    explicit Executor(const ExecutorOptions& options = ExecutorOptions());
    virtual ~Executor();

    // Node management
    virtual void add_node(std::shared_ptr<Node> node);
    virtual void remove_node(std::shared_ptr<Node> node);

    // Control
    void cancel();
    bool is_spinning() const;

protected:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// =============================================================================
// SingleThreadedExecutor - Desktop/std platforms (rclcpp-compatible)
// =============================================================================

/// Single-threaded executor with blocking spin (requires std)
///
/// This executor provides rclcpp-compatible API with blocking spin methods.
/// Use this on desktop platforms where std::thread is available.
///
/// Example:
/// @code
/// nano_ros::SingleThreadedExecutor executor;
/// executor.add_node(node);
/// executor.spin();  // Blocks until cancel() is called
/// @endcode
class SingleThreadedExecutor : public Executor {
public:
    explicit SingleThreadedExecutor(const ExecutorOptions& options = ExecutorOptions());
    ~SingleThreadedExecutor() override;

    /// Blocking spin loop - processes work until cancel() is called
    void spin();

    /// Execute one work item with optional timeout (rclcpp-compatible)
    /// @param timeout_ns Timeout in nanoseconds (-1 = block forever, 0 = non-blocking)
    void spin_once(int64_t timeout_ns = -1);

    /// Execute all immediately available work (non-blocking)
    /// @param max_duration_ns Maximum time to spend (0 = no limit)
    void spin_some(int64_t max_duration_ns = 0);
};

// Convenience functions (match rclcpp:: namespace)
void spin(std::shared_ptr<Node> node);
void spin_some(std::shared_ptr<Node> node);

// =============================================================================
// PollingExecutor - Embedded platforms (Zephyr, NuttX, bare-metal)
// =============================================================================

/// Polling executor for embedded platforms (no threads, no blocking)
///
/// This executor is designed for embedded systems where threads are not
/// available or not desired. The user must call spin_once() periodically
/// from their main loop or RTOS task.
///
/// Example (Zephyr):
/// @code
/// nano_ros::PollingExecutor executor;
/// executor.add_node(node);
///
/// while (true) {
///     executor.spin_once(10);  // 10ms since last call
///     k_msleep(10);
/// }
/// @endcode
class PollingExecutor : public Executor {
public:
    explicit PollingExecutor(const ExecutorOptions& options = ExecutorOptions());
    ~PollingExecutor() override;

    /// Process all available work (non-blocking)
    ///
    /// @param delta_ms Milliseconds elapsed since last call (for timer processing)
    /// @return Number of callbacks executed
    ///
    /// This method:
    /// 1. Polls transport for incoming messages
    /// 2. Invokes subscription callbacks for received messages
    /// 3. Fires ready timers based on delta_ms
    uint32_t spin_once(uint32_t delta_ms);

    /// Get the number of nodes in this executor
    size_t node_count() const;
};

}  // namespace nano_ros
```

#### 10.6.2 Rust Bridge Implementation
- [ ] Extend cxx bridge for executor operations
- [ ] Map C++ nodes to Rust NodeState
- [ ] Thread-safe cancel mechanism for SingleThreadedExecutor
- [ ] Polling interface for PollingExecutor

```rust
// src/lib.rs additions
extern "Rust" {
    // Executor types
    type RustSingleThreadedExecutor;
    type RustPollingExecutor;

    // SingleThreadedExecutor (std only)
    fn create_single_threaded_executor(
        context: &RustContext
    ) -> Result<Box<RustSingleThreadedExecutor>>;
    fn ste_add_node(exec: &mut RustSingleThreadedExecutor, node: &RustNode) -> Result<()>;
    fn ste_remove_node(exec: &mut RustSingleThreadedExecutor, node: &RustNode) -> Result<()>;
    fn ste_spin(exec: &mut RustSingleThreadedExecutor);  // Blocking
    fn ste_spin_once(exec: &mut RustSingleThreadedExecutor, timeout_ns: i64) -> u32;
    fn ste_spin_some(exec: &mut RustSingleThreadedExecutor, max_duration_ns: i64) -> u32;
    fn ste_cancel(exec: &RustSingleThreadedExecutor);
    fn ste_is_spinning(exec: &RustSingleThreadedExecutor) -> bool;

    // PollingExecutor (embedded compatible)
    fn create_polling_executor(
        context: &RustContext
    ) -> Result<Box<RustPollingExecutor>>;
    fn pe_add_node(exec: &mut RustPollingExecutor, node: &RustNode) -> Result<()>;
    fn pe_remove_node(exec: &mut RustPollingExecutor, node: &RustNode) -> Result<()>;
    fn pe_spin_once(exec: &mut RustPollingExecutor, delta_ms: u32) -> u32;
    fn pe_node_count(exec: &RustPollingExecutor) -> usize;
}
```

#### 10.6.3 Callback Handling
- [ ] Process subscription callbacks during spin
- [ ] Process timer callbacks during spin
- [ ] Exception-safe callback invocation (SingleThreadedExecutor)

```cpp
// SingleThreadedExecutor implementation (uses std::thread for sleep)
void SingleThreadedExecutor::spin_once(int64_t timeout_ns) {
    auto start = std::chrono::steady_clock::now();

    while (true) {
        uint32_t work_done = ffi::ste_spin_once(*impl_->rust_executor, 0);

        if (work_done > 0 || timeout_ns == 0) {
            return;  // Work done or non-blocking mode
        }

        if (timeout_ns < 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;  // Infinite timeout
        }

        auto elapsed = std::chrono::steady_clock::now() - start;
        if (elapsed >= std::chrono::nanoseconds(timeout_ns)) {
            return;  // Timeout expired
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

// PollingExecutor implementation (no std::thread, embedded-safe)
uint32_t PollingExecutor::spin_once(uint32_t delta_ms) {
    return ffi::pe_spin_once(*impl_->rust_executor, delta_ms);
}
```

### Usage Examples

#### Desktop (std) - rclcpp-compatible

```cpp
#include <nano_ros/nano_ros.hpp>
#include <std_msgs/msg/int32.hpp>

int main() {
    auto context = nano_ros::Context::from_env();
    auto node = context->create_node("desktop_node");

    auto subscription = node->create_subscription<std_msgs::msg::Int32>(
        "/topic",
        nano_ros::QoS(10),
        [](const std_msgs::msg::Int32& msg) {
            std::cout << "Received: " << msg.data << std::endl;
        }
    );

    // Option 1: Convenience function
    nano_ros::spin(node);

    // Option 2: Explicit executor
    nano_ros::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    return 0;
}
```

#### Embedded (Zephyr)

```cpp
#include <nano_ros/nano_ros.hpp>
#include <std_msgs/msg/int32.hpp>
#include <zephyr/kernel.h>

// Callback function (function pointer works, std::function also supported)
void on_message(const std_msgs::msg::Int32& msg) {
    printk("Received: %d\n", msg.data);
}

int main() {
    auto context = nano_ros::Context::from_env();
    auto node = context->create_node("zephyr_node");

    auto subscription = node->create_subscription<std_msgs::msg::Int32>(
        "/topic", nano_ros::QoS(10), on_message);

    nano_ros::PollingExecutor executor;
    executor.add_node(node);

    while (true) {
        executor.spin_once(10);  // 10ms elapsed
        k_msleep(10);
    }

    return 0;
}
```

### Acceptance Criteria
- [x] **SingleThreadedExecutor** (std platforms):
  - [x] `spin()` blocks until cancelled
  - [x] `spin_once(timeout)` waits for specified timeout
  - [x] `spin_some()` returns immediately (non-blocking)
  - [x] `cancel()` is thread-safe and stops spinning
  - [x] Convenience functions `spin(node)` and `spin_some(node)` work
- [x] **PollingExecutor** (embedded platforms):
  - [x] `spin_once(delta_ms)` non-blocking
  - [x] No `std::thread` or blocking operations used
  - [x] Suitable for Zephyr and other embedded platforms
- [x] Both executors:
  - [x] `add_node()`/`remove_node()` manage executor's node set

**Note:** C++ subscriptions use polling (`take()`) rather than callbacks. The executor
provides the rclcpp-compatible structure for node management and blocking spin, while
users poll subscriptions directly via `take()`. Timer callbacks are not yet supported
in the C++ bindings.

---

## 10.7 Service Support - COMPLETE

### Work Items

#### 10.7.1 Service Client Bridge
- [x] Expose Rust service client through cxx
- [x] Create templated C++ TypedServiceClient class
- [x] Implement synchronous `call()` method

#### 10.7.2 Service Server Bridge
- [x] Expose Rust service server through cxx
- [x] Create templated C++ TypedServiceServer class
- [x] Implement polling-based `try_recv_request()` / `send_reply()`

### Implementation Details

**Rust FFI (lib.rs):**
- `RustServiceClient` wrapper with `call_raw(request)` returning raw CDR bytes
- `RustServiceServer` wrapper with `try_recv_request()` and `send_reply()`
- Added buffer accessors to `ConnectedServiceClient/Server` for FFI use

**C++ API (service.hpp):**
```cpp
// Raw service client
class ServiceClient {
    std::vector<uint8_t> call_raw(const std::vector<uint8_t>& request);
    std::string get_service_name() const;
};

// Typed service client
template<typename ServiceT>
class TypedServiceClient {
    typename ServiceT::Response call(const typename ServiceT::Request& request);
};

// Raw service server
class ServiceServer {
    std::vector<uint8_t> try_recv_request();  // Non-blocking
    void send_reply(const std::vector<uint8_t>& response);
};

// Typed service server
template<typename ServiceT>
class TypedServiceServer {
    std::optional<typename ServiceT::Request> try_recv_request();
    void send_reply(const typename ServiceT::Response& response);
};
```

**Node methods:**
```cpp
std::shared_ptr<ServiceClient> create_client(const std::string& service_name);
std::shared_ptr<ServiceServer> create_service(const std::string& service_name);

template<typename ServiceT>
std::shared_ptr<TypedServiceClient<ServiceT>> create_client(const std::string& service_name);

template<typename ServiceT>
std::shared_ptr<TypedServiceServer<ServiceT>> create_service(const std::string& service_name);
```

### Acceptance Criteria
- [x] C++ service clients can call services with typed request/response
- [x] C++ service servers can receive requests and send responses
- [x] Polling-based server API (no callback support yet)
- [x] Wire-compatible CDR serialization with Rust services

**Note:** Async `call_async()` and callback-based servers are not yet implemented.
These can be added in a future iteration.

---

## 10.8 Examples and Documentation - COMPLETE

### Work Items

#### 10.8.1 Basic Examples (COMPLETE)
- [x] Create `examples/native-cpp-talker/` - Publisher example
- [x] Create `examples/native-cpp-listener/` - Subscriber example
- [x] Create `examples/native-cpp-custom-msg/` - Custom message example

#### 10.8.2 Service Examples (COMPLETE)
- [x] Create `examples/native-cpp-service-server/` - Service server example
- [x] Create `examples/native-cpp-service-client/` - Service client example

#### 10.8.3 Action Examples
- [ ] Create `examples/native-cpp-action-server/` - Action server example
- [ ] Create `examples/native-cpp-action-client/` - Action client example
- [ ] Requires C++ action API implementation (see 10.11)

#### 10.8.4 Embedded Examples
- [x] Create `examples/embedded-cpp-talker/` - Embedded publisher (no-std compatible)
- [x] Create `examples/embedded-cpp-listener/` - Embedded subscriber (no-std compatible)
- [ ] Create `examples/embedded-cpp-custom-msg/` - Embedded custom message demo

#### 10.8.5 CMake Integration Example
- [ ] Example `find_package(nano_ros_cpp)` usage
- [ ] Example ament_cmake integration
- [ ] Example standalone CMake project

#### 10.8.6 Documentation
- [ ] API reference documentation
- [ ] Migration guide from rclcpp
- [ ] Build integration guide

### Acceptance Criteria
- [x] Examples build and run correctly
- [x] Examples interoperate with ROS 2 nodes (via zenoh)
- [ ] Documentation is clear and complete (deferred)

---

## 10.9 Zephyr Support - NOT IMPLEMENTED

**Decision:** Zephyr C++ support was not implemented.

**Rationale:**
- Zephyr has limited C++ runtime support (often no RTTI, no exceptions, limited STL)
- C++ bindings require full libstdc++ or equivalent
- A pure C API is more portable and idiomatic for embedded RTOS targets
- The existing Rust API already works on Zephyr (see `zephyr-talker`/`zephyr-listener`)

**Alternative:** A dedicated C API phase (Phase 11) will provide embedded language bindings
using a pure C interface similar to rclc/micro-ROS.

---

## Implementation Order

```
Phase 10.1: Project Setup (Foundation)               ✓ COMPLETE
    └── Package structure, CMake/Cargo integration

Phase 10.2: Core Types Bridge                        ✓ COMPLETE
    └── Duration, Time, QoS, Error types

Phase 10.3: Context and Node                         ✓ COMPLETE
    └── Context, Node, NodeOptions, Clock

Phase 10.4: Publisher and Subscription               ✓ COMPLETE
    └── Pub/Sub with raw byte interface

Phase 10.5: Message Type Generation                  ✓ COMPLETE
    ├── C++ CdrWriter/CdrReader (header-only)
    ├── Python message generator
    ├── CMake nano_ros_generate_interfaces() macro
    └── Standard message types (std_msgs, builtin_interfaces)

Phase 10.6: Executor Integration                     ✓ COMPLETE
    └── Spin, node management, polling subscriptions

Phase 10.7: Service Support                          ✓ COMPLETE
    └── Client and Server (polling-based)

Phase 10.8: Examples and Documentation               ✓ COMPLETE
    └── Basic examples (cpp-custom-msg, cpp-service-client)

Phase 10.9: Zephyr Support                           ✗ NOT IMPLEMENTED
    └── Deferred to C API (Phase 11)

Phase 10.10: Embedded System Support (no_std)        ✗ NOT IMPLEMENTED
    └── Deferred to C API (Phase 11)
```

---

## 10.10 Embedded System Support (no_std) - NOT IMPLEMENTED

**Decision:** Embedded C++ support was not implemented.

**Rationale:**
- Freestanding C++ requires significant complexity (ETL, custom allocators, etc.)
- A pure C API is simpler, more portable, and widely supported on all embedded platforms
- C APIs are the standard approach for embedded ROS (rclc, micro-ROS)
- The complexity of maintaining two C++ variants (std vs freestanding) is not justified

**Alternative:** See Phase 11 (C API) for embedded language bindings.

---

## Appendix: Original 10.10 Design (For Reference)

The following design was planned but not implemented. It is preserved here for reference
if C++ embedded support is reconsidered in the future.

### Original Overview

This phase would have enabled C++ bindings for embedded systems without standard library support.
The goal was to provide a freestanding C++ API that mirrors the desktop API but uses
fixed-size containers and avoids heap allocation in critical paths.

### Current Limitations

The current C++ bindings require the full C++ standard library:

| Header | std Dependencies | Embedded Alternative |
|--------|------------------|---------------------|
| `cdr.hpp` | `std::string`, `std::vector`, `std::stdexcept` | ETL containers, error codes |
| `node.hpp` | `std::string`, `std::memory`, `std::functional` | `etl::string`, raw pointers |
| `publisher.hpp` | `std::string`, `std::vector`, `std::memory` | ETL containers |
| `subscription.hpp` | `std::string`, `std::vector`, `std::optional`, `std::functional` | ETL alternatives |
| `service.hpp` | `std::string`, `std::vector`, `std::optional`, `std::functional` | ETL alternatives |
| `context.hpp` | `std::string`, `std::memory` | ETL string, raw pointers |
| `executor.hpp` | `std::memory`, `std::vector` | Static allocation |

### Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                  Build Configuration                             │
├─────────────────────────────────────────────────────────────────┤
│  CMake Options:                                                  │
│    -DNANO_ROS_EMBEDDED=ON          Enable embedded mode          │
│    -DNANO_ROS_RUST_TARGET=<target> Cross-compile target          │
│    -DNANO_ROS_TRANSPORT=<backend>  Transport backend             │
│    -DNANO_ROS_USE_ETL=ON           Use ETL containers            │
└───────────────────────────────┬─────────────────────────────────┘
                                │
          ┌─────────────────────┴─────────────────────┐
          │                                           │
          ▼                                           ▼
┌─────────────────────┐                 ┌─────────────────────────┐
│   Desktop Build     │                 │   Embedded Build        │
│   (EMBEDDED=OFF)    │                 │   (EMBEDDED=ON)         │
├─────────────────────┤                 ├─────────────────────────┤
│ - std::string       │                 │ - etl::string<N>        │
│ - std::vector       │                 │ - etl::vector<T,N>      │
│ - std::shared_ptr   │                 │ - Raw pointers / static │
│ - std::function     │                 │ - Function pointers     │
│ - Exceptions        │                 │ - Error codes           │
│ - Dynamic alloc     │                 │ - Static allocation     │
└─────────────────────┘                 └─────────────────────────┘
```

### Work Items

#### 10.10.1 CMake Cross-Compilation Support

Add CMake options for embedded builds and cross-compilation:

```cmake
# CMakeLists.txt additions

# =============================================================================
# Embedded Build Options
# =============================================================================

option(NANO_ROS_EMBEDDED "Build for embedded systems (no std library)" OFF)
option(NANO_ROS_USE_ETL "Use Embedded Template Library for containers" OFF)

# Rust target for cross-compilation (empty = host)
set(NANO_ROS_RUST_TARGET "" CACHE STRING "Rust target triple for cross-compilation")

# Transport backend selection
set(NANO_ROS_TRANSPORT "zenoh" CACHE STRING "Transport backend")
set_property(CACHE NANO_ROS_TRANSPORT PROPERTY STRINGS
    "zenoh"           # Desktop: zenoh-pico POSIX
    "shim-zephyr"     # Zephyr RTOS
    "shim-smoltcp"    # Bare-metal with smoltcp
)

# Feature flags passed to Cargo
set(NANO_ROS_CARGO_FEATURES "" CACHE STRING "Additional Cargo features")

# =============================================================================
# Configure Corrosion for Cross-Compilation
# =============================================================================

if(NANO_ROS_RUST_TARGET)
    # Set Rust target for Corrosion
    set(Rust_CARGO_TARGET ${NANO_ROS_RUST_TARGET})

    # Disable default features and enable selected transport
    set(CARGO_FEATURE_ARGS "--no-default-features")

    if(NANO_ROS_EMBEDDED)
        list(APPEND CARGO_FEATURE_ARGS "--features" "alloc,${NANO_ROS_TRANSPORT}")
    else()
        list(APPEND CARGO_FEATURE_ARGS "--features" "std,alloc,${NANO_ROS_TRANSPORT}")
    endif()

    if(NANO_ROS_CARGO_FEATURES)
        list(APPEND CARGO_FEATURE_ARGS "--features" "${NANO_ROS_CARGO_FEATURES}")
    endif()

    corrosion_import_crate(
        MANIFEST_PATH Cargo.toml
        FLAGS ${CARGO_FEATURE_ARGS}
    )
else()
    # Default host build
    corrosion_import_crate(MANIFEST_PATH Cargo.toml)
endif()

# =============================================================================
# Embedded C++ Configuration
# =============================================================================

if(NANO_ROS_EMBEDDED)
    # Define macro for conditional compilation
    target_compile_definitions(nano_ros_cpp PUBLIC NANO_ROS_EMBEDDED=1)

    # Disable exceptions and RTTI for smaller binary
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(nano_ros_cpp PRIVATE -fno-exceptions -fno-rtti)
    endif()

    # Don't link against pthread/dl on embedded
    # (platform-specific libs handled by toolchain)
endif()

if(NANO_ROS_USE_ETL)
    # Fetch ETL if not found
    find_package(etl QUIET)
    if(NOT etl_FOUND)
        FetchContent_Declare(
            etl
            GIT_REPOSITORY https://github.com/ETLCPP/etl.git
            GIT_TAG 20.38.0
        )
        FetchContent_MakeAvailable(etl)
    endif()

    target_compile_definitions(nano_ros_cpp PUBLIC NANO_ROS_USE_ETL=1)
    target_link_libraries(nano_ros_cpp PUBLIC etl::etl)
endif()
```

**Toolchain file example (Zephyr):**
```cmake
# toolchain-zephyr.cmake
set(CMAKE_SYSTEM_NAME Zephyr)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Zephyr provides these
set(CMAKE_C_COMPILER ${ZEPHYR_SDK_INSTALL_DIR}/arm-zephyr-eabi/bin/arm-zephyr-eabi-gcc)
set(CMAKE_CXX_COMPILER ${ZEPHYR_SDK_INSTALL_DIR}/arm-zephyr-eabi/bin/arm-zephyr-eabi-g++)

# Rust target
set(NANO_ROS_RUST_TARGET "thumbv7em-none-eabihf")
```

#### 10.10.2 Cargo.toml Feature Gates

Update the Rust bridge crate to support no_std:

```toml
# crates/nano-ros-cpp/Cargo.toml

[package]
name = "nano-ros-cpp-bridge"
version = "0.1.0"
edition = "2021"

[lib]
crate-type = ["staticlib"]

[features]
default = ["std", "zenoh"]

# Standard library support (desktop)
std = ["nano-ros/std", "alloc"]

# Heap allocation (required for most functionality)
alloc = ["nano-ros/alloc"]

# Transport backends (mutually exclusive)
zenoh = ["nano-ros/zenoh", "std"]           # Requires std
shim-zephyr = ["nano-ros/shim-zephyr"]      # Zephyr RTOS
shim-smoltcp = ["nano-ros/shim-smoltcp"]    # Bare-metal

# Embedded executor modes
rtic = ["nano-ros/rtic"]
polling = ["nano-ros/polling"]

[dependencies]
cxx = { version = "1.0", default-features = false }
nano-ros = { path = "../nano-ros", default-features = false }
nano-ros-serdes = { path = "../nano-ros-serdes", default-features = false }

[build-dependencies]
cxx-build = "1.0"
```

#### 10.10.3 Freestanding C++ Headers

Create embedded-compatible headers using ETL or fixed-size alternatives:

**Type Abstraction Layer:**
```cpp
// include/nano_ros/embedded/types.hpp
#pragma once

#include <cstdint>
#include <cstddef>

#if defined(NANO_ROS_USE_ETL)
    #include <etl/string.h>
    #include <etl/vector.h>
    #include <etl/optional.h>
    #include <etl/span.h>
    #include <etl/delegate.h>
#elif defined(NANO_ROS_EMBEDDED)
    // Minimal freestanding implementations
    #include <nano_ros/embedded/string_view.hpp>
    #include <nano_ros/embedded/span.hpp>
    #include <nano_ros/embedded/optional.hpp>
#else
    #include <string>
    #include <vector>
    #include <optional>
    #include <span>
    #include <functional>
#endif

namespace nano_ros {

// =============================================================================
// Platform-Abstracted Types
// =============================================================================

#if defined(NANO_ROS_USE_ETL)

    // String types
    template<size_t N = 64>
    using String = etl::string<N>;
    using StringView = etl::string_view;

    // Container types
    template<typename T, size_t N = 16>
    using Vector = etl::vector<T, N>;

    template<typename T>
    using Span = etl::span<T>;

    template<typename T>
    using Optional = etl::optional<T>;

    // Callback types
    template<typename Signature>
    using Function = etl::delegate<Signature>;

    // Error handling (no exceptions)
    using ErrorCode = int32_t;
    constexpr ErrorCode OK = 0;
    constexpr ErrorCode ERR_INVALID = -1;
    constexpr ErrorCode ERR_NO_MEMORY = -2;
    constexpr ErrorCode ERR_TIMEOUT = -3;
    constexpr ErrorCode ERR_NOT_FOUND = -4;

#elif defined(NANO_ROS_EMBEDDED)

    // Minimal freestanding string view
    using StringView = nano_ros::embedded::StringView;

    // Fixed-size string (stack allocated)
    template<size_t N = 64>
    using String = nano_ros::embedded::FixedString<N>;

    // Static vector (fixed capacity)
    template<typename T, size_t N = 16>
    using Vector = nano_ros::embedded::StaticVector<T, N>;

    template<typename T>
    using Span = nano_ros::embedded::Span<T>;

    template<typename T>
    using Optional = nano_ros::embedded::Optional<T>;

    // Function pointer based callbacks
    template<typename Signature>
    using Function = Signature*;

    // Error handling
    using ErrorCode = int32_t;
    constexpr ErrorCode OK = 0;
    constexpr ErrorCode ERR_INVALID = -1;
    constexpr ErrorCode ERR_NO_MEMORY = -2;
    constexpr ErrorCode ERR_TIMEOUT = -3;
    constexpr ErrorCode ERR_NOT_FOUND = -4;

#else  // Desktop with std

    using String = std::string;
    using StringView = std::string_view;

    template<typename T, size_t N = 0>
    using Vector = std::vector<T>;

    template<typename T>
    using Span = std::span<T>;

    template<typename T>
    using Optional = std::optional<T>;

    template<typename Signature>
    using Function = std::function<Signature>;

#endif

// =============================================================================
// Memory Management
// =============================================================================

#if defined(NANO_ROS_EMBEDDED)

    // Static/placement-new based ownership
    template<typename T>
    class UniquePtr {
    public:
        UniquePtr() : ptr_(nullptr) {}
        explicit UniquePtr(T* ptr) : ptr_(ptr) {}
        ~UniquePtr() { reset(); }

        // Move only
        UniquePtr(UniquePtr&& other) noexcept : ptr_(other.release()) {}
        UniquePtr& operator=(UniquePtr&& other) noexcept {
            reset(other.release());
            return *this;
        }

        UniquePtr(const UniquePtr&) = delete;
        UniquePtr& operator=(const UniquePtr&) = delete;

        T* get() const { return ptr_; }
        T* operator->() const { return ptr_; }
        T& operator*() const { return *ptr_; }
        explicit operator bool() const { return ptr_ != nullptr; }

        T* release() {
            T* tmp = ptr_;
            ptr_ = nullptr;
            return tmp;
        }

        void reset(T* ptr = nullptr) {
            if (ptr_) {
                ptr_->~T();
                // Note: Memory not freed - user manages static storage
            }
            ptr_ = ptr;
        }

    private:
        T* ptr_;
    };

#else
    template<typename T>
    using UniquePtr = std::unique_ptr<T>;
#endif

}  // namespace nano_ros
```

**Embedded CDR Implementation:**
```cpp
// include/nano_ros/embedded/cdr.hpp
#pragma once

#include <nano_ros/embedded/types.hpp>
#include <cstring>

namespace nano_ros {

/// CDR Writer for embedded systems (no exceptions, fixed buffer)
class EmbeddedCdrWriter {
public:
    /// Initialize with external buffer
    EmbeddedCdrWriter(uint8_t* buffer, size_t capacity)
        : buffer_(buffer), capacity_(capacity), pos_(0), error_(OK) {}

    // No copy/move (buffer is external)
    EmbeddedCdrWriter(const EmbeddedCdrWriter&) = delete;
    EmbeddedCdrWriter& operator=(const EmbeddedCdrWriter&) = delete;

    /// Check if an error occurred
    bool has_error() const { return error_ != OK; }
    ErrorCode error() const { return error_; }

    /// Get written data
    const uint8_t* data() const { return buffer_; }
    size_t size() const { return pos_; }
    size_t remaining() const { return capacity_ - pos_; }

    /// Write encapsulation header
    void write_encapsulation() {
        write_u8(0x00);  // CDR_LE
        write_u8(0x01);
        write_u8(0x00);  // Options
        write_u8(0x00);
    }

    // Primitive writers (check capacity, set error on overflow)
    void write_u8(uint8_t v) { write_raw(&v, 1); }
    void write_i8(int8_t v) { write_raw(&v, 1); }
    void write_u16(uint16_t v) { align(2); write_raw(&v, 2); }
    void write_i16(int16_t v) { align(2); write_raw(&v, 2); }
    void write_u32(uint32_t v) { align(4); write_raw(&v, 4); }
    void write_i32(int32_t v) { align(4); write_raw(&v, 4); }
    void write_u64(uint64_t v) { align(8); write_raw(&v, 8); }
    void write_i64(int64_t v) { align(8); write_raw(&v, 8); }
    void write_f32(float v) { align(4); write_raw(&v, 4); }
    void write_f64(double v) { align(8); write_raw(&v, 8); }
    void write_bool(bool v) { uint8_t b = v ? 1 : 0; write_raw(&b, 1); }

    void write_string(StringView s) {
        write_u32(static_cast<uint32_t>(s.size() + 1));
        write_raw(s.data(), s.size());
        write_u8(0);  // Null terminator
    }

    template<typename T, size_t N>
    void write_array(const T (&arr)[N]) {
        for (size_t i = 0; i < N; ++i) {
            write_value(arr[i]);
        }
    }

    template<typename T, size_t N>
    void write_sequence(const Vector<T, N>& vec) {
        write_u32(static_cast<uint32_t>(vec.size()));
        for (const auto& v : vec) {
            write_value(v);
        }
    }

private:
    void align(size_t alignment) {
        size_t padding = (alignment - (pos_ % alignment)) % alignment;
        if (pos_ + padding > capacity_) {
            error_ = ERR_NO_MEMORY;
            return;
        }
        while (padding--) buffer_[pos_++] = 0;
    }

    void write_raw(const void* data, size_t len) {
        if (pos_ + len > capacity_) {
            error_ = ERR_NO_MEMORY;
            return;
        }
        std::memcpy(buffer_ + pos_, data, len);
        pos_ += len;
    }

    template<typename T>
    void write_value(const T& v);  // Specialized for primitives

    uint8_t* buffer_;
    size_t capacity_;
    size_t pos_;
    ErrorCode error_;
};

/// CDR Reader for embedded systems
class EmbeddedCdrReader {
public:
    EmbeddedCdrReader(const uint8_t* data, size_t size)
        : data_(data), size_(size), pos_(0), error_(OK) {}

    bool has_error() const { return error_ != OK; }
    ErrorCode error() const { return error_; }
    size_t remaining() const { return size_ - pos_; }

    void read_encapsulation() {
        read_u8(); read_u8(); read_u8(); read_u8();
    }

    uint8_t read_u8() { return read_primitive<uint8_t>(1); }
    int8_t read_i8() { return read_primitive<int8_t>(1); }
    uint16_t read_u16() { align(2); return read_primitive<uint16_t>(2); }
    int16_t read_i16() { align(2); return read_primitive<int16_t>(2); }
    uint32_t read_u32() { align(4); return read_primitive<uint32_t>(4); }
    int32_t read_i32() { align(4); return read_primitive<int32_t>(4); }
    uint64_t read_u64() { align(8); return read_primitive<uint64_t>(8); }
    int64_t read_i64() { align(8); return read_primitive<int64_t>(8); }
    float read_f32() { align(4); return read_primitive<float>(4); }
    double read_f64() { align(8); return read_primitive<double>(8); }
    bool read_bool() { return read_u8() != 0; }

    template<size_t N>
    void read_string(String<N>& out) {
        uint32_t len = read_u32();
        if (len > 0) len--;  // Exclude null terminator
        if (len > N) {
            error_ = ERR_NO_MEMORY;
            return;
        }
        out.assign(reinterpret_cast<const char*>(data_ + pos_), len);
        pos_ += len + 1;  // Include null terminator
    }

    template<typename T, size_t N>
    void read_sequence(Vector<T, N>& out) {
        uint32_t len = read_u32();
        if (len > N) {
            error_ = ERR_NO_MEMORY;
            return;
        }
        out.clear();
        for (uint32_t i = 0; i < len; ++i) {
            T v;
            read_value(v);
            out.push_back(v);
        }
    }

private:
    void align(size_t alignment) {
        pos_ += (alignment - (pos_ % alignment)) % alignment;
    }

    template<typename T>
    T read_primitive(size_t size) {
        if (pos_ + size > size_) {
            error_ = ERR_INVALID;
            return T{};
        }
        T v;
        std::memcpy(&v, data_ + pos_, size);
        pos_ += size;
        return v;
    }

    template<typename T>
    void read_value(T& v);  // Specialized for primitives

    const uint8_t* data_;
    size_t size_;
    size_t pos_;
    ErrorCode error_;
};

}  // namespace nano_ros
```

**Embedded Node API:**
```cpp
// include/nano_ros/embedded/node.hpp
#pragma once

#include <nano_ros/embedded/types.hpp>
#include <nano_ros/qos.hpp>

namespace nano_ros {

// Forward declarations
class EmbeddedPublisher;
class EmbeddedSubscriber;

/// Configuration for static buffer sizes
struct EmbeddedNodeConfig {
    static constexpr size_t MAX_NAME_LENGTH = 64;
    static constexpr size_t MAX_PUBLISHERS = 8;
    static constexpr size_t MAX_SUBSCRIBERS = 8;
    static constexpr size_t MAX_TOPIC_LENGTH = 128;
    static constexpr size_t DEFAULT_BUFFER_SIZE = 512;
};

/// Embedded-compatible Node (no heap allocation)
class EmbeddedNode {
public:
    using PublisherHandle = int32_t;
    using SubscriberHandle = int32_t;

    /// Create node with static storage
    static ErrorCode create(
        EmbeddedNode* storage,
        StringView name,
        StringView namespace_ = ""
    );

    /// Accessors
    StringView name() const;
    StringView namespace_() const;

    /// Create publisher (returns handle or negative error)
    PublisherHandle create_publisher(
        StringView topic,
        StringView type_name,
        const QoS& qos
    );

    /// Create subscriber (returns handle or negative error)
    SubscriberHandle create_subscriber(
        StringView topic,
        StringView type_name,
        const QoS& qos
    );

    /// Publish raw data
    ErrorCode publish(PublisherHandle handle, const uint8_t* data, size_t len);

    /// Try to receive data (non-blocking)
    /// Returns number of bytes read, 0 if no data, negative on error
    int32_t try_receive(
        SubscriberHandle handle,
        uint8_t* buffer,
        size_t buffer_size
    );

private:
    EmbeddedNode() = default;

    String<EmbeddedNodeConfig::MAX_NAME_LENGTH> name_;
    String<EmbeddedNodeConfig::MAX_NAME_LENGTH> namespace_;

    // Opaque handle to Rust node
    void* rust_node_;
};

}  // namespace nano_ros
```

#### 10.10.4 Embedded Examples

**Zephyr C++ Example:**
```cpp
// examples/zephyr-cpp-talker/src/main.cpp
#include <nano_ros/embedded/nano_ros.hpp>
#include <zephyr/kernel.h>

// Static storage for node
static nano_ros::EmbeddedNode node_storage;
static uint8_t pub_buffer[256];

// Message structure (generated or hand-written)
struct Int32Msg {
    int32_t data;

    void serialize(nano_ros::EmbeddedCdrWriter& w) const {
        w.write_i32(data);
    }
};

int main() {
    printk("nano-ros C++ Zephyr Talker\n");

    // Initialize node
    auto err = nano_ros::EmbeddedNode::create(
        &node_storage,
        "zephyr_talker"
    );
    if (err != nano_ros::OK) {
        printk("Failed to create node: %d\n", err);
        return 1;
    }

    // Create publisher
    auto pub_handle = node_storage.create_publisher(
        "/chatter",
        "std_msgs/msg/Int32",
        nano_ros::QoS(10).best_effort()
    );
    if (pub_handle < 0) {
        printk("Failed to create publisher: %d\n", pub_handle);
        return 1;
    }

    Int32Msg msg{0};

    while (true) {
        // Serialize message
        nano_ros::EmbeddedCdrWriter writer(pub_buffer, sizeof(pub_buffer));
        writer.write_encapsulation();
        msg.serialize(writer);

        if (!writer.has_error()) {
            err = node_storage.publish(pub_handle, writer.data(), writer.size());
            if (err == nano_ros::OK) {
                printk("Published: %d\n", msg.data);
            }
        }

        msg.data++;
        k_msleep(1000);
    }

    return 0;
}
```

**Zephyr CMakeLists.txt:**
```cmake
# examples/zephyr-cpp-talker/CMakeLists.txt
cmake_minimum_required(VERSION 3.20)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(zephyr_cpp_talker)

# Enable C++17
set(CMAKE_CXX_STANDARD 17)

# Configure nano-ros-cpp for embedded
set(NANO_ROS_EMBEDDED ON)
set(NANO_ROS_RUST_TARGET "thumbv7em-none-eabihf")  # Adjust for your target
set(NANO_ROS_TRANSPORT "shim-zephyr")

# Add nano-ros-cpp
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../crates/nano-ros-cpp nano-ros-cpp)

target_sources(app PRIVATE src/main.cpp)
target_link_libraries(app PRIVATE nano_ros_cpp)
```

**STM32 Bare-Metal Example (with smoltcp):**
```cpp
// examples/stm32-cpp-talker/src/main.cpp
#include <nano_ros/embedded/nano_ros.hpp>

// Static allocations
alignas(nano_ros::EmbeddedNode) static uint8_t node_mem[sizeof(nano_ros::EmbeddedNode)];
static uint8_t tx_buffer[512];

extern "C" void app_main() {
    auto* node = reinterpret_cast<nano_ros::EmbeddedNode*>(node_mem);

    auto err = nano_ros::EmbeddedNode::create(node, "stm32_talker");
    if (err != nano_ros::OK) {
        // Handle error
        return;
    }

    auto pub = node->create_publisher(
        "/sensor_data",
        "std_msgs/msg/Int32",
        nano_ros::QoS(10).best_effort()
    );

    int32_t counter = 0;
    while (true) {
        nano_ros::EmbeddedCdrWriter writer(tx_buffer, sizeof(tx_buffer));
        writer.write_encapsulation();
        writer.write_i32(counter++);

        node->publish(pub, writer.data(), writer.size());

        // Platform-specific delay
        delay_ms(100);
    }
}
```

#### 10.10.5 Message Generation for Embedded

Update the message generator to support embedded mode:

```python
# scripts/nano_ros_generate_cpp.py additions

def generate_embedded_message(msg_name, package, fields):
    """Generate embedded-compatible message struct."""
    return f'''
#pragma once

#include <nano_ros/embedded/cdr.hpp>
#include <nano_ros/embedded/types.hpp>

namespace {package}::msg {{

struct {msg_name} {{
{generate_embedded_fields(fields)}

    void serialize(nano_ros::EmbeddedCdrWriter& writer) const {{
{generate_embedded_serialize(fields)}
    }}

    nano_ros::ErrorCode deserialize(nano_ros::EmbeddedCdrReader& reader) {{
{generate_embedded_deserialize(fields)}
        return reader.error();
    }}

    static constexpr const char* type_name() {{
        return "{package}/msg/{msg_name}";
    }}
}};

}}  // namespace {package}::msg
'''
```

### Acceptance Criteria

#### CMake Integration
- [ ] `NANO_ROS_EMBEDDED=ON` builds without std library
- [ ] `NANO_ROS_RUST_TARGET` enables cross-compilation
- [ ] `NANO_ROS_TRANSPORT` selects correct backend
- [ ] Toolchain files work for Zephyr, STM32, etc.

#### Freestanding C++ Headers
- [ ] All headers compile with `-fno-exceptions -fno-rtti -ffreestanding`
- [ ] No dynamic memory allocation in critical paths
- [ ] ETL integration works when `NANO_ROS_USE_ETL=ON`
- [ ] Error codes used instead of exceptions

#### Cargo Feature Gates
- [ ] `--no-default-features` disables std
- [ ] `--features alloc,shim-zephyr` works for Zephyr
- [ ] `--features alloc,shim-smoltcp` works for bare-metal

#### Examples
- [ ] Zephyr C++ talker/listener examples build and run
- [ ] STM32 bare-metal example compiles
- [ ] Examples demonstrate pub/sub functionality
- [ ] Memory usage documented

#### Wire Compatibility
- [ ] Embedded CDR is wire-compatible with desktop CDR
- [ ] Embedded nodes can communicate with desktop nodes
- [ ] Messages interoperate with ROS 2

---

## 10.11 C++ Action Support (Future)

**Status: PLANNED**

C++ action support depends on the Rust action implementation (Phase 6) being complete.

### Work Items

#### 10.11.1 Action API Design
- [ ] Design C++ `ActionServer<T>` class template
- [ ] Design C++ `ActionClient<T>` class template
- [ ] Define goal handle types (`ServerGoalHandle`, `ClientGoalHandle`)
- [ ] Define feedback and result callback signatures

#### 10.11.2 FFI Bridge
- [ ] Add action server FFI to cxx bridge
- [ ] Add action client FFI to cxx bridge
- [ ] Handle goal state machine across FFI boundary
- [ ] Support feedback streaming

#### 10.11.3 Examples
- [ ] Create `examples/native-cpp-action-server/` - Fibonacci action server
- [ ] Create `examples/native-cpp-action-client/` - Fibonacci action client
- [ ] Test interop with ROS 2 action clients/servers

### Dependencies
- Phase 6 (Rust Actions) must be complete
- Action message generation must be implemented

### Acceptance Criteria
- [ ] C++ action server can accept/reject goals
- [ ] C++ action client can send goals and receive feedback
- [ ] C++ actions interoperate with ROS 2 action clients/servers
- [ ] Examples demonstrate full action workflow

---

## Dependencies

| Dependency | Version | Purpose |
|------------|---------|---------|
| cxx | 1.0+ | Safe Rust/C++ FFI |
| cxx-build | 1.0+ | Build-time bridge generation |
| Corrosion | 0.5+ | CMake/Cargo integration |
| nano-ros | workspace | Core Rust implementation |
| Python | 3.8+ | Message generator script |
| clang-format | - | C++ code formatting |
| clang-tidy | - | C++ static analysis |
| ETL | 20.38+ | Embedded Template Library (optional) |

---

## Acceptance Criteria (Phase Complete)

### API Compatibility
- [x] C++ API matches rclcpp patterns for core functionality
- [x] Existing rclcpp code can migrate with minimal changes
- [x] Full type safety at compile time

### Interoperability
- [x] C++ nano-ros nodes communicate with Rust nano-ros nodes
- [x] C++ nano-ros nodes communicate with ROS 2 nodes
- [x] Works via zenoh transport

### Build System
- [x] CMake integration via `add_subdirectory()` (examples demonstrate this)
- [ ] CMake integration via `find_package(nano_ros_cpp)` (deferred - requires install target)
- [ ] Optional ament_cmake integration for ROS 2 workspaces (deferred)

### Embedded Support (NOT IMPLEMENTED - Deferred to C API Phase)
- [ ] ~~Freestanding C++ headers compile without std library~~
- [ ] ~~CMake options enable cross-compilation~~
- [ ] ~~Cargo feature gates control std/alloc/transport~~
- [ ] ~~Zephyr and bare-metal examples work~~
- [ ] ~~Memory usage meets embedded constraints~~

### Documentation
- [ ] API reference generated (deferred)
- [ ] Migration guide from rclcpp complete (deferred)
- [x] Examples for major use cases (pub/sub, services)

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
- [ETL (Embedded Template Library)](https://www.etlcpp.com/) - STL-like containers for embedded
- [Zephyr C++ Support](https://docs.zephyrproject.org/latest/develop/languages/cpp/) - Zephyr RTOS C++ integration

### C API References (for future Phase 11)

- [rclc](https://github.com/ros2/rclc) - Official ROS 2 C client library
- [micro-ROS rclc](https://github.com/micro-ROS/rclc) - micro-ROS C client library (development fork)
- [micro-ROS Client Library Introduction](https://micro.ros.org/docs/concepts/client_library/introduction/) - Overview of rcl + rclc architecture
- [micro-ROS Tutorials](https://micro.ros.org/docs/tutorials/programming_rcl_rclc/overview/) - C API usage tutorials
