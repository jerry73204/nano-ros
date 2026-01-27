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
| Python | 3.8+ | Message generator script |
| clang-format | - | C++ code formatting |
| clang-tidy | - | C++ static analysis |

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
