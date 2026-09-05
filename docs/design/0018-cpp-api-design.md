---
rfc: 0018
title: "C++ API Design — nros-cpp"
status: Stable
since: 2026-03
last-reviewed: 2026-06
implements-tracked-by: []
supersedes: []
superseded-by: null
---

# C++ API Design — nros-cpp

## Goal

Provide a C++ API that mirrors rclcpp naming conventions while targeting
embedded systems with no heap allocation. The C++ layer wraps the Rust
`nros-node` core **directly via `extern "C"` FFI**, bypassing the C API
(`nros-c`) to preserve strong type safety through C++ templates.

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│  User code (C++)                                            │
│  nros::Node, nros::Publisher<M>, nros::Subscription<M>      │
├─────────────────────────────────────────────────────────────┤
│  nros-cpp  (header-only, freestanding C++)                  │
│  Thin wrappers: type safety, RAII, templates                │
│  Generated message types: std_msgs::msg::Int32, etc.        │
├─────────────────────────────────────────────────────────────┤
│  nros-cpp FFI  (Rust staticlib, extern "C" exports)          │
│  Typed FFI surface for C++ — not the C API                  │
├─────────────────────────────────────────────────────────────┤
│  nros-node (Rust core)                                      │
│  Executor<S, MAX_CBS, CB_ARENA>, Node, Publisher, etc.      │
└─────────────────────────────────────────────────────────────┘
```

### Why bypass the C API?

The existing `nros-c` API erases all type information into opaque handles
(`nros_publisher_t`, `nros_subscription_t`). Wrapping these in C++ templates
recovers type safety but cannot propagate it through the FFI boundary —
a `Publisher<Int32>` and a `Publisher<String>` both hold the same
`nros_publisher_t` handle, and nothing prevents mixing them at the FFI
level.

By having a dedicated Rust FFI layer in `nros-cpp`, we can:

1. **Carry type info through FFI**: The Rust side knows the concrete
   message type at compile time and can generate type-specific FFI
   functions (e.g., `nros_cpp_publish_std_msgs_Int32()`).
2. **Inline CDR serialization in Rust**: The C++ side passes a plain
   struct; Rust serializes it via the already-generated `RosMessage`
   impl. No duplicate serialization code in C++.
3. **Validate at the FFI boundary**: Rust can validate buffer sizes,
   message invariants, and session state before touching the transport.

### nros-cpp FFI Layer

The Rust FFI staticlib in `packages/api/nros-cpp/` that:

- Depends on `nros-node` and message crates (e.g., `nros-std-msgs`)
- Exports `extern "C"` functions for each message/service/action type
- Uses cbindgen to generate a C++ header (`nros_cpp_ffi.h`)
- Statically links into the final binary alongside `nros-node`

Example FFI surface:

```rust
// packages/api/nros-cpp/src/lib.rs (generated per message type)

#[repr(C)]
pub struct StdMsgsInt32 {
    pub data: i32,
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_cpp_publish_std_msgs_Int32(
    handle: *mut c_void,
    msg: *const StdMsgsInt32,
) -> i32 {
    // Deserialize C struct → Rust RosMessage, call publisher.publish()
}
```

The codegen tool generates both the C++ header struct and the Rust FFI
function for each message type, ensuring they stay in sync.

## Constraints

### Freestanding C++ (`-ffreestanding`)

The C++ API must work without the C++ standard library. This means:

- **No STL containers**: no `std::string`, `std::vector`, `std::map`,
  `std::shared_ptr`, `std::function`
- **No exceptions**: compile with `-fno-exceptions` (see Error Handling)
- **No RTTI**: compile with `-fno-rtti`
- **No `<iostream>`**: no `std::cout`
- **Available**: `<cstdint>`, `<cstddef>`, `<cstring>`, `<type_traits>`,
  templates, `constexpr`, placement new, move semantics

This is the C++ equivalent of Rust's `no_std`. The same binary runs on
bare-metal Cortex-M, Zephyr, FreeRTOS, NuttX, ThreadX, and Linux.

### Optional `std` mode

When `NROS_CPP_STD` is defined (or detected via `__STDC_HOSTED__`), the
API can optionally expose convenience overloads that accept `std::string`,
`std::function`, etc. These are `#ifdef`-guarded and never required.

## Error Handling

### No exceptions on embedded

C++ exceptions are **not available** on most RTOS and bare-metal targets:

- **Zephyr**: Compiles with `-fno-exceptions` by default. Enabling
  exceptions requires `CONFIG_EXCEPTIONS=y` + `CONFIG_CPP_EXCEPTIONS=y`,
  but not all architectures support it, and it adds significant code size.
- **FreeRTOS**: No built-in exception support. Toolchains for Cortex-M
  typically disable exceptions (`-fno-exceptions`).
- **NuttX**: Supports exceptions on some architectures, but not guaranteed.
- **Bare-metal**: Exception unwinding requires `libunwind` or equivalent,
  plus `.eh_frame` sections — often unavailable or too costly.

### Result-based error handling

nros-cpp uses a `Result` type for all fallible operations, matching the
Rust `Result<T, E>` pattern. This works universally on all platforms.

```cpp
namespace nros {

enum class ErrorCode : int32_t {
    Ok              = 0,
    Error           = -1,
    Timeout         = -2,
    InvalidArgument = -3,
    NotInitialized  = -4,
    Full            = -5,
    TransportError  = -100,
};

class Result {
public:
    constexpr Result(ErrorCode code) : code_(code) {}
    constexpr Result(int32_t raw) : code_(static_cast<ErrorCode>(raw)) {}

    bool ok() const { return code_ == ErrorCode::Ok; }
    explicit operator bool() const { return ok(); }
    ErrorCode code() const { return code_; }

    static constexpr Result success() { return Result(ErrorCode::Ok); }

private:
    ErrorCode code_;
};

// Macro for early return on error (replaces try/catch)
#define NROS_TRY(expr)              \
    do {                            \
        auto _r = (expr);          \
        if (!_r.ok()) return _r;   \
    } while (0)

} // namespace nros
```

Usage:

```cpp
nros::Result init_system() {
    NROS_TRY(nros::init());

    nros::Node node;
    NROS_TRY(nros::Node::create(node, "my_node"));

    nros::Publisher<std_msgs::msg::Int32> pub;
    NROS_TRY(node.create_publisher(pub, "/chatter"));

    return nros::Result::success();
}
```

### Optional exception mode

When `NROS_CPP_EXCEPTIONS` is defined **and** `-fno-exceptions` is NOT
active, methods can throw `nros::Exception` on error. This is strictly
opt-in and never the default.

## API Mapping: rclcpp vs nros-cpp

### Naming Convention

| rclcpp                     | nros-cpp                | Notes  |
|----------------------------|-------------------------|--------|
| `rclcpp::Node`             | `nros::Node`            |        |
| `rclcpp::Publisher<M>`     | `nros::Publisher<M>`    |        |
| `rclcpp::Subscription<M>`  | `nros::Subscription<M>` |        |
| `rclcpp::Service<S>`       | `nros::Service<S>`      | Server |
| `rclcpp::Client<S>`        | `nros::Client<S>`       | Client |
| `rclcpp_action::Server<A>` | `nros::ActionServer<A>` |        |
| `rclcpp_action::Client<A>` | `nros::ActionClient<A>` |        |
| `rclcpp::TimerBase`        | `nros::Timer`           |        |
| `rclcpp::Executor`         | `nros::Executor`        |        |
| `rclcpp::QoS`              | `nros::QoS`             |        |
| `rclcpp::GuardCondition`   | `nros::GuardCondition`  |        |
| `rclcpp::init()`           | `nros::init()`          |        |
| `rclcpp::spin()`           | `nros::spin()`          |        |
| `rclcpp::spin_some()`      | `nros::spin_some()`     |        |

### Key Differences from rclcpp

| rclcpp                                  | nros-cpp                           | Reason                        |
|-----------------------------------------|------------------------------------|-------------------------------|
| `std::shared_ptr<Publisher<M>>`         | `Publisher<M>` (value type)        | No heap; RAII move semantics  |
| `std::string topic_name`                | `const char* topic_name`           | No heap strings               |
| `std::function<void(M::SharedPtr)>`     | Function pointer + `void* context` | No `std::function` (heap)     |
| `std::shared_ptr<const M>` in callbacks | `const M&` in callbacks            | No shared ownership           |
| `rclcpp::QoS(KeepLast(10))`             | `nros::QoS::default_profile()`     | Compile-time profiles         |
| `std::chrono::duration`                 | `uint64_t period_ns`               | No `<chrono>` in freestanding |
| Template allocators                     | Fixed buffers (const generic)      | No allocator support          |
| `NodeOptions`                           | Template parameters                | Compile-time configuration    |
| Exceptions for errors                   | `Result` return values             | No exceptions on embedded     |

## Detailed API Design

### Initialization

```cpp
namespace nros {

// Initialize nros context (replaces rclcpp::init)
Result init(const char* locator = "tcp/127.0.0.1:7447",
            uint8_t domain_id = 0);

// Shutdown (replaces rclcpp::shutdown)
Result shutdown();

// Check if initialized (replaces rclcpp::ok)
bool ok();

} // namespace nros
```

**Wraps (Rust FFI):** `nros_cpp_init()` / `nros_cpp_fini()` — direct
calls into `nros-node` `Executor::open()` / `Executor::close()`.

### Node

Following the rclcpp pattern, entities are created through the Node.
The Node holds a reference to the underlying Rust executor session.

```cpp
namespace nros {

class Node {
public:
    // Create node (wraps Rust Executor::create_node)
    static Result create(Node& out, const char* name,
                         const char* ns = "");

    // Entity creation — mirrors rclcpp::Node methods
    template<typename M>
    Result create_publisher(Publisher<M>& out, const char* topic,
                            const QoS& qos = QoS::default_profile());

    template<typename M>
    Result create_subscription(Subscription<M>& out, const char* topic,
                               void (*callback)(const M&, void*),
                               void* context = nullptr,
                               const QoS& qos = QoS::default_profile());

    template<typename S>
    Result create_service(Service<S>& out, const char* name,
                          bool (*callback)(const typename S::Request&,
                                          typename S::Response&,
                                          void*),
                          void* context = nullptr);

    template<typename S>
    Result create_client(Client<S>& out, const char* name);

    template<typename A>
    Result create_action_server(ActionServer<A>& out, const char* name,
                                typename ActionServer<A>::Callbacks cbs);

    template<typename A>
    Result create_action_client(ActionClient<A>& out, const char* name);

    Result create_wall_timer(Timer& out, uint64_t period_ns,
                        void (*callback)(void*),
                        void* context = nullptr);

    // Parameter API
    Result declare_parameter(const char* name, const ParameterValue& default_val);
    bool get_parameter(const char* name, ParameterValue& out) const;
    bool get_parameter_bool(const char* name, bool& out) const;
    bool get_parameter_int(const char* name, int64_t& out) const;
    bool get_parameter_double(const char* name, double& out) const;
    bool get_parameter_string(const char* name, const char*& out) const;
    bool has_parameter(const char* name) const;

    const char* get_name() const;
    const char* get_namespace() const;

    ~Node();

    // Non-copyable, movable
    Node(Node&& other);
    Node& operator=(Node&& other);

private:
    Node(const Node&) = delete;
    Node& operator=(const Node&) = delete;
    void* rust_handle_; // Opaque pointer to Rust Node
};

} // namespace nros
```

### Publisher

```cpp
namespace nros {

template<typename M>
class Publisher {
public:
    // Publish a message — Rust side handles CDR serialization
    Result publish(const M& msg);

    const char* get_topic_name() const;
    bool is_valid() const;

    ~Publisher();

    Publisher(Publisher&& other);
    Publisher& operator=(Publisher&& other);

private:
    friend class Node;
    Publisher() = default;
    void* rust_handle_; // Opaque pointer to Rust Publisher
};

} // namespace nros
```

**FFI path:** `Publisher<M>::publish()` calls the generated
`nros_cpp_publish_<PackageName>_<MsgName>()` FFI function, which:
1. Copies the C++ `#[repr(C)]` struct fields into the Rust message type
2. Calls `Publisher::publish()` on the Rust side (CDR serialization)
3. Returns the error code

### Subscription

```cpp
namespace nros {

template<typename M>
class Subscription {
public:
    const char* get_topic_name() const;
    bool is_valid() const;

    ~Subscription();

    Subscription(Subscription&& other);
    Subscription& operator=(Subscription&& other);

private:
    friend class Node;
    Subscription() = default;
    void* rust_handle_; // Opaque pointer to Rust Subscription
};

} // namespace nros
```

The callback receives deserialized data. The Rust FFI function
deserializes CDR bytes into the `#[repr(C)]` struct and invokes the
C++ callback with `const M&`.

### Service Server

```cpp
namespace nros {

template<typename S>
class Service {
public:
    const char* get_service_name() const;
    bool is_valid() const;

    ~Service();

    Service(Service&& other);
    Service& operator=(Service&& other);

private:
    friend class Node;
    Service() = default;
    void* rust_handle_;
};

} // namespace nros
```

**Callback signature:**
```cpp
bool handler(const typename S::Request& req,
             typename S::Response& resp,
             void* context);
```

### Service Client

```cpp
namespace nros {

template<typename S>
class Client {
public:
    // Blocking call
    Result call(const typename S::Request& request,
                typename S::Response& response);

    const char* get_service_name() const;
    bool is_valid() const;

    ~Client();

    Client(Client&& other);
    Client& operator=(Client&& other);

private:
    friend class Node;
    Client() = default;
    void* rust_handle_;
};

} // namespace nros
```

### Action Server

```cpp
namespace nros {

template<typename A>
class ActionServer {
public:
    struct Callbacks {
        GoalResponse (*on_goal)(const GoalUUID&,
                                const typename A::Goal&,
                                void*);
        CancelResponse (*on_cancel)(GoalHandle<A>&, void*);
        void (*on_accepted)(GoalHandle<A>&, void*);
        void* context;
    };

    size_t get_active_goal_count() const;

    ~ActionServer();

    ActionServer(ActionServer&& other);
    ActionServer& operator=(ActionServer&& other);

private:
    friend class Node;
    ActionServer() = default;
    void* rust_handle_;
};

template<typename A>
class GoalHandle {
public:
    Result publish_feedback(const typename A::Feedback& feedback);
    Result succeed(const typename A::Result& result);
    Result abort(const typename A::Result& result);
    Result canceled(const typename A::Result& result);

    const GoalUUID& uuid() const;
    GoalStatus status() const;

private:
    void* rust_handle_;
};

} // namespace nros
```

### Action Client

```cpp
namespace nros {

template<typename A>
class ActionClient {
public:
    Result send_goal(const typename A::Goal& goal, GoalUUID& out_id);
    Result cancel_goal(const GoalUUID& goal_id);
    Result get_result(const GoalUUID& goal_id,
                      GoalStatus& status,
                      typename A::Result& result);

    void set_feedback_callback(
        void (*cb)(const GoalUUID&, const typename A::Feedback&, void*),
        void* context = nullptr);

    void set_result_callback(
        void (*cb)(const GoalUUID&, GoalStatus,
                   const typename A::Result&, void*),
        void* context = nullptr);

    ~ActionClient();

    ActionClient(ActionClient&& other);
    ActionClient& operator=(ActionClient&& other);

private:
    friend class Node;
    ActionClient() = default;
    void* rust_handle_;
};

} // namespace nros
```

### Timer

```cpp
namespace nros {

class Timer {
public:
    Result cancel();
    Result reset();
    bool is_ready(uint64_t current_time_ns) const;
    uint64_t get_period() const;

    ~Timer();

    Timer(Timer&& other);
    Timer& operator=(Timer&& other);

private:
    friend class Node;
    Timer() = default;
    void* rust_handle_;
};

} // namespace nros
```

### Executor

The Executor follows the rclcpp pattern: entities are created via
`Node`, then the node is added to the executor for spinning.

```cpp
namespace nros {

class Executor {
public:
    static Result create(Executor& out);

    // Add a node — registers all its entities for spinning
    Result add_node(Node& node);

    // Spin methods — match rclcpp naming
    Result spin();                              // Block forever
    Result spin_some(uint64_t timeout_ns = 0);  // Process available work
    Result spin_once(uint64_t timeout_ns);      // Process one batch
    Result spin_period(uint64_t period_ns);     // Fixed-rate loop
    Result stop();

    int get_handle_count() const;

    ~Executor();

    Executor(Executor&& other);
    Executor& operator=(Executor&& other);

private:
    Executor(const Executor&) = delete;
    Executor& operator=(const Executor&) = delete;
    void* rust_handle_;
};

// Free functions — match rclcpp::spin() etc.
Result spin(Executor& executor);
Result spin_some(Executor& executor, uint64_t timeout_ns = 0);

} // namespace nros
```

### QoS

```cpp
namespace nros {

class QoS {
public:
    constexpr QoS()
        : reliability_(Reliable), durability_(Volatile),
          history_(KeepLast), depth_(10) {}

    // Chainable setters — match rclcpp::QoS fluent API
    constexpr QoS& reliable()          { reliability_ = Reliable; return *this; }
    constexpr QoS& best_effort()       { reliability_ = BestEffort; return *this; }
    constexpr QoS& transient_local()   { durability_ = TransientLocal; return *this; }
    constexpr QoS& durability_volatile() { durability_ = Volatile; return *this; }
    constexpr QoS& keep_last(int depth) { history_ = KeepLast; depth_ = depth; return *this; }
    constexpr QoS& keep_all()          { history_ = KeepAll; return *this; }

    // Predefined profiles — match rclcpp named constructors
    static constexpr QoS default_profile() { return QoS(); }
    static constexpr QoS sensor_data()     { return QoS().best_effort().keep_last(5); }
    static constexpr QoS services()        { return QoS().reliable(); }

private:
    enum Reliability { Reliable, BestEffort } reliability_;
    enum Durability  { Volatile, TransientLocal } durability_;
    enum History     { KeepLast, KeepAll } history_;
    int depth_;
};

} // namespace nros
```

### Guard Condition

```cpp
namespace nros {

class GuardCondition {
public:
    static Result create(GuardCondition& out);

    Result trigger();
    bool is_triggered() const;
    Result clear();

    void set_callback(void (*callback)(void*), void* context = nullptr);

    ~GuardCondition();

    GuardCondition(GuardCondition&& other);
    GuardCondition& operator=(GuardCondition&& other);

private:
    void* rust_handle_;
};

} // namespace nros
```

## Generated Message Types

### ROS 2 namespace convention

Generated C++ types use standard ROS 2 namespaces:
`<package_name>::<interface_type>::<TypeName>`. This is the convention
used by rclcpp and all standard ROS 2 tooling:

- Messages: `std_msgs::msg::Int32`, `geometry_msgs::msg::Twist`
- Services: `example_interfaces::srv::AddTwoInts`
- Actions: `example_interfaces::action::Fibonacci`

This ensures maximum compatibility with existing ROS 2 C++ code. Users
migrating from rclcpp can keep the same `#include` paths and type names.

### Generated structure

The codegen tool produces two artifacts per message type:

1. **C++ header** (`.hpp`): `#[repr(C)]`-compatible struct + type traits
2. **Rust FFI glue** (`.rs`): Conversion between C++ struct and Rust
   `RosMessage` type + `extern "C"` functions

```cpp
// Generated: std_msgs/msg/int32.hpp
#pragma once

#include <cstdint>
#include <cstddef>

namespace std_msgs {
namespace msg {

struct Int32 {
    int32_t data;

    // Type metadata (used by Publisher/Subscription templates)
    static constexpr const char* TYPE_NAME =
        "std_msgs::msg::dds_::Int32_";
    static constexpr const char* TYPE_HASH =
        "TypeHashNotSupported";
    static constexpr size_t SERIALIZED_SIZE_MAX = 8;
};

} // namespace msg
} // namespace std_msgs
```

Service types:
```cpp
// Generated: example_interfaces/srv/add_two_ints.hpp
namespace example_interfaces {
namespace srv {

struct AddTwoInts {
    struct Request {
        int64_t a;
        int64_t b;
        static constexpr size_t SERIALIZED_SIZE_MAX = 20;
    };

    struct Response {
        int64_t sum;
        static constexpr size_t SERIALIZED_SIZE_MAX = 12;
    };

    static constexpr const char* SERVICE_NAME =
        "example_interfaces::srv::dds_::AddTwoInts_";
};

} // namespace srv
} // namespace example_interfaces
```

Action types:
```cpp
// Generated: example_interfaces/action/fibonacci.hpp
namespace example_interfaces {
namespace action {

struct Fibonacci {
    struct Goal {
        int32_t order;
        static constexpr size_t SERIALIZED_SIZE_MAX = 8;
    };

    struct Result {
        // Fixed-size array for no_std; size known at codegen time
        int32_t sequence[64];
        uint32_t sequence_len;
        static constexpr size_t SERIALIZED_SIZE_MAX = 264;
    };

    struct Feedback {
        int32_t partial_sequence[64];
        uint32_t partial_sequence_len;
        static constexpr size_t SERIALIZED_SIZE_MAX = 264;
    };
};

} // namespace action
} // namespace example_interfaces
```

**Note:** Nested `namespace a::b` syntax (C++17) is avoided for C++14
compatibility; nested `namespace a { namespace b { ... } }` is used
instead.

### CMake code generation

Message bindings are generated via CMake, using the same pipeline as
the existing C code generation. The `nros_find_interfaces()` function
takes a `LANGUAGE CPP` option and reads dependent packages from
`package.xml` `<depend>` rows (no explicit PACKAGES arg — `package.xml`
is the SSoT):

```cmake
# In the example's CMakeLists.txt:
nros_find_interfaces(LANGUAGE CPP)  # reads <depend> rows from package.xml

# Generates one `<pkg>__nano_ros_cpp` target per resolved <depend> row.
# Includes: ${CMAKE_CURRENT_BINARY_DIR}/generated/cpp/
# Links:    Rust FFI static library with type-specific functions
```

The CMake function:

1. Resolves `.msg`/`.srv`/`.action` files (local → ament → bundled)
2. Runs `nros-codegen --language cpp` to generate `.hpp` headers and
   `.rs` FFI glue files
3. Compiles the Rust FFI glue into a static library via Corrosion
4. Creates a CMake target that provides the include path and links
   the FFI library

This mirrors the existing `LANGUAGE C` pipeline in
`NanoRosGenerateInterfaces.cmake`, with a parallel `cpp` output
directory.

## Usage Example

```cpp
#include <nros/nros.hpp>
#include <std_msgs/msg/int32.hpp>

struct TalkerApp {
    nros::Publisher<std_msgs::msg::Int32>* pub;
    int count;
};

void timer_callback(void* ctx) {
    auto* app = static_cast<TalkerApp*>(ctx);
    std_msgs::msg::Int32 msg;
    msg.data = app->count++;
    app->pub->publish(msg);
}

int main() {
    NROS_TRY(nros::init());

    nros::Node node;
    NROS_TRY(nros::Node::create(node, "talker"));

    nros::Publisher<std_msgs::msg::Int32> pub;
    NROS_TRY(node.create_publisher(pub, "/chatter"));

    TalkerApp app{&pub, 0};

    nros::Timer timer;
    NROS_TRY(node.create_wall_timer(timer, 1000000000ULL, timer_callback, &app));

    nros::Executor executor;
    NROS_TRY(nros::Executor::create(executor));
    NROS_TRY(executor.add_node(node));
    executor.spin();

    nros::shutdown();
    return 0;
}
```

**Note on callbacks:** Freestanding C++ does not have `std::function`, so
callbacks are function pointers with a `void* context`. Lambdas work if
they are non-capturing (implicitly convertible to function pointers) or
the context pointer is used to pass state. This matches standard practice
in embedded C++ (AUTOSAR AP, POSIX callbacks).

For `std` mode, convenience overloads accepting `std::function` can be
provided behind `#ifdef NROS_CPP_STD`.

## Callback Pattern Comparison

### rclcpp (heap-allocated)
```cpp
auto sub = node->create_subscription<Int32>("/topic", 10,
    [this](Int32::SharedPtr msg) { process(msg->data); });
```

### nros-cpp (freestanding)
```cpp
nros::Subscription<Int32> sub;
node.create_subscription(sub, "/topic",
    [](const Int32& msg, void* ctx) {
        static_cast<MyApp*>(ctx)->process(msg.data);
    }, this);
```

### nros-cpp (std mode, optional)
```cpp
#ifdef NROS_CPP_STD
nros::Subscription<Int32> sub;
node.create_subscription(sub, "/topic",
    std::function<void(const Int32&)>([this](const Int32& msg) {
        process(msg.data);
    }));
#endif
```

## Implementation

See [docs/roadmap/phase-66-cpp-api.md](../roadmap/archived/phase-66-cpp-api.md) for
work items, acceptance criteria, and implementation schedule.

## Files

```
packages/api/nros-cpp/
├── Cargo.toml              # Rust staticlib (depends on nros-node + message crates)
├── src/
│   ├── lib.rs              # Core FFI exports (init, node, executor)
│   └── generated/          # Per-message FFI glue (from codegen)
├── build.rs                # cbindgen → nros_cpp_ffi.h + config header
├── cbindgen.toml           # cbindgen configuration
├── include/nros/
│   ├── nros.hpp              # Umbrella header
│   ├── result.hpp            # Result type + NROS_TRY macro
│   ├── qos.hpp               # QoS profiles
│   ├── node.hpp              # Node class
│   ├── publisher.hpp         # Publisher<M>
│   ├── subscription.hpp      # Subscription<M>
│   ├── service.hpp           # Service<S>
│   ├── client.hpp            # Client<S>
│   ├── action_server.hpp     # ActionServer<A>, GoalHandle<A>
│   ├── action_client.hpp     # ActionClient<A>
│   ├── timer.hpp             # Timer
│   ├── executor.hpp          # Executor
│   ├── guard_condition.hpp   # GuardCondition
│   └── parameter.hpp         # ParameterValue
└── CMakeLists.txt            # Header-only library target
```

## References

- Build/link determinism for C/C++ (one canonical `<nros/platform.h>`, board
  `[board.capabilities]` → `-DNROS_PLATFORM_HAS_*`, manifest-driven RMW link):
  [RFC-0042](0042-platform-build-determinism.md) +
  [c-api-cmake.md](../reference/c-api-cmake.md) "Board capabilities & deterministic
  build".
- rclcpp API: https://docs.ros.org/en/rolling/p/rclcpp/
- rclcpp_action API: https://docs.ros.org/en/rolling/p/rclcpp_action/
- Existing nros Rust API: `packages/core/nros-node/src/executor/`
- Existing nros C API: `packages/api/nros-c/`
- C codegen pipeline: `packages/codegen/packages/nros-codegen-c/cmake/NanoRosGenerateInterfaces.cmake`
- C message template: `packages/codegen/packages/rosidl-codegen/templates/message_c.h.jinja`
