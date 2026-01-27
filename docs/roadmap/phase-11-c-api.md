# Phase 11: C API (rclc-Compatible for Embedded Systems)

**Status: NOT STARTED**

## Executive Summary

Phase 11 implements a C API for nano-ros, providing an rclc-compatible interface for embedded systems.
The API is built on top of the existing Rust implementation via FFI, using zenoh-pico as the transport
layer. This enables C developers to use nano-ros with familiar ROS 2 patterns on resource-constrained
microcontrollers.

**Goals:**
1. Provide an rclc-compatible C API for nano-ros
2. Support static memory allocation (no runtime malloc)
3. Implement a deterministic executor for real-time applications
4. Enable ROS 2 interoperability via rmw_zenoh protocol
5. Target Zephyr RTOS and bare-metal embedded systems

**Non-Goals:**
- Full rclc API compatibility (only core features)
- DDS/XRCE-DDS support (use zenoh-pico instead)
- Dynamic memory allocation at runtime

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     User Application (C)                        │
├─────────────────────────────────────────────────────────────────┤
│                      nano_ros.h (C API)                         │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐           │
│  │  Node    │ │Publisher │ │Subscriber│ │ Executor │           │
│  └──────────┘ └──────────┘ └──────────┘ └──────────┘           │
├─────────────────────────────────────────────────────────────────┤
│                    nano_ros_impl.rs (Rust FFI)                  │
│  - Thin wrapper exposing Rust types to C                        │
│  - cbindgen-generated headers                                   │
├─────────────────────────────────────────────────────────────────┤
│                      nano-ros (Rust core)                       │
│  - nano-ros-node, nano-ros-transport, nano-ros-core             │
├─────────────────────────────────────────────────────────────────┤
│                      zenoh-pico-shim                            │
│  - Safe Rust wrapper for zenoh-pico                             │
├─────────────────────────────────────────────────────────────────┤
│                        zenoh-pico (C)                           │
│  - Lightweight zenoh client for embedded                        │
└─────────────────────────────────────────────────────────────────┘
```

## API Design

### Core Types

```c
// Support context (similar to rclc_support_t)
typedef struct nano_ros_support_t nano_ros_support_t;

// Node handle
typedef struct nano_ros_node_t nano_ros_node_t;

// Publisher/Subscriber handles
typedef struct nano_ros_publisher_t nano_ros_publisher_t;
typedef struct nano_ros_subscription_t nano_ros_subscription_t;

// Service handles
typedef struct nano_ros_client_t nano_ros_client_t;
typedef struct nano_ros_service_t nano_ros_service_t;

// Timer handle
typedef struct nano_ros_timer_t nano_ros_timer_t;

// Executor
typedef struct nano_ros_executor_t nano_ros_executor_t;

// Return codes (compatible with rcl_ret_t)
typedef int32_t nano_ros_ret_t;
#define NANO_ROS_RET_OK 0
#define NANO_ROS_RET_ERROR -1
#define NANO_ROS_RET_TIMEOUT -2
#define NANO_ROS_RET_INVALID_ARGUMENT -3
```

### Initialization API

```c
// Initialize support context
nano_ros_ret_t nano_ros_support_init(
    nano_ros_support_t *support,
    const char *locator,           // e.g., "tcp/192.168.1.1:7447"
    uint8_t domain_id);

// Finalize support context
nano_ros_ret_t nano_ros_support_fini(nano_ros_support_t *support);

// Create node
nano_ros_ret_t nano_ros_node_init(
    nano_ros_node_t *node,
    nano_ros_support_t *support,
    const char *name,
    const char *namespace_);

// Finalize node
nano_ros_ret_t nano_ros_node_fini(nano_ros_node_t *node);
```

### Publisher API

```c
// Message type info (generated per message type)
typedef struct {
    const char *type_name;      // e.g., "std_msgs::msg::dds_::Int32"
    const char *type_hash;      // RIHS hash
    size_t serialized_size_max; // Max serialized size (0 = dynamic)
} nano_ros_message_type_t;

// Create publisher
nano_ros_ret_t nano_ros_publisher_init(
    nano_ros_publisher_t *publisher,
    nano_ros_node_t *node,
    const nano_ros_message_type_t *type,
    const char *topic_name);

// Create publisher with QoS
nano_ros_ret_t nano_ros_publisher_init_with_qos(
    nano_ros_publisher_t *publisher,
    nano_ros_node_t *node,
    const nano_ros_message_type_t *type,
    const char *topic_name,
    const nano_ros_qos_t *qos);

// Publish raw CDR data
nano_ros_ret_t nano_ros_publish_raw(
    nano_ros_publisher_t *publisher,
    const uint8_t *data,
    size_t len);

// Finalize publisher
nano_ros_ret_t nano_ros_publisher_fini(nano_ros_publisher_t *publisher);
```

### Subscription API

```c
// Subscription callback type
typedef void (*nano_ros_subscription_callback_t)(
    const uint8_t *data,
    size_t len,
    void *context);

// Create subscription
nano_ros_ret_t nano_ros_subscription_init(
    nano_ros_subscription_t *subscription,
    nano_ros_node_t *node,
    const nano_ros_message_type_t *type,
    const char *topic_name,
    nano_ros_subscription_callback_t callback,
    void *context);

// Finalize subscription
nano_ros_ret_t nano_ros_subscription_fini(nano_ros_subscription_t *subscription);
```

### Service API

```c
// Service type info
typedef struct {
    const char *service_name;   // e.g., "example_interfaces::srv::dds_::AddTwoInts"
    const char *type_hash;
    size_t request_size_max;
    size_t response_size_max;
} nano_ros_service_type_t;

// Service callback type
typedef void (*nano_ros_service_callback_t)(
    const uint8_t *request,
    size_t request_len,
    uint8_t *response,
    size_t *response_len,
    void *context);

// Create service server
nano_ros_ret_t nano_ros_service_init(
    nano_ros_service_t *service,
    nano_ros_node_t *node,
    const nano_ros_service_type_t *type,
    const char *service_name,
    nano_ros_service_callback_t callback,
    void *context);

// Create service client
nano_ros_ret_t nano_ros_client_init(
    nano_ros_client_t *client,
    nano_ros_node_t *node,
    const nano_ros_service_type_t *type,
    const char *service_name);

// Call service (blocking)
nano_ros_ret_t nano_ros_client_call(
    nano_ros_client_t *client,
    const uint8_t *request,
    size_t request_len,
    uint8_t *response,
    size_t response_max_len,
    size_t *response_len,
    uint32_t timeout_ms);

// Finalize
nano_ros_ret_t nano_ros_service_fini(nano_ros_service_t *service);
nano_ros_ret_t nano_ros_client_fini(nano_ros_client_t *client);
```

### Timer API

```c
// Timer callback type
typedef void (*nano_ros_timer_callback_t)(nano_ros_timer_t *timer, void *context);

// Create timer
nano_ros_ret_t nano_ros_timer_init(
    nano_ros_timer_t *timer,
    nano_ros_support_t *support,
    uint64_t period_ns,
    nano_ros_timer_callback_t callback,
    void *context);

// Cancel timer
nano_ros_ret_t nano_ros_timer_cancel(nano_ros_timer_t *timer);

// Reset timer
nano_ros_ret_t nano_ros_timer_reset(nano_ros_timer_t *timer);

// Finalize timer
nano_ros_ret_t nano_ros_timer_fini(nano_ros_timer_t *timer);
```

### Executor API

```c
// Callback invocation mode
typedef enum {
    NANO_ROS_ON_NEW_DATA,  // Call only when new data available
    NANO_ROS_ALWAYS        // Always call (even with NULL data)
} nano_ros_executor_invocation_t;

// Initialize executor with fixed handle capacity
nano_ros_ret_t nano_ros_executor_init(
    nano_ros_executor_t *executor,
    nano_ros_support_t *support,
    size_t max_handles);

// Add subscription to executor
nano_ros_ret_t nano_ros_executor_add_subscription(
    nano_ros_executor_t *executor,
    nano_ros_subscription_t *subscription,
    nano_ros_executor_invocation_t invocation);

// Add timer to executor
nano_ros_ret_t nano_ros_executor_add_timer(
    nano_ros_executor_t *executor,
    nano_ros_timer_t *timer);

// Add service to executor
nano_ros_ret_t nano_ros_executor_add_service(
    nano_ros_executor_t *executor,
    nano_ros_service_t *service);

// Spin once (process pending callbacks)
nano_ros_ret_t nano_ros_executor_spin_some(
    nano_ros_executor_t *executor,
    uint64_t timeout_ns);

// Spin forever
nano_ros_ret_t nano_ros_executor_spin(nano_ros_executor_t *executor);

// Spin with period
nano_ros_ret_t nano_ros_executor_spin_period(
    nano_ros_executor_t *executor,
    uint64_t period_ns);

// Finalize executor
nano_ros_ret_t nano_ros_executor_fini(nano_ros_executor_t *executor);
```

### QoS Settings

```c
typedef enum {
    NANO_ROS_QOS_RELIABILITY_BEST_EFFORT,
    NANO_ROS_QOS_RELIABILITY_RELIABLE
} nano_ros_qos_reliability_t;

typedef enum {
    NANO_ROS_QOS_DURABILITY_VOLATILE,
    NANO_ROS_QOS_DURABILITY_TRANSIENT_LOCAL
} nano_ros_qos_durability_t;

typedef enum {
    NANO_ROS_QOS_HISTORY_KEEP_LAST,
    NANO_ROS_QOS_HISTORY_KEEP_ALL
} nano_ros_qos_history_t;

typedef struct {
    nano_ros_qos_reliability_t reliability;
    nano_ros_qos_durability_t durability;
    nano_ros_qos_history_t history;
    size_t depth;
} nano_ros_qos_t;

// Predefined QoS profiles
extern const nano_ros_qos_t NANO_ROS_QOS_DEFAULT;
extern const nano_ros_qos_t NANO_ROS_QOS_SENSOR_DATA;
extern const nano_ros_qos_t NANO_ROS_QOS_SERVICES;
```

## Message Type Generation

Messages are generated using the existing `cargo nano-ros generate` tool with C output support.

### Generated Header Example

```c
// std_msgs/msg/Int32.h (generated)
#ifndef STD_MSGS__MSG__INT32_H_
#define STD_MSGS__MSG__INT32_H_

#include <nano_ros.h>

typedef struct {
    int32_t data;
} std_msgs__msg__Int32;

// Type info for API
extern const nano_ros_message_type_t std_msgs__msg__Int32__type;

// Serialization functions
size_t std_msgs__msg__Int32__serialize(
    const std_msgs__msg__Int32 *msg,
    uint8_t *buffer,
    size_t buffer_size);

bool std_msgs__msg__Int32__deserialize(
    std_msgs__msg__Int32 *msg,
    const uint8_t *buffer,
    size_t buffer_size);

#endif
```

### Usage Example

```c
#include <nano_ros.h>
#include <std_msgs/msg/Int32.h>

// Callback for subscription
void on_message(const uint8_t *data, size_t len, void *ctx) {
    std_msgs__msg__Int32 msg;
    if (std_msgs__msg__Int32__deserialize(&msg, data, len)) {
        printf("Received: %d\n", msg.data);
    }
}

int main(void) {
    nano_ros_support_t support;
    nano_ros_node_t node;
    nano_ros_publisher_t pub;
    nano_ros_subscription_t sub;
    nano_ros_executor_t executor;

    // Initialize
    nano_ros_support_init(&support, "tcp/127.0.0.1:7447", 0);
    nano_ros_node_init(&node, &support, "my_node", "/");

    // Create publisher
    nano_ros_publisher_init(&pub, &node,
        &std_msgs__msg__Int32__type, "/chatter");

    // Create subscription
    nano_ros_subscription_init(&sub, &node,
        &std_msgs__msg__Int32__type, "/chatter",
        on_message, NULL);

    // Setup executor
    nano_ros_executor_init(&executor, &support, 2);
    nano_ros_executor_add_subscription(&executor, &sub, NANO_ROS_ON_NEW_DATA);

    // Publish a message
    std_msgs__msg__Int32 msg = { .data = 42 };
    uint8_t buffer[64];
    size_t len = std_msgs__msg__Int32__serialize(&msg, buffer, sizeof(buffer));
    nano_ros_publish_raw(&pub, buffer, len);

    // Spin
    nano_ros_executor_spin(&executor);

    // Cleanup
    nano_ros_executor_fini(&executor);
    nano_ros_subscription_fini(&sub);
    nano_ros_publisher_fini(&pub);
    nano_ros_node_fini(&node);
    nano_ros_support_fini(&support);

    return 0;
}
```

## Implementation Phases

### 11.1 Project Setup

**Status: NOT STARTED**

#### Work Items
- [ ] Create `crates/nano-ros-c/` crate structure
- [ ] Setup cbindgen for header generation
- [ ] Create CMake build integration
- [ ] Setup cross-compilation for ARM targets

#### Deliverables
- `crates/nano-ros-c/Cargo.toml`
- `crates/nano-ros-c/src/lib.rs`
- `crates/nano-ros-c/include/nano_ros.h` (generated)
- `crates/nano-ros-c/CMakeLists.txt`

### 11.2 Core Types and Initialization

**Status: NOT STARTED**

#### Work Items
- [ ] Implement `nano_ros_support_t` wrapping zenoh session
- [ ] Implement `nano_ros_node_t` wrapping Rust node
- [ ] Implement return codes and error handling
- [ ] Add QoS types and predefined profiles

#### API Coverage
- `nano_ros_support_init()` / `nano_ros_support_fini()`
- `nano_ros_node_init()` / `nano_ros_node_fini()`

### 11.3 Publisher

**Status: NOT STARTED**

#### Work Items
- [ ] Implement `nano_ros_publisher_t`
- [ ] Implement raw publish function
- [ ] Add QoS support
- [ ] Add liveliness token for ROS 2 discovery

#### API Coverage
- `nano_ros_publisher_init()` / `nano_ros_publisher_init_with_qos()`
- `nano_ros_publish_raw()`
- `nano_ros_publisher_fini()`

### 11.4 Subscription

**Status: NOT STARTED**

#### Work Items
- [ ] Implement `nano_ros_subscription_t`
- [ ] Implement callback dispatch
- [ ] Add QoS support
- [ ] Handle rmw_zenoh attachment parsing

#### API Coverage
- `nano_ros_subscription_init()`
- `nano_ros_subscription_fini()`

### 11.5 Executor

**Status: NOT STARTED**

#### Work Items
- [ ] Implement `nano_ros_executor_t` with static handle array
- [ ] Implement `spin_some()` with timeout
- [ ] Implement `spin()` infinite loop
- [ ] Implement `spin_period()` for periodic execution
- [ ] Add subscription and timer handle management

#### API Coverage
- `nano_ros_executor_init()` / `nano_ros_executor_fini()`
- `nano_ros_executor_add_subscription()`
- `nano_ros_executor_add_timer()`
- `nano_ros_executor_spin_some()` / `spin()` / `spin_period()`

### 11.6 Timer

**Status: NOT STARTED**

#### Work Items
- [ ] Implement `nano_ros_timer_t`
- [ ] Integrate with executor
- [ ] Support cancel and reset operations

#### API Coverage
- `nano_ros_timer_init()` / `nano_ros_timer_fini()`
- `nano_ros_timer_cancel()` / `nano_ros_timer_reset()`

### 11.7 Services

**Status: NOT STARTED**

#### Work Items
- [ ] Implement `nano_ros_service_t` (server)
- [ ] Implement `nano_ros_client_t` (client)
- [ ] Implement blocking service call
- [ ] Integrate service server with executor

#### API Coverage
- `nano_ros_service_init()` / `nano_ros_service_fini()`
- `nano_ros_client_init()` / `nano_ros_client_fini()`
- `nano_ros_client_call()`
- `nano_ros_executor_add_service()`

### 11.8 Message Generation

**Status: NOT STARTED**

#### Work Items
- [ ] Extend `cargo nano-ros generate` for C output
- [ ] Generate C struct definitions
- [ ] Generate serialization/deserialization functions
- [ ] Generate type info constants
- [ ] Support nested types and arrays

#### Deliverables
- Updated `colcon-nano-ros/` with C generation support
- Generated headers for `std_msgs`, `builtin_interfaces`
- Generated headers for `example_interfaces` services

### 11.9 Examples

**Status: NOT STARTED**

#### Work Items
- [ ] Create `examples/c-talker/` - Publisher example
- [ ] Create `examples/c-listener/` - Subscriber example
- [ ] Create `examples/c-service/` - Service server/client example
- [ ] Create Zephyr examples

#### Deliverables
- Working C examples with CMake build
- Zephyr project examples

### 11.10 Zephyr Integration

**Status: NOT STARTED**

#### Work Items
- [ ] Create Zephyr module structure
- [ ] CMake integration for Zephyr builds
- [ ] Test on native_sim
- [ ] Test on real hardware (STM32, nRF, etc.)

#### Deliverables
- Zephyr west module configuration
- Zephyr Kconfig options
- Zephyr sample applications

## Memory Model

### Static Allocation

All handles use static storage provided by the user:

```c
// User provides storage
static nano_ros_support_t support;
static nano_ros_node_t node;
static nano_ros_publisher_t pub;
static nano_ros_subscription_t sub;
static nano_ros_executor_t executor;

// Executor handle array (user-provided)
#define MAX_HANDLES 8
static nano_ros_executor_handle_t handles[MAX_HANDLES];
```

### Memory Requirements

| Component | RAM (bytes) | Notes |
|-----------|-------------|-------|
| Support context | ~64 | Zenoh session reference |
| Node | ~128 | Name, namespace, liveliness |
| Publisher | ~64 | Topic, zenoh publisher |
| Subscription | ~80 | Topic, callback, context |
| Timer | ~48 | Period, callback, state |
| Executor | ~32 + 16*N | Base + per-handle overhead |

Estimated total for minimal system (1 node, 2 pub, 2 sub, executor): **~600 bytes RAM**

## Build Integration

### CMake

```cmake
# Find nano-ros-c
find_package(nano_ros_c REQUIRED)

# Create executable
add_executable(my_app main.c)
target_link_libraries(my_app nano_ros_c::nano_ros_c)

# Generate messages
nano_ros_generate_messages(my_app
    PACKAGES std_msgs example_interfaces
    LANGUAGE C)
```

### Zephyr

```cmake
# In CMakeLists.txt
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/modules/nano-ros-c)

# In prj.conf
CONFIG_NANO_ROS=y
CONFIG_NANO_ROS_MAX_NODES=1
CONFIG_NANO_ROS_MAX_PUBLISHERS=4
CONFIG_NANO_ROS_MAX_SUBSCRIPTIONS=4
```

## Testing Strategy

### Unit Tests
- C API function tests using Unity framework
- Memory leak detection with sanitizers
- Static analysis with cppcheck

### Integration Tests
- C ↔ Rust interop tests
- C ↔ ROS 2 (rmw_zenoh) interop tests
- Zephyr native_sim tests

### Hardware Tests
- STM32F4 Discovery board
- Nordic nRF52840 DK
- Raspberry Pi Pico

## Dependencies

### Required
- Rust toolchain (for building nano-ros core)
- CMake 3.16+
- C11 compiler
- zenoh-pico (included via submodule)

### Optional
- Zephyr SDK (for Zephyr targets)
- ARM GCC toolchain (for embedded targets)

## References

- [rclc](https://github.com/ros2/rclc) - Official ROS 2 C API
- [Pico-ROS](https://github.com/Pico-ROS/Pico-ROS-software) - zenoh-pico based ROS client
- [micro-ROS](https://micro.ros.org/) - Embedded ROS 2 framework
- [cbindgen](https://github.com/mozilla/cbindgen) - Rust to C header generator

## Acceptance Criteria

### API Compatibility
- [ ] Core API matches rclc patterns
- [ ] Easy migration for rclc users
- [ ] Full type safety with message generation

### Interoperability
- [ ] C nano-ros nodes communicate with Rust nano-ros nodes
- [ ] C nano-ros nodes communicate with ROS 2 nodes (via rmw_zenoh)
- [ ] Works via zenoh transport

### Embedded Support
- [ ] Builds for Zephyr targets
- [ ] Runs on native_sim
- [ ] Runs on real embedded hardware
- [ ] Static memory allocation (no malloc at runtime)
- [ ] Memory footprint < 2KB RAM for minimal system

### Documentation
- [ ] API reference documentation
- [ ] Usage examples
- [ ] Zephyr integration guide
