# C++ API Guide

The nros C++ API (`nros-cpp`) provides a freestanding C++14 interface for embedded ROS 2 development. It mirrors rclcpp naming conventions while requiring no standard library, exceptions, or RTTI — making it suitable for Zephyr, FreeRTOS, NuttX, ThreadX, and bare-metal targets.

## Overview

- **Freestanding C++14** — no STL dependency in default mode
- **Direct Rust FFI** — wraps `nros-node` directly via typed `extern "C"` FFI (not the C API), preserving type safety per message type
- **rclcpp naming** — `Node`, `Publisher<M>`, `Subscription<M>`, `Service<S>`, `Client<S>`, `ActionServer<A>`, `ActionClient<A>`, `Timer`, `GuardCondition`, `Executor`
- **Result-based error handling** — `nros::Result` + `NROS_TRY` macro (no exceptions)
- **Generated message types** — `std_msgs::msg::Int32`, `example_interfaces::srv::AddTwoInts`, etc.
- **Optional std mode** — `NROS_CPP_STD` enables `std::string`, `std::function`, `std::chrono` conveniences

## Building with CMake

### Native Linux

```cmake
cmake_minimum_required(VERSION 3.22)
project(my_app LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(NANO_ROS_PLATFORM posix)
set(NANO_ROS_RMW     zenoh)
add_subdirectory(<path-to-nano-ros> nano_ros)

nano_ros_generate_interfaces(std_msgs
    "msg/Int32.msg"
    LANGUAGE CPP
    SKIP_INSTALL
)

add_executable(my_app src/main.cpp)
target_link_libraries(my_app
    PRIVATE
        std_msgs__nano_ros_cpp
        NanoRos::NanoRosCpp
)
nros_platform_link_app(my_app)
```

The `nano_ros_generate_interfaces()` function:
1. Resolves `.msg`/`.srv`/`.action` files (local directory, ament index, or bundled)
2. Generates C++ headers (`.hpp`) and Rust FFI glue (`.rs`)
3. Compiles the FFI glue into a static library via Corrosion
4. Creates a `<pkg>__nano_ros_cpp` CMake target with include paths and FFI library linkage

### FreeRTOS (ARM Cortex-M3)

Same shape as the native (host) build — set the toolchain + board + platform on the
cmake command line and the example's `add_subdirectory(nano-ros)` does
the rest. Phase 138's `cmake/platform/nano-ros-freertos.cmake` +
`cmake/board/nano-ros-board-mps2-an385-freertos.cmake` compose the
kernel + lwIP + LAN9118 driver in-tree; no install step needed.

```bash
cmake -S examples/qemu-arm-freertos/cpp/talker -B build/talker \
    -DCMAKE_TOOLCHAIN_FILE=$(pwd)/cmake/toolchain/arm-freertos-armcm3.cmake \
    -DNROS_RMW=zenoh
cmake --build build/talker
```

The example's own `CMakeLists.txt` consumes nano-ros via
`add_subdirectory(<repo>)` (Phase 144) and reaches
`NanoRos::NanoRosCpp` + `nros_platform_link_app()` directly.

### Zephyr

```cmake
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(my_app LANGUAGES CXX)

nros_generate_interfaces(std_msgs LANGUAGE CPP)
target_sources(app PRIVATE src/main.cpp)
```

Requires `CONFIG_NROS=y` and `CONFIG_NROS_CPP_API=y` in `prj.conf`.

## Core API

### Initialization

```cpp
#include <nros/nros.hpp>

// Global init (simple applications)
nros::Result ret = nros::init("tcp/127.0.0.1:7447", 0);

// Or explicit executor (multi-executor patterns)
nros::Executor executor;
NROS_TRY(nros::Executor::create(executor, "tcp/127.0.0.1:7447"));
```

### Node

```cpp
nros::Node node;
NROS_TRY(nros::create_node(node, "my_node"));

// Or with explicit executor:
NROS_TRY(executor.create_node(node, "my_node", "/namespace"));
```

### Publisher

```cpp
#include "std_msgs.hpp"

nros::Publisher<std_msgs::msg::Int32> pub;
NROS_TRY(node.create_publisher(pub, "/chatter"));

std_msgs::msg::Int32 msg;
msg.data = 42;
NROS_TRY(pub.publish(msg));
```

### Subscription

Subscriptions use manual polling — call `spin_once()` to drive I/O, then `take()` to check for messages:

```cpp
nros::Subscription<std_msgs::msg::Int32> sub;
NROS_TRY(node.create_subscription(sub, "/chatter"));

nros::spin_once(100);

std_msgs::msg::Int32 msg;
if (sub.take(msg)) {
    printf("Received: %d\n", msg.data);
}
```

### Service Server

```cpp
#include "example_interfaces.hpp"

using AddTwoInts = example_interfaces::srv::AddTwoInts;

nros::Service<AddTwoInts> srv;
NROS_TRY(node.create_service(srv, "/add_two_ints"));

// In your main loop:
nros::spin_once(10);
AddTwoInts::Request req;
int64_t seq;
if (srv.take_request(req, seq)) {
    AddTwoInts::Response resp;
    resp.sum = req.a + req.b;
    srv.send_response(seq, resp);
}
```

### Service Client

```cpp
nros::Client<AddTwoInts> client;
NROS_TRY(node.create_client(client, "/add_two_ints"));

AddTwoInts::Request req;
req.a = 1; req.b = 2;
AddTwoInts::Response resp;
NROS_TRY(client.call(req, resp));
// resp.sum == 3
```

### Action Server

```cpp
#include "example_interfaces.hpp"

using Fibonacci = example_interfaces::action::Fibonacci;

nros::ActionServer<Fibonacci> srv;
NROS_TRY(node.create_action_server(srv, "/fibonacci"));

// Goals are auto-accepted during spin_once()
nros::spin_once(10);
Fibonacci::Goal goal;
uint8_t goal_id[16];
if (srv.try_recv_goal(goal, goal_id)) {
    // Publish feedback
    Fibonacci::Feedback fb;
    // ... fill feedback ...
    srv.publish_feedback(goal_id, fb);

    // Complete goal
    Fibonacci::Result result;
    // ... fill result ...
    srv.complete_goal(goal_id, result);
}
```

### Action Client

`ActionClient<A>` is an arena-storage handle: the goal, feedback, and result buffers (plus the four underlying transport channels) live in fixed-size storage inside the `ActionClient<A>` instance itself. Nothing is heap-allocated per `send_goal` call, and the type is move-only — moves go through `nros_cpp_action_client_relocate` so the trampoline `context` pointer follows the new `this`.

**Blocking convenience.** `send_goal()` and `get_result()` spin the executor internally until the server replies or the per-call timeout expires:

```cpp
nros::ActionClient<Fibonacci> client;
NROS_TRY(node.create_action_client(client, "/fibonacci"));

Fibonacci::Goal goal;
goal.order = 10;
uint8_t goal_id[16];
NROS_TRY(client.send_goal(goal, goal_id));

Fibonacci::Result result;
NROS_TRY(client.get_result(goal_id, result));
```

These helpers are syntactic sugar over `send_goal_async()` + `spin_once()`; they cannot be called from inside a dispatch callback (returns `NROS_RET_REENTRANT`).

**Future-based async.** `send_goal_future()` / `get_result_future()` return a `Future<T>` that polls the same arena slot:

```cpp
auto fut = client.send_goal_future(goal);
typename decltype(client)::GoalAccept accept;
NROS_TRY(fut.wait(executor.handle(), 5000, accept));
if (accept.accepted) {
    auto rfut = client.get_result_future(accept.goal_id);
    Fibonacci::Result result;
    NROS_TRY(rfut.wait(executor.handle(), 10000, result));
}
```

`GoalAccept` is a nested type on `ActionClient<A>` (16-byte UUID + `bool accepted`).

**Feedback.** Feedback is not goal-scoped at the stream layer — `feedback_stream()` yields `FeedbackType` across every active goal for this client:

```cpp
auto& fb_stream = client.feedback_stream();
Fibonacci::Feedback fb;
while (fb_stream.try_next(fb).ok()) { /* ... */ }
// Or blocking:
NROS_TRY(fb_stream.wait_next(executor.handle(), 500, fb));
```

For per-goal feedback dispatch, use the callback API.

**Callback API: `SendGoalOptions` + `set_callbacks()`.** This is the rclcpp-style entry point. `SendGoalOptions` is a nested POD on `ActionClient<A>`; populate the three function-pointer fields (`goal_response`, `feedback`, `result`) plus an optional `context` pointer, then install once. Callbacks fire from `spin_once()`:

```cpp
typename decltype(client)::SendGoalOptions opts;
opts.goal_response = [](bool accepted, const uint8_t id[16], void* ctx) {
    auto* state = static_cast<MyState*>(ctx);
    state->accepted = accepted;
    std::memcpy(state->goal_id, id, 16);
};
opts.feedback = [](const uint8_t id[16], const uint8_t* data, size_t len, void* ctx) {
    Fibonacci::Feedback fb;
    if (Fibonacci::Feedback::ffi_deserialize(data, len, &fb) == 0) {
        // dispatch on `id`
    }
};
opts.result = [](const uint8_t id[16], int32_t status, const uint8_t* data, size_t len, void* ctx) {
    // status: 4=Succeeded, 5=Canceled, 6=Aborted
};
opts.context = &my_state;
NROS_TRY(client.set_callbacks(opts));

NROS_TRY(client.send_goal_async(goal, goal_id));  // fire-and-forget
while (!my_state.done) { nros::spin_once(10); }
```

Because callback storage lives in the arena, `set_callbacks()` may be called before or after `send_goal_async()` — the executor's trampoline reads the latest pointers on each dispatch. The C++-side trampoline always stashes the most recent feedback / result bytes too, so the same `ActionClient` can drive `feedback_stream().try_next()` and `get_result_future().wait()` even with callbacks installed.

### Timer

Timers fire during `spin_once()`:

```cpp
void on_timer(void* ctx) {
    // periodic work
}

nros::Timer timer;
NROS_TRY(node.create_wall_timer(timer, 1000, on_timer));      // 1000ms period
NROS_TRY(node.create_timer_oneshot(timer, 5000, on_timer)); // one-shot after 5s

timer.cancel();
timer.reset();  // restart from zero
```

### GuardCondition

Guard conditions allow cross-thread signaling:

```cpp
void on_signal(void* ctx) {
    // handle event
}

nros::GuardCondition guard;
NROS_TRY(node.create_guard_condition(guard, on_signal));

// From another thread:
guard.trigger();
// Callback fires on next spin_once()
```

### Executor

```cpp
nros::Executor executor;
NROS_TRY(nros::Executor::create(executor));

nros::Node node;
NROS_TRY(executor.create_node(node, "my_node"));

// Create publishers, subscriptions, etc. on node...

while (executor.ok()) {
    executor.spin_once(10);
}
executor.shutdown();
```

### Spinning

```cpp
// Global spin (after nros::init())
nros::spin_once(10);             // single poll, 10ms timeout
nros::spin(5000);                // spin for 5 seconds
nros::spin(5000, 50);            // spin for 5s, 50ms poll interval

// Explicit executor
executor.spin_once(10);
executor.spin(5000, 50);
```

## Error Handling

All fallible operations return `nros::Result`. Use `NROS_TRY` for early return:

```cpp
nros::Result setup() {
    NROS_TRY(nros::init());
    NROS_TRY(nros::create_node(node, "my_node"));
    NROS_TRY(node.create_publisher(pub, "/topic"));
    return nros::Result::success();
}
```

Check results manually when needed:

```cpp
nros::Result ret = pub.publish(msg);
if (!ret.ok()) {
    printf("Error: %d\n", ret.raw());
}
```

Error codes (`nros::ErrorCode`):
- `Ok` (0) — success
- `Error` (-1) — generic error
- `Timeout` (-2) — operation timed out
- `InvalidArgument` (-3) — bad parameter
- `NotInitialized` (-4) — entity not initialized
- `Full` (-5) — buffer full
- `TransportError` (-100) — middleware transport failure

## Optional `std` Mode (`NROS_CPP_STD`)

For any toolchain with a C++ standard library, define `NROS_CPP_STD` to enable STL convenience overloads. This is automatically available when including `<nros/nros.hpp>` with the macro defined.

```cpp
#define NROS_CPP_STD
#include <nros/nros.hpp>
```

### `std::string` overloads

```cpp
std::string locator = "tcp/127.0.0.1:7447";
nros::init(locator);

std::string topic = "/chatter";
nros::create_publisher<std_msgs::msg::Int32>(node, pub, topic);
```

### `std::function` callbacks

```cpp
nros::Timer timer;
nros::create_wall_timer(node, timer, std::chrono::milliseconds(1000), [&count, &pub]() {
    std_msgs::msg::Int32 msg;
    msg.data = ++count;
    pub.publish(msg);
});
```

### `std::chrono` durations

```cpp
using namespace std::chrono_literals;
nros::spin_once(100ms);
nros::spin(5s, 50ms);
```

## Zephyr Integration

### `prj.conf`

```ini
CONFIG_CPP=y
CONFIG_STD_CPP14=y

CONFIG_NROS=y
CONFIG_NROS_CPP_API=y
CONFIG_NROS_ZENOH_LOCATOR="tcp/192.0.2.2:7456"
CONFIG_NROS_DOMAIN_ID=0

CONFIG_POSIX_API=y
CONFIG_MAX_PTHREAD_MUTEX_COUNT=32
CONFIG_MAX_PTHREAD_COND_COUNT=16
```

### `CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(my_app LANGUAGES CXX)

nros_generate_interfaces(std_msgs LANGUAGE CPP)
target_sources(app PRIVATE src/main.cpp)
```

### `src/main.cpp`

```cpp
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

extern "C" {
#include <zpico_zephyr.h>
}

#include <nros/nros.hpp>
#include "std_msgs.hpp"

LOG_MODULE_REGISTER(my_app, LOG_LEVEL_INF);

int main(void)
{
    zpico_zephyr_wait_network(CONFIG_NROS_INIT_DELAY_MS);

    nros::Result ret = nros::init(CONFIG_NROS_ZENOH_LOCATOR, CONFIG_NROS_DOMAIN_ID);
    if (!ret.ok()) return 1;

    nros::Node node;
    NROS_TRY(nros::create_node(node, "my_node"));

    // ... create publishers, subscriptions, etc.

    while (true) {
        nros::spin_once(100);
    }
}
```

## Examples

| Directory | Description |
|-----------|-------------|
| `examples/native/cpp/talker/` | Publish Int32 on `/chatter` (native Linux) |
| `examples/native/cpp/listener/` | Subscribe to `/chatter` (native Linux) |
| `examples/native/cpp/service-server/` | AddTwoInts server (native Linux) |
| `examples/native/cpp/service-client/` | AddTwoInts client (native Linux) |
| `examples/zephyr/cpp/talker/` | Publish Int32 on `/chatter` (Zephyr) |
| `examples/zephyr/cpp/listener/` | Subscribe to `/chatter` (Zephyr) |

## See Also

- [creating-examples.md](creating-examples.md) — How to create new examples
- [message-generation.md](message-generation.md) — Message generation details
- [docs/roadmap/phase-66-cpp-api.md](../roadmap/archived/phase-66-cpp-api.md) — Phase 66 roadmap (design decisions)
- [docs/design/0018-cpp-api-design.md](../design/0018-cpp-api-design.md) — Full design rationale
