# Porting a ROS 2 C++ node to nano-ros

Goal: take a normal ROS 2 C++ node (one that compiles + runs under
`colcon build` against `ros-humble-*`) and run it under nano-ros — without
rewriting the source. The `rclcpp::` names are declared by nano-ros's own C++
headers — `#include <rclcpp/rclcpp.hpp>` resolves through
`cmake/compat/include/rclcpp/rclcpp.hpp` straight to `<nros/nros.hpp>`, with no
compat header in between (phase-417 stage 6 deleted `nros/rclcpp_compat.hpp`).
What remains of the compat layer is build glue —
`cmake/compat/NrosRclcppCompat.cmake` plus `nros-diagnostic-updater` — so the
only delta is **build-script glue**.

The canonical proof lives at
[`examples/templates/cpp-port-minimal-publisher/`](https://github.com/NEWSLabNTU/nano-ros/tree/main/examples/templates/cpp-port-minimal-publisher) —
the ROS 2 tutorial's `minimal_publisher.cpp` vendored unmodified, building
against nano-ros via the three glue lines below.

## Two layers of glue

### Per-package CMakeLists.txt — **zero nano-ros lines**

The ported pkg's `CMakeLists.txt` carries **only stock ROS 2 syntax**.
Same file builds under both `colcon build` AND a nano-ros build:

```cmake
cmake_minimum_required(VERSION 3.20)
project(my_node LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# … any other msg packages the source #include's.

add_executable(my_node src/my_node.cpp)
ament_target_dependencies(my_node rclcpp std_msgs)

ament_package()
```

`find_package(rclcpp)` resolves through the rclcpp Find-stub (which puts
`cmake/compat/include/` on the include path so `<rclcpp/rclcpp.hpp>` lands on
nano-ros's headers, and force-includes `nros/rclcpp_components_compat.hpp` for
the components macros); `find_package(std_msgs)`
resolves through the smart Find-stub (it walks
`NROS_INTERFACE_SEARCH_PATH > AMENT_PREFIX_PATH > bundled`); the
`ament_target_dependencies` compat shim wires both link targets.

### Workspace umbrella CMakeLists.txt — **one nano-ros include**

The umbrella `CMakeLists.txt` (sits at the workspace root, next to
`src/`) is the **only** nano-ros-aware file:

```cmake
cmake_minimum_required(VERSION 3.22)
project(my_workspace LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 14)

# 1) Pull nano-ros in.
set(NANO_ROS_PLATFORM posix)
set(NROS_RMW "zenoh" CACHE STRING "Active RMW.")
set(NANO_ROS_RMW "${NROS_RMW}")
add_subdirectory("/path/to/nano-ros" nano_ros)

# 2) Point the smart Find-stub at this workspace's src/ (must precede the
#    NrosRclcppCompat include so workspace-pkg Find<pkg>.cmake auto-emit
#    picks it up).
set(NROS_INTERFACE_SEARCH_PATH "${CMAKE_SOURCE_DIR}/src")

# 3) Drop-in source-compat surface.
include("/path/to/nano-ros/cmake/compat/NrosRclcppCompat.cmake")

# 4) Bulk-build every workspace msg pkg in topo order (one line instead of
#    N add_subdirectory(src/<pkg>) lines).
nros_workspace_interfaces()

# 5) Build consumer apps.
add_subdirectory(src/my_node)
```

No `nros_generate_interfaces(<pkg>)` calls per consumer — the smart
Find-stub does the codegen at `find_package(<pkg>)` time.

### Reference fixture

[`examples/templates/local-msg-package/`](https://github.com/NEWSLabNTU/nano-ros/tree/main/examples/templates/local-msg-package)
ships the pattern end-to-end: two workspace msg pkgs (`local_msgs`,
`extra_msgs`) with intra-workspace dep + a C++ consumer pulling msgs
from BOTH the workspace AND AMENT (`std_msgs`, `geometry_msgs`,
`sensor_msgs`) via one `find_package` shape. Cross-build proof: the
same `src/` builds under `colcon build` (CI-gated).

### Legacy `nros_generate_interfaces(<pkg>)` shape

(The full current-vs-legacy spelling table lives in
[Message Generation](../user-guide/message-generation.md#which-cmake-spelling-one-table).)

Per-package `nros_generate_interfaces(std_msgs LANGUAGE CPP SKIP_INSTALL)`
calls still work (back-compat preserved) but are **deprecated for new
code** — they bypass the ROS-convention smart Find-stub + workspace
discovery. Existing in-tree examples will migrate as part of Phase
210.E.3.

## What "just works" without source edits

The compat surface covers the patterns a typical ROS 2 C++ node uses:

| rclcpp surface | nano-ros mapping | Notes |
|---|---|---|
| `class MyNode : public rclcpp::Node` | `rclcpp::Node` shim → `nros::Executor` + `nros::Node` | Ctor takes `(name)`. |
| `std::make_shared<MyNode>()` | inherits `enable_shared_from_this` | `shared_from_this()` works. |
| `create_publisher<M>(topic, qos)` | shared_ptr-returning wrapper | `qos` can be `rclcpp::QoS(10)` or an int. |
| `create_subscription<M>(topic, qos, callback)` | polling pump dispatched from `spin*` | **Capturing lambdas + `std::function` all work**. |
| `create_wall_timer(period, callback)` | wall-timer dispatched from `spin*` | `std::chrono::duration` arg, capturing-lambda callback. |
| `rclcpp::init(argc, argv) / shutdown() / ok() / spin(n) / spin_some(n)` | wraps `nros::init/shutdown/ok/spin_once` | argc/argv ignored. |
| `RCLCPP_INFO / WARN / ERROR / DEBUG / FATAL` | dispatched through `NROS_*` macros | `_THROTTLE` variants degrade to plain log. |
| `rclcpp::QoS / KeepLast(n) / SystemDefaultsQoS()` | subclass of `nros::QoS` with the `(depth)` ctor | Chainable setters inherited. |
| `diagnostic_updater::Updater` + `DiagnosticStatusWrapper` | `nros-diagnostic-updater` shim | Publishes `/diagnostics`. |
| `rclcpp_action::Server<A> / Client<A>` | aliases for `nros::ActionServer/Client<A>` | The action call shapes (send_goal_async etc.) match. |
| `RCLCPP_COMPONENTS_REGISTER_NODE(class)` | no-op macro + cmake-side `rclcpp_components_register_node()` emits a thin `int main()` per registration | Single-binary embedded. |
| `find_package(ament_cmake_auto / rclcpp / rclcpp_components / diagnostic_updater / std_msgs / …)` | Find-stubs at `cmake/compat/stubs/` | ~24 of the most-cited ROS 2 packages stubbed; add your own under `cmake/compat/stubs/Find<pkg>.cmake` for more. |

## What's documented as "needs adapt" (codegen-side, not surface-side)

These are cosmetic codegen differences nano-ros's per-package codegen
and the upstream `rosidl_default_runtime` codegen don't share; both are
tracked as ROS-convention codegen work.

- **Message string fields.** nano-ros codegen emits `nros::FixedString<N>`,
  upstream emits `std::string`. Assigning a `std::string` needs a one-token
  adapter: `message.data = s.c_str()`. The reverse `(std::string{}.c_str())`
  is what `RCLCPP_INFO` already takes.
- **Generated message header path.** **CLOSED** (alias
  headers): nano-ros codegen emits BOTH the per-message form
  `<std_msgs/msg/string.hpp>` (upstream-shape) AND the umbrella
  `<std_msgs/std_msgs.hpp>`. Use whichever the original source picks.

## What's out of scope (will need code adapt or a follow-up phase)

- **`rclcpp_lifecycle::LifecycleNode`** — the compat shim does not map it.
  But nano-ros ships its own REP-2002 lifecycle surface (`nros/lifecycle.h`
  in C, `lifecycle-services` feature, state machine + lifecycle services —
  see the [C API reference](../reference/c-api.md) and
  `examples/native/rust/lifecycle-node/`), so port to that rather than
  hand-rolling configure/activate bookkeeping on a plain `Node`.
- **Yaml-loaded parameters.** `declare_parameter<T>("name", default)` reads
  from a launch yaml in stock ROS 2. nano-ros has no runtime yaml loader —
  parameter *initials* are compile-baked from the launch XML's
  `<param name="…" value="…"/>` entries (RFC-0004 §10), then live in a
  volatile store the standard parameter services can update until the next
  boot. Move yaml values into the launch file (or expose them as
  compile-time constants).
- **`tf2`, `image_transport`, `pluginlib`** — out of nano-ros scope. Project-
  specific helpers (autoware `universe_utils`, PX4 uORB shims) are not
  nano-ros's to ship; the porting user vendors them or replaces the call
  sites with raw `nros-cpp` ones.

## When the port hits a gap

Known open follow-ups: yaml-loaded parameter baking, `LifecycleNode`
compat, and the in-tree migration of legacy
`nros_generate_interfaces(<pkg>)` call sites. If your port surfaces a
*new* gap not covered by the compat header, file an issue — a fix
lands either tree-side (in `cmake/compat/` or
`packages/api/nros-cpp/`) or as a codegen change.

In-tree regression fixtures:

* [`local-msg-package`](https://github.com/NEWSLabNTU/nano-ros/tree/main/examples/templates/local-msg-package)
  — mixed workspace (workspace + AMENT msg sources) C++ + Rust consumers.
* [`cpp-port-minimal-publisher`](https://github.com/NEWSLabNTU/nano-ros/tree/main/examples/templates/cpp-port-minimal-publisher)
  — ROS 2 tutorial `minimal_publisher.cpp` verbatim.
* [`rclcpp-compat-smoke`](https://github.com/NEWSLabNTU/nano-ros/tree/main/examples/templates/rclcpp-compat-smoke)
  — minimal source-compat regression test.
* [`topic-state-monitor-port`](https://github.com/NEWSLabNTU/nano-ros/tree/main/examples/templates/topic-state-monitor-port)
  — multi-sub / wall-timer / diagnostic_updater exercise.

Drop your reduced-case node under `examples/templates/<your-port>/` and
add it to CI once the gap closes.
