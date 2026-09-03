# cpp-port-minimal-publisher — Phase 209.G iter 2

The canonical ROS 2 "minimal publisher" tutorial node
([source pattern](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)
from the upstream ROS 2 docs), **vendored unmodified**, building against
nano-ros through the Phase 209.A–D compat surface.

The acceptance for 209 lands here: a normal ROS 2 C++ node compiles + links +
runs against nano-ros by **swapping the build glue + zero `#include` edits** —
not by rewriting the source.

## What changed vs an upstream ROS 2 package

The C++ source (`src/minimal_publisher.cpp`) is the upstream tutorial's
`minimal_publisher.cpp` **verbatim** — true as of 2026-09-04 (phase-417 stage 1).
It was NOT true when first written: three lines carried adaptations, and the
README claimed verbatim anyway. What made the claim true is that
`rclcpp::Publisher<T>::SharedPtr` / `rclcpp::TimerBase::SharedPtr` now resolve
(the nested aliases exist on the entity types) and `FixedString<N>` accepts a
`std::string`, so the two member declarations and the
`message.data = "Hello, world! " + std::to_string(count_++);` assignment are
upstream's own lines again. The CMakeLists.txt's stock-ROS-2 shape
(`find_package(ament_cmake_auto)` / `ament_auto_add_executable` /
`ament_target_dependencies` / `ament_auto_package`) is **untouched**.

The only delta — three lines prepended:

```cmake
# 1) Pull nano-ros (NanoRos::NanoRos / NanoRosCpp / nros_generate_interfaces).
set(NANO_ROS_PLATFORM posix)
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../../.." nano_ros)

# 2) rclcpp / ament_cmake_auto / rclcpp_components source-compat.
include("${CMAKE_CURRENT_SOURCE_DIR}/../../../cmake/compat/NrosRclcppCompat.cmake")

# 3) Generate the message bindings the source includes (folded by 209.E).
nros_generate_interfaces(builtin_interfaces LANGUAGE CPP SKIP_INSTALL)
nros_generate_interfaces(std_msgs DEPENDENCIES builtin_interfaces LANGUAGE CPP SKIP_INSTALL)
```

## Build + run

```bash
cd examples/templates/cpp-port-minimal-publisher
cmake -B build -S . -DNROS_RMW=zenoh
cmake --build build -j

ZENOH_CONFIG_OVERRIDE='listen/endpoints=["tcp/127.0.0.1:7447"];scouting/multicast/enabled=false' ros2 run rmw_zenoh_cpp rmw_zenohd &
./build/minimal_publisher
# Publishing: 'Hello, world! 0'
# Publishing: 'Hello, world! 1'
# …
```

## Caveats found during this port (all in `book/.../porting-a-cpp-node.md`)

- nano-ros codegen emits message string fields as `nros::FixedString<N>` (not
  `std::string`). Assignments stay one-liners (`message.data = s.c_str()`)
  but cross-package code may need a small adapter. **Tracked: 209.E should
  emit a `std::string`-compatible field type alongside FixedString.**
- The generated message umbrella is `"std_msgs/std_msgs.hpp"` (the nano-ros
  layout). Upstream uses `<std_msgs/msg/string.hpp>` (per-message header).
  **Tracked: 209.E (codegen emits the upstream layout too).**

Both are codegen-side, not surface-side. The rclcpp surface itself (Node,
Publisher, Subscription, Timer, init/shutdown/spin/ok, log macros, QoS,
diagnostic_updater) lands the source unchanged.
