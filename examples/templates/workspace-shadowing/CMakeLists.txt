# Phase 210.F.4 — workspace-shadowing fixture umbrella.
#
# Smoke-test the layered-search-path contract: when a workspace pkg
# `std_msgs` collides with the AMENT-installed `std_msgs` under
# `/opt/ros/<distro>/share/std_msgs/`, the workspace copy MUST win.
#
# Layout:
#   src/std_msgs/    — workspace shadow: declares one msg the upstream
#                      AMENT std_msgs does NOT ship (`Marker.msg`,
#                      `string shadowed_marker`). If the consumer were
#                      to resolve to AMENT's std_msgs, the compile would
#                      FAIL (no `std_msgs/msg/marker.hpp` header).
#   src/consumer/    — verbatim ROS 2 C++ consumer that includes
#                      `std_msgs/msg/marker.hpp` + references
#                      `.shadowed_marker`, proving the workspace copy
#                      was linked.
#
# This umbrella is the ONLY nano-ros-specific file in the tree. Same
# `src/` builds under `colcon build` against an upstream-ROS workspace
# (where the workspace `std_msgs` similarly shadows the AMENT one via
# colcon's standard layered overlay behaviour).

cmake_minimum_required(VERSION 3.22)
project(workspace_shadowing_fixture LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1) Pull nano-ros.
set(NANO_ROS_PLATFORM posix)
set(NROS_RMW "zenoh" CACHE STRING "Active RMW (zenoh|cyclonedds|xrce).")
set(NANO_ROS_RMW "${NROS_RMW}")
# Resolve the nano-ros checkout root once (phase-277 W6 standard guard):
#   1. -DNANO_ROS_ROOT=<path> cache var (explicit, e.g. a copied-out example),
#   2. NROS_REPO_DIR env var (exported by nano-ros's activate.sh),
#   3. in-repo relative walk-up (this example inside the nano-ros tree).
if(NOT DEFINED NANO_ROS_ROOT)
    if(DEFINED ENV{NROS_REPO_DIR} AND NOT "$ENV{NROS_REPO_DIR}" STREQUAL "")
        set(NANO_ROS_ROOT "$ENV{NROS_REPO_DIR}")
    else()
        get_filename_component(NANO_ROS_ROOT
            "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
    endif()
endif()
add_subdirectory("${NANO_ROS_ROOT}" nano_ros)

# 2) Point the smart Find-stub at this workspace's `src/` BEFORE the
#    compat module is included — so the workspace-pkg Find<pkg>.cmake
#    auto-emit picks up our shadow `std_msgs`. With this set, the layered
#    resolver `_NrosFindRosMsgPackage.cmake` finds `src/std_msgs/` at
#    Layer 1 (NROS_INTERFACE_SEARCH_PATH) and skips Layer 2
#    (AMENT_PREFIX_PATH). The emitted Find<std_msgs>.cmake at
#    `${CMAKE_BINARY_DIR}/nros-find-stubs/Findstd_msgs.cmake` takes
#    precedence over the in-tree
#    `cmake/compat/stubs/Findstd_msgs.cmake` because
#    `CMAKE_MODULE_PATH` is PREPENDed by `_nros_emit_workspace_find_stubs`.
set(NROS_INTERFACE_SEARCH_PATH "${CMAKE_CURRENT_SOURCE_DIR}/src")

# 3) rclcpp source-compat surface.
include("${NANO_ROS_ROOT}/cmake/compat/NrosRclcppCompat.cmake")

# 4) Bulk-build every workspace msg pkg in topo order — picks up
#    `src/std_msgs/`.
nros_workspace_interfaces()

# 5) Consumer.
add_subdirectory(src/consumer)

# 6) The consumer's `find_package(std_msgs)` resolves to THIS workspace's
#    `std_msgs`, which the umbrella builds as a nano-ros cpp FFI binding —
#    so the consumer exe links `libnros_cpp.a`. nros-cpp's runtime references
#    the RMW-backend registration stub `nros_app_register_backends()`, whose
#    sole (strong) def is emitted by `nano_ros_link_rmw` (phase-249 P4a removed
#    the weak default → absence is a link error). Wire it on the consumer here,
#    in the umbrella (the only nano-ros-specific file), so
#    `src/consumer/CMakeLists.txt` stays verbatim-rclcpp.
if(COMMAND nano_ros_link_rmw AND TARGET consumer)
    nano_ros_link_rmw(consumer)
endif()
