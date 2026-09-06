cmake_minimum_required(VERSION 3.22)
project(cpp_lifecycle_talker_pkg VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(nano_ros REQUIRED)
find_package(std_msgs REQUIRED)


# Phase 269 W2 — typed managed-node C++ component: counter publisher on /chatter.
# Lifecycle autostart is wired in the entry by emit_cpp.rs W2.
nano_ros_auto_add_library(cpp_talker_lib STATIC src/LifecycleTalker.cpp)

nros_components_register_node(cpp_talker_lib
    PLUGIN cpp_lifecycle_talker_pkg::LifecycleTalker
    EXECUTABLE cpp_talker
    SHAPE configure)

# Phase 270 (#103) — managed node written with the nros::LifecycleNode wrapper
# (self-drives REP-2002 via the on_* overrides, not the entry autostart codegen).
nano_ros_auto_add_library(cpp_managed_talker_lib STATIC src/ManagedTalker.cpp)

nros_components_register_node(cpp_managed_talker_lib
    PLUGIN cpp_lifecycle_talker_pkg::ManagedTalker
    EXECUTABLE cpp_managed_talker
    SHAPE configure)

if(TARGET std_msgs__nano_ros_cpp)
    target_link_libraries(cpp_talker_lib
        PUBLIC std_msgs__nano_ros_cpp)
    target_link_libraries(cpp_talker_lib
        PUBLIC std_msgs__nano_ros_cpp)
endif()
