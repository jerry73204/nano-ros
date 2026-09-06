cmake_minimum_required(VERSION 3.22)
project(pkg_cpp_listener LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Phase 140 / 244.C4 — multi-package workspace demo, add_subdirectory shape.
# C++ listener as a TYPED component (RFC-0043): the node is a stateful
# `Listener` (src/Listener.cpp, `configure(node)` binds a raw /chatter sub by
# identity). RFC-0057 (phase-305) split shape — the ament pattern: the
# library owns the sources, the register owns the identity
# (`rclcpp_components_register_node` keyword parity).
find_package(nano_ros REQUIRED)
find_package(std_msgs REQUIRED)

# std_msgs C++ bindings — the component uses `std_msgs::msg::Int32::TYPE_NAME`.

nano_ros_auto_add_library(pkg_cpp_listener_lib STATIC src/Listener.cpp)

nros_components_register_node(pkg_cpp_listener_lib
    PLUGIN pkg_cpp_listener::Listener
    EXECUTABLE pkg_cpp_listener
    HEADER Listener.hpp
    SHAPE configure
    TYPED
    DEPLOY native)
