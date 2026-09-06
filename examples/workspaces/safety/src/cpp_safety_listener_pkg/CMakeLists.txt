cmake_minimum_required(VERSION 3.22)
project(cpp_safety_listener_pkg VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(nano_ros REQUIRED)
find_package(std_msgs REQUIRED)


# Phase 269 W3 — C++ safety-listener component. Uses
# `node.create_subscription_with_safety<M>()` to register an integrity-carrying
# subscription on /chatter; the typed callback receives (const Int32&, const
# nros_cpp_integrity_status_t&). Requires NANO_ROS_SAFETY_E2E=ON.
nano_ros_auto_add_library(cpp_safe_listener_lib STATIC src/SafetyListener.cpp)

nros_components_register_node(cpp_safe_listener_lib
    PLUGIN cpp_safety_listener_pkg::SafetyListener
    EXECUTABLE cpp_safe_listener
    SHAPE configure)

if(TARGET std_msgs__nano_ros_cpp)
    target_link_libraries(cpp_safe_listener_lib
        PUBLIC std_msgs__nano_ros_cpp)
endif()
