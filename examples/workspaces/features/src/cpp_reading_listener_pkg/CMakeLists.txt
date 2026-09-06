cmake_minimum_required(VERSION 3.22)
project(cpp_reading_listener_pkg VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(nano_ros REQUIRED)
find_package(custom_msgs REQUIRED)

# Typed component (RFC-0043) subscribing through the GENERATED
# `custom_msgs::msg::Reading` bindings (phase-293 / issue #212).
nano_ros_auto_add_library(cpp_reading_listener_lib STATIC src/ReadingListener.cpp)

nros_components_register_node(cpp_reading_listener_lib
    PLUGIN cpp_reading_listener_pkg::ReadingListener
    EXECUTABLE cpp_reading_listener
    SHAPE configure)
