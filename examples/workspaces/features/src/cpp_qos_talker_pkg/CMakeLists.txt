cmake_minimum_required(VERSION 3.22)
project(cpp_qos_talker_pkg VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(nano_ros REQUIRED)
find_package(std_msgs REQUIRED)


# Typed component (RFC-0043): a typed `Publisher<std_msgs::msg::Int32>` on /chatter.
# The differentiator vs the committed cpp_lifecycle_talker_pkg is the QoS: this declares a
# NON-DEFAULT profile via the `nros::QoS` builder
# (`.reliable().transient_local().keep_last(10)`) rather than the default profile
# (see src/QosTalker.cpp).
nano_ros_auto_add_library(cpp_qos_talker_lib STATIC src/QosTalker.cpp)

nros_components_register_node(cpp_qos_talker_lib
    PLUGIN cpp_qos_talker_pkg::QosTalker
    EXECUTABLE cpp_qos_talker
    SHAPE configure)

# `nros_find_interfaces` declares the `std_msgs__nano_ros_cpp` INTERFACE lib
# carrying the generated headers' include dirs; guard with `if(TARGET)` (the
# Zephyr generator whole-archives the interface into `app` instead).
if(TARGET std_msgs__nano_ros_cpp)
    target_link_libraries(cpp_qos_talker_lib
        PUBLIC std_msgs__nano_ros_cpp)
endif()
