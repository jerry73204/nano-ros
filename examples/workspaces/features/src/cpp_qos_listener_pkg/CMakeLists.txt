cmake_minimum_required(VERSION 3.22)
project(cpp_qos_listener_pkg VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(nano_ros REQUIRED)
find_package(std_msgs REQUIRED)


# Typed component (RFC-0043): a raw zero-copy `/chatter` subscription keyed on
# `std_msgs::msg::Int32::TYPE_NAME`. The differentiator vs the committed
# listener_pkg is the QoS: this declares the SAME non-default profile (reliable +
# transient-local + keep-last-10) as the talker so the QoS-matched endpoints
# connect (see src/QosListener.cpp).
nano_ros_auto_add_library(cpp_qos_listener_lib STATIC src/QosListener.cpp)

nros_components_register_node(cpp_qos_listener_lib
    PLUGIN cpp_qos_listener_pkg::QosListener
    EXECUTABLE cpp_qos_listener
    SHAPE configure)

# `nros_find_interfaces` declares the `std_msgs__nano_ros_cpp` INTERFACE lib (for
# `Int32::TYPE_NAME`); guard with `if(TARGET)` (the Zephyr generator whole-archives
# the interface into `app` instead).
if(TARGET std_msgs__nano_ros_cpp)
    target_link_libraries(cpp_qos_listener_lib
        PUBLIC std_msgs__nano_ros_cpp)
endif()
