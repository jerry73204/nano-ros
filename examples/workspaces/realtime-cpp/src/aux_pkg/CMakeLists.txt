cmake_minimum_required(VERSION 3.22)
project(aux_pkg VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(nano_ros REQUIRED)
find_package(std_msgs REQUIRED)


# Mid-tier auxiliary node. Publishes a counter on /aux every 50 ms.
# CALLBACK_GROUPS declares the "aux" group ID; system.toml group_tiers maps
# it to the [tiers.mid] tier (RFC-0015 §4.2). The mid tier is spawned by a
# spawned tier (boot→mid→low) — the middle hop the #144 chained-spawn fix
# serializes.
nano_ros_auto_add_library(aux_lib STATIC src/Aux.cpp)

nros_components_register_node(aux_lib
    HEADER aux_pkg/Aux.hpp
    PLUGIN aux_pkg::Aux
    EXECUTABLE aux
    SHAPE configure
    CALLBACK_GROUPS aux)

# `nros_find_interfaces` declares INTERFACE libs (`std_msgs__nano_ros_cpp`)
# carrying generated headers' include dirs. Guard with `if(TARGET)`: some
# embedded generators fold the generated interface into the app target
# instead of exposing a `<pkg>__nano_ros_cpp` lib.
if(TARGET std_msgs__nano_ros_cpp)
    target_link_libraries(aux_lib
        PUBLIC std_msgs__nano_ros_cpp)
endif()
