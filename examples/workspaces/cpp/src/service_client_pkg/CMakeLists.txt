cmake_minimum_required(VERSION 3.22)
project(service_client_pkg VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(nano_ros REQUIRED)
find_package(example_interfaces REQUIRED)

# phase-263 A1 (services, C++) — poll-model AddTwoInts client; republishes the sum on /sum.

nano_ros_auto_add_library(service_client_lib STATIC src/AddClient.cpp)

nros_components_register_node(service_client_lib
    PLUGIN service_client_pkg::AddClient
    EXECUTABLE add_client
    SHAPE configure
    # phase-412 -- `ENTITIES` is retired, and this component is the one
    # case with no replacement yet: its service has a CLIENT and no
    # server (the server is the separate add_server image), and the
    # contract schema has no `external:` mark for services the way it
    # has for topics -- `dangling-entity` makes a client-only service a
    # hard resolve error. So this image authors no contract and keeps
    # its CONFIGURED pool sizes, which is what every image had before
    # phase-403. Issue 1083.
)

if(TARGET example_interfaces__nano_ros_cpp)
    target_link_libraries(service_client_lib
        PUBLIC example_interfaces__nano_ros_cpp)
endif()
