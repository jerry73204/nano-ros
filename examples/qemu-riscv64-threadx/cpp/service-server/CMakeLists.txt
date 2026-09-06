cmake_minimum_required(VERSION 3.22)
project(cpp_service_server LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(nano_ros REQUIRED)
find_package(example_interfaces REQUIRED)

nano_ros_add_executable(cpp_service_server src/main.cpp)
ament_target_dependencies(cpp_service_server example_interfaces)

install(TARGETS cpp_service_server DESTINATION lib/${PROJECT_NAME})
ament_package()
