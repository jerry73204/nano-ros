cmake_minimum_required(VERSION 3.22)
project(cpp_logging LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(nano_ros REQUIRED)

nano_ros_add_executable(cpp_logging src/main.cpp)

install(TARGETS cpp_logging DESTINATION lib/${PROJECT_NAME})
ament_package()
