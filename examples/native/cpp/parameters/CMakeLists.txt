cmake_minimum_required(VERSION 3.22)
project(cpp_parameters LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(nano_ros REQUIRED)

nano_ros_add_executable(cpp_parameters src/main.cpp)

install(TARGETS cpp_parameters DESTINATION lib/${PROJECT_NAME})
ament_package()
