cmake_minimum_required(VERSION 3.15)
project(cpp_talker
    VERSION 0.1.0
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================================
# nano-ros-cpp dependency
# ============================================================================
# Option 1: Use installed nano_ros_cpp
# find_package(nano_ros_cpp REQUIRED)

# Option 2: Build from source using add_subdirectory
set(NANO_ROS_CPP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../crates/nano-ros-cpp")
add_subdirectory(${NANO_ROS_CPP_DIR} ${CMAKE_CURRENT_BINARY_DIR}/nano-ros-cpp)

# ============================================================================
# Executable
# ============================================================================
add_executable(cpp_talker src/main.cpp)

target_link_libraries(cpp_talker
    PRIVATE
        nano_ros_cpp
)

# Include pre-generated std_msgs and builtin_interfaces
target_include_directories(cpp_talker
    PRIVATE
        ${NANO_ROS_CPP_DIR}/include
)
