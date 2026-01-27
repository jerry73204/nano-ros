cmake_minimum_required(VERSION 3.15)
project(cpp_custom_msg
    VERSION 0.1.0
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================================
# nano-ros-cpp dependency
# ============================================================================
set(NANO_ROS_CPP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../crates/nano-ros-cpp")
add_subdirectory(${NANO_ROS_CPP_DIR} ${CMAKE_CURRENT_BINARY_DIR}/nano-ros-cpp)

# ============================================================================
# Message Generation
# ============================================================================
# Include the message generation macro
include(${NANO_ROS_CPP_DIR}/cmake/nano_ros_generate_interfaces.cmake)

# Generate C++ bindings for our custom messages
nano_ros_generate_interfaces(${PROJECT_NAME}
    "msg/SensorReading.msg"
    "msg/Status.msg"
)

# ============================================================================
# Executable
# ============================================================================
add_executable(cpp_custom_msg src/main.cpp)

target_link_libraries(cpp_custom_msg
    PRIVATE
        nano_ros_cpp
        ${PROJECT_NAME}_msg
)

# Include pre-generated std_msgs (for reference/comparison)
target_include_directories(cpp_custom_msg
    PRIVATE
        ${NANO_ROS_CPP_DIR}/include
)
