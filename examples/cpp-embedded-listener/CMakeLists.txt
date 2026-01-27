cmake_minimum_required(VERSION 3.15)
project(cpp_embedded_listener
    VERSION 0.1.0
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================================
# Embedded build options
# ============================================================================
# Enable embedded mode (no std library)
option(BUILD_FOR_EMBEDDED "Build for embedded systems" OFF)

# ============================================================================
# nano-ros-cpp dependency
# ============================================================================
set(NANO_ROS_CPP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../crates/nano-ros-cpp")

# Configure nano-ros-cpp for embedded if requested
if(BUILD_FOR_EMBEDDED)
    set(NANO_ROS_EMBEDDED ON CACHE BOOL "" FORCE)
endif()

add_subdirectory(${NANO_ROS_CPP_DIR} ${CMAKE_CURRENT_BINARY_DIR}/nano-ros-cpp)

# ============================================================================
# Message generation (embedded mode)
# ============================================================================
set(GENERATOR_SCRIPT "${NANO_ROS_CPP_DIR}/scripts/nano_ros_generate_cpp.py")
set(GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")

# Generate std_msgs/msg/Int32 for embedded
add_custom_command(
    OUTPUT ${GENERATED_DIR}/std_msgs/msg/int32.hpp
    COMMAND ${CMAKE_COMMAND} -E make_directory ${GENERATED_DIR}
    COMMAND Python3::Interpreter ${GENERATOR_SCRIPT}
        ${NANO_ROS_CPP_DIR}/msg/std_msgs/Int32.msg
        ${GENERATED_DIR}
        --package std_msgs
        $<$<BOOL:${BUILD_FOR_EMBEDDED}>:--embedded>
    DEPENDS ${GENERATOR_SCRIPT}
    COMMENT "Generating std_msgs/msg/Int32.hpp"
)

# Create a target for generated headers
add_custom_target(generated_msgs DEPENDS ${GENERATED_DIR}/std_msgs/msg/int32.hpp)

# Find Python3
find_package(Python3 COMPONENTS Interpreter REQUIRED)

# ============================================================================
# Executable
# ============================================================================
add_executable(cpp_embedded_listener src/main.cpp)

target_link_libraries(cpp_embedded_listener
    PRIVATE
        nano_ros_cpp
)

# Add generated headers and nano_ros includes
target_include_directories(cpp_embedded_listener
    PRIVATE
        ${NANO_ROS_CPP_DIR}/include
        ${GENERATED_DIR}
)

# Depend on generated messages
add_dependencies(cpp_embedded_listener generated_msgs)

# ============================================================================
# Embedded-specific configuration
# ============================================================================
if(BUILD_FOR_EMBEDDED)
    target_compile_definitions(cpp_embedded_listener
        PRIVATE
            NANO_ROS_EMBEDDED=1
    )

    # Disable exceptions and RTTI for smaller binary
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(cpp_embedded_listener
            PRIVATE
                -fno-exceptions
                -fno-rtti
        )
    endif()

    message(STATUS "Building cpp_embedded_listener in embedded mode")
endif()
