cmake_minimum_required(VERSION 3.15)
project(nano_ros_cpp
    VERSION 0.1.0
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ============================================================================
# Build Options
# ============================================================================

# Library type
option(BUILD_SHARED_LIBS "Build shared library" ON)

# Embedded system options
option(NANO_ROS_EMBEDDED "Build for embedded systems (no std library)" OFF)
option(NANO_ROS_USE_ETL "Use Embedded Template Library for containers" OFF)

# Rust target for cross-compilation (empty = host target)
set(NANO_ROS_RUST_TARGET "" CACHE STRING "Rust target triple for cross-compilation (e.g., thumbv7em-none-eabihf)")

# Transport backend selection
set(NANO_ROS_TRANSPORT "zenoh" CACHE STRING "Transport backend")
set_property(CACHE NANO_ROS_TRANSPORT PROPERTY STRINGS
    "zenoh"           # Desktop: zenoh-pico POSIX
    "shim-zephyr"     # Zephyr RTOS
    "shim-smoltcp"    # Bare-metal with smoltcp
)

# Additional Cargo features
set(NANO_ROS_CARGO_FEATURES "" CACHE STRING "Additional Cargo features (comma-separated)")

# ============================================================================
# Corrosion: CMake integration for Cargo
# ============================================================================
include(FetchContent)

FetchContent_Declare(
    Corrosion
    GIT_REPOSITORY https://github.com/corrosion-rs/corrosion.git
    GIT_TAG v0.5.1
)
FetchContent_MakeAvailable(Corrosion)

# ============================================================================
# Configure Cargo Features Based on Options
# ============================================================================

# Build the feature list
set(CARGO_FEATURES "")

if(NANO_ROS_EMBEDDED)
    # Embedded: no std, use alloc and selected transport
    list(APPEND CARGO_FEATURES "alloc")
    list(APPEND CARGO_FEATURES "${NANO_ROS_TRANSPORT}")

    # Add polling feature for embedded
    list(APPEND CARGO_FEATURES "polling")
else()
    # Desktop: std and zenoh (or selected transport)
    list(APPEND CARGO_FEATURES "std")
    list(APPEND CARGO_FEATURES "alloc")
    list(APPEND CARGO_FEATURES "${NANO_ROS_TRANSPORT}")
endif()

# Add any additional user-specified features
if(NANO_ROS_CARGO_FEATURES)
    string(REPLACE "," ";" EXTRA_FEATURES "${NANO_ROS_CARGO_FEATURES}")
    list(APPEND CARGO_FEATURES ${EXTRA_FEATURES})
endif()

# Remove duplicates
list(REMOVE_DUPLICATES CARGO_FEATURES)

# Convert to comma-separated string for Corrosion
string(REPLACE ";" "," CARGO_FEATURES_STR "${CARGO_FEATURES}")

message(STATUS "nano-ros-cpp: Embedded mode = ${NANO_ROS_EMBEDDED}")
message(STATUS "nano-ros-cpp: Transport = ${NANO_ROS_TRANSPORT}")
message(STATUS "nano-ros-cpp: Cargo features = ${CARGO_FEATURES_STR}")

# ============================================================================
# Import Rust Crate
# ============================================================================

if(NANO_ROS_RUST_TARGET)
    message(STATUS "nano-ros-cpp: Cross-compiling for ${NANO_ROS_RUST_TARGET}")

    corrosion_import_crate(
        MANIFEST_PATH Cargo.toml
        NO_DEFAULT_FEATURES
        FEATURES ${CARGO_FEATURES}
    )

    # Set the Rust target
    corrosion_set_hostbuild(nano_ros_cpp_bridge)
    set(Rust_CARGO_TARGET_CACHED ${NANO_ROS_RUST_TARGET})
else()
    # Host build with selected features
    corrosion_import_crate(
        MANIFEST_PATH Cargo.toml
        NO_DEFAULT_FEATURES
        FEATURES ${CARGO_FEATURES}
    )

    # Detect host target for cxx bridge directory
    if(NOT DEFINED Rust_CARGO_TARGET_CACHED)
        execute_process(
            COMMAND rustc -vV
            OUTPUT_VARIABLE RUSTC_VERSION_OUTPUT
        )
        string(REGEX MATCH "host: ([^\n]+)" _ ${RUSTC_VERSION_OUTPUT})
        set(Rust_CARGO_TARGET_CACHED ${CMAKE_MATCH_1})
    endif()
endif()

# cxx generates headers in cargo/build/<target>/cxxbridge/
set(CXX_BRIDGE_DIR "${CMAKE_BINARY_DIR}/cargo/build/${Rust_CARGO_TARGET_CACHED}/cxxbridge")

# ============================================================================
# ETL (Embedded Template Library) - Optional
# ============================================================================

if(NANO_ROS_USE_ETL)
    find_package(etl QUIET)
    if(NOT etl_FOUND)
        message(STATUS "nano-ros-cpp: Fetching ETL...")
        FetchContent_Declare(
            etl
            GIT_REPOSITORY https://github.com/ETLCPP/etl.git
            GIT_TAG 20.38.0
        )
        FetchContent_MakeAvailable(etl)
    endif()
endif()

# ============================================================================
# C++ Wrapper Library
# ============================================================================

# Source files - desktop vs embedded
if(NANO_ROS_EMBEDDED)
    set(NANO_ROS_CPP_SOURCES
        cpp/embedded.cpp
    )
else()
    set(NANO_ROS_CPP_SOURCES
        cpp/init.cpp
    )
endif()

add_library(nano_ros_cpp ${NANO_ROS_CPP_SOURCES})

# Include directories
target_include_directories(nano_ros_cpp
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CXX_BRIDGE_DIR}
)

# Link against the Rust static library
target_link_libraries(nano_ros_cpp
    PRIVATE
        nano_ros_cpp_bridge
)

# ============================================================================
# Platform-Specific Configuration
# ============================================================================

if(NANO_ROS_EMBEDDED)
    # Embedded mode definitions
    target_compile_definitions(nano_ros_cpp
        PUBLIC
            NANO_ROS_EMBEDDED=1
    )

    # Disable exceptions and RTTI for smaller binary
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(nano_ros_cpp
            PRIVATE
                -fno-exceptions
                -fno-rtti
        )
    endif()

    # Static library only for embedded
    set_target_properties(nano_ros_cpp PROPERTIES
        POSITION_INDEPENDENT_CODE OFF
    )
else()
    # Desktop: link against system libraries
    if(UNIX AND NOT APPLE)
        target_link_libraries(nano_ros_cpp PRIVATE pthread dl m)
    elseif(APPLE)
        target_link_libraries(nano_ros_cpp PRIVATE pthread dl)
    endif()
endif()

# ETL support
if(NANO_ROS_USE_ETL)
    target_compile_definitions(nano_ros_cpp PUBLIC NANO_ROS_USE_ETL=1)
    target_link_libraries(nano_ros_cpp PUBLIC etl::etl)
endif()

# ============================================================================
# Installation (Desktop only)
# ============================================================================

if(NOT NANO_ROS_EMBEDDED)
    include(GNUInstallDirs)

    install(TARGETS nano_ros_cpp
        EXPORT nano_ros_cpp-targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )

    install(DIRECTORY include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    )

    install(EXPORT nano_ros_cpp-targets
        FILE nano_ros_cpp-targets.cmake
        NAMESPACE nano_ros::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/nano_ros_cpp
    )

    # Generate package config
    include(CMakePackageConfigHelpers)

    configure_package_config_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/nano_ros_cpp-config.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/nano_ros_cpp-config.cmake
        INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/nano_ros_cpp
    )

    write_basic_package_version_file(
        ${CMAKE_CURRENT_BINARY_DIR}/nano_ros_cpp-config-version.cmake
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY SameMajorVersion
    )

    install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/nano_ros_cpp-config.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/nano_ros_cpp-config-version.cmake
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/nano_ros_generate_interfaces.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/nano_ros_cpp
    )

    # Install the generator script
    install(PROGRAMS
        ${CMAKE_CURRENT_SOURCE_DIR}/scripts/nano_ros_generate_cpp.py
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/nano_ros_cpp/../scripts
    )
endif()
