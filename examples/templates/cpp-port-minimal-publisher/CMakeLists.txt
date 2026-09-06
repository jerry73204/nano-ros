# cpp-port-minimal-publisher — Phase 209.G iter 2.
#
# Vendors the canonical ROS 2 "minimal publisher" tutorial node UNMODIFIED and
# builds it against nano-ros via the 209.A-D compat surface. The CMakeLists.txt
# nano-ros glue (the three lines below `# 1)`/`# 2)`/`# 3)`) is the only
# delta from a stock ROS 2 ament_cmake_auto package.

cmake_minimum_required(VERSION 3.22)
project(minimal_publisher LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1) Pull nano-ros (NanoRos::NanoRos / NanoRos::NanoRosCpp / nros_find_interfaces).
set(NANO_ROS_PLATFORM posix)
set(NROS_RMW "zenoh" CACHE STRING "Active RMW (zenoh|cyclonedds|xrce).")
set(NANO_ROS_RMW "${NROS_RMW}")
# Resolve the nano-ros checkout root once (phase-277 W6 standard guard):
#   1. -DNANO_ROS_ROOT=<path> cache var (explicit, e.g. a copied-out example),
#   2. NROS_REPO_DIR env var (exported by nano-ros's activate.sh),
#   3. in-repo relative walk-up (this example inside the nano-ros tree).
if(NOT DEFINED NANO_ROS_ROOT)
    if(DEFINED ENV{NROS_REPO_DIR} AND NOT "$ENV{NROS_REPO_DIR}" STREQUAL "")
        set(NANO_ROS_ROOT "$ENV{NROS_REPO_DIR}")
    else()
        get_filename_component(NANO_ROS_ROOT
            "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
    endif()
endif()
add_subdirectory("${NANO_ROS_ROOT}" nano_ros)

# 2) rclcpp / ament_cmake_auto / rclcpp_components source-compat.
include("${NANO_ROS_ROOT}/cmake/compat/NrosRclcppCompat.cmake")

# 3) Generate the message bindings the ported source includes. package.xml is the
#    single source of truth for the msg deps; nros_find_interfaces() resolves the
#    transitive closure (builtin_interfaces ← std_msgs) from it.
nros_find_interfaces(LANGUAGE CPP SKIP_INSTALL)

# --- below: unmodified stock-ROS-2 ament_cmake_auto pattern ---

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(minimal_publisher src/minimal_publisher.cpp)
ament_target_dependencies(minimal_publisher rclcpp)
target_link_libraries(minimal_publisher PRIVATE std_msgs__nano_ros_cpp)


# issue 0719 — an image path that does not go through `nano_ros_entry()` still
# has to apply the image's cross-cutting facts, so it calls the shared applier.
#
# Today that fact is the panic policy; the point of routing through ONE function
# is that whatever is added to it next arrives here too. The two paths that
# already did this (#0688's board seam, #0700's ESP-IDF shim) each got it after
# a build broke with `#[panic_handler] required` four crates from the cause.
#
# The include is guarded because these files are reached several ways
# (`find_package(nano_ros)`, `add_subdirectory`, an ament overlay) and not all
# of them have pulled `NanoRosEntry.cmake` first.
if(NOT COMMAND nros_apply_panic_policy)
    include("${NANO_ROS_ROOT}/cmake/NanoRosFeatureSet.cmake")
endif()
nros_apply_panic_policy(platform "examples/templates/cpp-port-minimal-publisher")

ament_auto_package()
