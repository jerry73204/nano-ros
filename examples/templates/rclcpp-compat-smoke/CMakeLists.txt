# rclcpp-compat smoke — Phase 209 MVP-quartet integration template.
#
# Pure ament_cmake_auto + rclcpp_components shape, mostly verbatim from what a
# ROS 2 user would write. The only nano-ros-specific lines are:
#   1) the platform / rmw selection + `add_subdirectory(<nano-ros>)`,
#   2) the `include(<nano-ros>/cmake/compat/NrosRclcppCompat.cmake)` one-liner,
#   3) the `nros_find_interfaces()` codegen call (package.xml drives the msg deps).
# Everything below those is unmodified `find_package + ament_auto_*`.

cmake_minimum_required(VERSION 3.22)
project(rclcpp_compat_smoke LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1) Pull nano-ros (provides NanoRos::NanoRos, NanoRos::NanoRosCpp + the codegen
#    function). Five levels up: examples/templates/rclcpp-compat-smoke/.
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

# 2) Drop-in source-compat for rclcpp / ament_cmake_auto / rclcpp_components /
#    diagnostic_updater. Everything below mirrors a stock ROS 2 CMakeLists.
include("${NANO_ROS_ROOT}/cmake/compat/NrosRclcppCompat.cmake")

# 3) Generate the C++ bindings for the message packages this node uses.
#    package.xml is the single source of truth for the msg deps;
#    nros_find_interfaces() resolves the transitive closure (builtin_interfaces,
#    std_msgs, geometry_msgs, diagnostic_msgs) from its <depend> entries.
nros_find_interfaces(LANGUAGE CPP SKIP_INSTALL)

# --- below: unmodified stock-ROS-2 ament_cmake_auto pattern ---

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_updater REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(rclcpp_compat_smoke src/talker.cpp)
ament_target_dependencies(rclcpp_compat_smoke rclcpp diagnostic_updater)
target_link_libraries(rclcpp_compat_smoke PRIVATE
    std_msgs__nano_ros_cpp
    diagnostic_msgs__nano_ros_cpp)


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
nros_apply_panic_policy(platform "examples/templates/rclcpp-compat-smoke")

ament_auto_package()
