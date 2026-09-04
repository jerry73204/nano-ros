# NrosRclcppCompat.cmake — Phase 209.B
#
# Maps a stock ROS 2 / ament_cmake_auto `CMakeLists.txt` onto the nano-ros
# consumption surface, so an unmodified ROS 2 package builds with **one new
# `include()` at the top** instead of a rewrite.
#
# Usage (from a ported package's CMakeLists.txt):
#
#   cmake_minimum_required(VERSION 3.20)
#   project(my_node LANGUAGES C CXX)
#   # 1) Bring nano-ros in (one of the two canonical shapes):
#   #    add_subdirectory(<path-to-nano-ros> nano_ros)
#   #    -OR- find_package(NanoRos CONFIG REQUIRED)
#   set(NANO_ROS_PLATFORM posix)        # or zephyr / freertos / nuttx / …
#   set(NANO_ROS_RMW      zenoh)
#   add_subdirectory(${NANO_ROS_SRC_DIR} nano_ros)
#   # 2) Pull this module — everything below is unmodified ament_cmake_auto:
#   include(${NANO_ROS_SRC_DIR}/cmake/compat/NrosRclcppCompat.cmake)
#
#   find_package(ament_cmake_auto REQUIRED)
#   ament_auto_find_build_dependencies()
#   ament_auto_add_library(my_node SHARED src/my_node.cpp)
#   rclcpp_components_register_node(my_node PLUGIN "my_ns::MyNode" EXECUTABLE my_node_exe)
#   ament_auto_package(INSTALL_TO_SHARE config launch)
#
# What this module does
# ---------------------
# * Prepends `cmake/compat/stubs/` to CMAKE_MODULE_PATH so `find_package(<pkg>)`
#   for the common ROS 2 packages (`ament_cmake_auto`, `ament_cmake`, `rclcpp`,
#   `rclcpp_components`, `std_msgs`, `builtin_interfaces`, …) resolves to a
#   no-op `Find<pkg>.cmake` stub. Where a stub also defines an `IMPORTED`
#   target (e.g. `rclcpp::rclcpp`), it transparently aliases to
#   `NanoRos::NanoRosCpp` so `target_link_libraries(... rclcpp::rclcpp)` works.
# * Defines the `ament_*` / `rclcpp_components_*` cmake **functions** so
#   `ament_auto_add_library` / `ament_auto_add_executable` /
#   `rclcpp_components_register_node` / `ament_target_dependencies` /
#   `ament_auto_package` translate to `add_library` + `target_link_libraries
#   (NanoRos::NanoRosCpp)` + (where applicable) `nros_platform_link_app(...)`.
# * Force-includes `nros/rclcpp_components_compat.hpp`
#   on each compat-built target so unmodified source compiling against
#   `#include <rclcpp/rclcpp.hpp>` resolves through the compat header without
#   even an include-line edit. (Phase 209.C ships the components stub.)
# * `rclcpp_components_register_node(... EXECUTABLE <bin>)` synthesises a thin
#   `int main()` that constructs the registered class + `rclcpp::spin`s it.
#   nano-ros is single-binary (no runtime composition); the macro turns a
#   component package into one self-contained executable per registration.
#
# Out of scope (kept as the porting user's call):
# * Launch / parameter yaml (Phase 209.F bakes yaml → header).
# * `ament_target_dependencies` for *project-specific* helper packages (e.g.
#   `autoware_universe_utils`) — the user vendors / replaces those.

if(_NROS_RCLCPP_COMPAT_INCLUDED)
    return()
endif()
set(_NROS_RCLCPP_COMPAT_INCLUDED TRUE)

# --- Find-stub directory ------------------------------------------------------
get_filename_component(_nros_compat_dir "${CMAKE_CURRENT_LIST_DIR}" ABSOLUTE)
if(NOT "${_nros_compat_dir}/stubs" IN_LIST CMAKE_MODULE_PATH)
    list(PREPEND CMAKE_MODULE_PATH "${_nros_compat_dir}/stubs")
endif()

# Pull the smart Find-stub helper proactively so it auto-emits workspace
# Find<pkg>.cmake stubs for every pkg under `NROS_INTERFACE_SEARCH_PATH`
# at compat-include time (Phase 210.A.2 + .A.4). Without this the emit
# only happens when a per-pkg delegator fires — which can be after a
# consumer's `find_package(<workspace_pkg>)` runs, defeating it.
include("${_nros_compat_dir}/stubs/_NrosFindRosMsgPackage.cmake")

# --- Sanity: nros-cpp must be loaded -----------------------------------------
# Two consumption shapes for `NanoRos::NanoRosCpp`:
#  1. Native / `add_subdirectory(<nano-ros>)` — the root CMakeLists.txt
#     publishes the IMPORTED INTERFACE target directly.
#  2. Zephyr — `find_package(Zephyr)` auto-loads the nros zephyr module
#     (`zephyr/CMakeLists.txt`) which calls `zephyr_library_named(nros)`
#     under `CONFIG_NROS_CPP_API=y`. There's no `NanoRos::NanoRosCpp`
#     target on Zephyr — the `nros` zephyr_library is its equivalent.
#     Bridge via ALIAS so the rest of this module + the per-pkg stubs
#     see one canonical name (Phase 210.E.3.c).
if(CONFIG_NROS_CPP_API AND NOT TARGET NanoRos::NanoRosCpp AND TARGET nros)
    add_library(NanoRos::NanoRosCpp ALIAS nros)
    set(_NROS_COMPAT_ON_ZEPHYR TRUE)
endif()

if(NOT TARGET NanoRos::NanoRosCpp)
    message(FATAL_ERROR
        "NrosRclcppCompat: NanoRos::NanoRosCpp not found.\n"
        "Include this module AFTER bringing nano-ros in:\n"
        "  add_subdirectory(<path-to-nano-ros> nano_ros)\n"
        "or `find_package(NanoRos CONFIG REQUIRED)`,\n"
        "or build inside a Zephyr application with CONFIG_NROS_CPP_API=y.")
endif()

# --- Compile flags applied to every compat-built target ----------------------
# `-include` force-pulls the headers so the ported source's
# `#include <rclcpp/rclcpp.hpp>` resolves without an edit.
# `SHELL:` keeps each `-include FILE` pair together as a single shell-tokenized
# arg — without it cmake dedupes the repeated `-include` flag and the second
# header path becomes a stray input file (`cannot specify '-o' with '-c' with
# multiple files`).
set(_NROS_COMPAT_FORCE_INCLUDES
    "$<$<COMPILE_LANGUAGE:CXX>:SHELL:-include nros/rclcpp_components_compat.hpp>"
)
# CACHE so functions defined below see it without recapturing scope.
set(_NROS_COMPAT_FORCE_INCLUDES "${_NROS_COMPAT_FORCE_INCLUDES}" CACHE INTERNAL "")

function(_nros_compat_apply_force_includes target)
    if(NOT TARGET ${target})
        return()
    endif()
    target_compile_options(${target} PRIVATE ${_NROS_COMPAT_FORCE_INCLUDES})
    # Make `#include <rclcpp/rclcpp.hpp>` (and the components register macro
    # header) resolve to the nano-ros compat shims.
    target_include_directories(${target} PRIVATE
        "${_nros_compat_dir}/include")
endfunction()

# Phase 210.E.3.c — Zephyr context: do NOT auto-apply force-include of
# `nros/rclcpp_compat.hpp` to the `app` target. The compat header pulls
# `<memory>` / `<string>` / `<functional>` / `<vector>` / `<chrono>`
# — Zephyr's minimal C++ stdlib doesn't ship most of those (only
# `<chrono>` is shimmed under `zephyr/cxx-compat/`). Existing Zephyr
# cpp examples use `nros::Node` directly (NOT rclcpp); polluting them
# with rclcpp_compat.hpp would break their build for no benefit.
#
# A user who actually ports rclcpp source to Zephyr opts in by adding
# `target_compile_options(app PRIVATE ${_NROS_COMPAT_FORCE_INCLUDES})`
# + `target_include_directories(app PRIVATE ${_nros_compat_dir}/include)`
# from their own CMakeLists — but they'd also hit the `<memory>` blocker
# (Phase 209.G.2 — Zephyr libstdc++ subset shim project).
#
# Native builds still get the auto-include via the ament_auto_* shim
# entry points.
if(_NROS_COMPAT_ON_ZEPHYR)
    # Just publish the include dir so an explicit `#include <nros/
    # rclcpp_compat.hpp>` resolves if a user reaches for it.
    zephyr_include_directories("${_nros_compat_dir}/include")
endif()

# --- ament_cmake_auto shims ---------------------------------------------------

function(ament_auto_find_build_dependencies)
    # nano-ros deps come from `target_link_libraries(NanoRos::NanoRosCpp)` (set
    # by the *_auto_add_* functions below) — there is no manifest-scan step.
endfunction()

function(ament_auto_add_library target kind)
    # kind ∈ SHARED | STATIC | MODULE — nano-ros prefers STATIC for the
    # single-binary embedded case; the kind argument is honoured (a host
    # tooling consumer may legitimately want SHARED) but the default is STATIC.
    set(_srcs ${ARGN})
    if("${kind}" STREQUAL "SHARED" OR "${kind}" STREQUAL "STATIC"
       OR "${kind}" STREQUAL "MODULE")
        add_library(${target} ${kind} ${_srcs})
    else()
        # If no kind keyword was passed (ament_cmake_auto often omits it),
        # treat the first arg as a source path.
        add_library(${target} STATIC ${kind} ${_srcs})
    endif()
    target_link_libraries(${target} PUBLIC NanoRos::NanoRosCpp)
    _nros_compat_apply_force_includes(${target})
endfunction()

function(ament_auto_add_executable target)
    add_executable(${target} ${ARGN})
    target_link_libraries(${target} PRIVATE NanoRos::NanoRosCpp)
    _nros_compat_apply_force_includes(${target})
    if(COMMAND nros_platform_link_app)
        nros_platform_link_app(${target})
    endif()
endfunction()

# --- ament_target_dependencies / ament_export_* shims -------------------------

function(ament_target_dependencies target)
    # Stock ROS 2 form: ament_target_dependencies(<target> rclcpp std_msgs …).
    # Each dep is a *package* whose `find_package(<dep>)` defined a target.
    # Wire only the deps the stubs actually create targets for (rclcpp +
    # rclcpp_components today); the rest are no-ops because nano-ros pulls in
    # the message + ROS surface through NanoRos::NanoRosCpp anyway.
    foreach(_dep IN LISTS ARGN)
        if(TARGET ${_dep}::${_dep})
            target_link_libraries(${target} PRIVATE ${_dep}::${_dep})
        endif()
    endforeach()
endfunction()

function(ament_export_dependencies)
    # No-op — nano-ros has no ament install layout; the embedded build does not
    # need exported package deps.
endfunction()

function(ament_export_include_directories)
    # No-op — INSTALL_INTERFACE include dirs are an ament-install concept.
endfunction()

function(ament_export_libraries)
endfunction()

function(ament_export_targets)
endfunction()

# --- rclcpp_components_register_node ------------------------------------------

function(rclcpp_components_register_node component_target)
    cmake_parse_arguments(_RCRN "" "PLUGIN;EXECUTABLE;RESOURCE_INDEX" "" ${ARGN})
    if(NOT _RCRN_EXECUTABLE OR NOT _RCRN_PLUGIN)
        # In the upstream macro, omitting EXECUTABLE installs a plugin index
        # consumed by the runtime ComponentManager. nano-ros has no dynamic
        # composer — without an EXECUTABLE name there's nothing to emit.
        return()
    endif()
    set(_gen_dir "${CMAKE_CURRENT_BINARY_DIR}/nros_compat_main")
    set(_gen_src "${_gen_dir}/${_RCRN_EXECUTABLE}_main.cpp")
    file(MAKE_DIRECTORY "${_gen_dir}")
    # Generated entry point. Prefer the stock ROS 2 component constructor
    # `T(rclcpp::NodeOptions{})`, then keep the older nano-ros smoke shapes
    # `T()` and `T(std::string)` source-compatible.
    file(GENERATE OUTPUT "${_gen_src}" CONTENT
        "// Generated by NrosRclcppCompat.cmake for ${_RCRN_EXECUTABLE}\n"
        "// (PLUGIN ${_RCRN_PLUGIN}). nano-ros is single-binary; this entry\n"
        "// point replaces the runtime ComponentManager loading dance.\n"
        "#include <nros/nros.hpp>\n"
        "#include <memory>\n"
        "#include <string>\n"
        "#include <type_traits>\n"
        "namespace nros_compat_component_detail {\n"
        "template <typename T>\n"
        "typename std::enable_if<std::is_constructible<T, rclcpp::NodeOptions>::value, std::shared_ptr<T>>::type\n"
        "make_component(const char*) {\n"
        "    return std::make_shared<T>(rclcpp::NodeOptions{});\n"
        "}\n"
        "template <typename T>\n"
        "typename std::enable_if<!std::is_constructible<T, rclcpp::NodeOptions>::value && std::is_constructible<T>::value, std::shared_ptr<T>>::type\n"
        "make_component(const char*) {\n"
        "    return std::make_shared<T>();\n"
        "}\n"
        "template <typename T>\n"
        "typename std::enable_if<!std::is_constructible<T, rclcpp::NodeOptions>::value && !std::is_constructible<T>::value && std::is_constructible<T, const std::string&>::value, std::shared_ptr<T>>::type\n"
        "make_component(const char* name) {\n"
        "    return std::make_shared<T>(std::string(name));\n"
        "}\n"
        "} // namespace nros_compat_component_detail\n"
        "extern \"C\" int main(int argc, char** argv) {\n"
        "    rclcpp::init(argc, argv);\n"
        "    auto node = nros_compat_component_detail::make_component<${_RCRN_PLUGIN}>(\"${_RCRN_EXECUTABLE}\");\n"
        "    rclcpp::spin(std::dynamic_pointer_cast<rclcpp::Node>(node));\n"
        "    rclcpp::shutdown();\n"
        "    return 0;\n"
        "}\n"
    )
    add_executable(${_RCRN_EXECUTABLE} "${_gen_src}")
    target_link_libraries(${_RCRN_EXECUTABLE} PRIVATE
        ${component_target} NanoRos::NanoRosCpp)
    _nros_compat_apply_force_includes(${_RCRN_EXECUTABLE})
    if(COMMAND nros_platform_link_app)
        nros_platform_link_app(${_RCRN_EXECUTABLE})
    endif()
endfunction()

# --- ament_auto_package + ament_package --------------------------------------

function(ament_auto_package)
    # Stock form: `ament_auto_package(INSTALL_TO_SHARE config launch)`. The
    # nano-ros embedded target has no `share/<pkg>` install layout; the args
    # are silently ignored. Launch/yaml is Phase 209.F.
endfunction()

function(ament_package)
endfunction()
