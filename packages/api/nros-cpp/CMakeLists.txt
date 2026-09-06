# packages/api/nros-cpp/CMakeLists.txt
#
# nros C++ API: header-only C++14 library + Rust FFI staticlib.
# Provides the NanoRos::NanoRosCpp target.
#
# This file is included from the top-level CMakeLists.txt via add_subdirectory().
# Two variables control the build:
#   NANO_ROS_RMW      — RMW backend: zenoh, xrce, or dds (default: zenoh)
#   NANO_ROS_PLATFORM — target platform: posix, freertos_armcm3, nuttx_armv7a
#                       (default: posix)

# ---- Platform (default: posix; may already be set by nros-c) ----
if(NOT DEFINED NANO_ROS_PLATFORM)
    set(NANO_ROS_PLATFORM "posix")
endif()

# ---- Cargo features: ONE computation, shared with nros-c + the umbrella ----
# phase-314 — was ~90 lines of inline rmw/platform mapping that hardcoded
# `ros-humble` and applied capabilities only on posix. See cmake/NanoRosFeatureSet.cmake
# and issue 0311 for why the duplication was a defect rather than a tidy-up.
# Relative to THIS file, not ${NANO_ROS_ROOT}: a raw `add_subdirectory(<repo>)`
# consumer (the phase-138 smoke shape) never defines NANO_ROS_ROOT.
include("${CMAKE_CURRENT_LIST_DIR}/../../../cmake/NanoRosFeatureSet.cmake")
option(NANO_ROS_SAFETY_E2E "Enable E2E message-integrity (CRC) — zenoh only" OFF)
set(_caps ${NANO_ROS_FEATURES})
if(NANO_ROS_SAFETY_E2E)
    list(APPEND _caps safety)
endif()
# phase-323 W2 — capabilities come from the DECLARATION on every platform.
#
# posix used to get `param_services` + `lifecycle` unconditionally. That was not
# a convenience: until W1 it was the ONLY route those axes took on hosted, since
# `NANO_ROS_FEATURES` was never populated on the workspace path (issue 0353). So
# hosted could not fail when a declaration was missing, and the same
# `system.toml` produced different runtimes depending on where it was built.
#
# W1 made `nano_ros_workspace()` resolve the axes from `SYSTEM` before the
# import, so the declaration now arrives on its own and this can go.
nros_feature_set(_cpp_features
    CRATE        cpp
    RMW          "${NANO_ROS_RMW}"
    PLATFORM     "${NANO_ROS_PLATFORM}"
    BOARD        "${NANO_ROS_BOARD}"
    CAPABILITIES "${_caps}")

# ---- Build nros-cpp via Corrosion ----
# phase-336 — PROFILE is passed explicitly. Without it Corrosion falls back to
# its own CMAKE_BUILD_TYPE mapping (Debug→dev, EVERYTHING else→release), so a
# Debug or MinSizeRel build of the C/C++ half still got a fat-LTO cargo build.
# Relative to THIS file, like the FeatureSet include above — a raw
# `add_subdirectory(<repo>)` consumer never defines NANO_ROS_ROOT.
include("${CMAKE_CURRENT_LIST_DIR}/../../../cmake/NanoRosCargoProfile.cmake")
# phase-347 W3 — `nros_rmw_dispatch()` / `nros_rmw_is_known()` (RFC-0071).
include("${CMAKE_CURRENT_LIST_DIR}/../../../cmake/NanoRosRmwDispatch.cmake")
nros_resolve_cargo_profile()
corrosion_import_crate(
    MANIFEST_PATH "${PROJECT_SOURCE_DIR}/Cargo.toml"
    CRATES        nros-cpp
    CRATE_TYPES   staticlib
    NO_DEFAULT_FEATURES
    FEATURES      ${_cpp_features}
    PROFILE       ${NROS_CARGO_PROFILE}
    LOCKED
)
# The preset's definition, so the crate resolves `--profile nros-…` without a
# `[profile.*]` block of its own. No-op for a user-owned profile.
nros_cargo_profile_env(nros_cpp-static)
# issue 0657 — the riscv64 float ABI, per target (see the helper's comment).
nros_riscv64_rustflags_env(nros_cpp-static)
# phase-372 W1 — the cortex-r52 FPU cflags, same class one arch over.
nros_armv8r_cflags_env(nros_cpp-static)

# phase-392 W5.g follow-up — the entity figures the RMW sizes its queryable
# table and payload classes from.
#
# HERE, and not only on the workspace umbrella, because `zpico-sys` is compiled
# once per CARGO ROOT and this staticlib imports from the REPO ROOT while the
# umbrella imports from its own synthesised one. Measured on
# `examples/workspaces/mixed`: 6 `zpico-sys` units, of which only the umbrella's
# could ever see the env. A pure-C/C++ workspace never even reaches the umbrella
# call site (`nros_synth_runtime_umbrella` returns early for it), so no unit was
# reachable at all — which is why W5.g found the delivery missing in every
# workspace it checked.
#
# DEFERRED: the accumulator is filled by `nano_ros_add_entry()`, which runs long
# after this file. Applying it inline reads an empty set, which is the W5.g
# defect itself.
# Relative to THIS file, not ${NANO_ROS_ROOT} — see the note above.
include("${CMAKE_CURRENT_LIST_DIR}/../../../cmake/NanoRosEntityFacts.cmake")
nros_entity_facts_env_deferred(nros_cpp-static)


# ---- Header-only C++ interface library ----
add_library(nros-cpp-headers INTERFACE)
add_library(NanoRos::NanoRosCpp ALIAS nros-cpp-headers)

target_include_directories(nros-cpp-headers INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Phase 144 — mirror the per-build `nros_cpp_config_generated.h` (produced
# by nros-cpp's build.rs into CMAKE_CURRENT_BINARY_DIR via corrosion) to
# a sibling `include/nros/` dir that we add to nros-cpp-headers' INTERFACE.
# Same pattern as the nros-c sub-project (see Phase 137 comment block in
# `packages/api/nros-c/CMakeLists.txt`). The mirrored copy is searched
# BEFORE the source-tree stub, so per-variant storage sizes take effect
# under add_subdirectory(<repo-root>) consumption.
set(_nros_cpp_intree_include_dir "${CMAKE_CURRENT_BINARY_DIR}/include")
file(MAKE_DIRECTORY "${_nros_cpp_intree_include_dir}/nros")
if(TARGET cargo-build_nros_cpp)
    # Issue 0088 — first-class OUTPUT + custom target instead of a POST_BUILD side
    # effect (see the matching comment in packages/api/nros-c/CMakeLists.txt). The
    # POST_BUILD `BYPRODUCTS` gave consumers no real edge to the mirrored headers, so
    # a consumer TU could compile before the copy ran and pick up the in-tree stub.
    # `cargo-build_nros_cpp` emits BOTH the C++ variant header AND (since the C++
    # umbrella bundles nros-c, Phase 241.D3-rev) the C variant header into this dir.
    set(_nros_cpp_gen_cpp_header "${_nros_cpp_intree_include_dir}/nros/nros_cpp_config_generated.h")
    set(_nros_cpp_gen_c_header "${_nros_cpp_intree_include_dir}/nros/nros_config_generated.h")
    # Issue 0268 — see the matching comment block in packages/api/nros-c/CMakeLists.txt.
    # `DEPENDS cargo-build_nros_cpp` alone is ORDER-ONLY in Ninja, so a rebuilt crate
    # never invalidates this copy and the mirror goes stale the moment a core struct
    # grows. Depend on the staticlib file itself — a real Ninja output — so every cargo
    # rebuild re-mirrors both headers.
    if(TARGET nros_cpp-static)
        set(_nros_cpp_mirror_trigger "$<TARGET_FILE:nros_cpp-static>")
    else()
        set(_nros_cpp_mirror_trigger "")
    endif()
    set(_NROS_CPP_MIRROR_SH "${CMAKE_CURRENT_LIST_DIR}/../../../scripts/build/mirror-generated-header.sh")
    add_custom_command(
        OUTPUT "${_nros_cpp_gen_cpp_header}" "${_nros_cpp_gen_c_header}"
        # issue 0805 — see the matching comment in nros-c/CMakeLists.txt: the
        # leaf's own copy is written only when the build script runs for THIS
        # leaf, which a shared cargo target dir makes untrue after the first.
        COMMAND bash "${_NROS_CPP_MIRROR_SH}"
            "${CMAKE_CURRENT_BINARY_DIR}/nros_cpp_config_generated.h"
            "${CMAKE_BINARY_DIR}" "nros-cpp-generated" "nros_cpp_config_generated.h"
            "${_nros_cpp_gen_cpp_header}"
        COMMAND bash "${_NROS_CPP_MIRROR_SH}"
            "${CMAKE_CURRENT_BINARY_DIR}/nros_config_generated.h"
            "${CMAKE_BINARY_DIR}" "nros-c-generated" "nros_config_generated.h"
            "${_nros_cpp_gen_c_header}"
        DEPENDS cargo-build_nros_cpp ${_nros_cpp_mirror_trigger}
        COMMENT "nros-cpp: mirroring nros_{,cpp_}config_generated.h for in-tree consumers"
        VERBATIM
    )
    add_custom_target(nros_cpp_config_header
        DEPENDS "${_nros_cpp_gen_cpp_header}" "${_nros_cpp_gen_c_header}")
    # Issue 0268 — heal an ALREADY-drifted mirror at configure time. The Ninja edge
    # above only fires when the crate rebuilds, so a tree that went stale under the
    # old order-only rule keeps its museum copy until something happens to touch the
    # crate. `file(COPY_FILE)` deliberately, NOT `configure_file`: the latter adds a
    # CMAKE_CONFIGURE_DEPENDS edge on a file cargo rewrites mid-build, which would
    # re-trigger configure on every build.
    # Issue 0985 — the heal runs the SAME resolver the build-time mirror runs.
    #
    # It used to `file(COPY_FILE)` straight from `${CMAKE_CURRENT_BINARY_DIR}`,
    # i.e. the leaf's OWN corrosion output — which is exactly the "present is
    # not current" source issue 0978 removed from the mirror script. Once
    # leaves share a cargo target dir the build script runs once per
    # (crate, feature set), so that copy is present and arbitrarily old for
    # every leaf after the first, and this loop wrote a museum header over a
    # correct one.
    #
    # It then made the drift UNREPAIRABLE, which is the half worth spelling out:
    # the copy stamps the mirror's OUTPUT with a NEW mtime, so ninja finds the
    # output newer than `$<TARGET_FILE:nros_cpp-static>` and skips the very
    # custom command that would have written the current bytes. A repair for
    # drift (issue 0268) that causes drift and then suppresses its own fix.
    #
    # Measured 2026-09-02 in examples/native/c/listener/build-zenoh: the C copy
    # current at 03:12:42, the C++ copy STALE at 03:12:43 — one second later,
    # same build, same generated content — and `c_listener` failing to link on
    # issue 0369's size anchor.
    #
    # Best-effort by construction: on a tree that has never built, neither
    # candidate exists and the script exits non-zero saying so. That is not a
    # configure error — the build-time edge is the real mechanism and this is
    # only a repair for a tree that is already drifted — so the result is
    # ignored, exactly as the old `if(EXISTS ...)` guard made it best-effort.
    foreach(_nros_cpp_heal
            "nros_cpp_config_generated.h;nros-cpp-generated"
            "nros_config_generated.h;nros-c-generated")
        list(GET _nros_cpp_heal 0 _nros_cpp_hdr)
        list(GET _nros_cpp_heal 1 _nros_cpp_gendir)
        execute_process(
            COMMAND bash "${_NROS_CPP_MIRROR_SH}"
                "${CMAKE_CURRENT_BINARY_DIR}/${_nros_cpp_hdr}"
                "${CMAKE_BINARY_DIR}" "${_nros_cpp_gendir}" "${_nros_cpp_hdr}"
                "${_nros_cpp_intree_include_dir}/nros/${_nros_cpp_hdr}"
            RESULT_VARIABLE _nros_cpp_heal_rc
            OUTPUT_QUIET ERROR_QUIET)
    endforeach()
    target_include_directories(nros-cpp-headers BEFORE INTERFACE
        $<BUILD_INTERFACE:${_nros_cpp_intree_include_dir}>)
    # Issue 0090 — export the absolute mirror-header paths (see nros-c/CMakeLists)
    # for a FILE-level OBJECT_DEPENDS on the threadx carrier's C++ TUs.
    set_property(GLOBAL PROPERTY NROS_CPP_CONFIG_HEADER_FILE
        "${_nros_cpp_gen_c_header};${_nros_cpp_gen_cpp_header}")
    # Issue 0123 — the mirror dir above is prepended to nros-cpp-headers' INTERFACE, but
    # the `nros_cpp_config_header` target (which mirrors BOTH the cpp and the c sizes
    # headers into it) only builds if something depends on it. A plain rclcpp/ament
    # consumer that pulls nros-cpp only transitively (e.g. workspace-shadowing's
    # `consumer` via the std_msgs binding) never wires that dep, so the mirror never
    # runs during `all` → its dir stays empty → the consumer reads the in-tree stub.
    # Make the real staticlib depend on the mirror so ANY consumer linking nano-ros C++
    # (directly or transitively) builds the per-build headers before compiling.
    if(TARGET nros_cpp-static)
        add_dependencies(nros_cpp-static nros_cpp_config_header)
    endif()
endif()

# Phase 241.D3-rev — the C++ umbrella (bundles nros-c + the backend) is the ONLY
# Rust staticlib on a C++ link line ⇒ `std` once ⇒ no `--allow-multiple-definition`.
# Linked NORMALLY (not whole-archive): whole-archiving a Rust staticlib drags its
# compiler-builtins intrinsics into a collision with the system libgcc. The surface
# is pulled lazily; the backend registers via the NanoRosLink.cmake stub.
target_link_libraries(nros-cpp-headers INTERFACE nros_cpp-static)

# Phase 241.D3-rev — pull NanoRos's C-side HEADERS + platform-compile-def + platform
# shim, but NOT its `nros_c-static` ARCHIVE: a C++ binary links ONLY libnros_cpp.a
# (already bundling nros-c). Linking the C archive too would put a SECOND Rust
# staticlib (a second `std`) on the line — the exact duplicate this phase removes.
# Deferred to the root dir where NanoRos + NanoRos::Platform are declared (after this
# add_subdirectory). 192.3: root via NANO_ROS_ROOT_DIR; relative fallback standalone.
if(NOT DEFINED NANO_ROS_ROOT_DIR)
    set(NANO_ROS_ROOT_DIR "${CMAKE_CURRENT_LIST_DIR}/../../..")
endif()
# C-side headers: pull from `nros_c-static`'s INTERFACE — it carries the public C
# headers AND the per-build variant header (`nros_config_generated.h`, the
# *_OPAQUE_U64S sizes). NanoRos's OWN INTERFACE_INCLUDE_DIRECTORIES does NOT contain
# them: NanoRos gets them transitively via its link to nros_c-static, and
# `$<TARGET_PROPERTY>` resolves only the named target's own property, not transitive
# usage requirements. Without the variant header the C++ TU reaches nros-c's stub
# `nros_config_generated.h` and fails (`*_OPAQUE_U64S not declared`). NuttX (no
# nros_c-static) falls back to NanoRos's manually-added include tree.
if(TARGET nros_c-static)
    cmake_language(DEFER DIRECTORY "${NANO_ROS_ROOT_DIR}" CALL
        target_include_directories nros-cpp-headers INTERFACE
        "$<TARGET_PROPERTY:nros_c-static,INTERFACE_INCLUDE_DIRECTORIES>")
else()
    cmake_language(DEFER DIRECTORY "${NANO_ROS_ROOT_DIR}" CALL
        target_include_directories nros-cpp-headers INTERFACE
        "$<TARGET_PROPERTY:NanoRos,INTERFACE_INCLUDE_DIRECTORIES>")
endif()
cmake_language(DEFER DIRECTORY "${NANO_ROS_ROOT_DIR}" CALL
    target_compile_definitions nros-cpp-headers INTERFACE
    "$<TARGET_PROPERTY:NanoRos,INTERFACE_COMPILE_DEFINITIONS>")
cmake_language(DEFER DIRECTORY "${NANO_ROS_ROOT_DIR}" CALL
    target_link_libraries nros-cpp-headers INTERFACE NanoRos::Platform)

# Phase 117.8: when the consumer asks for the Cyclone DDS RMW backend,
# pull in the standalone `nros-rmw-cyclonedds` library + Cyclone DDS
# itself transitively, and flip on the auto-register call inside
# `nros::init` via the `NROS_RMW_CYCLONEDDS` macro.
# phase-347 W3 — one descriptor-driven block, replacing a four-arm
# `if(NANO_ROS_RMW STREQUAL ...)` chain. That chain was the THIRD closed list of
# backends in the tree: it accepted uorb, `NanoRosFeatureSet` accepted uorb, and
# the generated `nros_rmw_dispatch` FATAL_ERROR'd on it. Three lists, and they
# disagreed.
#
# The defines are unchanged and come from each backend's `nros-rmw.toml`
# (`cpp_define`). Their spellings are INCONSISTENT by history — `_CFFI` on
# xrce/zenoh, bare on cyclonedds/uorb — and are preserved exactly, because
# consumer headers `#if` on them. Normalising them is a separate, breaking
# change; this wave moves the fact, it does not rename it.
#
# The per-backend comments that lived on each arm are now on the descriptors.
if(NANO_ROS_RMW AND NOT NANO_ROS_RMW STREQUAL "none")
    nros_rmw_is_known("${NANO_ROS_RMW}" _nros_cpp_rmw_known)
    if(NOT _nros_cpp_rmw_known)
        message(FATAL_ERROR
            "nros-cpp: unknown RMW '${NANO_ROS_RMW}' "
            "(provided by a descriptor: ${NROS_RMW_KNOWN})")
    endif()
    nros_rmw_dispatch("${NANO_ROS_RMW}")
    if(NROS_RMW_CPP_DEFINE)
        target_compile_definitions(nros-cpp-headers INTERFACE ${NROS_RMW_CPP_DEFINE}=1)
    endif()
    # uorb's PX4 shell add_subdirectory's the backend and exposes this target;
    # the guard stays because a consumer may select the rmw without that shell.
    if(NROS_RMW_CMAKE_TARGET AND TARGET ${NROS_RMW_CMAKE_TARGET})
        target_link_libraries(nros-cpp-headers INTERFACE ${NROS_RMW_CMAKE_TARGET})
    endif()
endif()

# Issue 0073 — when the E2E-safety cargo feature is compiled into nros-cpp (above),
# also surface the matching preprocessor define to HEADER CONSUMERS. `node.hpp` /
# `subscription.hpp` gate the validated-subscription API
# (`create_subscription_with_safety`) behind `#if defined(NANO_ROS_SAFETY_E2E)`, so a
# component TU that links `NanoRos::NanoRosCpp` needs the define to see it. Without
# this, the cargo symbol exists but the C++ method is hidden at compile time.
# phase-347 W4 — the backend condition is DECLARED, not named. This read
# `AND NANO_ROS_RMW STREQUAL "zenoh"`, which is a capability coupled to one
# backend by name in a user-facing library. The backend now declares `safety` in
# its `nros-rmw.toml` `[rmw.capabilities]`, and `nros_rmw_dispatch` reports the
# set as `NROS_RMW_CAPABILITIES` — so a third-party backend offering `safety`
# gets the define with no edit here.
if(NANO_ROS_SAFETY_E2E AND "safety" IN_LIST NROS_RMW_CAPABILITIES)
    target_compile_definitions(nros-cpp-headers INTERFACE NANO_ROS_SAFETY_E2E)
endif()

# THE declared C++ floor for everything that consumes nros-cpp — issue 1118.
#
# It read `cxx_std_14` while `component_node.hpp` used `if constexpr` at three
# sites (four with `NROS_CPP_STD`), which is C++17. GCC only WARNS on that
# (`'if constexpr' only available with '-std=c++17'`), so the declaration was a
# claim the code violated and nothing failed. `just check cpp`'s ~15
# `-std=c++14` probes could not see it either: the block sits behind
# `#if defined(NROS_SYSTEM_PARAM_SERVICES)`, a macro only
# `NanoRosCapabilities.cmake` defines and no probe does.
#
# C++17 and not C++20, deliberately: 17 is what the code already needs, what
# ROS 2 Humble's `rclcpp` exports (`cxx_std_17`), and what every toolchain this
# tree pins supports. Nothing in nros-cpp uses a C++20 feature, and C++20
# support varies across the embedded toolchains — that move needs its own
# survey.
#
# This is the REQUIREMENT. It is not the whole ENFORCEMENT: a compile FEATURE
# emits no `-std=` flag when the compiler default already satisfies it, so it
# cannot defend against a raw `-std=c++14` arriving from a toolchain file's
# `CMAKE_CXX_FLAGS_INIT` or from Zephyr's `CONFIG_STD_CPP*`. That is why the
# leaf packages still carry `set(CMAKE_CXX_STANDARD 17)` +
# `CXX_STANDARD_REQUIRED ON` — measured: with REQUIRED, CMake emits the
# standard flag LAST and it wins; without a leaf standard, the raw flag is the
# only `-std=` on the line. `check-cxx-standard-floor` reads the number BELOW
# out of this line and holds every other declaration in the tree at or above
# it, so the floor has exactly one home.
target_compile_features(nros-cpp-headers INTERFACE cxx_std_17)

# Phase 140 — install rules deleted. nros-cpp is consumed in-tree
# via `add_subdirectory(<repo-root>)`; the build-dir variant header
# path is wired via `target_include_directories(nros-cpp-headers
# BEFORE INTERFACE …)` above. NanoRos::NanoRosCpp resolves directly
# against the in-tree `nros-cpp-headers` INTERFACE target.
