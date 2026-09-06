# nros Zephyr Module
# Copyright (c) 2024 nros contributors
# SPDX-License-Identifier: MIT OR Apache-2.0
#
# This module compiles backend-specific C sources centrally and bridges
# Kconfig values to Cargo environment variables, so individual examples
# don't need to manually include them.

if(CONFIG_NROS)

# phase-337 W2.b — ONE spelling of "which flavour of Zephyr is this".
# The C-API branch and the C++ branch below each build
# nros-rmw-zenoh-staticlib, and each computed this feature string itself. When
# the bare-metal arm was added, patching only the first left the second
# silently on the old answer — so the two spellings become one macro here.
#
# Why it needs an arm at all: native_sim borrows the HOST's std, which supplies
# the global allocator and the panic handler. Every real board is no_std and
# must bring its own (`platform-zephyr-baremetal`). native_sim being the only
# Zephyr target this repo has ever built is why nothing noticed.
#
# A macro, not a function: it sets the caller's variable directly. A function
# would need PARENT_SCOPE, which is the same scope footgun the `_NROS_ENTRY_DIR`
# pattern exists to dodge (AGENTS.md CMake Pitfalls).
macro(_nros_zephyr_backend_features _out)
    set(${_out} "platform-zephyr")
    if(CONFIG_BOARD_NATIVE_SIM OR CONFIG_BOARD_NATIVE_POSIX)
        string(APPEND ${_out} ",std")
    else()
        string(APPEND ${_out} ",platform-zephyr-baremetal")
    endif()
endmacro()

include(${CMAKE_CURRENT_LIST_DIR}/cmake/nros_cargo_build.cmake)

# phase-351 W5 / issue 0605 — resolve the board facts ONCE per Zephyr configure,
# at MODULE scope.
#
# Two consumers need them and neither can resolve them itself: this module's own
# `nros_cargo_build()` (the C/C++ core crates), and zephyr-lang-rust's
# `rust_cargo_application()`, which builds the RUST entry's cargo command inside
# the module tree — reached only by the `${NROS_BOARD_FACTS_ENV}` the
# cargo-features patch injects there. Resolving here means the cache entry
# exists before either runs; the app's `APPLICATION_SOURCE_DIR` (an entry leaf,
# carrying `[package.metadata.nros.entry] deploy`) is what it resolves FROM.
nros_resolve_board_facts()
# Phase 212.H.1 — nros_system_generate(<bringup-pkg>) shells the
# `nros codegen-system` host-time bake at zephyr cmake configure
# time. Consumes <bringup>/system.toml + launch/*.xml; emits a baked
# tree under ${CMAKE_BINARY_DIR}/nros-system/ wired into `app`. See
# cmake/nros_system_generate.cmake (≤200 LoC adapter shim).
include(${CMAKE_CURRENT_LIST_DIR}/cmake/nros_system_generate.cmake)
# Phase 215.B.2 — nano_ros_use_board(<name>). Layers a board crate's
# board.cmake sidecar manifest (EXTRA_CONF_FILE / DTC_OVERLAY_FILE /
# BOARD / NANO_ROS_RMW / NROS_BOARD_RUNNER) into the downstream app
# build. The canonical call site is the consumer app's CMakeLists.txt
# BEFORE find_package(Zephyr); this include just makes the fn
# available wherever nano-ros is on ZEPHYR_EXTRA_MODULES (re-include
# is benign — cmake allows fn redefinition).
include(${CMAKE_CURRENT_LIST_DIR}/cmake/nano_ros_use_board.cmake)

# The nros repo root (parent of this zephyr/ directory).
# Phase 210.E.3.c — promoted to CACHE so consumer application's
# CMakeLists.txt can reference it (e.g.
# `include("${NROS_REPO_DIR}/cmake/compat/NrosRclcppCompat.cmake")`
# for the Phase 210 find_package shape). Without CACHE the var only
# lives in this directory's scope and the consumer sees an empty value.
set(NROS_REPO_DIR ${CMAKE_CURRENT_LIST_DIR}/.. CACHE PATH
    "nano-ros repo root (parent of zephyr/ module dir)")

# `zephyr_library()` would create a default library named after this
# directory's path (e.g. `..__..__nano-ros__zephyr`). All sources below
# are added via `zephyr_library_sources(...)` which targets the most
# recently declared library — the named one — so the unnamed one was
# always empty and Zephyr printed "No SOURCES given to Zephyr library:
# ..__..__nano-ros__zephyr". Use only the named form.
zephyr_library_named(nros)

# =============================================================================
# Platform shims (all RMW backends)
# =============================================================================

# Real-symbol wrappers around Zephyr kernel inlines (k_msleep, k_uptime_get,
# sys_rand_get, getaddrinfo) that nros-platform-zephyr needs to call via FFI.
zephyr_library_sources(${CMAKE_CURRENT_LIST_DIR}/nros_platform_zephyr_shims.c)

# Task stacks for `nros_zephyr_task_create`. Both tokens are #ifndef-guarded in
# the shim; NROS_ZEPHYR_STACK_SIZE otherwise derives from CONFIG_MAIN_STACK_SIZE,
# which couples main's budget to every task nano-ros creates and makes main's
# stack effectively unraisable on a tight image. That in turn makes
# CONFIG_HW_STACK_PROTECTION unusable there, which is the one tool that names an
# overflowing thread (issue 0821). 0 keeps the inherited default.
if(DEFINED CONFIG_NROS_ZEPHYR_TASK_STACK_SIZE AND NOT CONFIG_NROS_ZEPHYR_TASK_STACK_SIZE EQUAL 0)
    zephyr_compile_definitions(NROS_ZEPHYR_STACK_SIZE=${CONFIG_NROS_ZEPHYR_TASK_STACK_SIZE})
endif()
if(CONFIG_NROS_ZEPHYR_TASK_SLOTS)
    zephyr_compile_definitions(NROS_ZEPHYR_MAX_THREADS=${CONFIG_NROS_ZEPHYR_TASK_SLOTS})
endif()
if(CONFIG_NROS_ZEPHYR_STACKS_IN_DTCM)
    zephyr_compile_definitions(NROS_ZEPHYR_STACKS_IN_DTCM=1)
endif()
if(CONFIG_NROS_FAULT_LOG)
    zephyr_library_sources(${CMAKE_CURRENT_LIST_DIR}/nros_fault_log.c)
endif()
zephyr_include_directories(${NROS_REPO_DIR}/packages/platform/nros-platform-api/include)
zephyr_library_sources(
    ${NROS_REPO_DIR}/packages/platform/nros-platform-zephyr/src/platform.c
    ${NROS_REPO_DIR}/packages/platform/nros-platform-zephyr/src/timer.c
)

# net.c and net_wait.c are IP-stack code: net.c uses SOL_SOCKET, net_wait.c
# reads net_if_config.ip. They were compiled unconditionally, so an image whose
# only transport is a UART still had to enable NET_SOCKETS and carry the whole
# stack -- on a 320 KiB S32K344 that pushed a one-publisher serial talker to 94%
# of SRAM for code it never calls. Compile them only when there is a stack to
# talk to; the socket wrappers in nros_platform_zephyr_shims.c are gated on the
# same symbol.
if(CONFIG_NET_SOCKETS)
    zephyr_library_sources(
        ${NROS_REPO_DIR}/packages/platform/nros-platform-zephyr/src/net.c
        # Phase 200.1 — RMW-independent network-wait (compiled for every RMW; was
        # mis-filed as zpico_zephyr_wait_network in the zenoh-only zpico_zephyr.c).
        ${NROS_REPO_DIR}/packages/platform/nros-platform-zephyr/src/net_wait.c
    )
endif()

# phase-281 W3a — C/C++ multi-tier Zephyr entry seam
# (`nros_board_zephyr_run_tiers`, packages/boards/nros-board-zephyr/c/). A
# generated Zephyr C/C++ tiers entry emits `ZephyrBoard::run_tiers(...)` which
# calls this C seam; it opens ONE RMW session and spawns one `k_thread` per
# non-boot tier via the `nros_zephyr_tier_task_create` shim above (RFC-0015
# Model 1). It references nros_cpp_* (the typed runtime in nros-c / nros-cpp),
# so compile it only when the C or C++ API is present; it is inert (its symbol
# unreferenced) for single-tier / run_components entries. Added once here — the
# C++ API path always co-builds nros-c, so a single gate avoids adding the same
# source twice.
if(CONFIG_NROS_C_API OR CONFIG_NROS_CPP_API)
    zephyr_library_sources(
        ${NROS_REPO_DIR}/packages/boards/nros-board-zephyr/c/zephyr_run_tiers.c)
endif()

# =============================================================================
# Backend-specific C sources
# =============================================================================

set(NROS_ZEPHYR_DIR ${CMAKE_CURRENT_LIST_DIR})

# Resolve every pool knob (environment over Kconfig) BEFORE the backend modules
# run — nros_zephyr_configure_rmw_zenoh() emits compile definitions from the
# resolved values, and the cargo builds later export the same ones. One
# resolution, both consumers (issue 0316).
nros_resolve_knobs()

# phase-400 W5.a — and resolve the cargo target dir on the same terms, for the
# same reason: `nros_cargo_build()` writes the per-build config headers under it
# and seven consumers include from it, so it must have ONE value per configure
# no matter which of them runs first.
nros_resolve_cargo_dirs()

include(${CMAKE_CURRENT_LIST_DIR}/cmake/nros_rmw_zenoh.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/nros_rmw_xrce.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/nros_rmw_cyclonedds.cmake)

if(CONFIG_NROS_RMW_ZENOH)
    nros_zephyr_configure_rmw_zenoh()
    set(_nros_kconfig_rmw "zenoh")
elseif(CONFIG_NROS_RMW_XRCE)
    nros_zephyr_configure_rmw_xrce()
    set(_nros_kconfig_rmw "xrce")
elseif(CONFIG_NROS_RMW_CYCLONEDDS)
    nros_zephyr_configure_rmw_cyclonedds()
    set(_nros_kconfig_rmw "cyclonedds")
else()
    set(_nros_kconfig_rmw "")
endif() # RMW backend

# =============================================================================
# Native linker workaround for Rust staticlibs (C and C++ API)
#
# Rust staticlibs built with std for x86_64-unknown-linux-gnu reference
# malloc/free, pulling picolibc's nano-malloc into the link. This conflicts
# with Zephyr's COMMON_LIBC_MALLOC. Fix:
#   --allow-multiple-definition: Zephyr's malloc wins (linked first)
#   heap_stub_native.c: provides __heap_start/__heap_end for picolibc's sbrk
# =============================================================================

if((CONFIG_NROS_C_API OR CONFIG_NROS_CPP_API) AND
   (CONFIG_BOARD_NATIVE_SIM OR CONFIG_BOARD_NATIVE_POSIX))
    zephyr_ld_options(-Wl,--allow-multiple-definition)
    zephyr_library_sources(${CMAKE_CURRENT_LIST_DIR}/heap_stub_native.c)
endif()

# C++ Zephyr builds can link several Rust staticlibs in one final ELF:
# nros-cpp, optional generated message FFI libs, and sometimes nros-c for
# shared C/C++ surfaces. Each no_std staticlib is a Rust binary artifact and
# therefore carries allocator, panic, and critical-section lang items. Let the
# first definition win at the final Zephyr link.
if(CONFIG_NROS_CPP_API AND
   NOT (CONFIG_BOARD_NATIVE_SIM OR CONFIG_BOARD_NATIVE_POSIX))
    zephyr_ld_options(-Wl,--allow-multiple-definition)
endif()

# =============================================================================
# phase-366 / RFC-0077 — the image's ENDING, resolved ONCE for every API path
# =============================================================================
#
# `nano_ros_entry(... PANIC ...)` cannot reach this lane the way it reaches the
# cmake/Corrosion one. There, PANIC is applied with `corrosion_set_features()`
# on an imported crate target; here the Rust side is `nros_cargo_build()`, which
# builds its own `add_custom_target` plus an IMPORTED library named
# `nros_c_cargo` — not a Corrosion target, and not named `nros_c`. So
# `NanoRosEntry.cmake`'s scan finds nothing and `PANIC platform` died on "no
# nros-c/nros-cpp Rust target exists at this point in the configure".
#
# Ordering makes it structural rather than a slip: a Zephyr app calls
# `find_package(Zephyr)` — which runs THIS file, including every
# `nros_cargo_build()` below — before its `nano_ros_add_executable()`. The
# feature set is fixed before the entry speaks, which is exactly what phase-366
# W7.b says must not be true ("the staticlib's feature set has to be computable
# after entries are declared").
#
# The seam Zephyr already has for entry->module facts is a variable set BEFORE
# `find_package(Zephyr)`, as `NROS_WS_RUST_NODE_DIRS` is; PANIC uses the same
# one. Unset means `platform`, matching phase-366 M5's default, so entries that
# say `PANIC platform` need no change.
#
# Computed HERE, above both branches, and not inside the C-API one: a C++-only
# image sets `CONFIG_NROS_CPP_API` with `CONFIG_NROS_C_API` unset, so a
# computation nested in the C branch silently does not happen — which is how the
# first attempt at this fix still failed on `workspaces/{c,realtime-c,realtime-cpp}`.
include("${NROS_REPO_DIR}/cmake/NanoRosFeatureSet.cmake")
if(NOT DEFINED NROS_ENTRY_PANIC)
    set(NROS_ENTRY_PANIC platform)
endif()
nros_panic_policy_feature(_nros_panic_feature
    "${NROS_ENTRY_PANIC}" "zephyr NROS_ENTRY_PANIC")
# Tell `nano_ros_entry()` the ending was applied HERE, and with what, so it
# verifies agreement instead of scanning for a target this lane never creates.
set_property(GLOBAL PROPERTY NROS_ENTRY_PANIC_APPLIED "${NROS_ENTRY_PANIC}")
set_property(GLOBAL PROPERTY NROS_ENTRY_PANIC_APPLIED_BY "Zephyr")
set_property(GLOBAL PROPERTY NROS_ENTRY_PANIC_APPLIED_HOW
    "On Zephyr the policy reaches the staticlib through `NROS_ENTRY_PANIC`, \
which must be set BEFORE `find_package(Zephyr)` — the feature list is fixed \
there, and the entry call runs after it. Set that variable, or make the \
keyword match.")
# Appended to every `nros_cargo_build` FEATURES string below. Empty for
# `PANIC own`, where the image supplies its own `#[panic_handler]` and selecting
# either feature would be the second provider.
set(_nros_panic_suffix "")
if(_nros_panic_feature)
    set(_nros_panic_suffix ",${_nros_panic_feature}")
endif()

# phase-8 W4 — callback tracing. This is the ONE read of
# `CONFIG_NROS_TRACE_CALLBACKS`; every feature string below appends the result,
# so the knob cannot reach one cargo build and miss another.
#
# It is spelled as a suffix rather than as a per-site `if()` (the shape
# `param-services` uses) because the C/C++ lane compiles nros-c AND nros-cpp
# from the same nros-node dep graph, in two separate cargo invocations, and
# BOTH are linked into a C++-only image (see the `NOT CONFIG_NROS_C_API` block
# below, whose own comment requires "the SAME RMW + platform feature set").
# Two cargo units of nros-node differing in a feature is the 0135 ABI-split
# class, not a missing optimisation — so the sites must move together, and one
# variable is how they are kept in step.
#
# Appended to the feature STRINGS, not to the `nros_cargo_build` FEATURES
# argument like `_nros_panic_suffix`, because the mixed-workspace umbrella
# re-splits `_nros_cpp_features` (`string(REPLACE "," ";" ...)` below) and
# would otherwise build the one archive that actually gets linked without the
# feature.
set(_nros_trace_suffix "")
if(CONFIG_NROS_TRACE_CALLBACKS)
    set(_nros_trace_suffix ",trace-callbacks")
endif()

# =============================================================================
# C API path: build nros-c via Cargo and link it
# =============================================================================

if(CONFIG_NROS_C_API)
    # Phase 119.3: prepend `<cargo target dir>/nros-c-generated` to the include
    # path. nros-c's build.rs writes the per-build header there for this Zephyr
    # build's exact storage sizes. The source-tree header is a stub that
    # `#error`s if reached.
    # phase-400 W5.a — the directory is `nros_resolve_cargo_dirs()`'s answer, not
    # a literal repeated from nros_cargo_build.cmake. Same path today.
    zephyr_include_directories(${NROS_GENERATED_HEADER_DIR}/nros-c-generated)
    # Add nros-c include directories for user code
    zephyr_include_directories(${NROS_REPO_DIR}/packages/api/nros-c/include)

    # Define platform macro so nros/platform.h selects the Zephyr backend
    zephyr_compile_definitions(NROS_PLATFORM_ZEPHYR)

    # Make nros_generate_interfaces() + nros_find_interfaces() available
    # for C examples. nros_find_interfaces() is the package.xml SSoT entry
    # point (issue #11) — it resolves the transitive interface closure and
    # delegates to the Zephyr emit-into-`app` nros_generate_interfaces.
    include(${CMAKE_CURRENT_LIST_DIR}/cmake/nros_generate_interfaces.cmake)
    # nros_find_interfaces() comes from the shared core (Phase 246), pulled in by
    # the generator's include above — no separate zephyr copy.

    # Select Cargo features based on RMW backend
    if(CONFIG_NROS_RMW_ZENOH)
        set(_nros_features "rmw-cffi,cffi-zenoh-cffi,platform-zephyr,ros-humble")
    elseif(CONFIG_NROS_RMW_XRCE)
        # Phase 115.K.2.5.1.3-zephyr — XRCE now routes through the C
        # backend's cffi shim instead of the deleted Rust direct-impl.
        # `cffi-xrce-c` flips on the auto-register hook inside
        # `nros_support_init` (mirrors the cyclonedds wiring); the
        # cffi shim's `platform-zephyr` feature drops every POSIX TU
        # and emits `UCLIENT_PLATFORM_ZEPHYR` in the generated config.
        set(_nros_features "rmw-cffi,cffi-xrce-c,platform-zephyr,ros-humble")
    elseif(CONFIG_NROS_RMW_CYCLONEDDS)
        # Phase 168.X gap 2.B — cyclonedds is now usable from the C
        # API too. The backend's `nros_rmw_cyclonedds_register`
        # entry point is declared with C linkage in
        # `packages/rmw/cyclonedds/nros-rmw-cyclonedds/include/nros_rmw_cyclonedds.h`,
        # so the Phase 160.A strong stub below resolves it from the
        # standalone C++ library compiled by the RMW backend block
        # above (lines 196+). nros-c just needs the generic cffi
        # runtime + Zephyr platform feature.
        set(_nros_features "rmw-cffi,platform-zephyr,ros-humble")
    endif()

    # native_sim targets run on Linux and support std (needed for panic handler)
    if(CONFIG_BOARD_NATIVE_SIM OR CONFIG_BOARD_NATIVE_POSIX)
        string(APPEND _nros_features ",std")
    endif()

    # phase-8 W4 — callback tracing (see `_nros_trace_suffix` above).
    string(APPEND _nros_features "${_nros_trace_suffix}")

    # Issue 1100 — the nros-c features nros-cpp FORWARDS must be on this string
    # too, or the image carries TWO nros-c units.
    #
    # `nros-cpp`'s manifest forwards `alloc -> nros-c/alloc` and
    # `param-services -> nros-c/param-services`, and `_nros_cpp_features` below
    # sets both (`,alloc` unconditionally, `,param-services` under
    # `param_services IN_LIST NANO_ROS_FEATURES`). This string set neither. In a
    # mixed C + C++ image cargo therefore built nros-c twice, with different
    # `EXECUTOR_SIZE`, and both build scripts write the SAME shared
    # `nros-c-generated/nros/nros_config_generated.h`. The first build wins; the
    # second aborts with "written by another crate with DIFFERENT probed sizes"
    # — so a mixed image builds exactly once and every rebuild fails, including
    # the one `west build --target run` performs.
    #
    # This is the rule the `_nros_c_for_cpp_features` block below already states
    # for the inverse case: "a feature that reaches only one of them splits the
    # unit". It was stated for `trace-callbacks` and not applied to `alloc` or
    # `param-services`.
    #
    # Conditional on the C++ API being on, deliberately: a C-ONLY image has no
    # second unit to agree with, and adding `alloc` there would change what it
    # links for nothing. Where both are on, the image already contains the
    # alloc-enabled nros-node through nros-cpp — unifying removes a duplicate
    # rather than adding a capability.
    if(CONFIG_NROS_CPP_API)
        string(APPEND _nros_features ",alloc")
        if("param_services" IN_LIST NANO_ROS_FEATURES)
            string(APPEND _nros_features ",param-services")
        endif()
    endif()

    # Build nros-c static library via Cargo
    nros_cargo_build(
        PACKAGE nros-c
        FEATURES "${_nros_features}${_nros_panic_suffix}"
    )
    add_dependencies(app nros_c_cargo_build)

    # Link the built library
    zephyr_library_link_libraries(nros_c_cargo)

    # Phase 168.4 — Phase 134.fix dropped `dep:nros-rmw-zenoh` and
    # `dep:nros-rmw-dds` from `nros-c`'s feature graph so `libnros_c.a`
    # no longer carries `nros_rmw_<x>_register` for those backends.
    # The Phase 160.A strong stub below still references those symbols,
    # so the C-API Zephyr link fails for zenoh + dds without a
    # standalone backend staticlib. XRCE keeps `dep:nros-rmw-xrce-cffi`
    # under `cffi-xrce-c`, so it does NOT need a separate staticlib.
    if(CONFIG_NROS_RMW_ZENOH)
        _nros_zephyr_backend_features(_nros_backend_features)
        nros_cargo_build(
            PACKAGE nros-rmw-zenoh-staticlib
            FEATURES "${_nros_backend_features}"
        )
        add_dependencies(app nros_rmw_zenoh_staticlib_cargo_build)
        zephyr_library_link_libraries(nros_rmw_zenoh_staticlib_cargo)
    endif()
    # Phase 169.4 retired dust-dds; the Rust DDS path (and its
    # `nros-rmw-dds-staticlib`) is gone. Future Cyclone DDS Rust
    # support comes via Phase 169.5's `nros-rmw-cyclonedds-sys`
    # shim — wire it in here when it lands.

    # Phase 160.A — emit the `nros_app_register_backends` strong stub
    # that calls the active backend's `nros_rmw_<x>_register` entry.
    # `nros_support_init` invokes this hook explicitly (Phase 155.B.4)
    # because `linkme`'s distributed-slice ctor doesn't fire on Zephyr.
    # Without the strong def, the weak no-op (shipped by
    # nros-c/c-stubs/weak_register_backends.c) wins → zero backends
    # register → `default_vtable() -> InvalidArgument` → C-API caller
    # sees `nros_support_init_named(...) -> -3`. The `nano_ros_link_rmw()`
    # cmake helper emits the equivalent stub for `add_subdirectory`
    # consumers; Zephyr needs its own emission point since
    # `add_subdirectory(<repo>)` isn't on the Zephyr consumption
    # shape.
    if(CONFIG_NROS_RMW_XRCE)
        set(_nros_rmw_name "xrce")
    elseif(CONFIG_NROS_RMW_ZENOH)
        set(_nros_rmw_name "zenoh")
    elseif(CONFIG_NROS_RMW_CYCLONEDDS)
        set(_nros_rmw_name "cyclonedds")
    endif()
    if(_nros_rmw_name)
        set(_nros_app_register_stub "${CMAKE_BINARY_DIR}/nros_app_register_backends.c")
        file(WRITE "${_nros_app_register_stub}"
"/* Phase 160.A auto-generated by zephyr/CMakeLists.txt — strong def\n\
 * of nros_app_register_backends() overrides the weak no-op in\n\
 * libnros_c_weak_stubs (Phase 104.B.6). Calls the active RMW backend's\n\
 * register entry inside `nros_support_init` (Phase 155.B.4 explicit\n\
 * hook for RTOS targets where linkme distributed-slice ctor doesn't\n\
 * fire). Backend registered: ${_nros_rmw_name}\n\
 */\n\
extern int nros_rmw_${_nros_rmw_name}_register(void);\n\
void nros_app_register_backends(void) {\n\
    (void)nros_rmw_${_nros_rmw_name}_register();\n\
}\n")
        zephyr_library_sources("${_nros_app_register_stub}")
    endif()
endif()

# =============================================================================
# C++ API path: build nros-cpp via Cargo and link it
# =============================================================================

if(CONFIG_NROS_CPP_API)
    # Phase 119.3: prepend per-build variant header dirs (cpp + c) to
    # include path before source-tree dirs. C++ user code transitively
    # pulls `<nros/nros_config_generated.h>` (C-side) via parameter.hpp,
    # so both variants must resolve to the per-build versions.
    zephyr_include_directories(${NROS_GENERATED_HEADER_DIR}/nros-cpp-generated)
    zephyr_include_directories(${NROS_GENERATED_HEADER_DIR}/nros-c-generated)
    # `cxx-compat/` ships a stripped-down `<cstdlib>` / `<cstdio>` /
    # `<cstring>` / etc. for Zephyr targets whose C++ stdlib layer is
    # minimal. `zephyr/lib/cpp/minimal/include` only carries
    # `<cstddef>`, `<cstdint>`, `<new>` — `<cstdio>` is missing on
    # `native_sim` too (Phase 160.A surfaced this for the XRCE C++
    # examples that `#include <nros/log.hpp>` → `<cstdio>`). The
    # original PICOLIBC gate was over-conservative: every Zephyr cpp
    # path needs these shims when the SDK's libstdc++ headers aren't
    # in the include set. Newlib targets that DO ship full libstdc++
    # would prefer their `<cstdio>` over the shim, but the shim's
    # `using ::fprintf;` re-export pattern is benign on top of a real
    # `<cstdio>`.
    zephyr_include_directories(${NROS_REPO_DIR}/zephyr/cxx-compat)
    # Force-included libc gap shim. Zephyr's MINIMAL libc has no `setvbuf` and
    # no `_IO*BF`, and every C and C++ example calls
    # `setvbuf(stdout, NULL, _IONBF, 0)` so its prints appear promptly -- so on
    # minimal libc they all fail to build. Force-included rather than shipped as
    # a `<stdio.h>` override so it cannot disturb the include_next chain, and
    # keyed on `_IONBF` inside so it is inert on picolibc and newlib.
    # C and C++ ONLY. zephyr_compile_options() reaches EVERY target including
    # assembly, and `-include <C header>` on a .S file feeds the assembler C
    # declarations: Zephyr's own arch/arm64/core/early_mem_funcs.S then dies
    # with "unknown mnemonic `typedef'". Found by the ASI consumer on
    # fvp_baser_aemv8r_smp, where it broke the image build outright.
    #
    # issue 0845 — `SHELL:`, not two separate list elements.
    #
    # The two-element form was written because a space inside one generator
    # expression makes the flag and its path a SINGLE argument, which gcc
    # rejects. True, but splitting them creates a worse failure: CMake
    # DE-DUPLICATES compile options, Zephyr already passes
    # `-include .../undef_system_defines.h`, and the second bare `-include`
    # is dropped as a duplicate — leaving its path behind as an orphan input.
    # gcc then sees two input files and dies with
    #
    #     cannot specify `-o' with `-c', `-S' or `-E' with multiple files
    #
    # on every picolibc TU. Reproduced minimally: two `target_compile_options`
    # calls, the first a plain `-include first.h`, the second the split-genex
    # pair, yields `FLAGS = -include …/first.h …/second.h` — one flag, two
    # paths.
    #
    # SECOND WITNESS, cortex-m/picolibc (issue 0840 session, recorded before the
    # cause was known): the same defect killed every Zephyr fixture at
    # arch/arm/core/nmi_on_reset.S with 218 x "Error: bad instruction `typedef
    # signed char __int8_t'". Two arch families, one root cause — which is the
    # argument for fixing the pairing rather than either symptom.
    #
    # `SHELL:` is the CMake feature for exactly this: it splits on spaces into
    # separate arguments AND exempts the group from de-duplication, so the pair
    # stays together however many other `-include`s exist. Verified in the same
    # reproducer: `-include …/first.h -include …/second.h`, and it compiles.
    # issue 0918 — `SHELL:` must be the LITERAL PREFIX of the option, outside the
    # generator expression. CMake looks for `SHELL:` before it evaluates genexes,
    # so `$<$<COMPILE_LANGUAGE:C,CXX>:SHELL:...>` presents an option starting with
    # `$<` and the SHELL handling never engages — the group is not exempted from
    # de-duplication, and on Zephyr's `modules/picolibc` target the `-include`
    # token was dropped as a duplicate of an earlier one, stranding the path as a
    # bare input file. 932 of the 1258 occurrences in one `build.ninja` were
    # orphaned that way, all in that single target, and every C/C++ Zephyr fixture
    # died on the first picolibc TU with
    #     gcc: fatal error: cannot specify '-o' with '-c' … with multiple files
    #
    # Zephyr's own force-include has always had it in the right place
    # (`arch/posix/CMakeLists.txt`: `"SHELL:-include …/undef_system_defines.h"`),
    # which is why THAT one survived on the same command line while ours did not.
    #
    # The language guard stays, just inside: when the genex is empty (an .S file)
    # the option degrades to a bare `SHELL:` and contributes nothing, which is the
    # behaviour the ASM exclusion above needs.
    # issue 0918 — SCOPED to the targets that need it, not global.
    #
    # `zephyr_compile_options()` reaches EVERY target, including Zephyr's own
    # `modules/picolibc`. `SHELL:` splits the option into two arguments so gcc
    # gets a flag and a path; it is also supposed to exempt the group from
    # de-duplication, and it does for most targets. It does not survive whatever
    # picolibc's module does to its option list: measured in one `build.ninja`,
    # 932 of 1258 occurrences had the `-include` token dropped — ALL of them in
    # `modules/picolibc`, the other 326 correct — which strands the path as a bare
    # input file and kills the first picolibc TU with
    #     gcc: fatal error: cannot specify '-o' with '-c' … with multiple files
    # Moving `SHELL:` outside the genex (Zephyr's own spelling in
    # arch/posix/CMakeLists.txt) was tried and changed nothing: still 932.
    #
    # The shim is for USER code — the examples call `setvbuf(stdout, NULL,
    # _IONBF, 0)` and Zephyr's MINIMAL libc has neither `setvbuf` nor `_IO*BF`.
    # Zephyr's libc implementation never needed it, so sending it there was the
    # mistake; the de-duplication was only how that mistake surfaced.
    #
    # `app` is verified to exist at module time (probed on a real configure), and
    # `nros` is this file's own `zephyr_library_named`. The `if(TARGET)` guard is
    # a fail-loud, not a fallback: a silent skip would put us back to examples
    # that do not build, so it warns.
    set(_nros_libc_shim
        "SHELL:$<$<COMPILE_LANGUAGE:C,CXX>:-include ${NROS_REPO_DIR}/zephyr/libc-compat/nros_libc_compat.h>")
    if(TARGET app)
        target_compile_options(app PRIVATE "${_nros_libc_shim}")
    else()
        message(WARNING
            "nano-ros: no `app` target when the libc-compat shim was applied "
            "(issue 0918). Example sources calling setvbuf() will fail to build "
            "on a minimal-libc Zephyr.")
    endif()
    zephyr_library_compile_options("${_nros_libc_shim}")
    # Add nros-cpp include directories for user code
    zephyr_include_directories(${NROS_REPO_DIR}/packages/api/nros-cpp/include)
    # Phase 112.C — nros-c also exposes `<nros/app_main.h>` (the
    # unified user-entry contract used by both C and C++ examples).
    # Cpp Zephyr builds need the nros-c include dir for that header
    # plus the Zephyr Kconfig synthesis variant of `<nros/app_config.h>`
    # under `nros/zephyr/`.
    zephyr_include_directories(${NROS_REPO_DIR}/packages/api/nros-c/include)
    zephyr_include_directories(${NROS_REPO_DIR}/packages/api/nros-c/include/nros/zephyr)

    # Make nros_generate_interfaces() + nros_find_interfaces() available
    # for C++ examples (see the C-API path above for the rationale).
    include(${CMAKE_CURRENT_LIST_DIR}/cmake/nros_generate_interfaces.cmake)
    # nros_find_interfaces() comes from the shared core (Phase 246), pulled in by
    # the generator's include above — no separate zephyr copy.

    # Select Cargo features based on RMW backend
    if(CONFIG_NROS_RMW_ZENOH)
        zephyr_compile_definitions(NROS_RMW_ZENOH_CFFI=1)
        set(_nros_cpp_features "rmw-zenoh-cffi,platform-zephyr,ros-humble")
    elseif(CONFIG_NROS_RMW_XRCE)
        zephyr_compile_definitions(NROS_RMW_XRCE=1)
        # Phase 115.K.2.5.1.3-zephyr — same shape as the C path
        # above: cffi runtime + C backend + Zephyr platform feature.
        # issue 0730 follow-up — `alloc` here too. MEASURED on
        # `aarch64-unknown-none`: this composition fails the same E0599 as the
        # cyclonedds one.
        #
        #   rmw-cffi                 (cyclonedds)  E0599
        #   rmw-cffi,rmw-xrce-cffi                 E0599   <- this branch
        #   rmw-zenoh-cffi                         builds
        #
        # So the note below is one branch short: xrce is not a bare `rmw-cffi`
        # composition, but `rmw-xrce-cffi` pulls `dep:nros-rmw-xrce-cffi`, which
        # unlike nros-rmw-zenoh does NOT carry `alloc`. Reaching the capability
        # through whichever backend happens to depend on it is the shape that
        # produced the original bug; both sites now spell it.
        set(_nros_cpp_features "rmw-cffi,rmw-xrce-cffi,alloc,platform-zephyr,ros-humble")
    elseif(CONFIG_NROS_RMW_CYCLONEDDS)
        # cyclonedds bypasses Cargo: the standalone nros-rmw-cyclonedds
        # C++ lib (compiled into the Zephyr app library by the RMW
        # backend block above) registers the vtable via
        # nros_rmw_cyclonedds_register(). nros-cpp only needs the
        # generic cffi runtime + Zephyr platform feature.
        #
        # `alloc` is spelled here because this module IS the dep-site
        # composing the consumer's feature set (phase-361 W8.d): nros-cpp's
        # error mapper references the alloc-gated
        # `TransportError::BackendDynamic` un-gated (issue 0586 keeps
        # mappers exhaustive), and on embedded Zephyr the platform provides
        # a global allocator WITHOUT the `alloc` feature, so the image
        # links and only the variant is missing (E0599 — issue 0730). The
        # sibling lanes get `alloc` transitively (zenoh via
        # nros-rmw-zenoh's pinned nros-rmw-cffi/alloc; native_sim via
        # `,std`). NOTE: the xrce branch above needs it too — measured, see
        # its comment; this is not the only composition that lacked it.
        set(_nros_cpp_features "rmw-cffi,alloc,platform-zephyr,ros-humble")
    endif()


    # native_sim targets use the host Rust target; keep std enabled so
    # the final native_sim executable links Rust's panic personality.
    # (`std` implies `alloc`; the repeat above is harmless and keeps the
    # capability stated where it is required rather than where it is implied.)
    if(CONFIG_BOARD_NATIVE_SIM OR CONFIG_BOARD_NATIVE_POSIX)
        string(APPEND _nros_cpp_features ",std")
    endif()

    # Issue 0745 — the Zephyr lane has no workspace-style capability
    # resolution (`nros config show`) yet; the consumer passes
    # -DNANO_ROS_FEATURES=<axes> (semicolon list) and this mirrors the
    # workspace lowering: the param-services cargo feature (so the entry's
    # launch-param seeds link) + the directory-wide define ComponentNode's
    # seed-adoption gates on. Follow-up: resolve from the entry's BRINGUP
    # like nano_ros_workspace(SYSTEM ...) does.
    if("param_services" IN_LIST NANO_ROS_FEATURES)
        string(APPEND _nros_cpp_features ",param-services")
        add_compile_definitions(NROS_SYSTEM_PARAM_SERVICES)
    endif()

    # phase-8 W4 — callback tracing (see `_nros_trace_suffix` above). Appended
    # BEFORE the umbrella's `string(REPLACE "," ";" _umb_feats ...)` below, so
    # a mixed workspace's single linked archive carries it too.
    string(APPEND _nros_cpp_features "${_nros_trace_suffix}")

    # Phase 168.X gap 1 — `nros_log_emit` + `nros_log_emit_fmt`
    # symbols (the C-side log glue every `NROS_LOG_*` macro from
    # `<nros/log.h>` expands to) ship only with `libnros_c.a`.
    # `libnros_cpp.a` doesn't carry them, so C++-only Zephyr builds
    # (`CONFIG_NROS_C_API=n + CONFIG_NROS_CPP_API=y`) fail to link
    # the moment a cpp example uses `NROS_LOG_INFO`. Build nros-c
    # alongside nros-cpp (with the SAME RMW + platform feature set
    # the C-API path uses) so the log glue lands in the link line.
    # No code duplication: the C / C++ rlibs share their nros-node /
    # nros / nros-platform dep graph; the second cargo build wraps
    # them in nros-c's exported C symbols (init / log / executor /
    # …) — which is what nros-cpp's C++ classes call into anyway.
    #
    # Must run BEFORE the nros-cpp build below so the
    # nros-c-generated header byproducts are declared on the
    # nros_c_cargo_build target (nros-cpp's target then defers to it
    # via `TARGET nros_c_cargo_build` in nros_cargo_build.cmake).
    if(NOT CONFIG_NROS_C_API)
        if(CONFIG_NROS_RMW_ZENOH)
            set(_nros_c_for_cpp_features "rmw-cffi,cffi-zenoh-cffi,platform-zephyr,ros-humble")
        elseif(CONFIG_NROS_RMW_XRCE)
            set(_nros_c_for_cpp_features "rmw-cffi,cffi-xrce-c,platform-zephyr,ros-humble")
        elseif(CONFIG_NROS_RMW_CYCLONEDDS)
            set(_nros_c_for_cpp_features "rmw-cffi,platform-zephyr,ros-humble")
        endif()
        if(CONFIG_BOARD_NATIVE_SIM OR CONFIG_BOARD_NATIVE_POSIX)
            string(APPEND _nros_c_for_cpp_features ",std")
        endif()
        # phase-8 W4 — callback tracing. MUST match the nros-cpp string above:
        # both archives are linked into this image from one nros-node dep
        # graph, and a feature that reaches only one of them splits the unit.
        string(APPEND _nros_c_for_cpp_features "${_nros_trace_suffix}")
        # Issue 1156 — and so do `alloc` and `param-services`, by that same
        # rule. Issue 1100 applied it to `_nros_features`, the C-API-ON string;
        # this is the string a C++-ONLY image builds nros-c from, and it kept
        # the split: `nros_c_cargo_build` took
        # `rmw-cffi,platform-zephyr,ros-humble` while `nros_cpp_cargo_build`
        # took `rmw-cffi,alloc,platform-zephyr,ros-humble,param-services`, so
        # the image still carried two nros-c units with different
        # EXECUTOR_SIZE and the second build script still aborted.
        #
        # `alloc` unconditionally: this branch is inside `if(CONFIG_NROS_CPP_API)`,
        # so the C++ half has it by construction. `param-services` under the
        # same NANO_ROS_FEATURES condition `_nros_cpp_features` gates on.
        string(APPEND _nros_c_for_cpp_features ",alloc")
        if("param_services" IN_LIST NANO_ROS_FEATURES)
            string(APPEND _nros_c_for_cpp_features ",param-services")
        endif()
        nros_cargo_build(
            PACKAGE nros-c
            FEATURES "${_nros_c_for_cpp_features}${_nros_panic_suffix}"
        )
        add_dependencies(app nros_c_cargo_build)
    endif()

    # Build nros-cpp static library via Cargo
    nros_cargo_build(
        PACKAGE nros-cpp
        FEATURES "${_nros_cpp_features}${_nros_panic_suffix}"
    )
    add_dependencies(app nros_cpp_cargo_build)

    # phase-263 C2c-zephyr — a MIXED workspace (C + C++ + Rust node) bundles nros-cpp + every
    # workspace Rust node into the synthesised `nros_ws_runtime` umbrella staticlib (the
    # single-runtime invariant: ONE Rust staticlib, one nros-rmw-cffi REGISTRY — a node in its
    # own staticlib would split it). The entry CMakeLists sets NROS_WS_RUST_NODE_DIRS (a list of
    # node pkg dirs) BEFORE find_package(Zephyr). nros-c / nros-cpp above are still BUILT (their
    # build.rs writes the per-build headers the C/C++ TUs include) but NOT linked — the umbrella
    # (which deps nros-cpp, hence nros-c) is the single linked archive, so the node libs' nros
    # refs resolve from it without double-defining nros-rmw-cffi's REGISTRY. Mirrors the
    # cmake/Corrosion lane's archive swap (NanoRosRuntimeCrate.cmake).
    set(_nros_link_umbrella FALSE)
    if(NROS_WS_RUST_NODE_DIRS)
        set(_nros_link_umbrella TRUE)
    endif()

    if(_nros_link_umbrella)
        include("${NROS_REPO_DIR}/cmake/NanoRosRuntimeCrate.cmake")
        set(NANO_ROS_ROOT "${NROS_REPO_DIR}")
        # Backend key for the umbrella's force-link anchor (zenoh/xrce bundle a Rust rlib).
        if(CONFIG_NROS_RMW_XRCE)
            set(_umb_backend "xrce")
        elseif(CONFIG_NROS_RMW_CYCLONEDDS)
            set(_umb_backend "cyclone")
        else()
            set(_umb_backend "zenoh")
        endif()
        # native_sim uses the host std triple → std umbrella; a real Zephyr board is no_std.
        set(_umb_no_std TRUE)
        if(CONFIG_BOARD_NATIVE_SIM OR CONFIG_BOARD_NATIVE_POSIX)
            set(_umb_no_std FALSE)
        endif()
        string(REPLACE "," ";" _umb_feats "${_nros_cpp_features}")
        nros_write_runtime_umbrella_crate(
            OUT_DIR      "${CMAKE_BINARY_DIR}/nros_ws_runtime"
            BACKEND      "${_umb_backend}"
            NO_STD       "${_umb_no_std}"
            NODE_DIRS    ${NROS_WS_RUST_NODE_DIRS}
            CPP_FEATURES ${_umb_feats})
        nros_cargo_build(
            PACKAGE       nros_ws_runtime
            MANIFEST_PATH "${CMAKE_BINARY_DIR}/nros_ws_runtime/Cargo.toml"
        )
        # Headers (nros-c / nros-cpp build.rs byproducts) must be ready before the umbrella
        # build's TUs / the entry TU compile.
        add_dependencies(nros_ws_runtime_cargo_build nros_cpp_cargo_build)
        if(TARGET nros_c_cargo_build)
            add_dependencies(nros_ws_runtime_cargo_build nros_c_cargo_build)
        endif()
        add_dependencies(app nros_ws_runtime_cargo_build)
        zephyr_library_link_libraries(nros_ws_runtime_cargo)
    else()
        # issue 0734 — link nros-cpp ONLY. `libnros_cpp.a` is a `staticlib`
        # whose crate DEPENDS on nros-c (`nros-cpp/Cargo.toml:227`), and a
        # staticlib bundles its ENTIRE dependency closure — so it already
        # carries every C-ABI symbol `libnros_c.a` exports, `nros_log_emit`
        # included. Linking both therefore links the shared closure TWICE.
        #
        # That is not free: the two archives are separate cargo builds with
        # different `-C metadata`, so `nros-rmw-zenoh`'s statics do not even
        # collide — the linker allocates both copies. Measured on
        # mr_canhubk3/s32k344 (320 KiB SRAM):
        #
        #   0x20474cf1 0x20000 libnros_c.a  (…Cs*ewqHElJteY4*…SUBSCRIBER_BUFFERS)
        #   0x2049ccfb 0x20000 libnros_cpp.a(…Cs*hvwoP2UscId*…SUBSCRIBER_BUFFERS)
        #
        # ~195 KiB of duplicated `.bss`, and the subscriber ring state
        # (SUBSCRIBER_BUFFERS / NEXT_BUFFER_INDEX / OVERFLOW_DROPS) present in
        # two divergent copies.
        #
        # The Phase 168.X comment above still explains why nros-c is BUILT here
        # — nros-cpp compiles against its generated headers — and that stays.
        # Its further claim, that the log glue "ships only with libnros_c.a" and
        # so the archive must also be LINKED, is no longer true: verified with
        # `nm` on both archives, `nros_log_emit` is defined in each, and of the
        # 427 symbols present in libnros_c.a but not libnros_cpp.a, ZERO are
        # non-mangled C ABI — they are Rust internals whose names differ only by
        # `-C metadata`.
        zephyr_library_link_libraries(nros_cpp_cargo)
    endif()

    # Phase 168.4 — same backend staticlib link as the C-API branch
    # above. `nros-cpp`'s `rmw-zenoh-cffi` feature does NOT pull
    # `dep:nros-rmw-zenoh` (Phase 134.fix), so the Phase 160.A strong
    # stub below references `nros_rmw_zenoh_register` with no provider
    # unless we link the standalone backend staticlib. `rmw-xrce-cffi`
    # still carries its dep, so it doesn't need this extra step.
    if(CONFIG_NROS_RMW_ZENOH)
        _nros_zephyr_backend_features(_nros_cpp_backend_features)
        if(NOT TARGET nros_rmw_zenoh_staticlib_cargo)
            nros_cargo_build(
                PACKAGE nros-rmw-zenoh-staticlib
                FEATURES "${_nros_cpp_backend_features}"
            )
        endif()
        add_dependencies(app nros_rmw_zenoh_staticlib_cargo_build)
        zephyr_library_link_libraries(nros_rmw_zenoh_staticlib_cargo)
    endif()

    # Phase 160.A — same register-stub emission as the C-API branch
    # above. C++ apps call `nros_cpp_init` which internally invokes
    # `nros_app_register_backends`; without the strong def the weak
    # no-op fires and `default_vtable() -> InvalidArgument` surfaces
    # as `nros_support_init_named(...) -> -3`.
    if(CONFIG_NROS_RMW_XRCE)
        set(_nros_cpp_rmw_name "xrce")
    elseif(CONFIG_NROS_RMW_ZENOH)
        set(_nros_cpp_rmw_name "zenoh")
    elseif(CONFIG_NROS_RMW_CYCLONEDDS)
        set(_nros_cpp_rmw_name "cyclonedds")
    endif()
    if(_nros_cpp_rmw_name)
        set(_nros_cpp_app_register_stub "${CMAKE_BINARY_DIR}/nros_app_register_backends.c")
        file(WRITE "${_nros_cpp_app_register_stub}"
"/* Phase 160.A auto-generated by zephyr/CMakeLists.txt (CPP API). */\n\
extern int nros_rmw_${_nros_cpp_rmw_name}_register(void);\n\
void nros_app_register_backends(void) {\n\
    (void)nros_rmw_${_nros_cpp_rmw_name}_register();\n\
}\n")
        zephyr_library_sources("${_nros_cpp_app_register_stub}")
    endif()
endif()

# =============================================================================
# Rust API path: bridge Kconfig → Cargo env vars
# =============================================================================

if(CONFIG_NROS_RUST_API)
    nros_set_cargo_env_from_kconfig()

    # Issue 0155 (Phase 160.A sibling) — pure-Rust images need the same
    # strong `nros_app_register_backends` stub the C/C++ branches emit:
    # `nros::zephyr_component_main!` calls the hook before `Executor::open`
    # (Rust `.init_array` ctors don't run on Zephyr, and only the C/C++
    # `nros_cpp_init` path used to call it — a Rust-only image reached
    # open with no backend registered → Transport(ConnectionFailed)).
    # Skip when a C/C++ branch already emitted the stub (same file path).
    if(NOT CONFIG_NROS_C_API AND NOT CONFIG_NROS_CPP_API)
        set(_nros_rust_rmw_name "")
        if(CONFIG_NROS_RMW_XRCE)
            set(_nros_rust_rmw_name "xrce")
        elseif(CONFIG_NROS_RMW_ZENOH)
            set(_nros_rust_rmw_name "zenoh")
        elseif(CONFIG_NROS_RMW_CYCLONEDDS)
            set(_nros_rust_rmw_name "cyclonedds")
        endif()
        if(_nros_rust_rmw_name)
            set(_nros_rust_app_register_stub
                "${CMAKE_BINARY_DIR}/nros_app_register_backends.c")
            # WEAK reference (unlike the C/C++ branch): a pure-Rust app crate
            # does not link libnros_c.a, and its `rmw-<x>` cargo features are
            # inert markers (#60 T5) — for zenoh no TU in the image defines
            # `nros_rmw_zenoh_register` (cyclonedds gets it from the module's
            # C++ lib). A strong extern made every pure-Rust zenoh image a
            # LINK error (issue 0163). With the weak guard the image links and
            # `Executor::open` fails loudly at runtime until the backend-dep
            # decision in 0163 lands.
            file(WRITE "${_nros_rust_app_register_stub}"
"/* Issue 0155 auto-generated by zephyr/CMakeLists.txt (RUST API). */
extern int nros_rmw_${_nros_rust_rmw_name}_register(void) __attribute__((weak));
void nros_app_register_backends(void) {
    if (nros_rmw_${_nros_rust_rmw_name}_register) {
        (void)nros_rmw_${_nros_rust_rmw_name}_register();
    }
}
")
            zephyr_library_sources("${_nros_rust_app_register_stub}")
        endif()
    endif()
endif()

# RFC-0085 D14 — Kconfig above is the MECHANISM; when this application belongs
# to a nano-ros workspace its `[image.*]` is a second answer to the same
# question, and the two must not disagree silently. A plain Zephyr app is in no
# workspace, and this is a no-op for it.
#
# LAST, not beside the RMW block it checks: the nros CLI is resolved by
# `nros_generate_interfaces.cmake`, included further down, so a call up there
# found no CLI and returned having checked nothing — silently, because a
# missing CLI must never fail a build that was not asking it anything.
# `_nros_kconfig_rmw` is captured where the choice is made and read here.
# `NanoRosCodegenCore.cmake` supplies `nros_resolve_cli`, which the check uses
# when no lane has already populated a tool cache var.
include("${NROS_REPO_DIR}/cmake/NanoRosCodegenCore.cmake")
include("${NROS_REPO_DIR}/cmake/NanoRosImageAgreement.cmake")
nros_check_image_agreement(KCONFIG_RMW "${_nros_kconfig_rmw}")

endif() # CONFIG_NROS
