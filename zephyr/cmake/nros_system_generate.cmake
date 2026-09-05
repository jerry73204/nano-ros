# nros_system_generate.cmake — Phase 212.H.1 Zephyr adapter shim.
# Copyright (c) 2026 nros contributors
# SPDX-License-Identifier: MIT OR Apache-2.0
#
# Adapter shim for the Phase 212.E "nros codegen system" host-time bake.
# Reads `<bringup>/system.toml` + `<bringup>/launch/*.xml` at Zephyr
# cmake configure time, shells the `nros` CLI, and consumes the emitted
# tree under `${CMAKE_BINARY_DIR}/nros-system/`:
#
#   system_config.h    — baked compile-time C config (domain, rmw, ...)
#   system_config.cmake — baked cmake-side config mirror
#   (system_main.c retired in phase-258 — install-seam registration)
#   Cargo.toml         — workspace stub (if any Rust components)
#   nros-plan.json     — resolved plan (debug + tests inspect this)
#
# The shim is host-callable independently of `just`: any Zephyr app
# CMakeLists.txt that has `find_package(Zephyr ...)` plus the nros
# module loaded (via west or `ZEPHYR_EXTRA_MODULES=<repo>/zephyr`) gets
# `nros_system_generate(<bringup-pkg>)` for free — the parent
# zephyr/CMakeLists.txt includes this file unconditionally.
#
# Retires the per-example `<nros/app_config.h>` Kconfig-synthesis path
# (packages/api/nros-c/include/nros/zephyr/app_config.h) for any app
# that opts into Phase 212. That header stays in tree as a fallback for
# pre-212 examples; once every example migrates, it's deletable. See
# `docs/roadmap/phase-212-ux-cargo-native-and-file-consolidation.md`
# section 212.H.1.

# Resolve the `nros` CLI binary. Priority: NROS_CLI env, PATH (incl
# `packages/cli/target/release/` via `activate.sh`), NROS_HOME/bin,
# ~/.nros/bin (transitional). Matches scripts/build/cargo.sh::nros_cli_bin.
# issue #219 — thin shim over the shared resolver in NanoRosCodegenCore
# (which owns the PATHS-not-HINTS rule this function's comment used to
# document: a stale provisioned ~/.nros/bin/nros must never shadow the
# activate.sh-wired in-tree CLI — the museum binary bakes the retired
# pre-258 shape and every nros_system_generate fixture goes red).
include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/NanoRosCodegenCore.cmake")

# phase-206 W2 — the bringup's own RMW config. Included at FILE scope, not
# inside `nros_system_generate`: `include()` in a function body pops the
# file's normal vars with the frame (the `_NROS_ENTRY_DIR` class), and a
# module included once per call is a module re-read once per call.
include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/NanoRosRmwUserConfig.cmake")
function(_nros_system_resolve_cli outvar)
    nros_resolve_cli(_cli OPTIONAL)
    if(_cli)
        set(${outvar} "${_cli}" PARENT_SCOPE)
    else()
        set(${outvar} "NROS_CLI-NOTFOUND" PARENT_SCOPE)
    endif()
endfunction()

# Detect M-F.3 self-pkg bringup shape. Returns TRUE in outvar when
# `<abs>/Cargo.toml` carries `[package.metadata.nros.deploy.zephyr*]`
# OR `<abs>/package.xml` carries the RFC-0048 §4 tuple
# `<nano_ros deploy="zephyr" …/>` (the C/C++ deploy SSoT; replaces the
# retired `nano_ros_deploy(TARGET zephyr …)` CMakeLists grep). A
# self-pkg eats its own bringup role — workspace AND bringup dir are
# the pkg itself.
function(_nros_system_detect_self_pkg abs outvar)
    set(${outvar} FALSE PARENT_SCOPE)
    if(EXISTS "${abs}/Cargo.toml")
        file(READ "${abs}/Cargo.toml" _toml)
        # Match the bare table OR the per-RMW target-name variants. Each
        # is a literal substring of the rendered Cargo.toml — no regex
        # escape gymnastics needed.
        if(_toml MATCHES "\\[package\\.metadata\\.nros\\.deploy\\.zephyr")
            set(${outvar} TRUE PARENT_SCOPE)
            return()
        endif()
    endif()
    if(EXISTS "${abs}/package.xml")
        file(READ "${abs}/package.xml" _pxml)
        if(_pxml MATCHES "<nano_ros[^>]*deploy=\"zephyr")
            set(${outvar} TRUE PARENT_SCOPE)
        endif()
    endif()
endfunction()

# Resolve a bringup-pkg argument to an absolute directory. Accepts an
# absolute path, a path relative to the app's source dir, or a sibling
# dir name (walks one level up — workspace shape). Returns a dir that
# is EITHER a Path A bringup (`system.toml` present) OR an M-F.3
# self-pkg (Cargo.toml + deploy.zephyr OR package.xml +
# <nano_ros deploy="zephyr" .../>).
function(_nros_system_resolve_bringup arg outvar)
    if(IS_ABSOLUTE "${arg}" AND IS_DIRECTORY "${arg}")
        set(${outvar} "${arg}" PARENT_SCOPE)
        return()
    endif()
    set(_candidates
        "${CMAKE_CURRENT_SOURCE_DIR}/${arg}"
        "${CMAKE_SOURCE_DIR}/${arg}"
        "${CMAKE_CURRENT_SOURCE_DIR}/../${arg}"
        "${APPLICATION_SOURCE_DIR}/../${arg}")
    foreach(_c ${_candidates})
        get_filename_component(_abs "${_c}" ABSOLUTE)
        if(IS_DIRECTORY "${_abs}")
            if(EXISTS "${_abs}/system.toml")
                set(${outvar} "${_abs}" PARENT_SCOPE)
                return()
            endif()
            _nros_system_detect_self_pkg("${_abs}" _is_self)
            if(_is_self)
                set(${outvar} "${_abs}" PARENT_SCOPE)
                return()
            endif()
        endif()
    endforeach()
    set(${outvar} "BRINGUP-NOTFOUND" PARENT_SCOPE)
endfunction()

# Public function: bake the system, wire the generated sources into the
# Zephyr `app` target. Single positional arg: the bringup pkg name or
# path (Path A: contains `system.toml`, no `Cargo.toml`).
function(nros_system_generate bringup_pkg)
    _nros_system_resolve_cli(_nros_cli)
    _nros_system_resolve_bringup("${bringup_pkg}" _bringup_dir)
    # codegen-system's --out is the PARENT of the bake tree (`nros-system/`
    # subdir is created inside). The downstream cmake target reads
    # ${CMAKE_BINARY_DIR}/nros-system/{system_config.h,system_config.cmake}, so
    # pass CMAKE_BINARY_DIR (not CMAKE_BINARY_DIR/nros-system) to the CLI.
    set(_out_parent "${CMAKE_BINARY_DIR}")
    set(_out_dir    "${_out_parent}/nros-system")
    file(MAKE_DIRECTORY "${_out_parent}")

    if(_bringup_dir STREQUAL "BRINGUP-NOTFOUND")
        message(FATAL_ERROR
            "nros_system_generate: bringup pkg '${bringup_pkg}' not "
            "found. Looked relative to ${CMAKE_CURRENT_SOURCE_DIR}, "
            "${CMAKE_SOURCE_DIR}, and their parents. The dir must "
            "contain system.toml (Path A bringup) OR a Cargo.toml "
            "with [package.metadata.nros.deploy.zephyr*] / a "
            "package.xml with a <nano_ros deploy=zephyr .../> tuple "
            "(M-F.3 self-pkg bringup).")
    endif()

    if(_nros_cli STREQUAL "NROS_CLI-NOTFOUND")
        message(FATAL_ERROR
            "nros_system_generate: `nros` CLI not on PATH and "
            "NROS_CLI/$NROS_HOME/bin/~/.nros/bin all unset/missing. "
            "Run `just setup-cli` + `source ./activate.sh` (Phase 218).")
    endif()

    # Map Kconfig RMW to a --target string the CLI understands. Zephyr
    # is the platform; the RMW comes from the prj-<rmw>.conf overlay.
    if(CONFIG_NROS_RMW_ZENOH)
        set(_rmw "zenoh")
    elseif(CONFIG_NROS_RMW_XRCE)
        set(_rmw "xrce")
    elseif(CONFIG_NROS_RMW_CYCLONEDDS)
        set(_rmw "cyclonedds")
    else()
        set(_rmw "zenoh")
    endif()

    message(STATUS "nros_system_generate: baking ${_bringup_dir} → ${_out_dir} (rmw=${_rmw})")

    # Workspace choice:
    #   * Path A (sibling `system.toml`): workspace = parent dir; the
    #     bringup pkg is one member of the surrounding cargo workspace.
    #   * M-F.3 self-pkg: workspace == bringup dir; the pkg is its own
    #     workspace root (the upstream nros-cli planner treats it as a
    #     1-member synth bringup). Detected by re-running the self-pkg
    #     check on the resolved abs dir.
    _nros_system_detect_self_pkg("${_bringup_dir}" _is_self)
    if(_is_self)
        set(_workspace "${_bringup_dir}")
    else()
        get_filename_component(_workspace "${_bringup_dir}" DIRECTORY)
    endif()
    # issue 1018 — `codegen-system` writes `system_config.h`, which is compiled
    # into the image, so the bake outlives the configure that produced it. This
    # runs unconditionally per configure, which means its freshness is entirely
    # the question of whether a configure happens at all.
    nros_codegen_tool_reconfigure("${_nros_cli}")

    execute_process(
        COMMAND "${_nros_cli}" codegen-system
                --workspace "${_workspace}"
                --bringup   "${_bringup_dir}"
                --target    "zephyr-${_rmw}"
                --out       "${_out_parent}"
        WORKING_DIRECTORY "${_workspace}"
        RESULT_VARIABLE   _rc
        OUTPUT_VARIABLE   _stdout
        ERROR_VARIABLE    _stderr)

    if(NOT _rc EQUAL 0)
        message(FATAL_ERROR
            "nros codegen-system failed (rc=${_rc}):\n${_stdout}\n${_stderr}")
    endif()

    # Issue 0154 — phase-258 retired the `system_main.c` C-baker (its
    # `nros_component_*_register` externs died with the post-257 install
    # seam), so the bake contract is now the config header + config cmake
    # (+ Cargo.toml / nros-plan.json). No generated TU is attached to `app`
    # any more; components register through `__nros_component_<pkg>_install`.
    set(_config_h     "${_out_dir}/system_config.h")
    set(_config_cmake "${_out_dir}/system_config.cmake")
    if(NOT EXISTS "${_config_h}" OR NOT EXISTS "${_config_cmake}")
        message(FATAL_ERROR
            "nros codegen-system produced no system_config.h / "
            "system_config.cmake in ${_out_dir} (verb may be unimplemented "
            "in this CLI build).")
    endif()

    if(TARGET app)
        target_include_directories(app PRIVATE "${_out_dir}")
    else()
        message(WARNING
            "nros_system_generate called before find_package(Zephyr); "
            "deferring include attach. Call after project().")
    endif()
    zephyr_include_directories("${_out_dir}")

    # phase-206 W2 — bake `<bringup>/rmw/<backend>.<ext>` so the user's own
    # middleware config reaches the backend's own parser.
    #
    # HERE, and not in `nros_rmw_cyclonedds.cmake`, for two reasons. This is the
    # function that knows the bringup: `NROS_SYSTEM_BRINGUP_DIR` is set at the
    # BOTTOM of it, while `zephyr/CMakeLists.txt` includes the backend module at
    # line 162 -- long before any leaf calls this -- so the backend module could
    # only ever read a value left in the cache by a PREVIOUS configure. And this
    # is where the include attach already happens, so the generated header
    # reaches the backend's TUs by the same route `system_config.h` does.
    #
    # `_rmw` is already the backend's own name here (the Kconfig mapping above),
    # which is the vocabulary `nros_bake_rmw_user_config` takes -- no second
    # spelling to keep in sync.
    set(_rmw_cfg_dir "${_out_parent}/nros-rmw-user-config")
    nros_bake_rmw_user_config(
        BRINGUP "${_bringup_dir}"
        BACKEND "${_rmw}"
        OUT_DIR "${_rmw_cfg_dir}"
        RESULT  _rmw_cfg_header)
    if(_rmw_cfg_header)
        # Only when a config exists: the consumer guards on `__has_include`, so
        # an image with no config must see no directory and no header. Adding
        # the dir unconditionally would be harmless today and a trap the first
        # time a stale header survived in it.
        zephyr_include_directories("${_rmw_cfg_dir}")
        if(TARGET app)
            target_include_directories(app PRIVATE "${_rmw_cfg_dir}")
        endif()
        message(STATUS
            "nano-ros: ${_rmw} user config baked for bringup ${_bringup_dir}")
    endif()

    # Re-export for downstream consumers (tests, follow-up cmake fns).
    set(NROS_SYSTEM_DIR        "${_out_dir}"   CACHE PATH "Phase 212.E baked system tree" FORCE)
    set(NROS_SYSTEM_BRINGUP_DIR "${_bringup_dir}" CACHE PATH "Phase 212.H.1 bringup pkg" FORCE)
endfunction()
