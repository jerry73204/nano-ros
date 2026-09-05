# cmake/NanoRosEntry.cmake — Phase 212.N.6
#
# Defines the C++ cmake fn `nano_ros_entry(NAME <name> SOURCES <files...>
#   [BOARD <board>] DEPLOY <target1> [<target2> ...])`.
#
# This is the rename of the pre-N.6 `nano_ros_application` per Phase
# 212.L.9 + 212.N — see `cmake/NanoRosNodeRegister.cmake` for the
# legacy alias (DEPRECATION shim that forwards here). Both names
# resolve so the in-tree caller migration (212.N.7 wave) can land
# incrementally without breaking the configure step.
#
# Semantics (= pre-N.6 `nano_ros_application` body):
#   * `NAME` (required) — exe target + Entry pkg entity name.
#   * `SOURCES` (required, multi-value) — sources passed to
#     `add_executable`.
#   * `DEPLOY` (required, multi-value) — `native` is always allowed.
#     Phase 235.B: a non-`native` DEPLOY target is the embedded path and
#     REQUIRES a resolved Board — either an explicit `BOARD <key>` or one
#     derived from the Phase 215 `nano_ros_use_board(<name>)` import
#     (`NROS_BOARD_RUNNER`). Without a Board, a non-`native` DEPLOY
#     rejects configure with FATAL_ERROR (the pre-235 native-only rule).
#   * `BOARD` (optional, single-value) — Phase 212.N.6 addition: the
#     codegen board key (`native`, `zephyr`, `fvp-aemv8r-smp`, …) the
#     Entry pkg targets; flows to `nros codegen entry --board` and
#     selects the C++ Board adapter (`LinuxBoard` / `ZephyrBoard`).
#     Stored as the `NANO_ROS_BOARD` target property. Absent BOARD is
#     valid for host-native pkgs; for embedded DEPLOY it is auto-derived
#     from `NROS_BOARD_RUNNER` (Phase 235.B) when not passed explicitly.
#
# Side effect: appends an entry to the GLOBAL `NROS_APPLICATIONS_JSON`
# property and rewrites `${CMAKE_BINARY_DIR}/nros-metadata.json` via
# `_nros_metadata_emit()` (defined in NanoRosNodeRegister.cmake;
# we depend on it being included alongside this module).

if(DEFINED _NROS_ENTRY_INCLUDED)
    return()
endif()
set(_NROS_ENTRY_INCLUDED TRUE)

# phase-263 C2b — capture this module's dir GLOBALLY (cache) so the `nano_ros_entry`
# function can locate `cmake/templates/` no matter which subdirectory scope calls it.
# `_NROS_NODE_REGISTER_DIR` is only set in scopes that `include()` NanoRosNodeRegister
# directly (the standalone examples); a workspace entry pkg reaches us via the workspace
# guard, so that var is empty there.
set(_NROS_ENTRY_DIR "${CMAKE_CURRENT_LIST_DIR}" CACHE INTERNAL "nano_ros_entry module dir")

# Pull in the shared metadata-emit helper + GLOBAL property
# definitions. `NanoRosNodeRegister.cmake` is the SSoT for those
# (it predates this module). The include is guarded inside that file,
# so re-including is a no-op when callers already loaded it.
include("${CMAKE_CURRENT_LIST_DIR}/NanoRosNodeRegister.cmake")
# Shared helpers (nros_resolve_cli — issue #219). include_guard'd.
include("${CMAKE_CURRENT_LIST_DIR}/NanoRosCodegenCore.cmake")
# `nros_panic_policy_feature` — the ONE policy->feature mapping, shared with the
# workspace umbrella (phase-366). include_guard'd.
include("${CMAKE_CURRENT_LIST_DIR}/NanoRosFeatureSet.cmake")
# `nano_ros_support_library` / `nano_ros_link_support_libraries` — RFC-0065 D12,
# phase-383 W7.e/W7.f. Included HERE because this module is the seam BOTH the
# standalone path and the workspace-pkg guard (`_nros_import_once`) go through,
# so a support package's `CMakeLists.txt` can call the keyword wherever it sits.
# include_guard(GLOBAL)'d, and it defines only functions — no directory vars to
# lose when this include's frame pops (the `_NROS_ENTRY_DIR` hazard).
include("${CMAKE_CURRENT_LIST_DIR}/NanoRosSupportLibrary.cmake")
# `_nros_resolve_entry_locator` — the ONE producer of the connect locator, shared
# with `NanoRosNodeRegister.cmake`'s RTOS carriers (issue 0946). Its literals are
# CACHE INTERNAL precisely so they survive being included from inside a function
# frame, which is how the node-register side reaches it.
include("${CMAKE_CURRENT_LIST_DIR}/NanoRosEntryLocator.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/NanoRosRmwUserConfig.cmake")  # phase-206 W2

# --------------------------------------------------------------------------
# Platform-link wrappers (phase-287 W6). Defined HERE (not NanoRosBootstrap):
# the workspace guard path loads NanoRosEntry without Bootstrap, and both the
# C2 gate below and Bootstrap's `nano_ros_link` call these.
# --------------------------------------------------------------------------
# Idempotent wrapper around the platform overlay's `nros_platform_link_app`.
# The ament verb path runs the phase-263 C2 embedded link pass inside
# `nano_ros_entry` AND then calls `nano_ros_link` — on NuttX the second pass
# is fatal (`nros_nuttx_build_example` creates a named `<name>_build` custom
# target; duplicate = CMP0002 error), on other platforms it is silent
# duplication. A target property marks the first pass (phase-287 W6).
function(nros_platform_link_app_once target)
    get_property(_nros_linked TARGET ${target} PROPERTY _NROS_PLATFORM_LINK_DONE)
    if(_nros_linked)
        return()
    endif()
    set_property(TARGET ${target} PROPERTY _NROS_PLATFORM_LINK_DONE TRUE)
    nros_platform_link_app(${target})
endfunction()

# Deferred variant — runs the (once-guarded) platform link at END of the
# calling directory's scope. The ament shape links interface deps via
# `ament_target_dependencies(<t> std_msgs)` AFTER the `nano_ros_add_*` verb
# returns, but the NuttX board's `nros_platform_link_app` materialises the
# target's LINK_LIBRARIES / include closure into text files for the cargo
# cross-link AT CALL TIME — an immediate call sees no interface libs and the
# firmware compile dies with `std_msgs.h: No such file`. Deferring to scope
# end sees the fully-wired target; the once-guard collapses the gate's and
# `nano_ros_link`'s deferrals into one pass (phase-287 W6).
function(nros_platform_link_app_deferred target)
    # DEFER CALL arguments are re-evaluated when the deferred call runs (where
    # `${target}` no longer exists) — bake the value into the code string now.
    cmake_language(EVAL CODE
        "cmake_language(DEFER CALL nros_platform_link_app_once [[${target}]])")
endfunction()

function(nano_ros_entry)
    # Phase 219.D — LAUNCH + ARGS + LANG keyword args.
    # R1 / W4.2 — MODEL <system_model.yaml>: the canonical resolved-model
    # input (RFC-0052); mutually exclusive with LAUNCH.
    # phase-405 W1 — LOCATOR, ARGS and LAUNCH_ARGS are GONE. Each had zero
    # authored users in the tree (generated CMakeLists excluded, since those are
    # tool output rather than a caller's choice), and each carried live-looking
    # code that could never run. Dropping them from the parse turns such a call
    # into an UNPARSED_ARGUMENTS error rather than a line that silently does
    # nothing.
    #
    # phase-405 W4 — MODEL is GONE. Zero callers passed it, authored OR
    # generated, and it named a resolved artifact directly while BRINGUP+LAUNCH
    # name the INPUT that produces one (phase-330 W4.a). `_NRA_MODEL` is now
    # written only by the launch resolution below, so there is one way in.
    #
    # LAUNCH stays PARSED but is no longer required: 18 generated CMakeLists
    # pass a real launch file, and a generator should be explicit. What changes
    # is that a HUMAN need not write it — every one of the nine authored entries
    # said `LAUNCH default`, which is what BRINGUP alone now means.
    #
    # HOST is kept solely to fail loudly for a pre-phase-326 caller.
    cmake_parse_arguments(_NRA
        "TYPED"
        "NAME;BOARD;LAUNCH;LANG;HOST;BRINGUP;PANIC"
        "SOURCES;DEPLOY"
        ${ARGN})

    # BRINGUP with no LAUNCH means this bringup's default launch. Conditional on
    # BRINGUP on purpose: defaulting unconditionally would turn an entry with
    # nothing at all — a typo, a half-written CMakeLists — from the error below
    # into a silently accepted launch-addressed entry.
    if(_NRA_BRINGUP AND NOT _NRA_LAUNCH)
        set(_NRA_LAUNCH "default")
    endif()
    # phase-326 (issue 0364) — the bake-time host partition is gone with
    # `<node machine=>` (ROS 1 syntax). HOST stays PARSED so an old caller
    # fails here with guidance instead of the keyword silently joining
    # SOURCES via UNPARSED_ARGUMENTS.
    if(_NRA_HOST)
        message(FATAL_ERROR
            "nano_ros_entry(${_NRA_NAME}): HOST was removed (phase-326 / "
            "issue 0364) — multi-host partitions at RESOLVE time now. Point "
            "MODEL at the per-host SystemModel instead (resolved with "
            "`host:=${_NRA_HOST}`, e.g. "
            "MODEL config/multihost_${_NRA_HOST}_model.yaml).")
    endif()
    foreach(_req NAME DEPLOY)
        if(NOT _NRA_${_req})
            message(FATAL_ERROR
                "nano_ros_entry: ${_req} required")
        endif()
    endforeach()
    # SOURCES becomes optional when LAUNCH present — the generated TU
    # carries `int main()`. Standalone single-Node entry still needs
    # SOURCES (the caller provides their own `main`).
    if(NOT _NRA_LAUNCH AND NOT _NRA_SOURCES)
        message(FATAL_ERROR
            "nano_ros_entry: SOURCES required when LAUNCH is absent "
            "(single-Node self-bringup mode).")
    endif()
    # Phase 235.B — derive the board key from the Phase 215 board import.
    # `nano_ros_use_board(<name>)` caches `NROS_BOARD_RUNNER`
    # (armfvp / qemu / native / …). When the caller didn't pass an
    # explicit BOARD and an *embedded* board was imported (runner is set
    # and not "native"), default the codegen board key to "zephyr" — the
    # single metadata-driven embedded adapter (RFC-0032 §8a). Everything
    # board-specific (Zephyr BOARD id, DTS overlay, default RMW, runner)
    # already came from board.cmake at the `nano_ros_use_board` call, so
    # the C++ adapter needs only native-vs-Zephyr granularity here.
    if(NOT _NRA_BOARD AND DEFINED NROS_BOARD_RUNNER
       AND NOT "${NROS_BOARD_RUNNER}" STREQUAL ""
       AND NOT "${NROS_BOARD_RUNNER}" STREQUAL "native")
        set(_NRA_BOARD "zephyr")
        message(STATUS
            "nano_ros_entry(${_NRA_NAME}): embedded board imported "
            "(runner=${NROS_BOARD_RUNNER}) — codegen board key => zephyr "
            "(nros::board::ZephyrBoard).")
    endif()

    # DEPLOY gate. `native` is always allowed. A non-`native` DEPLOY
    # target is the embedded path (Phase 235.B) and REQUIRES a resolved
    # BOARD — either passed explicitly or derived above from the Phase 215
    # import. Without one, fail loudly (the pre-235 native-only rule).
    foreach(_t IN LISTS _NRA_DEPLOY)
        if(NOT _t STREQUAL "native" AND NOT _NRA_BOARD)
            message(FATAL_ERROR
                "nano_ros_entry: DEPLOY target '${_t}' rejected — "
                "embedded Entry pkgs need a Board. Either import a board "
                "via `nano_ros_use_board(<name>)` (sets NROS_BOARD_RUNNER) "
                "or pass `BOARD <key>` (e.g. zephyr). Native pkgs use "
                "`DEPLOY native`.")
        endif()
    endforeach()

    # phase-366 — resolved AFTER the argument gates above, not before them.
    #
    # This block ends the configure with a FATAL_ERROR when no nros-c/nros-cpp
    # Rust target is importable, which is the right answer for a real entry and
    # the WRONG one to reach first: a malformed call (embedded DEPLOY with no
    # Board, missing DEPLOY, ...) has no Rust target either, so running this
    # early replaced every one of those diagnostics with a complaint about the
    # panic policy. `cmake_node_register_misuse` caught exactly that — it asserts
    # the embedded-deploy rejection and got "PANIC platform cannot be applied"
    # instead. Validate the call, then act on it.
    # ---- phase-366 M4 / RFC-0077 — this image's ending -------------------
    #
    # `PANIC platform|halt|own`. Applied to the `nros-c`/`nros-cpp` staticlib
    # with `corrosion_set_features()`, which APPENDS, so it reaches the archive
    # even though the crate was imported earlier with its base feature set.
    #
    # It has to be the ENTRY that says this, and it has to be applied after the
    # entry is declared. `nros_feature_set()`'s per-platform table runs at import
    # time from globals and structurally cannot answer the question: "does
    # anything else already supply the handler?" is a property of the link step,
    # not of the RTOS. The same platform gives opposite answers for a
    # Zephyr-hosted C/C++ image (the `zephyr` crate supplies it, so `own`) and a
    # bare one linking `libnros_c.a` (nothing does, so `platform`).
    #
    # phase-366 M5/R2 — absent PANIC now means `platform`, and the per-platform
    # table no longer appends `panic-halt` (those four rows are gone). So an
    # entry that says nothing gets the board's ending, which is the same default
    # `nros::main!()` gives a Rust entry.
    #
    # This carries the behaviour change RFC-0077 called out rather than shipping
    # silently: an embedded C/C++ image used to halt on panic because the table
    # said so, and now ends the way its board ends — printing where the port
    # prints, `k_panic()` on Zephyr, exiting QEMU on the ThreadX RV64 board. An
    # image that wants the old behaviour says `PANIC halt`.
    # issue 0719 — ONE applier, shared with the image paths that cannot call
    # this function (board seams, the ESP-IDF component shim). Everything that
    # used to be inline here — the default, the mapper, cross-entry conflict
    # detection, the externally-applied-lane check (#0689), the four target
    # spellings, and the fail-loud — lives in `nros_apply_panic_policy`, because
    # two of those paths had hand-copied HALF of it.
    nros_apply_panic_policy("${_NRA_PANIC}" "nano_ros_entry(${_NRA_NAME})")

    # Phase 241.D3-rev — infer LANG from the source extensions when not given.
    # The C and C++ umbrellas are now DISTINCT staticlibs (`libnros_c.a` vs
    # `libnros_cpp.a`, one `std` each), so a C binary must link NanoRos (nros_c)
    # and a C++ binary NanoRosCpp (nros_cpp) — NEVER both, or `std`/compiler-builtins
    # collide. LANG used to default to `cpp`; harmless when NanoRosCpp was an ALIAS of
    # NanoRos, but post-single-runtime that dragged a second Rust staticlib into every
    # C example. A `.cpp`/`.cxx`/`.cc`/`.C` source ⇒ cpp; otherwise c. LAUNCH-only
    # (no SOURCES) keeps the historical `cpp` default.
    if(NOT _NRA_LANG)
        if(_NRA_SOURCES)
            set(_NRA_LANG c)
            foreach(_src ${_NRA_SOURCES})
                if(_src MATCHES "\\.(cpp|cxx|cc|C)$")
                    set(_NRA_LANG cpp)
                    break()
                endif()
            endforeach()
        else()
            set(_NRA_LANG cpp)
        endif()
    endif()

    # Phase 219.D — LAUNCH-aware fast path: shell `nros codegen entry`
    # at configure time, append the generated TU + auto-link sidecar.
    set(_sources_for_exe ${_NRA_SOURCES})
    if(_NRA_LAUNCH)
        # phase-296 R-code.1 — the launch-XML bake is REMOVED; the canonical
        # input is a play_launch-resolved SystemModel.
        if(_NRA_LAUNCH)
            # phase-330 W7 — the INPUT-ADDRESSED spelling: the entry names its
            # launch file (+ BRINGUP dir + optional LAUNCH_ARGS k=v bindings);
            # the model is a build artifact the entry never names. The mapping
            # rule lives ONCE in nros_orchestration_ir::model_location, reached
            # through `nros model-path` — NOT re-implemented here (the
            # second-spelling drift class). This is not the phase-296-deleted
            # parse path: nothing parses launch XML at configure; the launch
            # file ADDRESSES the resolver output the build layer produced.
            if(NOT _NRA_BRINGUP)
                message(FATAL_ERROR
                    "nano_ros_entry: LAUNCH requires BRINGUP <dir> (the bringup "
                    "package directory holding system.toml + launch/).")
            endif()
            set(_nra_mp_args --bringup-dir "${_NRA_BRINGUP}")
            if(NOT _NRA_LAUNCH STREQUAL "default")
                list(APPEND _nra_mp_args --launch "${_NRA_LAUNCH}")
            endif()
            nros_resolve_cli(_nra_mp_tool CONTEXT "nano_ros_entry LAUNCH")
            execute_process(
                COMMAND "${_nra_mp_tool}" model-path ${_nra_mp_args}
                OUTPUT_VARIABLE _NRA_MODEL
                OUTPUT_STRIP_TRAILING_WHITESPACE
                RESULT_VARIABLE _nra_mp_rc
                ERROR_VARIABLE _nra_mp_err)
            if(NOT _nra_mp_rc EQUAL 0)
                message(FATAL_ERROR
                    "nano_ros_entry: `nros model-path` failed for LAUNCH "
                    "'${_NRA_LAUNCH}' (bringup ${_NRA_BRINGUP}):\n${_nra_mp_err}")
            endif()
            # Re-resolve when the USER'S inputs change. DEDUPED: several
            # entries share one bringup (workspaces/c has 7), and a duplicate
            # CMAKE_CONFIGURE_DEPENDS entry becomes a duplicate OUTPUT on the
            # generated missing-inputs phony rule — pinned ninja 1.13 hard-
            # errors "defined as an output multiple times" (1.10 tolerated it).
            # NORMALIZED paths: each entry spells the shared bringup via its
            # own ${CMAKE_CURRENT_SOURCE_DIR}/../…, so string-dedupe alone
            # still emits N distinct spellings of ONE file — which ninja
            # normalizes into duplicate outputs on the missing-inputs phony
            # rule (pinned 1.13 hard-errors; 1.10 tolerated it).
            file(REAL_PATH "${_NRA_BRINGUP}/system.toml" _nra_dep_toml)
            get_property(_nra_cfg_deps DIRECTORY PROPERTY CMAKE_CONFIGURE_DEPENDS)
            if(NOT "${_nra_dep_toml}" IN_LIST _nra_cfg_deps)
                set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
                    "${_nra_dep_toml}")
            endif()
            if(NOT _NRA_LAUNCH STREQUAL "default")
                file(REAL_PATH "${_NRA_BRINGUP}/launch/${_NRA_LAUNCH}" _nra_dep_launch)
                get_property(_nra_cfg_deps DIRECTORY PROPERTY CMAKE_CONFIGURE_DEPENDS)
                if(NOT "${_nra_dep_launch}" IN_LIST _nra_cfg_deps)
                    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
                        "${_nra_dep_launch}")
                endif()
            endif()
            set(_NRA_LAUNCH "")
        endif()
        if(NOT _NRA_LANG)
            set(_NRA_LANG cpp)
        endif()
        if(NOT (_NRA_LANG STREQUAL "cpp" OR _NRA_LANG STREQUAL "c"))
            message(FATAL_ERROR
                "nano_ros_entry: LANG '${_NRA_LANG}' rejected — "
                "expected 'cpp' (default) or 'c'.")
        endif()
        # Phase 240.2b (RFC-0043) — TYPED routes each launch node to the real
        # executor via its component object (`--typed`), reading the cmake
        # metadata for each node's C++ class + header. C++ only; the metadata
        # must already list every component, so the node pkgs' add_subdirectory
        # has to precede the entry's (it links them anyway).
        # Phase 257 (W0-A) — TYPED now supports LANG c too: the generated C TU
        # routes each node to the real executor via its `NROS_C_COMPONENT`
        # factory/configure seam + `nros_board_native_run_components`.
        # phase-330 W3.b (RFC-0063) — prefer a BUILD-OUTPUT model over the
        # committed one the caller pointed at. Same order the Rust consumers use
        # via `nros_orchestration_ir::model_location`: an explicit shared dir
        # (`NROS_MODEL_DIR`, set by RFC-0065's builder) beats the per-build
        # `${CMAKE_BINARY_DIR}/nros/`, which beats the source copy. With nothing
        # generating into either, `MODEL` is used exactly as passed.
        if(_NRA_MODEL)
            get_filename_component(_nra_model_name "${_NRA_MODEL}" NAME)
            if(DEFINED ENV{NROS_MODEL_DIR} AND
               EXISTS "$ENV{NROS_MODEL_DIR}/${_nra_model_name}")
                set(_NRA_MODEL "$ENV{NROS_MODEL_DIR}/${_nra_model_name}")
            elseif(EXISTS "${CMAKE_BINARY_DIR}/nros/${_nra_model_name}")
                set(_NRA_MODEL "${CMAKE_BINARY_DIR}/nros/${_nra_model_name}")
            endif()
        endif()

        # phase-392 W5.c — the entity figures this entry DECLARES, folded into
        # the configure's accumulated view. One runtime staticlib serves every
        # entry here, so the union is applied once by
        # `nros_synth_runtime_umbrella`, which runs after the SUBDIRS loop.
        include("${_NROS_ENTRY_DIR}/NanoRosEntityFacts.cmake")
        nros_record_entity_facts("${_NRA_MODEL}")

        _nros_entry_invoke_codegen(
            NAME      "${_NRA_NAME}"
            LANG      "${_NRA_LANG}"
            LAUNCH    "${_NRA_LAUNCH}"
            MODEL     "${_NRA_MODEL}"
            BOARD     "${_NRA_BOARD}"
            ARGS_LIST ""
            TYPED     "${_NRA_TYPED}"
            OUT_VAR_GEN     _gen_tu
            OUT_VAR_LINKLIB _link_libs_cmake)
        list(APPEND _sources_for_exe "${_gen_tu}")
    endif()
    if(_NRA_LANG STREQUAL "c")
        set(_lang_tag "c")
    else()
        set(_lang_tag "cpp")
    endif()

    # phase-263 C2d — Zephyr build model. Unlike FreeRTOS/NuttX/ThreadX (add_executable +
    # `nros_platform_link_app`), a Zephyr app is the `app` target that the entry
    # CMakeLists' `find_package(Zephyr)` created. `nano_ros_entry` is the C/C++ analog of
    # zephyr-lang-rust's `rust_cargo_application()`: after find_package it wires the
    # generated entry TU into `app`, rather than building its own executable.
    set(_nra_is_zephyr FALSE)
    # issue 0326 — shared predicate.
    _nros_is_zephyr(_nra_zephyr_detected)
    if(_nra_zephyr_detected)
        set(_nra_is_zephyr TRUE)
    endif()

    if(_nra_is_zephyr)
        if(NOT TARGET app)
            message(FATAL_ERROR
                "nano_ros_entry(BOARD zephyr …): no Zephyr `app` target. A Zephyr workspace "
                "entry CMakeLists must call `find_package(Zephyr REQUIRED HINTS "
                "$ENV{ZEPHYR_BASE})` BEFORE project() so the `app` target exists.")
        endif()
        # The generated entry TU defines `int main(void)` (board_is_zephyr shape). It MUST
        # go directly into `app`: Zephyr links `libapp.a` whole-archive (so `main` is pulled
        # as a strong symbol, overriding Zephyr's weak default `main`), and the TU inherits
        # `app`'s include set — incl. the per-build `<nros/nros_{,cpp_}config_generated.h>`
        # storage-size headers the Zephyr module emits into
        # the cargo target dir's `nros-{c,cpp}-generated` via
        # `zephyr_include_directories` — and `app`'s `nros_cpp_cargo_build` ordering, for
        # free. (A separate static lib does NOT get whole-archived → `main` silently dropped.)
        target_sources(app PRIVATE ${_sources_for_exe})
        # The node component libs are linked by the sidecar (below) to ${_NRA_NAME}; carry
        # them in a placeholder static lib (Zephyr needs a real target for the sidecar's
        # `target_link_libraries`) and link it into `app`, so the component archives reach
        # `app`'s final link and resolve the `__nros_c_component_*` seams the TU references.
        set(_nra_app_stub "${CMAKE_CURRENT_BINARY_DIR}/${_NRA_NAME}_zephyr_components_stub.c")
        if(NOT EXISTS "${_nra_app_stub}")
            file(WRITE "${_nra_app_stub}"
                "/* phase-263 C2d — placeholder so the auto-link sidecar has a real target;\n"
                "   the node component libs link PRIVATE here and propagate to `app`. */\n")
        endif()
        add_library(${_NRA_NAME} STATIC "${_nra_app_stub}")
        target_link_libraries(app PRIVATE ${_NRA_NAME})
    elseif(NOT TARGET ${_NRA_NAME})
        add_executable(${_NRA_NAME} ${_sources_for_exe})
        # Phase 257 (W0-A) — a TYPED C entry drives the `nros_cpp_*` runtime
        # (`nros_board_native_run_components`, `nros_cpp_node_create`), which lives
        # in the C++ umbrella, so it links NanoRosCpp like a C++ entry — NOT the
        # C-only NanoRos. A legacy (non-typed) C entry keeps NanoRos.
        # issue 0425 — the C++ umbrella wins whenever it exists: it BUNDLES
        # nros-c, so a legacy C entry's `nros_*` calls still resolve and the
        # executable links exactly ONE Rust staticlib, which is the "NEVER both"
        # rule above. A pure-C workspace has no `NanoRosCpp` target and keeps
        # `NanoRos` unchanged.
        if(TARGET NanoRos::NanoRosCpp)
            target_link_libraries(${_NRA_NAME} PRIVATE NanoRos::NanoRosCpp)
        elseif(TARGET NanoRos::NanoRos)
            target_link_libraries(${_NRA_NAME} PRIVATE NanoRos::NanoRos)
        endif()
    else()
        # Target may have been declared by a sibling call (e.g. the
        # user supplied an empty SOURCES + LAUNCH/MODEL). Ensure the
        # generated TU still ends up on the target.
        if(_NRA_LAUNCH)
            target_sources(${_NRA_NAME} PRIVATE "${_gen_tu}")
        endif()
    endif()

    # Phase 219.J — auto-link every Node-pkg static lib the launch XML /
    # SystemModel named. The sidecar `<bin>/<exe>_link_libs.cmake` is
    # emitted by the same CLI invocation as the generated TU; it carries
    # one `target_link_libraries(<exe> PRIVATE <pkg>_<exec>_component)`
    # call per unique `(pkg, exec)` pair. RFC-0052 W4.3: the MODEL arm
    # emits the same sidecar (plan_from_model resolves the components), so
    # it must be included on the MODEL path too — otherwise a TYPED entry's
    # generated TU never sees the component class headers those libs carry
    # (their PUBLIC include dirs) and fails with `<pkg>/<Class>.hpp: No
    # such file`.
    if((_NRA_LAUNCH OR _NRA_MODEL) AND _link_libs_cmake)
        include("${_link_libs_cmake}")
    endif()

    # phase-263 C2c — on Zephyr the generated entry TU is compiled INTO `app` (not into
    # ${_NRA_NAME}), and a TYPED C++ entry `#include`s each node's component CLASS header
    # (`<pkg>/<Class>.hpp`). The sidecar above linked the component libs PRIVATE to the
    # ${_NRA_NAME} placeholder, so their PUBLIC include dirs do NOT reach `app`. Propagate
    # each linked component's interface include dirs onto `app` so the entry TU finds the
    # class headers. (Native/embedded entries compile the TU into ${_NRA_NAME}/the exe, which
    # links the components directly, so they already see these includes.)
    if(_nra_is_zephyr AND TARGET app AND TARGET ${_NRA_NAME})
        get_target_property(_nra_comp_libs ${_NRA_NAME} LINK_LIBRARIES)
        if(_nra_comp_libs)
            foreach(_nra_comp ${_nra_comp_libs})
                if(TARGET ${_nra_comp})
                    target_include_directories(app PRIVATE
                        $<TARGET_PROPERTY:${_nra_comp},INTERFACE_INCLUDE_DIRECTORIES>)
                endif()
            endforeach()
        endif()
    endif()

    # Phase 249 P4a (issue #57) — wire the strong `nros_app_register_backends()`
    # for native (host) C/C++ entries. `nros_cpp_init` / `nros_support_init`
    # call that symbol unconditionally, and P4a removed its weak default — the only
    # def is the one `nano_ros_link_rmw()` (via `nros_platform_link_app`) generates.
    # The `nano_ros_node_register` native carrier (244.C4) calls it, but the
    # LAUNCH-based `nano_ros_entry` path created the exe here, so that carrier's
    # `NOT TARGET` guard skips it — leaving a workspace native C/C++ Entry with an
    # undefined `nros_app_register_backends` at link (the cpp/mixed workspace fixture
    # link failure). The Rust workspace Entry is cargo-built (linkme, no
    # `nros_cpp_init`) so it is exempt; embedded Entries take the board link path.
    # `nano_ros_link_rmw` is idempotent (single accumulated stub), so this is safe
    # even when a node-register call also wired the same target.
    if(TARGET ${_NRA_NAME}
       AND NANO_ROS_PLATFORM STREQUAL "posix"
       AND COMMAND nros_platform_link_app
       AND ("native" IN_LIST _NRA_DEPLOY))
        nros_platform_link_app_deferred(${_NRA_NAME})
    endif()

    # phase-206 W2 — bake this bringup's own RMW config, on the ENTRY path.
    #
    # The sibling wiring is `nros_system_generate` (the Zephyr 212.H.1 shim).
    # Both are needed and neither covers the other: NO in-tree example calls the
    # shim -- its only callers are the `multi_pkg_workspace_zephyr` and
    # `zephyr_self_pkg` test fixtures -- while a real workspace/west image comes
    # through HERE. Wiring only the shim left the mechanism reachable from the
    # fixtures and from nothing a user builds.
    #
    # THE INCLUDE GOES ON THE BACKEND TARGET, not on the entry. `session.cpp` is
    # the TU that reads the generated header, and it compiles into
    # `nros_rmw_<backend>` -- a different target from the executable. Attaching
    # the directory to the entry alone would configure cleanly, compile cleanly,
    # and bake nothing into the image; that is the failure mode this whole work
    # item exists to stop being possible. The entry gets it too, for a C++ entry
    # that includes the header itself.
    #
    # Guarded on the target EXISTING because a consumer that finds NanoRos as an
    # installed package has an IMPORTED backend it cannot recompile. That case
    # gets a warning rather than silence -- a config that does not apply must not
    # look like one that did.
    if(_NRA_BRINGUP)
        # Same resolution ORDER as `nano_ros_link_rmw`'s fallback list
        # ("NANO_ROS_DEFAULT_RMW;NANO_ROS_RMW"), not that function itself: it
        # FATAL_ERRORs when nothing resolves, and an entry with no RMW chosen yet
        # is a legitimate state here.
        set(_nra_rmw "")
        if(DEFINED NANO_ROS_DEFAULT_RMW AND NOT "${NANO_ROS_DEFAULT_RMW}" STREQUAL "")
            set(_nra_rmw "${NANO_ROS_DEFAULT_RMW}")
        elseif(DEFINED NANO_ROS_RMW AND NOT "${NANO_ROS_RMW}" STREQUAL "")
            set(_nra_rmw "${NANO_ROS_RMW}")
        endif()

        if(_nra_rmw)
            set(_nra_cfg_dir "${CMAKE_BINARY_DIR}/nros-rmw-user-config")
            nros_bake_rmw_user_config(
                BRINGUP "${_NRA_BRINGUP}"
                BACKEND "${_nra_rmw}"
                OUT_DIR "${_nra_cfg_dir}"
                RESULT  _nra_cfg_header)
            if(_nra_cfg_header)
                if(TARGET nros_rmw_${_nra_rmw})
                    target_include_directories(nros_rmw_${_nra_rmw}
                        PRIVATE "${_nra_cfg_dir}")
                    message(STATUS
                        "nano-ros: ${_nra_rmw} user config reaches "
                        "nros_rmw_${_nra_rmw} (entry ${_NRA_NAME})")
                else()
                    message(WARNING
                        "nano_ros_entry(${_NRA_NAME}): ${_NRA_BRINGUP} ships an "
                        "${_nra_rmw} config, but target nros_rmw_${_nra_rmw} is "
                        "not in this build tree (an imported/installed backend "
                        "cannot be recompiled), so the config CANNOT take "
                        "effect. Build the backend from source, or set "
                        "CYCLONEDDS_URI at run time on a hosted target.")
                endif()
                if(TARGET ${_NRA_NAME})
                    target_include_directories(${_NRA_NAME}
                        PRIVATE "${_nra_cfg_dir}")
                endif()
            endif()
        endif()
    endif()

    # phase-263 C2b — bake the connect locator BEFORE the embedded link pass. On NuttX the
    # board overlay's `nros_platform_link_app` FERRIES the target's COMPILE_DEFINITIONS into the
    # cargo cc-rs kernel build AT CONFIGURE TIME, so `NROS_ENTRY_LOCATOR` must already be on the
    # target when the link pass runs — otherwise the entry TU bakes the `<nros/main.hpp>` default
    # (tcp/127.0.0.1:7447) and never connects to the host router. (FreeRTOS/ThreadX link at BUILD
    # time, so their order is immaterial — but setting it here is correct for them too; the later
    # block then only does the FreeRTOS app-config TU + header-mirror ordering.) Zephyr is exempt
    # (Kconfig locator). Precedence: -DNROS_ENTRY_LOCATOR cache > per-board default
    # (threadx-linux dials host loopback; QEMU boards dial the slirp host 10.0.2.2).
    # Whether THIS entry targets the active board. Two spellings (phase-287 W5):
    #   * workspace system.toml deploy targets are NAMED BY BOARD — the active
    #     `NANO_ROS_BOARD` appears in the entry's DEPLOY list (phase-263 C2);
    #   * the RFC-0048 `package.xml <export><nano_ros deploy=…/>` tuple carries
    #     the deploy/PLATFORM token ("freertos", …) in DEPLOY while the board
    #     rides the `board=` attr — there the active platform matches instead.
    # Normalize legacy/variant platform spellings to the deploy token before
    # comparing (`threadx_linux` / `threadx_riscv64` → `threadx`,
    # `freertos_armcm3` → `freertos`, `nuttx_armv7a` → `nuttx`) — the just
    # recipes still pass `-DNANO_ROS_PLATFORM=threadx_linux`, which the root
    # maps to the threadx module but would never equal the tuple's `threadx`.
    set(_nra_platform_norm "${NANO_ROS_PLATFORM}")
    if(_nra_platform_norm MATCHES "^(threadx|freertos|nuttx)_")
        string(REGEX REPLACE "_.*$" "" _nra_platform_norm "${_nra_platform_norm}")
    endif()
    # issue 0735 — the board-name test is ONE of the three arms, not a gate over
    # all of them. It used to open `if(DEFINED NANO_ROS_BOARD)`, which makes a
    # board NAME the precondition for being a board. That is a proxy for "this
    # is an embedded build", and it is exactly inverted for RFC-0064: a board
    # arriving through an integration shell contributes NO files to this tree
    # and therefore has no `NANO_ROS_BOARD` to define — that is the design (the
    # FreeRTOS platform module's board-less mode exists for it; ESP-IDF's shell
    # is 146 lines with no board name at all).
    #
    # So every shell-integrated entry answered FALSE and silently lost all
    # three dependent blocks below: the `NROS_ENTRY_LOCATOR` + domain bake (the
    # entry connects nowhere), `nros_platform_link_app_deferred()` (NO platform
    # link — no family C glue, no `freertos_platform` on the target), and the
    # FreeRTOS `NROS_APP_CONFIG` TU (network + task sizing fall back to
    # defaults). Configure succeeded, the build succeeded, the image was wrong.
    #
    # Dropping the outer guard cannot change any in-tree board's answer: every
    # one of them defines `NANO_ROS_BOARD`, and with it unset the first clause
    # is simply false. All three dependent blocks independently require a
    # non-posix platform, so the host lane is untouched either way.
    set(_nra_board_active FALSE)
    if(("${NANO_ROS_BOARD}" IN_LIST _NRA_DEPLOY)
       OR ("${NANO_ROS_PLATFORM}" IN_LIST _NRA_DEPLOY)
       OR ("${_nra_platform_norm}" IN_LIST _NRA_DEPLOY))
        set(_nra_board_active TRUE)
    endif()

    set(_nra_locator "")
    if(TARGET ${_NRA_NAME}
       AND NOT NANO_ROS_PLATFORM STREQUAL "posix"
       AND NOT _nra_is_zephyr
       AND _nra_board_active)
        # issue 0946 — ONE producer. This ladder used to be spelled out here AND
        # again, with different values, in NanoRosNodeRegister.cmake's three RTOS
        # carriers. Both are now rungs of `_nros_resolve_entry_locator()`, which
        # preserves each lane's own answers rather than merging them (they cite
        # different networks; see NanoRosEntryLocator.cmake).
        _nros_resolve_entry_locator(entry
            "${NANO_ROS_PLATFORM}" "${NANO_ROS_BOARD}" _nra_locator)
        target_compile_definitions(${_NRA_NAME} PRIVATE
            "NROS_ENTRY_LOCATOR=\"${_nra_locator}\"")
        # phase-287 W6 — bake the domain the same way (fixture pairs pass
        # `-DNROS_DOMAIN_ID=<n>`; portable sources read NROS_ENTRY_DOMAIN_ID).
        if(DEFINED NROS_DOMAIN_ID)
            target_compile_definitions(${_NRA_NAME} PRIVATE
                "NROS_ENTRY_DOMAIN_ID=${NROS_DOMAIN_ID}")
        endif()
    endif()

    # Issue 1050 defect (3) — bake the RMW SELECTOR, and do it OUTSIDE the block
    # above.
    #
    # The locator and domain bakes are gated on a non-posix, non-Zephyr, active
    # board, because a locator is an embedded-deployment fact. The selector is
    # not: the case it exists for is a HOSTED one. A Rust backend compiled into
    # `libnros_cpp.a` registers itself from an `.init_array` ctor, which runs
    # before `main` and therefore before the image's own generated
    # `nros_app_register_backends()`. So on hosted POSIX the registry can hold a
    # backend the image never declared, and — until this — the image had no way
    # to say which one it meant.
    #
    # `NANO_ROS_RMW` is exactly the registry's vocabulary: each backend
    # registers under the same four names this variable takes
    # (`nros_rmw_cffi_register_named("zenoh"/"cyclonedds"/"xrce"/"uorb", …)`).
    # `check-entry-rmw-vocabulary` holds the two sides together, because a
    # selector that names no registered backend is a hard error rather than a
    # fallback — a drift here would turn a working image into one that cannot
    # open a session.
    if(TARGET ${_NRA_NAME} AND NANO_ROS_RMW)
        target_compile_definitions(${_NRA_NAME} PRIVATE
            "NROS_ENTRY_RMW=\"${NANO_ROS_RMW}\"")
    endif()

    # phase-263 C2 (issue 0097) — embedded LAUNCH Entry link pass. The standalone
    # `nano_ros_node_register` carrier calls `nros_platform_link_app` for the embedded
    # boards (startup source + app_define + linker script + kernel/netstack umbrella +
    # RMW stub), but the LAUNCH path builds the exe HERE. The active platform module
    # (selected by the workspace-root `NANO_ROS_PLATFORM` + `NANO_ROS_BOARD`) loaded the
    # board overlay, so `nros_platform_link_app` does the correct per-board work. Gated on
    # `NANO_ROS_BOARD IN_LIST DEPLOY` so only the entry matching the active board links.
    if(TARGET ${_NRA_NAME}
       AND NOT NANO_ROS_PLATFORM STREQUAL "posix"
       AND NOT _nra_is_zephyr
       AND COMMAND nros_platform_link_app
       AND _nra_board_active)
        nros_platform_link_app_deferred(${_NRA_NAME})
    endif()

    # phase-263 C2a (issue 0097) — two wirings the embedded LAUNCH Entry needs that the
    # `nano_ros_node_register` carrier already does, but the LAUNCH path (exe built HERE)
    # was missing. Both gated identically to the link pass (embedded + active board in
    # DEPLOY), so a posix configure / a non-matching board entry is untouched.
    #
    #  (1) Baked connect locator. The generated TU calls the locator-LESS
    #      `<Board>::run_components(&setup)` overload, which reads the compile-time
    #      `NROS_ENTRY_LOCATOR` macro. Its header default is "" → backend discovery, which
    #      finds no router over the nsos POSIX-connect shim (threadx-linux host-sim) or
    #      QEMU slirp, so `nros::init` fails and `run_components` returns before the spin
    #      (no publish). Define the macro on the target. Precedence:
    #      `-DNROS_ENTRY_LOCATOR=…` cache override > per-board default
    #      (threadx-linux dials the host loopback — nsos `connect()` reaches it with NO
    #      veth bridge / root; QEMU boards dial the slirp host 10.0.2.2). The C2a /
    #      rtos_e2e fixture threads the per-fixture port via the cache var, mirroring the
    #      carrier's `-DNROS_THREADX_LOCATOR`.
    #
    #  (2) Header-mirror ordering. The generated `.cpp` TU includes
    #      <nros/nros_{,cpp_}config_generated.h> (the *_OPAQUE_U64S sizes). On a FRESH
    #      embedded build it can compile BEFORE Corrosion mirrors the per-build header,
    #      reading the in-tree stub (`*_OPAQUE_U64S undeclared`) — issues 0088/0090. Add
    #      the same edges the carrier uses: `add_dependencies` on the mirror targets + a
    #      HARD file-level OBJECT_DEPENDS on the mirrored header(s).
    if(TARGET ${_NRA_NAME}
       AND NOT NANO_ROS_PLATFORM STREQUAL "posix"
       AND _nra_board_active)
        # (1) locator — baked earlier (BEFORE the link pass, for the NuttX configure-time
        # COMPILE_DEFINITIONS ferry). `_nra_locator` is in scope here. Zephyr is EXEMPT (its
        # locator threads via the `CONFIG_NROS_ZENOH_LOCATOR` Kconfig), so the earlier block left
        # `_nra_locator` empty for it; the FreeRTOS `NROS_APP_CONFIG` TU below is board-startup-only
        # and likewise skipped on Zephyr.
        if(NOT _nra_is_zephyr)
            # (1b) phase-263 C2b — FreeRTOS `NROS_APP_CONFIG` TU. The board's `startup.c`
            # reads `NROS_APP_CONFIG` (LAN9118/lwIP bring-up + FreeRTOS task prio/stacks);
            # unlike ThreadX (whose `nros_platform_link_app` bakes its own app config) the
            # FreeRTOS platform link does NOT, so the `nano_ros_node_register` carrier
            # generates it from `templates/freertos_app_config.c.in`. The LAUNCH path builds
            # the exe here, so generate + attach the same TU (only LOCATOR + DOMAIN vary; the
            # network/scheduling fields are board defaults).
            #
            # issue 0946 — this block used to close with "Mirrors
            # NanoRosNodeRegister.cmake freertos branch; keep in sync". That
            # instruction is gone because the thing it asked a human to sync is
            # gone: the locator now has ONE producer
            # (`_nros_resolve_entry_locator`, gated by
            # `check-entry-locator-ssot`), and the two lanes' values are
            # DELIBERATELY different rungs of it rather than a duplication
            # anyone has to keep equal. What remains shared with the carrier is
            # the TEMPLATE, `templates/freertos_app_config.c.in`, which is one
            # file both paths `configure_file()` — a shared file needs no sync
            # note.
            if(NANO_ROS_PLATFORM STREQUAL "freertos")
                # The `@NROS_ENTRY_LOCATOR@` substitution the template reads —
                # an assignment FROM the resolved value, not a second producer.
                set(NROS_ENTRY_LOCATOR "${_nra_locator}")
                set(NROS_ENTRY_APP_DOMAIN_ID 0)
                # Per-image IP last octet (default .10). Test pairs bake a
                # distinct value per member (`-DNROS_ENTRY_IP_LAST=11` via
                # fixtures.toml) — identical IP+MAC seeds → identical ZIDs →
                # the router sees one peer and delivery silently dies.
                if(NOT DEFINED NROS_ENTRY_IP_LAST)
                    set(NROS_ENTRY_IP_LAST 10)
                endif()
                set(_nra_appcfg "${CMAKE_CURRENT_BINARY_DIR}/${_NRA_NAME}_nros_app_config_def.c")
                configure_file(
                    "${_NROS_ENTRY_DIR}/templates/freertos_app_config.c.in"
                    "${_nra_appcfg}" @ONLY)
                target_sources(${_NRA_NAME} PRIVATE "${_nra_appcfg}")
            endif()
        endif()

        # (2) header-mirror ordering for the generated TU (all embedded boards, incl. Zephyr)
        if(_gen_tu)
            if(COMMAND _nros_node_register_config_header_deps)
                _nros_node_register_config_header_deps(${_NRA_NAME})
            endif()
            get_property(_nra_c_hdr   GLOBAL PROPERTY NROS_C_CONFIG_HEADER_FILE)
            get_property(_nra_cpp_hdr GLOBAL PROPERTY NROS_CPP_CONFIG_HEADER_FILE)
            set(_nra_cfg_hdrs "")
            if(_nra_c_hdr)
                list(APPEND _nra_cfg_hdrs "${_nra_c_hdr}")
            endif()
            if(_nra_cpp_hdr)
                list(APPEND _nra_cfg_hdrs "${_nra_cpp_hdr}")
            endif()
            if(_nra_cfg_hdrs)
                if(COMMAND _nros_config_header_stamp)
                    _nros_config_header_stamp(_nra_cfg_stamp "${_NRA_NAME}" ${_nra_cfg_hdrs})
                else()
                    set(_nra_cfg_stamp ${_nra_cfg_hdrs})
                endif()
                set_source_files_properties("${_gen_tu}" PROPERTIES
                    OBJECT_DEPENDS "${_nra_cfg_stamp}")
            endif()
        endif()
    endif()

    # Issue 0114 — the header-mirror race (issues 0088/0090) ALSO hits the NATIVE
    # (posix) C/C++ cmake fixtures, which the embedded-only block above skips. An
    # example like cpp `safety-listener` compiles `main.cpp` (→ <nros/nros.hpp> →
    # <nros/nros_generated.h> → the `*_OPAQUE_U64S` sizes) BEFORE Corrosion's
    # `nros_{c,cpp}_config_header` mirror custom command runs. The mirror dir is on
    # the include path AHEAD of the in-tree stub, but the mirrored file does not
    # exist yet, so the compile falls through to the stub (`#error` /
    # `*_OPAQUE_U64S undeclared` → cascade `Subscription has no member storage_`).
    # `add_dependencies` alone orders the TARGET but not each TU (issues 0088/0090),
    # and the embedded block only guards the generated TU — so here set a HARD
    # file-level `OBJECT_DEPENDS` on EVERY source of the entry (incl. the user
    # `main.cpp`) pointing at the mirrored header(s).
    if(TARGET ${_NRA_NAME} AND NANO_ROS_PLATFORM STREQUAL "posix")
        get_property(_nra_c_hdr   GLOBAL PROPERTY NROS_C_CONFIG_HEADER_FILE)
        get_property(_nra_cpp_hdr GLOBAL PROPERTY NROS_CPP_CONFIG_HEADER_FILE)
        set(_nra_cfg_hdrs "")
        if(_nra_c_hdr)
            list(APPEND _nra_cfg_hdrs "${_nra_c_hdr}")
        endif()
        if(_nra_cpp_hdr)
            list(APPEND _nra_cfg_hdrs "${_nra_cpp_hdr}")
        endif()
        if(_nra_cfg_hdrs)
            foreach(_dep nros_c_config_header nros_cpp_config_header)
                if(TARGET ${_dep})
                    add_dependencies(${_NRA_NAME} ${_dep})
                endif()
            endforeach()
            get_target_property(_nra_srcs ${_NRA_NAME} SOURCES)
            if(_nra_srcs)
                # issue 0740 — a LOCAL stamp, not the cross-directory mirror
                # path. `add_dependencies` above orders the TARGET; it does not
                # give this directory's .o rule a prerequisite it can BUILD, so
                # under Makefiles a clean consumer build dies with "No rule to
                # make target". The stamp carries the same content edge.
                if(COMMAND _nros_config_header_stamp)
                    _nros_config_header_stamp(_nra_cfg_stamp "${_NRA_NAME}" ${_nra_cfg_hdrs})
                else()
                    set(_nra_cfg_stamp ${_nra_cfg_hdrs})
                endif()
                set_source_files_properties(${_nra_srcs} PROPERTIES
                    OBJECT_DEPENDS "${_nra_cfg_stamp}")
            endif()
        endif()
    endif()

    # Phase 212.N.6 — stash the BOARD selection on the target so the
    # later N.4 / N.5 codegen planner can read it. Empty when caller
    # didn't pass BOARD.
    if(DEFINED _NRA_BOARD)
        set_target_properties(${_NRA_NAME} PROPERTIES
            NANO_ROS_BOARD "${_NRA_BOARD}")
    endif()

    _nros_json_strlist(_sources_json ${_sources_for_exe})
    _nros_json_strlist(_deploy_json  ${_NRA_DEPLOY})
    get_property(_acc GLOBAL PROPERTY NROS_APPLICATIONS_JSON)
    if(_acc)
        set(_sep ",")
    else()
        set(_sep "")
    endif()
    set(_entry
"${_sep}\n    {\"name\": \"${_NRA_NAME}\", \"sources\": [${_sources_json}], \
\"deploy\": [${_deploy_json}], \"pkg_dir\": \"${CMAKE_CURRENT_SOURCE_DIR}\", \
\"lang\": \"${_lang_tag}\"}")
    set_property(GLOBAL APPEND_STRING PROPERTY NROS_APPLICATIONS_JSON "${_entry}")
    _nros_metadata_emit()

    # phase-383 W7.e — RFC-0065 D12: under D4 the entry package is GENERATED, so
    # a user has no `CMakeLists.txt` here to add a link line to. The entry is the
    # image, therefore the entry is the consumer of every support package this
    # configure declared. Registering unconditionally is safe and deliberate:
    # `_nano_ros_support_flush()` returns immediately when no support library
    # exists, and the attach itself is deferred to the end of the top-level
    # scope, so the support package may be read before OR after this one.
    if(TARGET ${_NRA_NAME} AND COMMAND nano_ros_link_support_libraries)
        nano_ros_link_support_libraries(${_NRA_NAME})
    endif()
endfunction()

# ---------------------------------------------------------------------------
# Helper — resolve a `nros` CLI binary, shell `nros codegen entry`, and
# slurp the depfile into `CMAKE_CONFIGURE_DEPENDS`.
#
# Inputs (single-value kw):
#   NAME      — Entry exe target name (used to build the sidecar path).
#   LANG      — "cpp" (default) or "c".
#   LAUNCH    — `<bringup>:<file>.launch.xml` spec.
#   BOARD     — optional board key (empty => CLI defaults to native).
#   ARGS_LIST — semicolon-separated `k=v` pairs; relayed to the CLI.
#
# Outputs (PARENT_SCOPE):
#   <OUT_VAR_GEN>     — path of the generated TU (to add to SOURCES).
#   <OUT_VAR_LINKLIB> — path of the sidecar `.cmake` carrying the
#                       `target_link_libraries(<NAME> PRIVATE …)` call.
# ---------------------------------------------------------------------------
function(_nros_entry_invoke_codegen)
    cmake_parse_arguments(_NRX
        ""
        "NAME;LANG;LAUNCH;MODEL;BOARD;TYPED;OUT_VAR_GEN;OUT_VAR_LINKLIB"
        "ARGS_LIST"
        ${ARGN})

    # Resolve the nros CLI binary: NROS_CLI_BIN cache override, else the
    # shared resolver (issue #219 — env NROS_CLI, the shared codegen-tool
    # cache, then PATH before the provisioned-store fallback; FATAL with the
    # setup-cli guidance when absent).
    set(_nros_bin "")
    if(NROS_CLI_BIN)
        set(_nros_bin "${NROS_CLI_BIN}")
    else()
        nros_resolve_cli(_nros_bin CONTEXT "nano_ros_entry(LAUNCH …)")
    endif()

    # Workspace root: walk up from the Entry-pkg dir until we hit a
    # `package.xml`-bearing sibling that's NOT the Entry pkg itself.
    # Practically: the caller workspace-root is two levels up
    # (`src/<entry_pkg>/CMakeLists.txt` → `..` → `..`). Falls back to
    # CMAKE_SOURCE_DIR if the assumed layout doesn't hold.
    get_filename_component(_pkg_parent "${CMAKE_CURRENT_SOURCE_DIR}" DIRECTORY)
    get_filename_component(_pkg_grandparent "${_pkg_parent}" DIRECTORY)
    if(EXISTS "${_pkg_grandparent}/CMakeLists.txt" OR EXISTS "${_pkg_grandparent}/Cargo.toml")
        set(_ws_root "${_pkg_grandparent}")
    else()
        set(_ws_root "${CMAKE_SOURCE_DIR}")
    endif()

    # phase-263 C2 (issue 0097) — an embedded C entry's generated TU is C++ (it drives
    # the C++ board runner `ThreadxBoard::run_components`, calling each C node via its
    # `extern "C"` seam), so emit `.cpp` even for `LANG c` on a non-posix configure. The
    # codegen routes embedded-C through the C++ emitter to match. Native C stays `.c`.
    if(_NRX_LANG STREQUAL "c" AND DEFINED NANO_ROS_PLATFORM
       AND NOT NANO_ROS_PLATFORM STREQUAL "posix")
        set(_ext "cpp")
    elseif(_NRX_LANG STREQUAL "c")
        set(_ext "c")
    else()
        set(_ext "cpp")
    endif()

    # Per-target output paths under the build dir. Sidecars share the
    # current dir to keep the relative location simple for the
    # `include()` call back in `nano_ros_entry`.
    set(_gen_path
        "${CMAKE_CURRENT_BINARY_DIR}/${_NRX_NAME}_nros_main_generated.${_ext}")
    set(_depfile_path
        "${CMAKE_CURRENT_BINARY_DIR}/${_NRX_NAME}_nros_main_generated.d")
    set(_link_libs_path
        "${CMAKE_CURRENT_BINARY_DIR}/${_NRX_NAME}_link_libs.cmake")

    set(_cli_args
        codegen entry
        --lang "${_NRX_LANG}"
        --workspace "${_ws_root}"
        --out "${_gen_path}"
        --depfile "${_depfile_path}"
        --emit-link-libs "${_NRX_NAME}=${_link_libs_path}")
    # R-code.1 — MODEL (resolved SystemModel) is the ONLY input; the
    # launch-XML bake was removed (phase-296 R4).
    if(_NRX_LAUNCH)
        message(FATAL_ERROR
            "nano_ros_entry: the LAUNCH keyword was removed (phase-296 R4) — "
            "resolve a SystemModel (play_launch resolve … -o "
            "<bringup>/config/system_model.yaml) and pass MODEL instead")
    endif()
    if(_NRX_MODEL)
        list(APPEND _cli_args --model "${_NRX_MODEL}")
    else()
        message(FATAL_ERROR "nano_ros_entry codegen: MODEL required")
    endif()
    if(_NRX_BOARD)
        list(APPEND _cli_args --board "${_NRX_BOARD}")
    endif()
    # Phase 240.2b — typed executor Entry: pass the cmake metadata so the
    # codegen can map each launch `(pkg, exec)` to its C++ class + header.
    if(_NRX_TYPED)
        list(APPEND _cli_args
            --typed
            --metadata "${CMAKE_BINARY_DIR}/nros-metadata.json")
    endif()
    if(_NRX_ARGS_LIST)
        # The cmake list uses `;` separators; the CLI expects `,`.
        string(REPLACE ";" "," _cli_args_csv "${_NRX_ARGS_LIST}")
        list(APPEND _cli_args --args "${_cli_args_csv}")
    endif()

    execute_process(
        COMMAND "${_nros_bin}" ${_cli_args}
        WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
        RESULT_VARIABLE _rc
        OUTPUT_VARIABLE _stdout
        ERROR_VARIABLE  _stderr)
    if(NOT _rc EQUAL 0)
        message(FATAL_ERROR
            "nano_ros_entry(LAUNCH \"${_NRX_LAUNCH}\"): "
            "`nros codegen entry` failed (rc=${_rc}).\n"
            "  CLI: ${_nros_bin} ${_cli_args}\n"
            "  stdout: ${_stdout}\n"
            "  stderr: ${_stderr}")
    endif()

    # phase-403 W9 (issue 0965) — compose this image's ENTITY inventory.
    #
    # HERE and not in `nano_ros_node_register()` because the answer is a
    # property of the whole IMAGE: `NROS_EXECUTOR_MAX_CBS` sizes ONE executor
    # and an image has one. Same reason `nros_derive_message_bound_knobs` runs
    # at the end of `nros_find_interfaces()` rather than per package.
    #
    # And here specifically because this is the first point in a configure that
    # is guaranteed to be AFTER every `nano_ros_node_register()`: the entry is
    # declared last, which is exactly what the `--metadata` argument two blocks
    # up already relies on.
    #
    # Never fatal on absence. The knobs file is a promise on a clean tree, and
    # every image built before this wave has no `ENTITIES` at all — those keep
    # their configured `MAX_CBS` and are unaffected.
    #
    # Issue 0991 -- and because it is composed HERE, every reader of it already
    # ran: the Zephyr knob resolver during `find_package(Zephyr)`, the
    # payload-class join at the end of each `nros_find_interfaces()`. If this
    # composition writes different bytes than those readers saw, the build about
    # to start is sized from the PREVIOUS answer, which on a clean build dir is
    # the "no inventory yet" placeholder -- and on a memory-tight image that is
    # a link failure naming a byte count and no knob. `nros_reconfigure_*` makes
    # cmake run again before the build proceeds; see NanoRosReconfigure.cmake
    # for why the `CMAKE_CONFIGURE_DEPENDS` recovery this used to rely on never
    # fired.
    #
    # "Again" is not "once" (issue 1002). This fragment feeds the payload-class
    # join, whose OWN output is what the knob resolver reads, so a clean build
    # dir takes THREE configures to deliver a derived size: one to learn the
    # entities, one to re-derive the sizes from them, one for a reader to see
    # those sizes. Ninja runs all three inside the same `west build`.
    include("${_NROS_ENTRY_DIR}/NanoRosEntityInventory.cmake")
    include("${_NROS_ENTRY_DIR}/NanoRosReconfigure.cmake")
    nros_entity_inventory_knobs_file(_entity_knobs_path)
    nros_reconfigure_snapshot("${_entity_knobs_path}" _entity_knobs_before)
    nros_derive_entity_inventory_knobs(CLI "${_nros_bin}")
    nros_reconfigure_on_change("${_entity_knobs_path}" "${_entity_knobs_before}"
        LABEL "this image's entity inventory")
    # issue 1033 — tell the deferred non-entry composer to stand down. An entry
    # composes HERE, after its own registrations, which is the earliest correct
    # point; the deferred one exists only for images that never call this verb.
    set_property(GLOBAL PROPERTY NROS_ENTITY_INVENTORY_COMPOSED TRUE)

    # #182 — the generated TU is a function of the CODEGEN TOOL too, not just
    # its inputs: a `nros` rebuild that changes the emitter (e.g. the fd32a0f75
    # group-split fallback, the phase-281 tier seams) must re-run this
    # configure-time codegen, or an existing build dir keeps linking a museum
    # TU while every source-level dep looks current (the #147 resolver probe
    # reads the toolchain dep graph and is equally blind to the tool). Depend
    # on the CLI binary itself: its mtime change → cmake re-configure →
    # `nros codegen entry` re-runs.
    #
    # issue 1018 — this WAS four lines of `set_property` here, and it was the
    # only configure-time emitter in the tree that had them. Same rule, shared
    # spelling, so the sibling emitters cannot drift back out of it.
    nros_codegen_tool_reconfigure("${_nros_bin}")

    # CONFIGURE_DEPENDS from the depfile so any change to the launch
    # XML, any package.xml the pkg-index walked, or the bringup's
    # `system.toml` re-runs cmake configure.
    if(EXISTS "${_depfile_path}")
        file(READ "${_depfile_path}" _dep_text)
        # Strip the `<target>: \` prefix.
        string(REGEX REPLACE "^[^:]*:" "" _dep_text "${_dep_text}")
        # Split on backslash-newline and whitespace; cmake list semantics
        # tokenise on `;`, so we first normalise whitespace runs to a
        # single space, then turn each space + path into a list entry
        # by replacing space with `;` after backslash removal.
        string(REPLACE "\\\n" " " _dep_text "${_dep_text}")
        string(REPLACE "\n" " " _dep_text "${_dep_text}")
        # Collapse runs of whitespace.
        string(REGEX REPLACE "[ \t]+" " " _dep_text "${_dep_text}")
        string(STRIP "${_dep_text}" _dep_text)
        # Convert space-separated paths to a cmake list.
        string(REPLACE " " ";" _dep_list "${_dep_text}")
        foreach(_dep IN LISTS _dep_list)
            if(_dep AND EXISTS "${_dep}")
                # Canonicalize before appending: two entries that share one
                # input (e.g. native_entry_robot1/robot2 both baking the same
                # multihost MODEL) reference it via different `../` spellings
                # relative to their own CMAKE_CURRENT_SOURCE_DIR. Ninja
                # aggregates every directory's CMAKE_CONFIGURE_DEPENDS into one
                # phony and rejects the same file declared as an output twice
                # ("defined as an output multiple times") unless the spellings
                # are identical. REALPATH collapses them to one canonical path.
                get_filename_component(_dep_real "${_dep}" REALPATH)
                set_property(DIRECTORY APPEND PROPERTY
                    CMAKE_CONFIGURE_DEPENDS "${_dep_real}")
            endif()
        endforeach()
    endif()

    set(${_NRX_OUT_VAR_GEN}     "${_gen_path}"       PARENT_SCOPE)
    set(${_NRX_OUT_VAR_LINKLIB} "${_link_libs_path}" PARENT_SCOPE)
endfunction()
