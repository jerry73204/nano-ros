# cmake/NanoRosVerbs.cmake — RFC-0048 (phase-287 W3): the two role verbs.
#
# `find_package(nano_ros)` pulls this in. It defines the two verbs a nano-ros
# package uses in place of a bare `add_executable`, matching the two roles a
# ROS 2 developer already distinguishes:
#
#   * nano_ros_add_executable(<name> <sources…>) — a STANDALONE ENTRY (own
#     `main` / self-bringup). Emits `add_executable` on native/FreeRTOS/NuttX/
#     ThreadX and `add_library`-into-Zephyr's-`app` on Zephyr; the platform
#     choice is hidden inside `nano_ros_entry`, so the call is identical
#     everywhere.
#   * nano_ros_add_node(<name> <sources…> CLASS <ns::Class>) — a WORKSPACE
#     COMPONENT (no own `main`; registered into a carrier ELF). Always a
#     component library.
#
# Interface codegen is authoritative through `nros_find_interfaces`, which reads
# the package's `package.xml` `<depend>` closure and shells `nros codegen
# resolve-deps` (the proven path every example uses; it resolves well-known ROS
# packages the CLI knows, so no in-tree bundle or sourced ROS install is needed).
# A leaf's `find_package(<msg_pkg> REQUIRED)` line satisfies the ament shape (via
# the compat find-stubs) and validates the dependency; the generation itself is
# driven here from `package.xml`, so C and C++ leaves stay byte-identical.

include_guard(GLOBAL)

# issue 0326 — `_nros_is_zephyr()` lives here.
include("${CMAKE_CURRENT_LIST_DIR}/NanoRosCodegenCore.cmake")
# `nros_read_package_xml_body()` — see phase-348 W1.
include("${CMAKE_CURRENT_LIST_DIR}/NanoRosPackageXml.cmake")

# `_nros_bootstrap` / `_nros_link` (root resolve + auto-link of the generated
# interface libs + platform — config internals since W8 retired the public
# spelling). Imported already by the config, but keep the include so the verbs
# are usable if a caller pulls this module directly.
include("${CMAKE_CURRENT_LIST_DIR}/NanoRosBootstrap.cmake")

# ---------------------------------------------------------------------------
# _nros_infer_lang(<out_var> <sources…>)
#   CPP if any source has a C++ extension, else C. Mirrors the inference
#   `nano_ros_entry` does so the verb and the entry agree on LANG.
# ---------------------------------------------------------------------------
function(_nros_infer_lang out_var)
    set(_lang c)
    foreach(_src ${ARGN})
        if(_src MATCHES "\\.(cpp|cxx|cc|C)$")
            set(_lang cpp)
        endif()
    endforeach()
    set(${out_var} "${_lang}" PARENT_SCOPE)
endfunction()

# ---------------------------------------------------------------------------
# _nros_generate_declared_interfaces(<lang>)
#   Run interface codegen for the invoking package's declared interface deps —
#   but only when its package.xml actually declares one. A no-dep leaf (a pure
#   `nros::init` demo, or an own-msg pkg that generates via
#   nano_ros_generate_interfaces) would otherwise trip a spurious
#   "no interface packages resolved" warning from nros_find_interfaces.
# ---------------------------------------------------------------------------
function(_nros_generate_declared_interfaces lang)
    set(_pkgxml "${CMAKE_CURRENT_SOURCE_DIR}/package.xml")
    if(NOT EXISTS "${_pkgxml}")
        return()
    endif()
    nros_read_package_xml_body("${_pkgxml}" _body)
    if(_body MATCHES "<(depend|build_depend|exec_depend|run_depend|build_export_depend)>")
        # phase-403 -- a leaf's own `nros-codegen.toml`, if it has one.
        #
        # A package that CONSUMES an interface cannot bound it any other way.
        # The .msg belongs to someone else, so there is no bound to add there,
        # and `CapacityResolver::discover` walks up from the codegen OUTPUT
        # directory under the CMake binary tree -- not from the consumer's
        # source -- so a config sitting beside package.xml is not found.
        #
        # Beside package.xml is the discoverable place: it is where a consumer
        # already declares WHICH interfaces it wants, so it is where it says
        # how big they may be.
        set(_leaf_cfg)
        if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/nros-codegen.toml")
            set(_leaf_cfg CODEGEN_CONFIG "${CMAKE_CURRENT_SOURCE_DIR}/nros-codegen.toml")
        endif()
        nros_find_interfaces(LANGUAGE ${lang} SKIP_INSTALL ${_leaf_cfg})
    endif()
endfunction()

# ---------------------------------------------------------------------------
# nano_ros_add_executable(<name> <sources…> [DEPLOY <target>…] [BOARD <board>]
#     [BRINGUP <dir>] [LAUNCH <launch.xml>] [TYPED])
#
# Standalone entry. DEPLOY/BOARD default to the package.xml `<export>` tuple in
# W4; until then DEPLOY defaults to `native` and an embedded board is passed
# explicitly (or comes from a prior `nano_ros_use_board`).
#
# 287-W6 workspace slice 3 — LAUNCH/TYPED pass through to
# `nano_ros_entry` so a workspace Entry pkg (multi-node carrier generated from
# a bringup launch manifest) can use the ament verb instead of the raw
# `nano_ros_entry(...)` call.
# ---------------------------------------------------------------------------
function(nano_ros_add_executable name)
    # phase-405 W1 — LOCATOR and ARGS dropped in lockstep with
    # `nano_ros_entry`. A verb that still accepted them would forward keywords
    # the callee no longer parses, which lands them in SOURCES.
    cmake_parse_arguments(_NRE "TYPED" "BOARD;BRINGUP;LAUNCH;MODEL;HOST;LANG" "DEPLOY;SOURCES" ${ARGN})
    set(_srcs ${_NRE_SOURCES} ${_NRE_UNPARSED_ARGUMENTS})
    if(NOT _srcs AND NOT _NRE_LAUNCH AND NOT _NRE_BRINGUP AND NOT _NRE_MODEL)
        message(FATAL_ERROR
            "nano_ros_add_executable(${name}): no sources given "
            "(a BRINGUP/LAUNCH-generated entry may omit sources; anything "
            "else needs at least one).")
    endif()
    _nros_infer_lang(_lang ${_srcs})

    # DEPLOY/BOARD default to the package.xml <export><nano_ros> tuple that
    # find_package(nano_ros) parsed into NROS_DEPLOY / NROS_BOARD (RFC-0048 §4);
    # an explicit keyword still wins.
    if(NOT _NRE_DEPLOY)
        if(NROS_DEPLOY)
            set(_NRE_DEPLOY "${NROS_DEPLOY}")
        else()
            set(_NRE_DEPLOY native)
        endif()
    endif()
    if(NOT _NRE_BOARD AND NROS_BOARD)
        set(_NRE_BOARD "${NROS_BOARD}")
    endif()

    # issue 0755 — the entry knows WHICH deploy this build is (explicit
    # `DEPLOY`, else the package.xml tuple). Board facts resolve further down
    # the call chain, and without this the resolver could only ask by BOARD —
    # which a `system.toml` carrying several deploys for one board answers with
    # an ambiguity refusal, and this lane's soft error handling turns that into
    # a silent skip. cmake functions are dynamically scoped, so setting it here
    # reaches the nested resolution without threading a parameter through every
    # frame between.
    set(NROS_DEPLOY "${_NRE_DEPLOY}")

    # Generate the package's declared interface closure in the leaf's language
    # (no-op when package.xml declares no interface deps).
    _nros_generate_declared_interfaces(${_lang})

    set(_board_arg "")
    if(_NRE_BOARD)
        set(_board_arg BOARD ${_NRE_BOARD})
    endif()
    # Entry-carrier knobs (LAUNCH-generated multi-node entries + typed
    # components + connection overrides) forward verbatim.
    set(_entry_extra "")
    # phase-330 W7 — BRINGUP is the INPUT-addressed spelling and the one a
    # human writes: it names the package the user authors (system.toml +
    # launch/ + the launch file's contract sidecar), and `nano_ros_entry`
    # resolves the SystemModel BUILD ARTIFACT from it via `nros model-path`.
    # It was parsed by `nano_ros_entry` and NOT by this verb, so every caller
    # reaching the entry through the ament verb had no way to spell it and
    # was pushed back onto MODEL — which names a resolved artifact directly
    # and is exactly what phase-405 W4 retired for letting an entry build
    # from a stale model.
    if(_NRE_BRINGUP)
        list(APPEND _entry_extra BRINGUP ${_NRE_BRINGUP})
    endif()
    if(_NRE_LAUNCH)
        list(APPEND _entry_extra LAUNCH ${_NRE_LAUNCH})
    endif()
    # R1 / W4.2 — the canonical resolved-model input (RFC-0052). Deprecated
    # expert override; prefer BRINGUP.
    if(_NRE_MODEL)
        list(APPEND _entry_extra MODEL ${_NRE_MODEL})
    endif()
    if(_NRE_TYPED)
        list(APPEND _entry_extra TYPED)
    endif()
    # phase-326 (issue 0364) — HOST removed with `<node machine=>` (ROS 1
    # syntax); kept PARSED so an old caller fails loud with guidance instead
    # of the keyword silently joining SOURCES via UNPARSED_ARGUMENTS.
    if(_NRE_HOST)
        message(FATAL_ERROR
            "nano_ros_add_executable(${name}): HOST was removed (phase-326 / "
            "issue 0364) — multi-host partitions at RESOLVE time now. Point "
            "MODEL at the per-host SystemModel instead (resolved with "
            "`host:=${_NRE_HOST}`, e.g. "
            "MODEL config/multihost_${_NRE_HOST}_model.yaml).")
    endif()
    # Language: explicit LANG wins (the only way a LAUNCH-only entry — no
    # sources to infer from — can select C; nano_ros_entry's sourceless
    # default is cpp). With sources, infer; without either, let nano_ros_entry
    # default.
    set(_lang_arg "")
    if(_NRE_LANG)
        set(_lang_arg LANG ${_NRE_LANG})
    elseif(_srcs)
        set(_lang_arg LANG ${_lang})
    endif()
    nano_ros_entry(
        NAME ${name}
        SOURCES ${_srcs}
        DEPLOY ${_NRE_DEPLOY}
        ${_lang_arg}
        ${_board_arg}
        ${_entry_extra})

    _nros_link(${name})
endfunction()

# ---------------------------------------------------------------------------
# nano_ros_add_node(<name> <sources…> CLASS <ns::Class> [LANGUAGE C|CPP]
#                   [DEPLOY <target>…])
#
# Workspace component. Registers a component library via `nano_ros_node_register`;
# the carrier entry ELF is assembled by the workspace root / `nros plan`.
#
# LANGUAGE is optional and normally inferred from the source extensions. State
# it when SOURCES arrives through a variable or a generator expression: cmake
# expands those before inferring, `nros`'s static CMakeLists scanner cannot, and
# the two readers then disagree about which ABI seam the component has
# (issue 1062).
# ---------------------------------------------------------------------------
function(nano_ros_add_node name)
    # RFC-0057 (phase-305 W4.1): this fused spelling remains supported as the
    # compat verb; the reference shape is nano_ros_auto_add_library +
    # nros_components_register_node. Opt-in nudge only — no default noise.
    if(NROS_WARN_LEGACY_VERBS)
        message(DEPRECATION
            "nano_ros_add_node(${name}): consider the RFC-0057 split shape "
            "(nano_ros_auto_add_library + nros_components_register_node).")
    endif()
    cmake_parse_arguments(_NRN "TYPED" "CLASS;HEADER;SHAPE;LANGUAGE" "SOURCES;DEPLOY;CALLBACK_GROUPS" ${ARGN})
    set(_srcs ${_NRN_SOURCES} ${_NRN_UNPARSED_ARGUMENTS})
    if(NOT _srcs)
        message(FATAL_ERROR "nano_ros_add_node(${name}): no sources given.")
    endif()
    if(NOT _NRN_CLASS)
        message(FATAL_ERROR
            "nano_ros_add_node(${name}): CLASS <ns::Class> required "
            "(a workspace component registers a class; use "
            "nano_ros_add_executable for a standalone entry with its own main).")
    endif()
    # DEPLOY defaults to the package.xml tuple; with neither, the component is
    # registered CARRIER-LESS (no DEPLOY forwarded) — a workspace member's
    # carrier is assembled by its Entry pkg / the workspace root, exactly like
    # the pre-verb `nano_ros_node_register` calls that omitted DEPLOY. (287-W6
    # slice 3: the earlier implicit `native` default forced every member onto
    # the per-node carrier path — fatal on FreeRTOS, whose carrier requires
    # TYPED, and a spurious extra exe on posix.)
    if(NOT _NRN_DEPLOY AND NROS_DEPLOY)
        set(_NRN_DEPLOY "${NROS_DEPLOY}")
    endif()
    # Explicit LANGUAGE wins over inference. Inference reads EXPANDED sources,
    # which is exactly what the static CMakeLists scanner in `nros` cannot do —
    # so a component whose SOURCES hides behind `${var}` states the language
    # here and both readers agree (issue 1062).
    if(_NRN_LANGUAGE)
        string(TOLOWER "${_NRN_LANGUAGE}" _lang)
        if(_lang STREQUAL "cxx")
            set(_lang cpp)
        endif()
        if(NOT _lang STREQUAL "c" AND NOT _lang STREQUAL "cpp")
            message(FATAL_ERROR
                "nano_ros_add_node(${name}): LANGUAGE '${_NRN_LANGUAGE}' rejected "
                "— this verb compiles C/C++ SOURCES into a component library, so "
                "LANGUAGE is C, CPP, or CXX. A Rust component registers through "
                "nano_ros_node_register(LANGUAGE RUST) against its Cargo.toml.")
        endif()
    else()
        _nros_infer_lang(_lang ${_srcs})
    endif()

    # Generate the package's declared interface closure (no-op when package.xml
    # declares none — a TYPED component publishes raw topics with no bindings).
    _nros_generate_declared_interfaces(${_lang})

    # TYPED (RFC-0043): a typed component carries the type name as a string, no
    # generated bindings — forward the flag to the register.
    set(_typed_arg "")
    if(_NRN_TYPED)
        set(_typed_arg TYPED)
    endif()
    # 287-W6 workspace slice 2 — pass-through for the register's remaining
    # per-component knobs (rclcpp-shape components, custom class headers,
    # RFC-0047 callback-group declarations) so every node member can use the
    # ament verb, not just the plain TYPED ones.
    set(_extra_args "")
    if(_NRN_HEADER)
        list(APPEND _extra_args HEADER ${_NRN_HEADER})
    endif()
    if(_NRN_SHAPE)
        list(APPEND _extra_args SHAPE ${_NRN_SHAPE})
    endif()
    if(_NRN_CALLBACK_GROUPS)
        list(APPEND _extra_args CALLBACK_GROUPS ${_NRN_CALLBACK_GROUPS})
    endif()
    set(_deploy_arg "")
    if(_NRN_DEPLOY)
        set(_deploy_arg DEPLOY ${_NRN_DEPLOY})
    endif()
    nano_ros_node_register(
        NAME ${name}
        CLASS ${_NRN_CLASS}
        LANGUAGE ${_lang}
        SOURCES ${_srcs}
        ${_deploy_arg}
        ${_typed_arg}
        ${_extra_args})
endfunction()

# ---------------------------------------------------------------------------
# nano_ros_auto_add_library(<name> [STATIC] <sources…>)      (RFC-0057 D1/D3)
#
# The `ament_auto_add_library` analog: creates the component library AND
# wires everything a nano-ros component TU needs — the declared interface
# closure (from package.xml), the generated interface libs (per-package FFI
# crates since phase-306; consumers never hand-pick archives), the nros
# runtime lib, the per-build config-header ordering, and the Zephyr compile
# context. Registration is a separate step (`nros_components_register_node`),
# exactly like ament: sources belong to the library, identity to the register.
# ---------------------------------------------------------------------------
function(nano_ros_auto_add_library name)
    set(_srcs ${ARGN})
    list(REMOVE_ITEM _srcs STATIC SHARED) # STATIC accepted for ament parity; SHARED tolerated, built STATIC
    if(NOT _srcs)
        message(FATAL_ERROR "nano_ros_auto_add_library(${name}): no sources given.")
    endif()
    _nros_infer_lang(_lang ${_srcs})
    _nros_generate_declared_interfaces(${_lang})

    add_library(${name} STATIC ${_srcs})
    string(REGEX REPLACE "[^A-Za-z0-9_]" "_" _pkg_sym "${PROJECT_NAME}")
    if(_lang STREQUAL "C")
        set_target_properties(${name} PROPERTIES LINKER_LANGUAGE C)
    endif()
    # Zephyr compile context (see the fused register path for rationale).
    if(TARGET zephyr_interface)
        target_link_libraries(${name} PRIVATE zephyr_interface)
    endif()
    _nros_node_register_config_header_deps(${name})
    # Runtime lib: C++ always links the umbrella; a C component's choice
    # depends on TYPED (declarative C keeps NanoRos), which is a
    # register-time fact — nros_components_register_node adds it.
    if(NOT _lang STREQUAL "C" AND TARGET NanoRos::NanoRosCpp)
        target_link_libraries(${name} PUBLIC NanoRos::NanoRosCpp)
    endif()
    target_include_directories(${name} PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
        "${CMAKE_CURRENT_SOURCE_DIR}/src")
    target_compile_definitions(${name} PRIVATE NROS_PKG_NAME=${_pkg_sym})
    set_target_properties(${name} PROPERTIES
        NROS_COMPONENT_PKG_SYM "${_pkg_sym}"
        NROS_COMPONENT_LANG "${_lang}")
    # Generated interface libs (220.G.2 mechanics; per-package FFI crates
    # since phase-306, so any subset links cleanly; zephyr gets headers via
    # the app include mirror instead — non-target lib names there).
    # issue 0326 — a SEVENTH instance of the class, not listed in that issue:
    # same inverted guard as NanoRosNodeRegister.cmake:418. With
    # NANO_ROS_PLATFORM unset this read TRUE on Zephyr and linked the
    # non-target `-l<pkg>__nano_ros_cpp` names the comment above says Zephyr
    # must NOT link. One helper call serves both sites in this function.
    _nros_is_zephyr(_nrv_is_zephyr)
    if(NOT _nrv_is_zephyr)
        get_directory_property(_nros_iface_libs NROS_GENERATED_INTERFACE_LIBS)
        if(_nros_iface_libs)
            list(REMOVE_DUPLICATES _nros_iface_libs)
            target_link_libraries(${name} PUBLIC ${_nros_iface_libs})
        endif()
    endif()
    # Zephyr detection: NANO_ROS_PLATFORM is NOT set in a `find_package(Zephyr)`
    # app build (issue 0282) — the previous `STREQUAL "zephyr"` guard silently
    # never fired there, so the per-build config headers got no ordering edge
    # and a component TU could compile before the nros-cpp cargo build emitted
    # `nros_cpp_config_generated.h` (reaching the in-tree `#error` stub). The
    # global `app` target + ZEPHYR_BASE is the reliable signal.
    # issue 0326 — this site carried the second idiom introduced by #282; it is
    # now the shared helper (resolved once above) so there is one spelling.
    if(_nrv_is_zephyr)
        target_include_directories(${name} PRIVATE
            $<TARGET_PROPERTY:app,INCLUDE_DIRECTORIES>)
        # APPEND, not set: the Zephyr interface-codegen module also stamps
        # OBJECT_DEPENDS on these same sources (the generated msg headers), and
        # a plain `set_source_files_properties` from either side silently
        # CLOBBERS the other — the surviving set decides which race is closed.
        # Losing the nros-cpp entry lets a C TU compile before
        # `nros_cpp_config_generated.h` exists and fall through to the in-tree
        # `#error` stub (issue 0088/0090 class).
        # Target-level ordering too: the file-level OBJECT_DEPENDS below closes
        # the compile race, this makes the dependency explicit for consumers
        # that only look at target edges.
        foreach(_cargo_tgt nros_cpp_cargo_build nros_c_cargo_build
                           cargo-build_nros_cpp cargo-build_nros_c)
            if(TARGET ${_cargo_tgt})
                add_dependencies(${name} ${_cargo_tgt})
            endif()
        endforeach()
        _nros_generated_header_dir(_nrv_gen_dir)
        set_property(SOURCE ${_srcs} APPEND PROPERTY OBJECT_DEPENDS
            "${_nrv_gen_dir}/nros-cpp-generated/nros/nros_cpp_config_generated.h"
            "${_nrv_gen_dir}/nros-c-generated/nros/nros_config_generated.h")
    endif()
endfunction()

# ---------------------------------------------------------------------------
# nros_components_register_node(<target>                     (RFC-0057 D1)
#     PLUGIN <ns::Class> EXECUTABLE <node_name>
#     [HEADER <hdr>] [SHAPE rclcpp|configure] [TYPED]
#     [DEPLOY <t>…] [CALLBACK_GROUPS <g>…])
#
# Keyword-parity analog of `rclcpp_components_register_node`: PLUGIN is the
# component class (any qualified name — L.4 retired per RFC-0057), EXECUTABLE
# the node/exec identity. Operates on an EXISTING library target (created by
# `nano_ros_auto_add_library` or plain `add_library`). SHAPE defaults to
# `rclcpp` — the construct-with-handle IS-A-node shape upstream components
# have; the legacy `configure(Node&)` shape is the explicit opt-in.
# ---------------------------------------------------------------------------
function(nros_components_register_node target)
    cmake_parse_arguments(_NCR "TYPED" "PLUGIN;EXECUTABLE;HEADER;SHAPE" "DEPLOY;CALLBACK_GROUPS;ENTITIES" ${ARGN})
    foreach(_req PLUGIN EXECUTABLE)
        if(NOT _NCR_${_req})
            message(FATAL_ERROR
                "nros_components_register_node(${target}): ${_req} required")
        endif()
    endforeach()
    if(NOT _NCR_SHAPE)
        set(_NCR_SHAPE rclcpp)
    endif()
    # Same DEPLOY defaulting as nano_ros_add_node: the workspace/platform
    # harness communicates the deploy tuple via NROS_DEPLOY when the call
    # omits it (287-W6 slice 3 semantics — absent both, the component is
    # carrier-less and the Entry pkg / workspace root selects).
    if(NOT _NCR_DEPLOY AND NROS_DEPLOY)
        set(_NCR_DEPLOY "${NROS_DEPLOY}")
    endif()
    # A C component links its runtime by TYPED-ness (see auto_add_library).
    get_target_property(_ncr_lang ${target} NROS_COMPONENT_LANG)
    if(_ncr_lang STREQUAL "C")
        # issue 0425 — prefer the C++ umbrella whenever it exists, TYPED or not:
        # it BUNDLES nros-c, so a C component's `nros_*` calls resolve from it
        # and the binary links ONE Rust staticlib. Keeping `NanoRos` for a
        # non-typed C node is what made a MIXED workspace link both archives and
        # die on ~96 duplicate C-ABI symbols. A pure-C workspace instantiates no
        # `NanoRosCpp` target and is unaffected.
        if(TARGET NanoRos::NanoRosCpp)
            target_link_libraries(${target} PUBLIC NanoRos::NanoRosCpp)
        elseif(TARGET NanoRos::NanoRos)
            target_link_libraries(${target} PUBLIC NanoRos::NanoRos)
        endif()
    endif()
    set(_extra "")
    if(_NCR_HEADER)
        list(APPEND _extra HEADER ${_NCR_HEADER})
    endif()
    if(_NCR_TYPED)
        list(APPEND _extra TYPED)
    endif()
    if(_NCR_DEPLOY)
        list(APPEND _extra DEPLOY ${_NCR_DEPLOY})
    endif()
    if(_NCR_CALLBACK_GROUPS)
        list(APPEND _extra CALLBACK_GROUPS ${_NCR_CALLBACK_GROUPS})
    endif()
    # phase-403 W9 (issue 0965) — forward the ENTITY declaration.
    #
    # `DEFINED` and not truthiness, unlike every sibling above. A `list(APPEND)`
    # gated on the VALUE cannot express "the caller declared something", only
    # "the caller declared something cmake reads as true" — and cmake reads
    # `0`, `OFF`, `NO`, `N`, `IGNORE`, `NOTFOUND` and anything `*-NOTFOUND` as
    # false. None of those is a legal entity spec today, so the difference is
    # currently invisible; it stops being invisible the first time the grammar
    # grows a spelling that collides, and what it would cost then is exactly
    # the under-report this wave exists to make impossible. Forwarding the
    # keyword-missing case too is deliberate: `nano_ros_node_register` raises
    # the error, so a bare `ENTITIES` fails identically through either spelling.
    if(DEFINED _NCR_ENTITIES OR "ENTITIES" IN_LIST _NCR_KEYWORDS_MISSING_VALUES)
        list(APPEND _extra ENTITIES ${_NCR_ENTITIES})
    endif()
    nano_ros_node_register(
        NAME ${_NCR_EXECUTABLE}
        CLASS ${_NCR_PLUGIN}
        SHAPE ${_NCR_SHAPE}
        EXISTING_TARGET ${target}
        ${_extra})
endfunction()

# ---------------------------------------------------------------------------
# nano_ros_generate_interfaces(<name> <files…> [DEPENDENCIES <pkgs…>])
#
# For a package that DEFINES its own .msg/.srv/.action — the `rosidl_generate_
# interfaces` analogue (RFC-0048 §5). Thin alias over the low-level generator so
# the ament shape reads uniformly; defaults to C++ bindings like rosidl does.
# ---------------------------------------------------------------------------
function(nano_ros_generate_interfaces name)
    cmake_parse_arguments(_NRG "" "LANGUAGE;ROS_EDITION" "DEPENDENCIES" ${ARGN})
    if(NOT _NRG_LANGUAGE)
        set(_NRG_LANGUAGE CPP)
    endif()
    # phase-304 W2b (RFC-0056) — forward an explicit ROS_EDITION; when omitted,
    # nros_generate_interfaces inherits the workspace `NANO_ROS_ROS_EDITION`.
    if(_NRG_ROS_EDITION)
        set(_nrg_edition ROS_EDITION ${_NRG_ROS_EDITION})
    endif()
    nros_generate_interfaces(${name}
        ${_NRG_UNPARSED_ARGUMENTS}
        DEPENDENCIES ${_NRG_DEPENDENCIES}
        LANGUAGE ${_NRG_LANGUAGE}
        ${_nrg_edition}
        SKIP_INSTALL)
endfunction()
