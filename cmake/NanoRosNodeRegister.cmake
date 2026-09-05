# cmake/NanoRosNodeRegister.cmake — Phase 212.L.9 / 212.N.6
#
# C++ cmake fn surface for the three Phase 212.L pkg shapes:
#
#   * `nano_ros_node_register(NAME <name> CLASS <UserClass>
#       [LANGUAGE C|CPP|RUST] SOURCES <files...> DEPLOY <target1> [<target2> ...])`
#       — declares a Component pkg entity. Compiles SOURCES into a
#         STATIC `<pkg>_<name>_component` lib linked to the C or C++
#         nano-ros target. Rust packages import `Cargo.toml` through
#         Corrosion and expose the same component target name for entry
#         link glue. CLASS is any namespace-qualified name (RFC-0057; the
#         L.4 prefix rule is retired — pkg is explicit metadata).
#
#     `ENTITIES` was phase-403 W9's way of saying WHICH entities a component's
#     constructor creates. It is RETIRED (phase-412) and now fails loud.
#
#     The reasoning that put it here still holds and is worth keeping: RFC-0043
#     /0044 components wire themselves in CONSTRUCTORS, at runtime, so anything
#     the macros emit is a link-section fact that exists only after linking,
#     while `NROS_EXECUTOR_MAX_CBS` and the arena are `const` sizes compiled
#     INTO nros-node before a component TU is compiled. Emitted evidence can
#     VERIFY a count; it can never SUPPLY one. Somebody has to STATE it.
#
#     What changed is WHO states it and WHERE. A list in a component's own
#     CMakeLists is hand-maintained beside the code with nothing comparing the
#     two, and that is not a hypothetical: the safety island's mrm_handler
#     declared six subscriptions for a node that creates seven, every pool
#     derived from the list was short by one, and the eleventh subscription
#     failed at boot with a transport error that named nothing. It cost days on
#     real silicon.
#
#     The replacement is a CONTRACT SIDECAR beside the launch file --
#     `<stem>.contract.yaml` next to `<stem>.launch.xml` in the bringup package
#     -- which the resolver folds into the SystemModel. One file per system
#     rather than one per component, sitting next to the launch file that says
#     which nodes run, and `nros ws entity-inventory --model` sizes the pools
#     from it. `nano_ros_entry` passes the model automatically; nothing here
#     needs an argument.
#
#   * `nano_ros_entry(NAME <name> SOURCES <files...> [BOARD <board>]
#       DEPLOY <target1> [<target2> ...])`
#       — declares an Entry pkg entity. Renamed from
#         `nano_ros_application` per Phase 212.L.9 / 212.N.6. Defined
#         in `NanoRosEntry.cmake` (auto-included below); see that
#         module for the body + the BOARD-arg semantics.
#
#   (The `nano_ros_application` / `nano_ros_component_register` deprecation
#   shims were retired in 287-W8; `nano_ros_deploy` post-287 — the per-package
#   deploy/rmw tuple lives in package.xml `<export><nano_ros …/>`.)
#
# Side effect: every fn appends to GLOBAL props and rewrites
# `${CMAKE_BINARY_DIR}/nros-metadata.json` so `nros codegen-system`
# can consume it at configure time. No sidecar TOML for C++ pkgs.
#
# The function is deliberately declarative/glue-only; entry generation
# lives in `NanoRosEntry.cmake`.

# issue 0326 — `_nros_is_zephyr()` lives here.
include("${CMAKE_CURRENT_LIST_DIR}/NanoRosCodegenCore.cmake")

# issue 0342-adjacent — the SPLIT spelling's other half.
#
# `nros_components_register_node` (below) attaches metadata to a library the
# caller created with `nano_ros_auto_add_library`, which lives in
# NanoRosVerbs.cmake. A consumer reaching this module through
# `find_package(nano_ros)` gets both; one that includes it DIRECTLY — the
# build-stage fixtures do, deliberately (issue 0041) — used to get only half the
# spelling and failed at configure time with `Unknown CMake command
# "nano_ros_auto_add_library"`. The 305-W2 sweep migrated 80 files to the split
# verbs and this include did not follow.
#
# `NanoRosVerbs.cmake` is `include_guard(GLOBAL)`, so this is idempotent and
# costs nothing on the find_package path.
include("${CMAKE_CURRENT_LIST_DIR}/NanoRosVerbs.cmake")
# phase-403 step 2 -- `nros_entity_inventory_metadata_file()`, so the declared
# QoS header below reads the same `nros-metadata.json` the knob lane does.
# Included at FILE scope, never from inside a function: with
# `include_guard(GLOBAL)` a file-scope `set()` that lands in a function frame
# is gone when the frame pops and never comes back.
include("${CMAKE_CURRENT_LIST_DIR}/NanoRosEntityInventory.cmake")

# issue 0946 — `_nros_resolve_entry_locator`, the ONE producer of the connect
# locator. The three RTOS typed-entry carriers below used to each carry their
# own per-platform default, independently of the ladder in NanoRosEntry.cmake.
include("${CMAKE_CURRENT_LIST_DIR}/NanoRosEntryLocator.cmake")

if(DEFINED _NROS_NODE_REGISTER_INCLUDED)
    return()
endif()
set(_NROS_NODE_REGISTER_INCLUDED TRUE)

# Capture this module's directory at include time. `CMAKE_CURRENT_LIST_DIR`
# is dynamic — inside a function body it resolves to the *caller's* list
# dir, not this file's — so the Phase 238 carrier `configure_file` must use
# this captured path to find `templates/nuttx_entry_main.cpp.in`.
# CACHE INTERNAL (like NanoRosEntry's `_NROS_ENTRY_DIR`): the workspace path
# includes this module inside a FUNCTION scope (`_nros_import_once`), so a
# normal variable dies with that frame and a member pkg's embedded-carrier
# `configure_file` resolved `/templates/…` (287-W6: every freertos workspace
# member failed "File /templates/freertos_entry_main_c_typed.cpp.in does not
# exist"; posix never touches the templates, which hid it).
set(_NROS_NODE_REGISTER_DIR "${CMAKE_CURRENT_LIST_DIR}" CACHE INTERNAL
    "nano_ros_node_register module dir")

# Issue 0088 — order any target whose TUs `#include <nros/nros_config_generated.h>`
# (or the C++ variant) AFTER the per-build header generators. The headers are
# produced by the nros-c / nros-cpp corrosion builds and mirrored into the in-tree
# `include/nros/` dir by the `nros_{c,cpp}_config_header` custom targets (OUTPUT
# edges, not POST_BUILD side effects). Depending only on `cargo-build_nros_{c,cpp}`
# is NOT enough: that orders against the cargo primary output, so a consumer TU can
# still compile before the mirror copy runs and pick up the in-tree stub header.
#
# The wiring is DEFERRED to end-of-configure: under `add_subdirectory(<repo-root>)`
# (Zephyr / examples) the consumer's `nano_ros_node_register()` often runs BEFORE
# the nros-c / nros-cpp subdirs are added, so the generator targets don't exist yet
# and an immediate `add_dependencies` would silently no-op (the `if(TARGET)` guard
# skips) — leaving the race in place (pass/fail purely by build order). A deferred
# call (`cmake_language(DEFER)` at the top-level dir) runs after every subdir is
# processed, when all targets exist, so the edge is always applied.
# ---------------------------------------------------------------------------
# issue 1033 — compose the entity inventory on the NON-ENTRY path.
#
# `nros_derive_entity_inventory_knobs` had exactly one caller, inside
# `nano_ros_entry()`. A standalone image — every `examples/zephyr/**` leaf —
# registers a node and never calls that verb, so its `ENTITIES` declaration
# reached `nros-metadata.json` and was never composed: the inventory read
# `refused / "no entity inventory composed yet"` and every derivable knob fell
# to its crate default. The declaration was captured and thrown away.
#
# DEFERRED to the TOP-LEVEL directory, for the reason the support-library flush
# documents one file over: the composer must run after EVERY registration, and
# deferring to the current directory fires at the end of whichever package
# registered first. On Zephyr the reader (`nros_resolve_knobs()`, inside
# `find_package(Zephyr)`) runs BEFORE the app's CMakeLists body, so there is no
# ordering that lets this configure both compose and read its own answer —
# which is why `nros_reconfigure_on_change` is not optional here. It is the same
# machinery issue 0991 built for exactly this shape, and the same reason: a
# clean build dir would otherwise size the image from the "no inventory yet"
# placeholder.
#
# The deferred call takes NO ARGUMENTS. `cmake_language(DEFER … CALL fn(arg))`
# delivers them EMPTY, so the CLI path travels as a GLOBAL property instead.
# ---------------------------------------------------------------------------
function(_nros_node_register_schedule_inventory)
    get_property(_scheduled GLOBAL PROPERTY NROS_ENTITY_INVENTORY_SCHEDULED)
    if(_scheduled)
        return()
    endif()
    set_property(GLOBAL PROPERTY NROS_ENTITY_INVENTORY_SCHEDULED TRUE)
    cmake_language(DEFER DIRECTORY "${CMAKE_SOURCE_DIR}"
        CALL _nros_node_register_compose_inventory)
endfunction()

function(_nros_node_register_compose_inventory)
    get_property(_composed GLOBAL PROPERTY NROS_ENTITY_INVENTORY_COMPOSED)
    if(_composed)
        # An entry already composed, after its own registrations. Re-composing
        # would read the same metadata and write the same bytes, so it is not
        # WRONG — it is a CLI invocation for nothing, and a second writer of one
        # artifact is the shape this repo keeps paying for.
        return()
    endif()

    get_property(_nros_bin GLOBAL PROPERTY NROS_ENTITY_INVENTORY_CLI)
    if(NOT _nros_bin)
        return()
    endif()

    include("${_NROS_NODE_REGISTER_DIR}/NanoRosEntityInventory.cmake")
    include("${_NROS_NODE_REGISTER_DIR}/NanoRosReconfigure.cmake")
    nros_entity_inventory_knobs_file(_knobs_path)
    nros_reconfigure_snapshot("${_knobs_path}" _knobs_before)
    nros_derive_entity_inventory_knobs(CLI "${_nros_bin}")
    nros_reconfigure_on_change("${_knobs_path}" "${_knobs_before}"
        LABEL "this image's entity inventory (standalone)")
endfunction()

function(_nros_node_register_apply_config_header_deps _tgt)
    if(NOT TARGET ${_tgt})
        return()
    endif()
    # Two header-generation flavours, depending on the consuming build:
    #  * Native / examples (add_subdirectory(<repo-root>) → Corrosion): the
    #    `cargo-build_nros_{c,cpp}` cargo targets + the `nros_{c,cpp}_config_header`
    #    OUTPUT-mirror targets (packages/core/nros-{c,cpp}/CMakeLists.txt).
    #  * Zephyr (`zephyr/CMakeLists.txt` → `nros_cargo_build`): the
    #    `nros_{c,cpp}_cargo_build` targets, which emit the per-build header into
    #    the cargo target dir's `nros-{c,cpp}-generated`. `app` already
    #    depends on these, but a `nano_ros_node_register` component lib is a
    #    SEPARATE target and otherwise races them → in-tree stub (issue 0088).
    # Guarded by `if(TARGET)`, so each flavour contributes only where it exists.
    foreach(_dep cargo-build_nros_cpp cargo-build_nros_c
                 nros_cpp_config_header nros_c_config_header
                 nros_cpp_cargo_build nros_c_cargo_build)
        if(TARGET ${_dep})
            add_dependencies(${_tgt} ${_dep})
        endif()
    endforeach()
endfunction()

function(_nros_node_register_config_header_deps _tgt)
    cmake_language(DEFER DIRECTORY "${CMAKE_SOURCE_DIR}"
        CALL _nros_node_register_apply_config_header_deps "${_tgt}")
endfunction()

define_property(GLOBAL PROPERTY NROS_COMPONENTS_JSON
    BRIEF_DOCS "Accumulated component JSON fragments"
    FULL_DOCS  "Phase 212.L.9 — appended by nano_ros_node_register().")
define_property(GLOBAL PROPERTY NROS_APPLICATIONS_JSON
    BRIEF_DOCS "Accumulated application JSON fragments"
    FULL_DOCS  "Phase 212.L.9 / 212.N.6 — appended by nano_ros_entry().")
set_property(GLOBAL PROPERTY NROS_COMPONENTS_JSON "")
set_property(GLOBAL PROPERTY NROS_APPLICATIONS_JSON "")

# Emit the JSON file. Idempotent — called after every fn so the file
# is always current. Keep small: writes the whole doc each time.
function(_nros_metadata_emit)
    get_property(_comps   GLOBAL PROPERTY NROS_COMPONENTS_JSON)
    get_property(_apps    GLOBAL PROPERTY NROS_APPLICATIONS_JSON)
    set(_doc "{\n")
    string(APPEND _doc "  \"components\": [${_comps}\n  ],\n")
    string(APPEND _doc "  \"applications\": [${_apps}\n  ]\n")
    string(APPEND _doc "}\n")
    file(WRITE "${CMAKE_BINARY_DIR}/nros-metadata.json" "${_doc}")
endfunction()

# Helper: render a string list as a JSON array body.
function(_nros_json_strlist out_var)
    set(_acc "")
    set(_first TRUE)
    foreach(_v IN LISTS ARGN)
        if(_first)
            set(_acc "\"${_v}\"")
            set(_first FALSE)
        else()
            set(_acc "${_acc}, \"${_v}\"")
        endif()
    endforeach()
    set(${out_var} "${_acc}" PARENT_SCOPE)
endfunction()

# Issue 1003 — derive the RTOS entry template's family holes from the family
# NAME, so a family is named once and everything else follows.
#
# NuttX, ThreadX and FreeRTOS all boot the same way (the board's `startup.c`
# owns `main` and dispatches to `nros_app_main`), so they share ONE entry
# template, `templates/rtos_entry_main{,_c}_typed.cpp.in`. They used to have
# three near-identical templates apiece; the copies differed only in these
# three tokens, which is exactly the duplication issue 1003 is about — a fix
# applied to one copy leaves the others wrong AND plausible.
#
# Every value below is COMPUTED from `_fam`. A table would be a second place
# to forget a family.
macro(_nros_rtos_entry_family _fam)
    set(NROS_ENTRY_RTOS_TAG "${_fam}")
    string(TOUPPER "${_fam}" NROS_ENTRY_RTOS_UPPER)
    # `nuttx` -> `Nuttx`, so `::nros::board::NuttxBoard`.
    string(SUBSTRING "${_fam}" 0 1 _nros_fam_head)
    string(SUBSTRING "${_fam}" 1 -1 _nros_fam_tail)
    string(TOUPPER "${_nros_fam_head}" _nros_fam_head)
    set(NROS_ENTRY_BOARD_CPP "::nros::board::${_nros_fam_head}${_nros_fam_tail}Board")
endmacro()

function(nano_ros_node_register)
    cmake_parse_arguments(_NRC "TYPED" "NAME;CLASS;LANGUAGE;HEADER;SHAPE;EXISTING_TARGET" "SOURCES;DEPLOY;CALLBACK_GROUPS;ENTITIES" ${ARGN})

    # Issue 1017 — the entry's session name is DERIVED here, once, from the
    # node's own name, and every entry shape below reads it.
    #
    # It used to be set separately in each of the FIVE shape branches, all
    # spelling the same assignment. Five copies of one fact is the shape that
    # produced issue 1003 one layer up: if a single branch drifted to a
    # literal, every image built through THAT shape would share a name — and
    # the templates, which only substitute `@NROS_ENTRY_NODE_NAME@`, would
    # still look correct. Hoisting removes the possibility rather than
    # checking for it.
    set(NROS_ENTRY_NODE_NAME "${_NRC_NAME}")
    # RFC-0057 (phase-305 W1.1) — EXISTING_TARGET mode: the component library
    # was created separately (`nano_ros_auto_add_library`); attach
    # registration (class define, metadata row, carrier glue) to it instead
    # of creating one. SOURCES is recovered from the target for the carrier
    # branches + metadata.
    if(_NRC_EXISTING_TARGET)
        if(NOT TARGET ${_NRC_EXISTING_TARGET})
            message(FATAL_ERROR
                "nano_ros_node_register: EXISTING_TARGET '${_NRC_EXISTING_TARGET}' is not a target")
        endif()
        if(NOT _NRC_SOURCES)
            get_target_property(_nrc_tgt_srcs ${_NRC_EXISTING_TARGET} SOURCES)
            if(_nrc_tgt_srcs)
                set(_NRC_SOURCES ${_nrc_tgt_srcs})
            endif()
        endif()
    endif()
    # Phase 248 C6b (#60 T5) — DEPLOY is OPTIONAL on a Node pkg. A reusable Node
    # pkg must NOT name a deploy target; the Entry pkg (`nano_ros_entry(... DEPLOY
    # …)`) + the bringup `system.toml` select RMW/platform/deploy. Embedded Node
    # pkgs that drive a single-node carrier (NuttX/ThreadX/Zephyr branches below)
    # still pass `DEPLOY <rtos>` — those branches gate on `<rtos> IN_LIST
    # _NRC_DEPLOY`, so absence is a no-op (the metadata `deploy` array is empty
    # and the Entry/system.toml is the selection point).
    set(_nrc_required NAME CLASS SOURCES)
    if(_NRC_EXISTING_TARGET)
        set(_nrc_required NAME CLASS) # sources live on the existing target
    endif()
    foreach(_req ${_nrc_required})
        if(NOT _NRC_${_req})
            message(FATAL_ERROR
                "nano_ros_node_register: ${_req} required")
        endif()
    endforeach()
    # Phase 242.4 (RFC-0044) — component SHAPE: `rclcpp` (IS-A-node, ctor-wired,
    # construct-with-handle — the typed entry placement-news it with the executor
    # handle *after* init and checks `ok()`) or `configure` (RFC-0043, the
    # default/back-compat: default-construct + `configure(node)`). Recorded in the
    # metadata JSON (the CLI `emit_typed` reads it onto `PlanNode.shape`) AND
    # surfaced to the carrier template as `NROS_ENTRY_SHAPE_RCLCPP` (0|1).
    if(_NRC_SHAPE)
        string(TOLOWER "${_NRC_SHAPE}" _nrc_shape)
    else()
        set(_nrc_shape "configure")
    endif()
    if(NOT (_nrc_shape STREQUAL "rclcpp" OR _nrc_shape STREQUAL "configure"))
        message(FATAL_ERROR
            "nano_ros_node_register: SHAPE '${_NRC_SHAPE}' rejected — "
            "expected rclcpp or configure")
    endif()
    if(_nrc_shape STREQUAL "rclcpp")
        set(_nrc_shape_rclcpp 1)
    else()
        set(_nrc_shape_rclcpp 0)
    endif()
    if(_NRC_LANGUAGE)
        string(TOUPPER "${_NRC_LANGUAGE}" _nrc_lang)
    else()
        # Back-compat: old C examples omitted LANGUAGE. If every source is a C
        # TU, record/link it as C; otherwise preserve the historical C++ default.
        set(_nrc_lang C)
        foreach(_src IN LISTS _NRC_SOURCES)
            get_filename_component(_ext "${_src}" EXT)
            string(TOLOWER "${_ext}" _ext_lc)
            if(NOT _ext_lc STREQUAL ".c")
                set(_nrc_lang CPP)
            endif()
        endforeach()
    endif()
    if(_nrc_lang STREQUAL "CXX")
        set(_nrc_lang CPP)
    endif()
    if(_nrc_lang STREQUAL "RUST" OR _nrc_lang STREQUAL "RS")
        set(_nrc_lang RUST)
    endif()
    if(NOT (_nrc_lang STREQUAL "C" OR _nrc_lang STREQUAL "CPP" OR _nrc_lang STREQUAL "RUST"))
        message(FATAL_ERROR
            "nano_ros_node_register: LANGUAGE '${_NRC_LANGUAGE}' rejected — "
            "expected C, CPP, or RUST")
    endif()
    string(TOLOWER "${_nrc_lang}" _nrc_lang_lc)
    # RFC-0057 D2 — the Phase 212.L.4 class-prefix rule is RETIRED: the pkg
    # is written explicitly into the metadata row below (`"pkg":
    # "${PROJECT_NAME}"`), so CLASS may carry any qualified C++ name
    # (verbatim upstream namespaces, e.g. `autoware::x::Node`). CLASS must
    # still be namespace-qualified — the entry codegen needs a real type name.
    string(FIND "${_NRC_CLASS}" "::" _idx)
    if(_idx EQUAL -1)
        message(FATAL_ERROR
            "nano_ros_node_register: CLASS '${_NRC_CLASS}' must be a "
            "namespace-qualified C++ name (`ns::Class`).")
    endif()

    # Phase 240.2b (RFC-0043) — the typed Entry emitter `#include`s the
    # component's class header to construct it. Accept an explicit HEADER or
    # derive it from CLASS by convention: `pkg::Sub::Class` → `pkg/Sub/Class.hpp`
    # (namespace `::` → `/`, `.hpp` suffix), which resolves against the component
    # lib's `include/` (added to its PUBLIC include dirs below). Recorded in the
    # metadata JSON so the codegen can populate `PlanNode.class_header`.
    if(_NRC_HEADER)
        set(_nrc_header "${_NRC_HEADER}")
    else()
        string(REPLACE "::" "/" _nrc_header "${_NRC_CLASS}")
        set(_nrc_header "${_nrc_header}.hpp")
    endif()

    # Issue 0088 (Zephyr path) — on Zephyr the per-build nros config headers are
    # BYPRODUCTS of an always-run `nros_{c,cpp}_cargo_build` target
    # (zephyr/cmake/nros_cargo_build.cmake), written into
    # the cargo target dir's `nros-{c,cpp}-generated`. The Zephyr component
    # lib is a plain `add_library` that inherits the generated + stub include dirs
    # (via `zephyr_include_directories`) but has no hard edge to the byproduct, so
    # its TUs can compile before the cargo build writes the real header and pick up
    # the in-tree stub (`#error "must be supplied per-build"`). `add_dependencies`
    # on the cargo target proved insufficient across the configure-order / typed-C
    # matrix; a file-level `OBJECT_DEPENDS` is a HARD Ninja edge — the TU won't
    # compile until the byproduct exists — independent of target name / order.
    # The C-variant header is a byproduct of EVERY nros Zephyr build (declared on
    # nros-c, or on nros-cpp when nros-c isn't built separately), so listing it is
    # always safe; depending on it also transitively orders after the same cargo
    # target that writes the C++ variant, covering C / typed-C / C++ consumers.
    # issue 0326 — NANO_ROS_PLATFORM is directory-scoped, so it is UNSET in an
    # add_subdirectory'd package and the bare STREQUAL silently took the
    # non-Zephyr path.
    _nros_is_zephyr(_nrc_is_zephyr)
    if(_nrc_is_zephyr AND _NRC_SOURCES)
        # phase-400 W5.a — ask where the headers ARE, do not restate where they
        # were. A file-level OBJECT_DEPENDS naming a path cargo no longer writes
        # is a ninja edge to a file that never appears.
        _nros_generated_header_dir(_nrc_gen_dir)
        set_source_files_properties(${_NRC_SOURCES} PROPERTIES OBJECT_DEPENDS
            "${_nrc_gen_dir}/nros-c-generated/nros/nros_config_generated.h;${_nrc_gen_dir}/nros-c-generated/nros/nros_generated.h")
    endif()

    set(_lib "${PROJECT_NAME}_${_NRC_NAME}_component")
    if(_NRC_EXISTING_TARGET)
        # RFC-0057: the register decorations the fused path applies inside
        # the add_library block were already applied by
        # nano_ros_auto_add_library; here we add the per-register class
        # define and keep the conventional `<pkg>_<exec>_component` name
        # alive as an INTERFACE wrapper (the CLI-emitted entry sidecar
        # links that name).
        target_compile_definitions(${_NRC_EXISTING_TARGET} PRIVATE
            "NROS_NODE_CLASS_NAME=\"${_NRC_CLASS}\"")
        if(NOT _lib STREQUAL "${_NRC_EXISTING_TARGET}" AND NOT TARGET ${_lib})
            add_library(${_lib} INTERFACE)
            target_link_libraries(${_lib} INTERFACE ${_NRC_EXISTING_TARGET})
            # phase-305 W2 fix: consumers that reach a component by its
            # CONVENTIONAL name must still find the facts they read off it —
            # notably the NuttX kernel link (cmake/board/nano-ros-board-nuttx-*),
            # which recompiles each C component's SOURCES for the ARM target
            # tagged with its own `-DNROS_PKG_NAME` (phase-263 C2b) and pulls the
            # include dirs. An INTERFACE wrapper carries none of that by default:
            # the symbols `__nros_c_component_<pkg>_{create,configure}` then never
            # get built for the target arch and the kernel link fails undefined.
            # Mirror the properties from the real library (INTERFACE targets may
            # carry SOURCES since CMake 3.19; they are metadata, never compiled).
            get_target_property(_et_srcs ${_NRC_EXISTING_TARGET} SOURCES)
            if(_et_srcs)
                set_property(TARGET ${_lib} PROPERTY SOURCES ${_et_srcs})
            endif()
            get_target_property(_et_pkg_sym ${_NRC_EXISTING_TARGET} NROS_COMPONENT_PKG_SYM)
            if(_et_pkg_sym)
                set_property(TARGET ${_lib} PROPERTY NROS_COMPONENT_PKG_SYM "${_et_pkg_sym}")
            endif()
            get_target_property(_et_lang ${_NRC_EXISTING_TARGET} NROS_COMPONENT_LANG)
            if(_et_lang)
                set_property(TARGET ${_lib} PROPERTY NROS_COMPONENT_LANG "${_et_lang}")
            endif()
            get_target_property(_et_incs ${_NRC_EXISTING_TARGET} INTERFACE_INCLUDE_DIRECTORIES)
            if(_et_incs)
                set_property(TARGET ${_lib} APPEND PROPERTY
                    INTERFACE_INCLUDE_DIRECTORIES ${_et_incs})
            endif()
            # Issue 0149 descent (NuttX): consumers read the component's
            # LINK_LIBRARIES to find its generated interface libs
            # (`<pkg>__nano_ros_{c,cpp}`) — their include dirs, codegen order
            # and (C lane) serdes SOURCES all flow from there. Those links live
            # on the REAL library, so re-expose them on the wrapper's own
            # LINK_LIBRARIES, otherwise the walk sees only the wrapper→lib edge
            # and the cross-compile dies on `std_msgs/std_msgs.h`.
            get_target_property(_et_links ${_NRC_EXISTING_TARGET} LINK_LIBRARIES)
            if(_et_links)
                foreach(_et_l IN LISTS _et_links)
                    if(_et_l MATCHES "__nano_ros_(c|cpp)$")
                        set_property(TARGET ${_lib} APPEND PROPERTY LINK_LIBRARIES ${_et_l})
                    endif()
                endforeach()
            endif()
        endif()
        set(_lib "${_NRC_EXISTING_TARGET}")
    endif()
    if(NOT TARGET ${_lib})
        # Phase 212.M.5.a.1 — package symbol used by C/C++ macros and
        # mirrored by Rust `nros::node!()`.
        string(REGEX REPLACE "[^A-Za-z0-9_]" "_" _pkg_sym "${PROJECT_NAME}")

        if(_nrc_lang STREQUAL "RUST")
            # Phase 241 W11 (Option D) — a Rust Node pkg is NO LONGER imported as its own
            # Corrosion staticlib. A per-node `lib<pkg>.a` bundles its full `nros` closure
            # (incl. `nros-rmw-cffi`'s `#[no_mangle]` C ABI + REGISTRY); linked beside the
            # umbrella it collided (`multiple definition`) once single-runtime dropped
            # `--allow-multiple-definition`, and split the stateful REGISTRY (issue the W1
            # un-gate closed). Instead the workspace's per-configure runtime umbrella
            # (`nros_ws_runtime`, synthesised by `nano_ros_workspace`) bundles this node as
            # a cargo **rlib** path-dep — one Rust staticlib for the whole binary.
            #
            # `${_lib}` stays as an EMPTY INTERFACE target so the CLI-emitted entry
            # auto-link sidecar (`target_link_libraries(<entry> PRIVATE
            # <pkg>_<exec>_component)`, Phase 219.J) resolves to a harmless no-op — the
            # node's `__nros_component_<pkg>_register` symbol arrives via the runtime
            # umbrella, which the entry already links through `NanoRos::NanoRosCpp`.
            if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/Cargo.toml")
                message(FATAL_ERROR
                    "nano_ros_node_register(LANGUAGE RUST): expected Cargo.toml "
                    "in ${CMAKE_CURRENT_SOURCE_DIR}")
            endif()
            add_library(${_lib} INTERFACE)
        else()
            add_library(${_lib} STATIC ${_NRC_SOURCES})
            if(_nrc_lang STREQUAL "C")
                set_target_properties(${_lib} PROPERTIES LINKER_LANGUAGE C)
            endif()
            # Phase 215.J / 242 — on Zephyr the component lib is a plain
            # add_library(STATIC), so unlike the `find_package(Zephyr)`-owned
            # `app` target it does NOT inherit Zephyr's compile context (the
            # C++ standard from CONFIG_STD_CPP17, the zephyr + autogen include
            # dirs, the CONFIG_* defines). Without it, C++ sources that compiled
            # in a monolithic Zephyr app (e.g. ASI's vendored autoware libs)
            # fail (default `-std` + missing zephyr headers). `zephyr_interface`
            # is the INTERFACE target carrying exactly that build context; link
            # it so the component sources compile identically to `app`.
            if(TARGET zephyr_interface)
                target_link_libraries(${_lib} PRIVATE zephyr_interface)
            endif()
            # Phase 242 — the per-build `<nros/nros_cpp_config_generated.h>` /
            # `<nros/nros_config_generated.h>` (storage sizes, etc.) are emitted
            # as byproducts of the nros-cpp / nros-c cargo builds into
            # the cargo target dir's `nros-{cpp,c}-generated` (prepended
            # to the include path by zephyr/CMakeLists.txt). `app` already
            # depends on those targets, but this component lib is a SEPARATE
            # add_library; without the same dependency its TUs can compile
            # before the headers exist (clean-build race) and pick up the
            # in-tree stub header, which #errors. Order it after the generators.
            # (Target names are `cargo-build_nros_{cpp,c}` — the corrosion cargo
            # targets; the pre-257 names `nros_{cpp,c}_cargo_build` never matched.
            # Issue 0088: also depend on the `nros_{c,cpp}_config_header` mirror
            # targets — the cargo dep alone races the POST_BUILD-era copy.)
            _nros_node_register_config_header_deps(${_lib})
            # Phase 257 (W0-A) — a TYPED C component (`NROS_C_COMPONENT`) calls the
            # `nros_cpp_*` seam (publisher/subscription/timer), which lives in the
            # C++ umbrella (nros-cpp), so it links NanoRosCpp like a C++ component —
            # NOT the C-only NanoRos. (A legacy declarative C node keeps NanoRos.)
            # The umbrella bundles nros-c's C ABI, so `nros_*` C calls still resolve,
            # and only ONE Rust staticlib is linked (no double `std`/REGISTRY).
            #
            # issue 0425 — the C-only branch is taken ONLY when no C++ umbrella
            # exists in this configure. A MIXED workspace (C node pkg + C++ node
            # pkg + C++ entry) otherwise links BOTH archives into one
            # executable: the C node drags `libnros_c.a`, the C++ side drags
            # `libnros_cpp.a`, and since Phase 241.D3-rev the latter BUNDLES
            # nros-c — ~96 duplicate `nros_log_*` / `nros_lifecycle_*`
            # definitions, which is the "NEVER both" rule stated in
            # NanoRosEntry.cmake being violated through a DEPENDENCY rather than
            # by the entry's own LANG.
            #
            # Preferring the C++ umbrella when it is present is exactly the
            # reasoning the TYPED branch above already relies on — it bundles
            # nros-c, so a C node's `nros_*` calls still resolve and the binary
            # links ONE Rust staticlib. A pure-C workspace instantiates no
            # `NanoRosCpp` target and so keeps `NanoRos`, unchanged.
            if(TARGET NanoRos::NanoRosCpp)
                target_link_libraries(${_lib} PUBLIC NanoRos::NanoRosCpp)
            elseif(TARGET NanoRos::NanoRos)
                target_link_libraries(${_lib} PUBLIC NanoRos::NanoRos)
            endif()
            target_include_directories(${_lib} PUBLIC
                "${CMAKE_CURRENT_SOURCE_DIR}/include"
                "${CMAKE_CURRENT_SOURCE_DIR}/src")
            target_compile_definitions(${_lib} PRIVATE
                NROS_PKG_NAME=${_pkg_sym}
                "NROS_NODE_CLASS_NAME=\"${_NRC_CLASS}\"")
            # phase-263 C2b — record the pkg sym + language on the component lib so a
            # consumer that must RECOMPILE this node's sources with the right
            # `-DNROS_PKG_NAME` (the NuttX kernel link: cc-rs builds each component source
            # for the ARM target separately, since the host-built `.a` is the wrong arch)
            # can recover it without re-deriving from the target name. SOURCES + SOURCE_DIR
            # are standard properties the consumer also reads.
            set_target_properties(${_lib} PROPERTIES
                NROS_COMPONENT_PKG_SYM "${_pkg_sym}"
                NROS_COMPONENT_LANG "${_nrc_lang}")
        endif()

        # Phase 220.G.2 — auto-link every `<pkg>__nano_ros_{c,cpp}`
        # interface lib that `nros_generate_interfaces` registered in
        # this directory's scope. Without this, an example whose src
        # `#include "std_msgs.h"` (or `.hpp`) fails with
        # `No such file or directory` because the include dirs live on
        # the interface lib's INTERFACE_INCLUDE_DIRECTORIES. Pre-220.G
        # every example had to append a per-pkg manual
        # `target_link_libraries(<component> PUBLIC <pkg>__nano_ros_X)`
        # (the 220.G.1 boilerplate, now revertible).
        # DIRECTORY scope — see the property write in
        # NanoRosGenerateInterfaces.cmake.
        if(NOT _nrc_lang STREQUAL "RUST")
            # phase-263 C2c — on Zephyr the generated interface FFI is whole-archived into
            # `app` directly (the Zephyr generator), NOT exposed as a per-pkg
            # `<pkg>__nano_ros_cpp` linkable lib, so `NROS_GENERATED_INTERFACE_LIBS` may carry
            # a NON-target name → the component link tries `-l<name>` and fails ("cannot find
            # -lstd_msgs__nano_ros_cpp"). Skip the interface-lib link on Zephyr; the component
            # gets the generated msg headers via the `app` include mirror below.
            # issue 0326 — inverted guard: with NANO_ROS_PLATFORM unset this read TRUE
    # on Zephyr and linked non-target `-lstd_msgs__nano_ros_cpp` names.
    _nros_is_zephyr(_nrc_is_zephyr_link)
    if(NOT _nrc_is_zephyr_link)
                get_directory_property(_nros_iface_libs NROS_GENERATED_INTERFACE_LIBS)
                if(_nros_iface_libs)
                    list(REMOVE_DUPLICATES _nros_iface_libs)
                    target_link_libraries(${_lib} PUBLIC ${_nros_iface_libs})
                endif()
            endif()
            # Phase 244.C2 — on Zephyr the generated message include dirs
            # (std_msgs.hpp, example_interfaces, …) are added by the Zephyr
            # `nros_generate_interfaces` directly to `app` PRIVATE
            # (zephyr/cmake/nros_generate_interfaces.cmake:290), NOT via the
            # NROS_GENERATED_INTERFACE_LIBS interface-lib path that native/nuttx
            # use. This component lib is a SEPARATE add_library (not `app`), so it
            # never sees those headers and a TYPED component that #includes a
            # generated msg header fails (`std_msgs.hpp: No such file`). Mirror
            # `app`'s full include set onto it — it compiles the same TUs `app`
            # would. Genexpr → captured at generate time, so it picks up includes
            # `find_package(<msg pkg>)` adds to `app` after this point too.
            # issue 0326 — the helper already requires TARGET app.
    _nros_is_zephyr(_nrc_is_zephyr_mirror)
    if(_nrc_is_zephyr_mirror)
                target_include_directories(${_lib} PRIVATE
                    $<TARGET_PROPERTY:app,INCLUDE_DIRECTORIES>)
                # phase-263 C2c — HARD file edge for the per-build sizes headers. A C++
                # component pulls the full nros-cpp surface (action/client/service →
                # `<nros/nros_{,cpp_}config_generated.h>`); on a PRISTINE multi-node build a
                # SEPARATE component lib can compile before the Zephyr cargo build emits those
                # headers, falling through the include path to the in-tree stub (`#error`).
                # `_nros_node_register_config_header_deps` (above) orders the target but ninja
                # can still start the object compile early (issue 0090); a file-level
                # OBJECT_DEPENDS forces each TU to wait for the generated headers. (A C node /
                # the single-node carrier compiles into `app`, which already depends on the
                # cargo build, so neither hits this.)
                _nros_generated_header_dir(_nrc_gen_dir)
                set_source_files_properties(${_NRC_SOURCES} PROPERTIES OBJECT_DEPENDS
                    "${_nrc_gen_dir}/nros-cpp-generated/nros/nros_cpp_config_generated.h;${_nrc_gen_dir}/nros-c-generated/nros/nros_config_generated.h")
            endif()
        endif()
    endif()

    # Phase 238 — NuttX bootable-ELF carrier. The Component lib above is
    # build-coverage only; the rtos_e2e harness + `build_nuttx_cpp_*`
    # resolvers need a bootable kernel ELF at `build-zenoh/<PROJECT_NAME>`.
    # When this Node pkg deploys to nuttx AND the NuttX platform/board
    # overlay is active (`nros_platform_link_app` defined), synthesise a
    # single-node entry TU + a carrier `add_executable(<PROJECT_NAME> …)`
    # and delegate to `nros_platform_link_app` (→ `nros_board_link_app` →
    # `nros_nuttx_build_example`), which drives the cargo `nros-nuttx-ffi`
    # kernel link and copies the ELF to `build-zenoh/<PROJECT_NAME>`.
    #
    # Scope: pub/sub (talker/listener), C AND C++ (238.C). The generated
    # entry is ALWAYS C++ (it drives the header-only C++ EntryNodeRuntime);
    # a C example's declarative node (`Talker.c`) is added as an extra source
    # and compiled as C by the mixed-language cargo build
    # (nros-board-common::nuttx_ffi_build), so its C-linkage
    # `__nros_component_<pkg>_register` symbol matches the entry's
    # `extern "C"` decl. Services / actions register but do not execute
    # (interpreter limit; deferred — see phase-238).
    if((_nrc_lang STREQUAL "CPP" OR _nrc_lang STREQUAL "C")
       AND "nuttx" IN_LIST _NRC_DEPLOY
       AND NANO_ROS_PLATFORM STREQUAL "nuttx"
       AND COMMAND nros_platform_link_app
       AND NOT TARGET ${PROJECT_NAME})
        string(REGEX REPLACE "[^A-Za-z0-9_]" "_" _pkg_sym "${PROJECT_NAME}")
        set(NROS_ENTRY_PKG_SYM "${_pkg_sym}")
        # Baked connect locator. QEMU slirp routes the guest to the host
        # zenoh router at `10.0.2.2:<port>`. Override per-build with
        # `-DNROS_NUTTX_LOCATOR=tcp/10.0.2.2:<port>` (the rtos_e2e harness
        # passes the per-cell `zenohd_port_for` port); the default 7447
        # serves manual `zenohd` runs. Mirrors the Rust `*_entry`
        # `[…entry] locator = …` bake.
        #
        # issue 0946 — the default lives in NanoRosEntryLocator.cmake now, with
        # the entry lane's, so the two cannot drift apart unnoticed. They still
        # DIFFER where they always did; the `node-register` rung preserves this
        # lane's answer exactly. `NROS_ENTRY_LOCATOR` here is the
        # `@NROS_ENTRY_LOCATOR@` template substitution the carrier TU reads, not
        # a second producer.
        _nros_resolve_entry_locator(node-register
            "${NANO_ROS_PLATFORM}" "${NANO_ROS_BOARD}" NROS_ENTRY_LOCATOR)
        set(_entry_dir "${CMAKE_CURRENT_BINARY_DIR}/nros-entry")
        set(_entry_src "${_entry_dir}/main.cpp")
        # Phase 240.3 (RFC-0043) — TYPED routes the carrier to the real
        # executor via the component object (`NuttxBoard::run_components`
        # constructs `CLASS` + calls `configure(node)`), instead of the legacy
        # register-symbol → `EntryNodeRuntime` interpreter. Substitution vars
        # `NROS_ENTRY_CLASS` / `NROS_ENTRY_CLASS_HEADER` / `NROS_ENTRY_NODE_NAME`
        # feed the typed template. C++ only (the C path is 240.4).
        if(_NRC_TYPED)
            set(NROS_ENTRY_SHAPE_RCLCPP "${_nrc_shape_rclcpp}")
            _nros_rtos_entry_family(nuttx)
            if(_nrc_lang STREQUAL "CPP")
                set(NROS_ENTRY_CLASS "${_NRC_CLASS}")
                set(NROS_ENTRY_CLASS_HEADER "${_nrc_header}")
                configure_file(
                    "${_NROS_NODE_REGISTER_DIR}/templates/rtos_entry_main_typed.cpp.in"
                    "${_entry_src}" @ONLY)
            elseif(_nrc_lang STREQUAL "C")
                # Phase 240.4 — C typed component. The entry TU is C++ but
                # constructs the C component via its `__nros_c_component_<pkg>_*`
                # factory/configure seam (NROS_C_COMPONENT). `NROS_ENTRY_PKG_SYM`
                # is already set above to the sanitized pkg.
                configure_file(
                    "${_NROS_NODE_REGISTER_DIR}/templates/rtos_entry_main_c_typed.cpp.in"
                    "${_entry_src}" @ONLY)
            else()
                message(FATAL_ERROR
                    "nano_ros_node_register(TYPED): NuttX carrier supports "
                    "LANGUAGE C or CPP (got '${_nrc_lang}').")
            endif()
        else()
            # Phase 257 (Stage-3) — the non-TYPED NuttX carrier drove the retired
            # `EntryNodeRuntime` interpreter (`NuttxBoard::run`); it is gone. Every
            # NuttX Node pkg is now TYPED (`NROS_C_COMPONENT` / `configure`).
            message(FATAL_ERROR
                "nano_ros_node_register: the non-typed NuttX carrier is retired "
                "(phase-257). Pass `TYPED` (LANGUAGE C or CPP) for the real-executor "
                "carrier.")
        endif()

        # Carrier executable named after the pkg so the ELF lands at
        # `build-zenoh/${PROJECT_NAME}`. SOURCES = entry (main.cpp, picked
        # up as MAIN_SOURCE by nros_board_link_app's `/main\.cpp$` match) +
        # the Component class source(s) (compiled as APP_EXTRA_SOURCES).
        add_executable(${PROJECT_NAME} "${_entry_src}" ${_NRC_SOURCES})
        target_include_directories(${PROJECT_NAME} PRIVATE
            "${CMAKE_CURRENT_SOURCE_DIR}/include"
            "${CMAKE_CURRENT_SOURCE_DIR}/src")
        # NROS_PKG_NAME reaches the class TU through nros_board_link_app's
        # COMPILE_DEFINITIONS → APP_COMPILE_DEFS forwarding (Phase 238).
        target_compile_definitions(${PROJECT_NAME} PRIVATE
            NROS_PKG_NAME=${_pkg_sym})
        if(TARGET NanoRos::NanoRosCpp)
            target_link_libraries(${PROJECT_NAME} PRIVATE NanoRos::NanoRosCpp)
        elseif(TARGET NanoRos::NanoRos)
            target_link_libraries(${PROJECT_NAME} PRIVATE NanoRos::NanoRos)
        endif()
        get_directory_property(_nros_iface_libs NROS_GENERATED_INTERFACE_LIBS)
        if(_nros_iface_libs)
            list(REMOVE_DUPLICATES _nros_iface_libs)
            target_link_libraries(${PROJECT_NAME} PRIVATE ${_nros_iface_libs})
        endif()
        # Issue 0088 — the carrier executable compiles ${_NRC_SOURCES} (C/C++ TUs
        # that include <nros/nros_config_generated.h>); order them after the header
        # mirror targets so they never pick up the in-tree stub.
        _nros_node_register_config_header_deps(${PROJECT_NAME})
        nros_platform_link_app(${PROJECT_NAME})
    endif()

    # Phase 246 (RFC-0043) — ThreadX typed-entry carrier. Mirrors the NuttX
    # branch above (bare-metal riscv64 + threadx-linux host sim both set
    # `NANO_ROS_PLATFORM threadx`): synthesise a single-node C++ entry TU that
    # routes the component to the real executor via `ThreadxBoard::run_components`
    # (construct `CLASS` + `configure(node)`), then delegate to
    # `nros_platform_link_app` for the kernel/netstack/startup link. The board's
    # `startup.c` dispatches to the entry's `app_main` inside the app thread, so
    # the typed entry's `NROS_APP_MAIN_REGISTER_VOID()` symbol is the boot target.
    #
    # TYPED-only: the legacy declarative-register + `NanoRosThreadxSystemCodegen`
    # NULL-context stub is retired on ThreadX (phase-246). Both C and C++.
    if((_nrc_lang STREQUAL "CPP" OR _nrc_lang STREQUAL "C")
       AND NANO_ROS_PLATFORM STREQUAL "threadx"
       AND COMMAND nros_platform_link_app
       AND _NRC_DEPLOY
       AND NOT TARGET ${PROJECT_NAME})
        if(NOT _NRC_TYPED)
            message(FATAL_ERROR
                "nano_ros_node_register: the ThreadX carrier requires TYPED — "
                "the RFC-0043 real-callback component path. The legacy "
                "declarative-register / NULL-context baker entry is retired on "
                "ThreadX (phase-246).")
        endif()
        string(REGEX REPLACE "[^A-Za-z0-9_]" "_" _pkg_sym "${PROJECT_NAME}")
        set(NROS_ENTRY_PKG_SYM "${_pkg_sym}")
        # Baked connect locator. QEMU slirp routes the guest to the host zenoh
        # router at `10.0.2.2:<port>`. Override with `-DNROS_THREADX_LOCATOR=…`;
        # the default 7553 matches the qemu-riscv64-threadx fixture port.
        # CycloneDDS ignores the locator (no router); domain id is compile-time.
        #
        # issue 0946 — resolved through the ONE producer. Note this lane's
        # threadx default (7553, slirp host) is NOT the entry lane's
        # (127.0.0.1:7447 on threadx-linux); both are preserved deliberately.
        _nros_resolve_entry_locator(node-register
            "${NANO_ROS_PLATFORM}" "${NANO_ROS_BOARD}" NROS_ENTRY_LOCATOR)
        set(NROS_ENTRY_SHAPE_RCLCPP "${_nrc_shape_rclcpp}")
        _nros_rtos_entry_family(threadx)
        set(_entry_dir "${CMAKE_CURRENT_BINARY_DIR}/nros-entry")
        set(_entry_src "${_entry_dir}/main.cpp")
        if(_nrc_lang STREQUAL "CPP")
            set(NROS_ENTRY_CLASS "${_NRC_CLASS}")
            set(NROS_ENTRY_CLASS_HEADER "${_nrc_header}")
            configure_file(
                "${_NROS_NODE_REGISTER_DIR}/templates/rtos_entry_main_typed.cpp.in"
                "${_entry_src}" @ONLY)
        else() # C
            configure_file(
                "${_NROS_NODE_REGISTER_DIR}/templates/rtos_entry_main_c_typed.cpp.in"
                "${_entry_src}" @ONLY)
        endif()

        add_executable(${PROJECT_NAME} "${_entry_src}" ${_NRC_SOURCES})
        target_include_directories(${PROJECT_NAME} PRIVATE
            "${CMAKE_CURRENT_SOURCE_DIR}/include"
            "${CMAKE_CURRENT_SOURCE_DIR}/src")
        target_compile_definitions(${PROJECT_NAME} PRIVATE
            NROS_PKG_NAME=${_pkg_sym})
        if(TARGET NanoRos::NanoRosCpp)
            target_link_libraries(${PROJECT_NAME} PRIVATE NanoRos::NanoRosCpp)
        elseif(TARGET NanoRos::NanoRos)
            target_link_libraries(${PROJECT_NAME} PRIVATE NanoRos::NanoRos)
        endif()
        get_directory_property(_nros_iface_libs NROS_GENERATED_INTERFACE_LIBS)
        if(_nros_iface_libs)
            list(REMOVE_DUPLICATES _nros_iface_libs)
            target_link_libraries(${PROJECT_NAME} PRIVATE ${_nros_iface_libs})
        endif()
        # Issue 0088 — the carrier executable compiles ${_NRC_SOURCES} (C/C++ TUs
        # that include <nros/nros_config_generated.h>); order them after the header
        # mirror targets so they never pick up the in-tree stub.
        _nros_node_register_config_header_deps(${PROJECT_NAME})
        # Issue 0090 — `add_dependencies` (above) orders the TARGET but does not stop
        # a TU compiling before the per-build header is mirrored on the threadx link
        # path, so it reads the in-tree stub (`*_OPAQUE_U64S undeclared`) — exactly
        # the 0088 zephyr failure on yet another provisioning path. Add a HARD
        # file-level OBJECT_DEPENDS on the Corrosion mirror header (abs path exported
        # as a global by nros-{c,cpp}/CMakeLists), so the TU won't compile until it
        # exists. C carrier reads the C header; C++ reads both — list whichever the
        # build provides (harmless extra edge).
        if(_NRC_SOURCES)
            get_property(_nros_c_cfg_hdr GLOBAL PROPERTY NROS_C_CONFIG_HEADER_FILE)
            get_property(_nros_cpp_cfg_hdr GLOBAL PROPERTY NROS_CPP_CONFIG_HEADER_FILE)
            set(_nros_cfg_hdrs "")
            if(_nros_c_cfg_hdr)
                list(APPEND _nros_cfg_hdrs "${_nros_c_cfg_hdr}")
            endif()
            if(_nros_cpp_cfg_hdr)
                list(APPEND _nros_cfg_hdrs "${_nros_cpp_cfg_hdr}")
            endif()
            if(_nros_cfg_hdrs)
                # issue 0740 — local stamp, not the cross-directory mirror path.
                _nros_config_header_stamp(_nros_cfg_stamp "${PROJECT_NAME}" ${_nros_cfg_hdrs})
                set_source_files_properties(${_NRC_SOURCES} PROPERTIES
                    OBJECT_DEPENDS "${_nros_cfg_stamp}")
            endif()
        endif()
        nros_platform_link_app(${PROJECT_NAME})
    endif()

    # Phase 240.6 (RFC-0043) — FreeRTOS typed-entry carrier. Mirrors the NuttX /
    # ThreadX branches above (NANO_ROS_PLATFORM freertos, QEMU MPS2-AN385 + lwIP):
    # synthesise a single-node C++ entry TU that routes the component to the real
    # executor via `FreertosBoard::run_components` (construct `CLASS` +
    # `configure(node)`), then delegate to `nros_platform_link_app` for the
    # kernel/lwIP/netif/startup link. The board's `startup.c` `_start` spawns the
    # app task + starts the scheduler; that task's `app_task_entry` brings up the
    # network + poll/zenoh tasks, then dispatches to the entry's `app_main`, so the
    # typed entry's `NROS_APP_MAIN_REGISTER_VOID()` symbol is the boot target —
    # same shape as the NuttX carrier (network is up by the time `app_main` runs).
    #
    # Unlike the Rust FreeRTOS path (which links the board crate's build.rs-emitted
    # NROS_APP_CONFIG), the cmake C/C++ carrier does not pull the Rust board crate,
    # so it generates the NROS_APP_CONFIG TU that startup.c reads (network +
    # scheduling) from `templates/freertos_app_config.c.in`.
    #
    # TYPED-only: the legacy declarative-register / NULL-context baker entry is
    # retired on FreeRTOS (phase-240.6). Both C and C++.
    if((_nrc_lang STREQUAL "CPP" OR _nrc_lang STREQUAL "C")
       AND NANO_ROS_PLATFORM STREQUAL "freertos"
       AND COMMAND nros_platform_link_app
       AND _NRC_DEPLOY
       AND NOT TARGET ${PROJECT_NAME})
        if(NOT _NRC_TYPED)
            message(FATAL_ERROR
                "nano_ros_node_register: the FreeRTOS carrier requires TYPED — "
                "the RFC-0043 real-callback component path. The legacy "
                "declarative-register / NULL-context baker entry is retired on "
                "FreeRTOS (phase-240.6).")
        endif()
        string(REGEX REPLACE "[^A-Za-z0-9_]" "_" _pkg_sym "${PROJECT_NAME}")
        set(NROS_ENTRY_PKG_SYM "${_pkg_sym}")
        # Baked connect locator. QEMU slirp routes the guest to the host zenoh
        # router at `10.0.2.2:<port>`. Override with `-DNROS_FREERTOS_LOCATOR=…`;
        # the default 7447 matches the qemu-arm-freertos example deploy + the
        # rtos_e2e harness's manual `zenohd` default.
        #
        # issue 0946 — resolved through the ONE producer. This is the rung that
        # DISAGREES most visibly with the entry lane: 10.0.2.2 (slirp host) here
        # against 192.0.3.1 (static-lwIP gateway) there. Both justifications are
        # on the record and they name different networks, so the disagreement is
        # carried forward rather than resolved by reading — deciding it needs a
        # QEMU run per lane.
        _nros_resolve_entry_locator(node-register
            "${NANO_ROS_PLATFORM}" "${NANO_ROS_BOARD}" NROS_ENTRY_LOCATOR)
        set(NROS_ENTRY_SHAPE_RCLCPP "${_nrc_shape_rclcpp}")
        _nros_rtos_entry_family(freertos)
        set(_entry_dir "${CMAKE_CURRENT_BINARY_DIR}/nros-entry")
        set(_entry_src "${_entry_dir}/main.cpp")
        if(_nrc_lang STREQUAL "CPP")
            set(NROS_ENTRY_CLASS "${_NRC_CLASS}")
            set(NROS_ENTRY_CLASS_HEADER "${_nrc_header}")
            configure_file(
                "${_NROS_NODE_REGISTER_DIR}/templates/rtos_entry_main_typed.cpp.in"
                "${_entry_src}" @ONLY)
        else() # C
            configure_file(
                "${_NROS_NODE_REGISTER_DIR}/templates/rtos_entry_main_c_typed.cpp.in"
                "${_entry_src}" @ONLY)
        endif()

        # NROS_APP_CONFIG definition TU (network + scheduling) for startup.c.
        # `.zenoh.locator` is cosmetic on the typed path; bake the entry locator
        # for consistency and a domain id of 0 (the deploy DOMAIN_ID default —
        # the typed path's runtime domain is the compile-time NROS_ENTRY_DOMAIN_ID).
        set(NROS_ENTRY_APP_DOMAIN_ID 0)
        # Per-image IP last octet (default .10; distinct per test-pair member —
        # identical IP+MAC seeds give identical zenoh ZIDs → one-peer collapse).
        if(NOT DEFINED NROS_ENTRY_IP_LAST)
            set(NROS_ENTRY_IP_LAST 10)
        endif()
        set(_appcfg_src "${_entry_dir}/nros_app_config_def.c")
        configure_file(
            "${_NROS_NODE_REGISTER_DIR}/templates/freertos_app_config.c.in"
            "${_appcfg_src}" @ONLY)

        add_executable(${PROJECT_NAME} "${_entry_src}" "${_appcfg_src}" ${_NRC_SOURCES})
        target_include_directories(${PROJECT_NAME} PRIVATE
            "${CMAKE_CURRENT_SOURCE_DIR}/include"
            "${CMAKE_CURRENT_SOURCE_DIR}/src")
        target_compile_definitions(${PROJECT_NAME} PRIVATE
            NROS_PKG_NAME=${_pkg_sym})
        if(TARGET NanoRos::NanoRosCpp)
            target_link_libraries(${PROJECT_NAME} PRIVATE NanoRos::NanoRosCpp)
        elseif(TARGET NanoRos::NanoRos)
            target_link_libraries(${PROJECT_NAME} PRIVATE NanoRos::NanoRos)
        endif()
        get_directory_property(_nros_iface_libs NROS_GENERATED_INTERFACE_LIBS)
        if(_nros_iface_libs)
            list(REMOVE_DUPLICATES _nros_iface_libs)
            target_link_libraries(${PROJECT_NAME} PRIVATE ${_nros_iface_libs})
        endif()
        # Issue 0088 — the carrier executable compiles ${_NRC_SOURCES} (C/C++ TUs
        # that include <nros/nros_config_generated.h>); order them after the header
        # mirror targets so they never pick up the in-tree stub.
        _nros_node_register_config_header_deps(${PROJECT_NAME})
        # Issue 0090 — `add_dependencies` (above) orders the TARGET but does not stop
        # a TU compiling before the per-build header is mirrored on the freertos link
        # path, so it reads the in-tree stub (`*_OPAQUE_U64S undeclared`) — exactly the
        # 0088/0090 race the zephyr + threadx carriers already guard. Add a HARD
        # file-level OBJECT_DEPENDS on the Corrosion mirror header(s) so each TU waits
        # for the generated header to exist. Apply to the WHOLE carrier source set: the
        # generated C++ entry (`${_entry_src}`) drives the full nros-cpp surface and the
        # C node source(s) (`${_NRC_SOURCES}`) include <nros/nros_config_generated.h>.
        # The source-file property is directory-scoped, so listing `${_NRC_SOURCES}`
        # ALSO covers the SEPARATE build-coverage component lib (`${_lib}`) that
        # compiles the same file. C carrier reads the C header; C++ reads both — list
        # whichever the build provides (harmless extra edge).
        get_property(_nros_c_cfg_hdr GLOBAL PROPERTY NROS_C_CONFIG_HEADER_FILE)
        get_property(_nros_cpp_cfg_hdr GLOBAL PROPERTY NROS_CPP_CONFIG_HEADER_FILE)
        set(_nros_cfg_hdrs "")
        if(_nros_c_cfg_hdr)
            list(APPEND _nros_cfg_hdrs "${_nros_c_cfg_hdr}")
        endif()
        if(_nros_cpp_cfg_hdr)
            list(APPEND _nros_cfg_hdrs "${_nros_cpp_cfg_hdr}")
        endif()
        if(_nros_cfg_hdrs)
            # issue 0740 — local stamp, not the cross-directory mirror path.
            _nros_config_header_stamp(_nros_cfg_stamp "${PROJECT_NAME}" ${_nros_cfg_hdrs})
            set_source_files_properties("${_entry_src}" "${_appcfg_src}" ${_NRC_SOURCES}
                PROPERTIES OBJECT_DEPENDS "${_nros_cfg_stamp}")
        endif()
        nros_platform_link_app(${PROJECT_NAME})
    endif()

    # Phase 244.C4 (RFC-0043) — native (host) typed-entry carrier. Mirrors
    # the FreeRTOS self-executable branch above (add_executable + the generated
    # entry + the component sources + nros_platform_link_app), but the host board
    # resolves locator/domain from $NROS_LOCATOR / $ROS_DOMAIN_ID at runtime
    # (`LinuxBoard::run_components` -> `nros::init()`), so there is no baked
    # locator and no FreeRTOS app-config TU.
    #
    # TYPED gates the branch (not a FATAL): native supports BOTH the typed carrier
    # AND the imperative hand-written `main` via `nano_ros_entry`. A non-TYPED
    # posix node pkg (declarative / Component-only, e.g. a workspace node compiled
    # only into its component lib above) must fall through here — FATALing would
    # break every non-TYPED posix `nano_ros_node_register` (the 244.C4-collision
    # the phase-247 template sweep hit).
    if((_nrc_lang STREQUAL "CPP" OR _nrc_lang STREQUAL "C")
       AND NANO_ROS_PLATFORM STREQUAL "posix"
       AND _NRC_TYPED
       AND COMMAND nros_platform_link_app
       AND NOT TARGET ${PROJECT_NAME})
        string(REGEX REPLACE "[^A-Za-z0-9_]" "_" _pkg_sym "${PROJECT_NAME}")
        set(NROS_ENTRY_PKG_SYM "${_pkg_sym}")
        set(NROS_ENTRY_SHAPE_RCLCPP "${_nrc_shape_rclcpp}")
        set(_entry_dir "${CMAKE_CURRENT_BINARY_DIR}/nros-entry")
        set(_entry_src "${_entry_dir}/main.cpp")
        # `CMAKE_CURRENT_FUNCTION_LIST_DIR` (CMake ≥3.17) resolves to THIS module's
        # dir regardless of include context — unlike the captured
        # `_NROS_NODE_REGISTER_DIR`, which is empty when the module is reached
        # through a workspace add_subdirectory chain (the 244.C4 workspace-subdir
        # bug: `configure_file` resolved a bogus `/templates/...` root path).
        if(_nrc_lang STREQUAL "CPP")
            set(NROS_ENTRY_CLASS "${_NRC_CLASS}")
            set(NROS_ENTRY_CLASS_HEADER "${_nrc_header}")
            configure_file(
                "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/templates/native_entry_main_typed.cpp.in"
                "${_entry_src}" @ONLY)
        else() # C
            configure_file(
                "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/templates/native_entry_main_c_typed.cpp.in"
                "${_entry_src}" @ONLY)
        endif()

        add_executable(${PROJECT_NAME} "${_entry_src}" ${_NRC_SOURCES})
        target_include_directories(${PROJECT_NAME} PRIVATE
            "${CMAKE_CURRENT_SOURCE_DIR}/include"
            "${CMAKE_CURRENT_SOURCE_DIR}/src")
        target_compile_definitions(${PROJECT_NAME} PRIVATE
            NROS_PKG_NAME=${_pkg_sym})
        if(TARGET NanoRos::NanoRosCpp)
            target_link_libraries(${PROJECT_NAME} PRIVATE NanoRos::NanoRosCpp)
        elseif(TARGET NanoRos::NanoRos)
            target_link_libraries(${PROJECT_NAME} PRIVATE NanoRos::NanoRos)
        endif()
        get_directory_property(_nros_iface_libs NROS_GENERATED_INTERFACE_LIBS)
        if(_nros_iface_libs)
            list(REMOVE_DUPLICATES _nros_iface_libs)
            target_link_libraries(${PROJECT_NAME} PRIVATE ${_nros_iface_libs})
        endif()
        # Issue 0088 — the carrier executable compiles ${_NRC_SOURCES} (C/C++ TUs
        # that include <nros/nros_config_generated.h>); order them after the header
        # mirror targets so they never pick up the in-tree stub.
        _nros_node_register_config_header_deps(${PROJECT_NAME})
        nros_platform_link_app(${PROJECT_NAME})
    endif()

    # Phase 240.8 (RFC-0043) — Zephyr typed-entry carrier. Unlike NuttX (a
    # standalone bootable ELF via add_executable + nros_platform_link_app), a
    # Zephyr app IS the find_package(Zephyr)-owned monolithic `app` target. The
    # carrier APPENDS the generated typed entry TU to `app` and links the
    # component lib (`${_lib}`, built above) into it — no second executable, no
    # per-node component lib the build has to expose separately. The component
    # lib's PUBLIC include dirs (the class header + generated interface libs)
    # propagate to `app`, so the entry TU's `#include "<class_header>"` resolves.
    #
    # Each Node pkg is its own `project(<pkg>)` subdirectory (e.g. ASI
    # `add_subdirectory(controller_pkg)` with `project(controller_pkg)`); the
    # Zephyr `app` target is global, so `target_sources(app …)` from that subdir
    # composes into the outer app. SINGLE-NODE per app: one Node pkg deploys to
    # zephyr per `app` (it owns the one `int main`). Multi-node Zephyr uses the
    # `nros codegen entry --typed` multi-node emitter (one entry constructs all
    # nodes) — out of scope here.
    # issue 0326 — the fused Zephyr carrier guard; the helper subsumes both the
    # platform test and the `TARGET app` requirement.
    _nros_is_zephyr(_nrc_is_zephyr_carrier)
    if((_nrc_lang STREQUAL "CPP" OR _nrc_lang STREQUAL "C")
       AND "zephyr" IN_LIST _NRC_DEPLOY
       AND _nrc_is_zephyr_carrier
       AND NOT TARGET ${PROJECT_NAME}_nros_zephyr_entry)
        if(NOT _NRC_TYPED)
            message(FATAL_ERROR
                "nano_ros_node_register: the Zephyr carrier requires TYPED — "
                "the RFC-0043 real-callback component path. The legacy "
                "declarative-register entry is not generated on Zephyr.")
        endif()
        set(_zephyr_entry_src "${CMAKE_CURRENT_BINARY_DIR}/nros-entry/zephyr_entry_main.cpp")
        if(_nrc_lang STREQUAL "CPP")
            set(NROS_ENTRY_CLASS "${_NRC_CLASS}")
            set(NROS_ENTRY_CLASS_HEADER "${_nrc_header}")
            set(NROS_ENTRY_SHAPE_RCLCPP "${_nrc_shape_rclcpp}")
            configure_file(
                "${_NROS_NODE_REGISTER_DIR}/templates/zephyr_entry_main_typed.cpp.in"
                "${_zephyr_entry_src}" @ONLY)
        else()
            # Phase 244.C2 — Zephyr C typed carrier (mirrors the NuttX C path).
            # The entry TU is C++ but constructs the C component via its
            # `__nros_c_component_<pkg>_*` factory/configure seam
            # (NROS_C_COMPONENT); `NROS_ENTRY_PKG_SYM` is the sanitized pkg name
            # the C source was compiled with.
            string(REGEX REPLACE "[^A-Za-z0-9_]" "_" _pkg_sym "${PROJECT_NAME}")
            set(NROS_ENTRY_PKG_SYM "${_pkg_sym}")
            configure_file(
                "${_NROS_NODE_REGISTER_DIR}/templates/zephyr_entry_main_c_typed.cpp.in"
                "${_zephyr_entry_src}" @ONLY)
        endif()
        # Idempotency marker — guard one entry TU per Node pkg (re-runnable
        # configure). PROJECT_NAME is the Node pkg (its own project()), so the
        # marker is per-pkg, not per-app.
        add_custom_target(${PROJECT_NAME}_nros_zephyr_entry)
        target_sources(app PRIVATE "${_zephyr_entry_src}")
        # issue 0638 — the entry TU needs the FILE-level edge too, and the
        # block above says why while exempting this very case: "a C node / the
        # single-node carrier compiles into `app`, which already depends on the
        # cargo build, so neither hits this". But three lines earlier the same
        # comment records that a target edge "orders the target [while] ninja
        # can still start the object compile early (issue 0090)" — which is
        # exactly what `app` gives this source. The exemption assumes a target
        # dependency does something it explicitly does not do.
        #
        # Measured: in one `just zephyr build-fixtures`, `build-c-talker-
        # cyclonedds` compiled this TU after the cargo build and linked, while
        # `build-c-listener-cyclonedds` compiled it before and fell through the
        # include path to the in-tree stub — `#error
        # "nros_cpp_config_generated.h must be supplied per-build"`. Same
        # platform, same language, same RMW, same configure run: the only
        # difference was scheduling, which is the signature of a race rather
        # than a missing feature.
        #
        # APPEND for the reason the sibling site gives: the Zephyr
        # interface-codegen module stamps OBJECT_DEPENDS on sources too, and a
        # plain set CLOBBERS whichever side ran first.
        _nros_generated_header_dir(_nrc_gen_dir)
        set_property(SOURCE "${_zephyr_entry_src}" APPEND PROPERTY OBJECT_DEPENDS
            "${_nrc_gen_dir}/nros-cpp-generated/nros/nros_cpp_config_generated.h"
            "${_nrc_gen_dir}/nros-c-generated/nros/nros_config_generated.h")
        target_link_libraries(app PRIVATE ${_lib})
    endif()

    _nros_json_strlist(_sources_json ${_NRC_SOURCES})
    _nros_json_strlist(_deploy_json  ${_NRC_DEPLOY})
    _nros_json_strlist(_cbgs_json    ${_NRC_CALLBACK_GROUPS})

    # phase-412 — `ENTITIES` is RETIRED. It stays PARSED (and
    # `KEYWORDS_MISSING_VALUES` still catches the valueless form) purely so an
    # existing caller fails HERE, naming its replacement, instead of having the
    # keyword and its specs silently join SOURCES via UNPARSED_ARGUMENTS and
    # become source files nobody can find. Same shape as the HOST removal in
    # `nano_ros_entry`.
    #
    # There is no metadata `"entities"` key any more. The three-valued rule it
    # existed for — "did not say" vs "creates none" vs the specs — is not gone,
    # it MOVED: a component the model describes is stated, a component the model
    # does not describe is "did not say" and still makes the image refuse rather
    # than derive a total that is short.
    set(_entities_field "")
    if(DEFINED _NRC_ENTITIES OR "ENTITIES" IN_LIST _NRC_KEYWORDS_MISSING_VALUES)
        message(FATAL_ERROR
            "nano_ros_node_register(${_NRC_NAME}): ENTITIES was retired (phase-412).\n"
            "  What this component creates is now stated ONCE PER SYSTEM, in a contract\n"
            "  sidecar beside the launch file that runs it:\n"
            "\n"
            "      <bringup>/launch/<stem>.contract.yaml   (beside <stem>.launch.xml)\n"
            "\n"
            "  nodes: names each node's `pub` / `sub` / `srv` / `cli` endpoints and its\n"
            "  `paths:` (a path whose trigger is `{ timer: { rate_hz: N } }` IS a timer);\n"
            "  topics: / services: / actions: wire those endpoints to absolute names.\n"
            "  The resolver folds it into the SystemModel and `nano_ros_entry` passes\n"
            "  that model to `nros ws entity-inventory`, so the pools size themselves.\n"
            "\n"
            "  Delete the ENTITIES argument. A system with no contract yet keeps its\n"
            "  configured NROS_EXECUTOR_MAX_CBS, exactly as it did before phase-403.")
    endif()

    get_property(_acc GLOBAL PROPERTY NROS_COMPONENTS_JSON)
    if(_acc)
        set(_sep ",")
    else()
        set(_sep "")
    endif()
    set(_entry
"${_sep}\n    {\"name\": \"${_NRC_NAME}\", \"pkg\": \"${PROJECT_NAME}\", \"class\": \"${_NRC_CLASS}\", \
\"class_header\": \"${_nrc_header}\", \"shape\": \"${_nrc_shape}\", \
\"sources\": [${_sources_json}], \"deploy\": [${_deploy_json}], \
\"pkg_dir\": \"${CMAKE_CURRENT_SOURCE_DIR}\", \"lang\": \"${_nrc_lang_lc}\", \
\"callback_groups\": [${_cbgs_json}]${_entities_field}}")
    set_property(GLOBAL APPEND_STRING PROPERTY NROS_COMPONENTS_JSON "${_entry}")
    _nros_metadata_emit()

    # issue 1033 — arm the deferred composer. The metadata is written above; on
    # a standalone image nothing else ever composes it into knobs, so this is
    # where that gets scheduled. Idempotent (guarded on a GLOBAL property) and
    # a no-op when an entry composes first.
    #
    # The CLI travels by GLOBAL property because the deferred call takes no
    # arguments. OPTIONAL: a tree without the CLI resolvable still registers
    # fine, it simply derives nothing — which is the behaviour it had before
    # this existed, not a new failure.
    get_property(_nros_inv_cli GLOBAL PROPERTY NROS_ENTITY_INVENTORY_CLI)
    if(NOT _nros_inv_cli)
        nros_resolve_cli(_nros_inv_cli OPTIONAL
            CONTEXT "entity inventory (standalone image)")
        if(_nros_inv_cli)
            set_property(GLOBAL PROPERTY NROS_ENTITY_INVENTORY_CLI
                "${_nros_inv_cli}")
        endif()
    endif()
    _nros_node_register_schedule_inventory()

    # phase-403 step 2 carried a declared `@depth=` to the COMPILER from here,
    # via `_nros_emit_declared_qos_header`. Its only producer was `ENTITIES`,
    # which is retired, so the call is gone and no component gets a depth table.
    # The function itself is still there and still tested; see the note above it
    # for why, and issue 1084 for what a new producer costs.
endfunction()

# _nros_emit_declared_qos_header(<component> <target> <lang>)
#
# phase-403 step 2 — render this component's declared QoS depths as a C++
# header and put it on its own library's include path, so `NROS_SUBSCRIBE` can
# NO PRODUCTION CALLER, and that is deliberate rather than an oversight.
# `ENTITIES` was this function's only caller and it is retired (phase-412), so
# nothing in a real configure renders a depth table today. The function and its
# test (`tests/cmake-declared-qos-header-tests.sh`, a REAL configure) stay
# because the mechanism is correct and the renderer behind it is still exercised
# by `check-cpp`; what is missing is a producer, not the machinery.
#
# Restoring one means rendering from the model's
# `contracts.sub_endpoints[*].qos.depth`, which `EntityInventory::from_model`
# already reads. The obstacle is WHERE: this runs per component, before the
# entry has resolved a model, and the table has to land on that component's own
# include path. Issue 1084.
#
# static_assert the QoS at a call site against the `@depth=` the register call
# beside it declared.
#
# =============================================================================
# Why PER COMPONENT, and why HERE
# =============================================================================
#
# The knob half of this inventory is composed image-wide by `nano_ros_entry()`,
# because `NROS_EXECUTOR_MAX_CBS` sizes ONE executor. The DEPTH table is not
# that shape. It answers a question a single translation unit asks about its own
# call sites, and there are two reasons that makes per-component the right unit:
#
#   * ORDERING. `nano_ros_entry()` runs after every register call, which is
#     after this target's include path is already set. A table written there
#     would reach a component's TUs one configure late — the same lag the
#     payload-class join lives with, but here it would mean a check that
#     silently does not run on the configure that changed the declaration.
#   * KEYS. The table is keyed `(type, topic)`. Two components in one image may
#     legitimately subscribe to the same topic at different depths, and an
#     image-wide table would have to pick one.
#
# The CLI still owns the grammar and the rendering — `--component` narrows the
# metadata that was just written to this one row. Nothing is parsed in cmake.
#
# =============================================================================
# ABSENT IS THE NORMAL CASE
# =============================================================================
#
# No `ENTITIES`, no CLI, a RUST or INTERFACE target, a component that declares
# no depth: no header is written, `nros/declared_qos.hpp` finds none, the table
# is empty and every call site compiles exactly as it did before. A missing
# table means "nobody declared", never "declared zero" — which is why this
# function is silent on every one of those paths rather than warning: an image
# that has not opted in is not an image in error.
function(_nros_emit_declared_qos_header _component _target _lang)
    # A Rust component has no C++ call site to check, and a target that is not
    # a real library has no include path to put a header on.
    if(_lang STREQUAL "RUST")
        return()
    endif()
    if(NOT TARGET ${_target})
        return()
    endif()
    get_target_property(_nrq_type ${_target} TYPE)
    if(_nrq_type STREQUAL "INTERFACE_LIBRARY")
        return()
    endif()
    nros_resolve_cli(_nrq_cli OPTIONAL CONTEXT "nano_ros_node_register(ENTITIES @depth=)")
    if(NOT _nrq_cli OR NOT EXISTS "${_nrq_cli}")
        # Same rule the knob lane holds: no CLI is a refusal, never a guess.
        return()
    endif()
    # ONE spelling of the metadata path, shared with the knob lane that also
    # reads it. A second one here is how a producer and a consumer of the same
    # file quietly stop meeting.
    nros_entity_inventory_metadata_file(_nrq_metadata)
    if(NOT EXISTS "${_nrq_metadata}")
        return()
    endif()

    # One dir per component, and the `nros/` level is what makes the include
    # spelling `<nros/nros_declared_qos_generated.h>` — the same shape the
    # per-build sizes headers use, so nothing here needs a new convention.
    set(_nrq_dir "${CMAKE_CURRENT_BINARY_DIR}/nros-declared-qos/${_component}")
    set(_nrq_hdr "${_nrq_dir}/nros/nros_declared_qos_generated.h")
    file(MAKE_DIRECTORY "${_nrq_dir}/nros")
    execute_process(
        COMMAND "${_nrq_cli}" ws entity-inventory
                --metadata "${_nrq_metadata}"
                --component "${PROJECT_NAME}::${_component}"
                --output-header "${_nrq_hdr}"
        OUTPUT_VARIABLE _nrq_out
        ERROR_VARIABLE _nrq_err
        RESULT_VARIABLE _nrq_rc
        OUTPUT_STRIP_TRAILING_WHITESPACE)
    if(NOT _nrq_rc EQUAL 0)
        # A broken declaration is fatal and names the component, exactly as the
        # image-wide lane makes it fatal. The difference from "no CLI" above is
        # the difference between "you have not declared" and "what you declared
        # is wrong", and they license different actions.
        message(FATAL_ERROR
            "nros: could not render the declared QoS depths of "
            "${PROJECT_NAME}::${_component}.\n"
            "  ${_nrq_err}\n"
            "  Usually the `ENTITIES` argument of that nano_ros_node_register() is "
            "malformed -- an unknown kind, an unknown `@attr=`, a `@depth=0`.\n"
            "  FATAL and not skipped on purpose: a component whose table is missing "
            "compiles with every declared-depth check disabled, which looks exactly "
            "like a component whose depths all agree.")
    endif()
    if(NOT EXISTS "${_nrq_hdr}")
        return()
    endif()

    # PRIVATE: this table describes THIS component's call sites. A consumer that
    # links the component must not inherit it, or its own NROS_SUBSCRIBE calls
    # would be checked against somebody else's declaration.
    target_include_directories(${_target} PRIVATE "${_nrq_dir}")
    # The CLI writes write-if-changed, so re-running it on every configure does
    # not re-arm a rebuild; this edge is what makes an EDITED declaration reach
    # the next compile.
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_nrq_hdr}")
endfunction()

# (The 212.N.6 `nano_ros_application` and 213.B.1 `nano_ros_component_register`
# deprecation shims were retired in 287-W8 — both caller sweeps completed long
# ago; zero callers remained.)

# (`nano_ros_deploy` was retired post-287 — nothing consumed its
# deploy_targets JSON: the CLI's MetadataDoc reads only `components`, and the
# per-package deploy/rmw tuple lives in package.xml `<export><nano_ros …/>`
# since 287-W4. `nros_system_generate`'s self-pkg detection now keys on the
# package.xml tuple instead of the retired cmake call.)

# Phase 212.N.6 — pull in `nano_ros_entry`. The Entry module
# back-includes this file (guarded) for the shared helpers
# (`_nros_metadata_emit`, `_nros_json_strlist`) + GLOBAL property
# definitions; doing the include LAST ensures those helpers are
# already defined by the time NanoRosEntry's body runs, and that the
# deprecation shim above can resolve `nano_ros_entry` at call time.
include("${CMAKE_CURRENT_LIST_DIR}/NanoRosEntry.cmake")

# ---------------------------------------------------------------------------
# _nros_config_header_stamp(<out-var> <owner-target> <header>...)  — issue 0740,
#                                                          keyed per target by 0990
#
# A LOCAL proxy for cross-directory generated headers, so a file-level
# `OBJECT_DEPENDS` works under the Unix Makefiles generator.
#
# The mirrored config headers are `add_custom_command(OUTPUT ...)` products of
# `packages/api/nros-{c,cpp}/`. Ninja keeps one global graph, so a consumer in
# another directory can name the file and get the edge. Make does not: a custom
# command's OUTPUT rule exists only in the makefile of the directory that
# declared it, so a consumer elsewhere names a prerequisite nothing can build —
#
#     No rule to make target '.../nros-c/include/nros/nros_config_generated.h',
#     needed by '.../<entry>_nros_main_generated.cpp.o'.
#
# `add_dependencies` does NOT fix this, and that is the trap: it was already
# there (`_nros_node_register_apply_config_header_deps`) when issue 0740 was
# filed. Target-level ordering says "build that target first"; it does not give
# the .o's prerequisite a RULE. Only the second build passes, because by then
# the file exists — which is why in-tree lanes never saw it and a clean
# downstream consumer build always does.
#
# So give the CONSUMER's own directory a rule. The stamp is a `copy_if_different`
# of the headers, which means:
#
#   * Make has a local rule, so the prerequisite resolves on a clean tree;
#   * `DEPENDS` names the producing TARGETS (legal, and the ordering edge);
#   * the stamp's mtime moves only when the header CONTENT does, so the
#     rebuild-on-change edge issues 0088/0268 exist for is preserved. A bare
#     `touch` stamp would order correctly and rebuild every consumer TU on every
#     build, which is how a correctness fix becomes a build-time regression.
#
# Idempotent per directory: several entries in one CMakeLists share one stamp.
function(_nros_config_header_stamp _out_var _owner)
    set(_stamps "")
    # The stamp must go stale EXACTLY when the mirror does, in BOTH generators.
    #
    # Naming only the mirror TARGETS is not enough: `DEPENDS <target>` is
    # ORDER-ONLY in Ninja, so a rebuilt crate would refresh the mirror and leave
    # the stamp — and therefore every consumer TU — reading the old sizes. That
    # is issue 0268's museum mirror, reintroduced one level out, on the
    # generator CI actually uses. So take the mirrors' OWN triggers: the cargo
    # targets for ordering, plus the staticlib FILE, which is a real Ninja edge
    # and is legal cross-directory because CMake knows which target produces it.
    set(_deps "")
    foreach(_t cargo-build_nros_c cargo-build_nros_cpp
               nros_c_config_header nros_cpp_config_header
               nros_c_cargo_build nros_cpp_cargo_build)
        if(TARGET ${_t})
            list(APPEND _deps ${_t})
        endif()
    endforeach()
    foreach(_lib nros_c-static nros_cpp-static)
        if(TARGET ${_lib})
            list(APPEND _deps "$<TARGET_FILE:${_lib}>")
        endif()
    endforeach()
    foreach(_hdr ${ARGN})
        # One stamp per header, rather than one stamp for all of them:
        # `cmake -E copy_if_different` takes many sources only with a DIRECTORY
        # destination, and concatenating would need a shell redirect that
        # `add_custom_command` cannot express portably. Per-header also keeps
        # `copy_if_different`'s exact semantics — a header that did not change
        # does not touch its stamp.
        get_filename_component(_stem "${_hdr}" NAME_WE)
        # Per CONSUMING TARGET, not per directory — issue 0990.
        #
        # A custom command whose OUTPUT is reached through `OBJECT_DEPENDS` has
        # no owning target, so the Makefile generator emits its rule into the
        # build.make of EVERY target that consumes it. That duplication is what
        # makes issue 0740's fix work (the consumer's own directory gets a rule
        # for the prerequisite), and it cannot be removed without reintroducing
        # 0740 — but with a shared OUTPUT path it also gives N independent make
        # rules writing ONE file. Measured in
        # `examples/workspaces/features/build/posix-zenoh-native/cmake`: 16
        # targets each declaring `_nros_cfg_stamp/nros_config_generated.stamp`.
        # Under `make -j` several can run `copy_if_different` on it at once, and
        # a failure in any one makes GNU make delete the file the others just
        # wrote.
        #
        # Keying the path by target keeps every property that mattered — a rule
        # local to the consumer, and `copy_if_different` so the stamp's mtime
        # still moves only when the header CONTENT does — while giving each rule
        # an output nothing else writes. The cost is one small copy per entry.
        set(_stamp "${CMAKE_CURRENT_BINARY_DIR}/_nros_cfg_stamp/${_owner}/${_stem}.stamp")
        list(APPEND _stamps "${_stamp}")
        # Idempotent per (directory, header): several entries in one
        # CMakeLists.txt share one rule, and declaring it twice is an error.
        get_property(_declared DIRECTORY PROPERTY _NROS_CFG_STAMPS)
        if(NOT "${_stamp}" IN_LIST _declared)
            add_custom_command(
                OUTPUT "${_stamp}"
                COMMAND ${CMAKE_COMMAND} -E make_directory
                        "${CMAKE_CURRENT_BINARY_DIR}/_nros_cfg_stamp/${_owner}"
                COMMAND ${CMAKE_COMMAND} -E copy_if_different "${_hdr}" "${_stamp}"
                DEPENDS ${_deps}
                COMMENT "nano-ros: config-header stamp ${_stem} (issue 0740)"
                VERBATIM)
            set_property(DIRECTORY APPEND PROPERTY _NROS_CFG_STAMPS "${_stamp}")
        endif()
    endforeach()
    set(${_out_var} "${_stamps}" PARENT_SCOPE)
endfunction()
