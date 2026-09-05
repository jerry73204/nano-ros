# NanoRosEntityInventory.cmake -- phase-403 W9 (issue 0965): the READER for the
# ENTITY inventory, and the second half `NanoRosMessageBounds.cmake` says it
# needs by name.
#
# That module's own header states the split:
#
#   NOT DERIVABLE HERE -- a question about entity COUNTS, which needs a second
#   source (an entity inventory), not this one:
#     NROS_EXECUTOR_MAX_CBS, NROS_EXECUTOR_ARENA_SIZE, ...
#
# This is that second source. A bound inventory prices a TYPE; this one counts
# the ENTITIES an image creates, which is the half `NROS_EXECUTOR_MAX_CBS` has
# always needed and never had.
#
# =============================================================================
# Where the answer comes from
# =============================================================================
#
# Every `nano_ros_node_register(... ENTITIES ...)` in the image, composed
# through the file that call already writes -- `${CMAKE_BINARY_DIR}/
# nros-metadata.json`. `nros ws entity-inventory` reads it and renders the same
# three transports `bounds.rs` does (JSON, this CMake fragment, and env lines
# for the cargo carrier), from one data model.
#
# It is AUTHOR-STATED, and the reason is timing rather than taste. RFC-0043/0044
# components wire themselves in CONSTRUCTORS; `NROS_SUBSCRIBE` /
# `create_publisher` / `NROS_CREATE_WALL_TIMER` do know the kind and the type `M`,
# and a descriptor emitted from them would name every entity -- but that
# descriptor is a LINK-SECTION fact and exists only after linking, while
# `NROS_EXECUTOR_MAX_CBS` is a `const` compiled into `nros-node` before a single
# component TU is compiled. Emitted evidence can VERIFY a count; it cannot
# SUPPLY one. So the declaration supplies, and the running image verifies.
#
# =============================================================================
# What it derives, and what it does not
# =============================================================================
#
#   DERIVABLE HERE:
#     NROS_EXECUTOR_MAX_CBS      the executor's callback-entry slot demand
#
#   SUPPLIED HERE, derived NEXT DOOR (phase-403 step 1):
#     the zenoh payload classes  `NROS_ENTITY_SUBSCRIBED_TYPES` is the JOIN KEY
#                                `nros_derive_message_bound_knobs` reads through
#                                its `ENTITY_INVENTORY` argument. This module
#                                says WHICH types are received; that one prices
#                                them. Neither half derives a class alone.
#     the arena                  `NROS_ENTITY_RECEIVED_TYPES` is the wider set
#                                it needs -- a service server, a service client
#                                and both action roles all carry receive
#                                buffers that no payload class covers.
#
#   SUPPLIED HERE, consumed NEXT DOOR (phase-403 step 2):
#     the declared QoS DEPTHS    `NROS_ENTITY_DECLARED_DEPTHS` is the multiplier
#                                the arena needs on top of the type bound, and
#                                `NROS_ENTITY_UNDECLARED_DEPTH_COUNT` is what a
#                                consumer must refuse on -- a table over the
#                                endpoints that happened to be annotated sizes
#                                an image from a subset of itself.
#                                The same declaration also reaches the COMPILER:
#                                `nano_ros_node_register()` renders it as
#                                `<nros/nros_declared_qos_generated.h>` on the
#                                component's own include path, and
#                                `NROS_SUBSCRIBE` static_asserts the QoS at each
#                                call site against it.
#
#   NOT YET, and deliberately left alone:
#     NROS_EXECUTOR_ARENA_SIZE   the type set and the depths are both necessary
#                                and still not sufficient: the arena needs a
#                                size probe per entry KIND, which is step 3.
#
# =============================================================================
# A PUBLISHER CLAIMS NO SLOT
# =============================================================================
#
# `NROS_EXECUTOR_MAX_CBS` sizes the executor's callback-entry table. Every
# registration that claims an entry calls `Executor::next_entry_slot()`, and the
# 24 sites that do are subscriptions, timers, services, service clients, action
# servers, action clients and guard conditions. `create_publisher` is not one:
# on the C++ path it writes an `RmwPublisher` into caller-owned storage, and on
# the C path there is no `nros_executor_add_publisher` to increment
# `handle_count`.
#
# This is why the derived number is SMALLER than the entity count, and both are
# published. The mr-canhubk344 bring-up recorded "33 handles" for the island and
# set `MAX_CBS=36` from it; 33 is the entity total and 14 of those are
# publishers. `nros_cli_core::entity_inventory::EntityKind::callback_slots` is
# the one place that difference is spelled, and
# `scripts/check-entity-slot-costs.py` holds it to those 24 sites.
#
# The other way this could be short is an entity the executor creates that no
# component declares. There are two candidates and neither claims a slot:
# `ParamState` is stored outside the arena precisely so it does not consume
# `MAX_CBS` slots, and the five REP-2002 lifecycle servers go through
# `create_lc_srv`, which calls `session.create_service` directly rather than
# `Executor::register_service_*`. The declared application entities are the
# whole demand.
#
# =============================================================================
# An incomplete inventory is REFUSED, never averaged
# =============================================================================
#
# If ANY component in the image states no `ENTITIES`, nothing is derived at all
# and every knob keeps its configured value. That is the same rule
# `nros_derive_message_bound_knobs` holds when any type in the closure is
# unbounded, and for the same reason: a count composed over the components that
# did answer is SMALLER than the image needs, and a short `MAX_CBS` fails entity
# creation on a board.
#
# The derived value also carries NO headroom. It is exactly the declared demand,
# which makes the running image a checker of its own declaration: one entity
# more than declared and registration returns `NodeError::ExecutorFull`, which
# names this knob, and `ComponentNode`'s `ok()` flag halts boot naming the
# failing node. `MAX_CBS` is the right FIRST consumer precisely because that
# failure is loud -- an under-sized ARENA halts during entity creation, before
# the first spin, which is why issue 0900 W1's advisory cannot cover it.
#
# =============================================================================
# Usage
# =============================================================================
#
#   include(NanoRosEntityInventory.cmake)
#   nros_derive_entity_inventory_knobs(
#       CLI <path to nros>                  # required
#       [METADATA <nros-metadata.json>]     # default ${CMAKE_BINARY_DIR}/...
#       [MODEL <system_model.yaml>]         # resolved SystemModel, if any
#       [OUTPUT_FILE <path>]                # default the knobs file below
#       [QUIET])
#
# MODEL is a SECOND source of the same wiring, not a replacement: the verb
# takes the larger of the two per component and per kind. The metadata
# declaration knows the timers the model's schema cannot express; the model
# knows the wiring an `ENTITIES` list can be silently short of. It is ignored
# when the path is empty or missing, and the verb abstains on a model that
# describes no wiring, so an image with no contract sees no change.
#
# Sets in the CALLER's scope:
#
#   NROS_ENTITY_INVENTORY_STATUS          derived | refused
#   NROS_ENTITY_INVENTORY_REASON          prose, when refused
#   NROS_ENTITY_INVENTORY_COMPONENT_COUNT components composed
#   NROS_ENTITY_INVENTORY_ENTITY_TOTAL    entities declared (slots or not)
#   NROS_ENTITY_COUNT_<KIND>              per-kind counts
#   NROS_DERIVED_EXECUTOR_MAX_CBS         unset when not derivable -- ABSENT
#                                         means "no answer", never a default
#   NROS_ENTITY_DECLARED_DEPTH_STATUS     resolved | refused (phase-403 step 2)
#   NROS_ENTITY_DECLARED_DEPTHS           `type|topic=depth` triples
#   NROS_ENTITY_DECLARED_DEPTH_COUNT      endpoints that stated a depth
#   NROS_ENTITY_UNDECLARED_DEPTH_COUNT    endpoints that COULD have and did not.
#                                         A consumer that sizes from depth must
#                                         refuse while this is non-zero
#
# A derived value is a DEFAULT. Every consumer applies it only where nothing
# else stated a number -- see `_nros_resolve_derivable_knob` in
# `zephyr/cmake/nros_cargo_build.cmake` for the precedence ladder
# (env > Kconfig/board > derived > crate default).

include_guard(GLOBAL)

# The inventory schema this reader understands --
# `nros_cli_core::entity_inventory::ENTITY_INVENTORY_SCHEMA_VERSION`. A fragment
# that states anything else is REFUSED, never read field-by-field on the hope
# that nothing moved.
#
# `CACHE INTERNAL` and not a plain variable, for the reason
# `NROS_MESSAGE_BOUNDS_SCHEMA_SUPPORTED` gives at length: this file is reachable
# from inside a function frame, and with `include_guard(GLOBAL)` a file-scope
# `set()` that lands in such a frame is gone when it pops and never comes back.
#
# **2** (phase-403 step 1) added the join key -- `NROS_ENTITY_SUBSCRIBED_TYPES`
# and `NROS_ENTITY_RECEIVED_TYPES`, each with its own status. Nothing MOVED, and
# it still bumps: a version-1 fragment carries neither, and a reader that took
# their absence for "this image receives nothing" would derive a payload class
# over an EMPTY set. Absence has to be distinguishable from zero here for the
# same reason `ENTITIES NONE` exists.
#
# **3** (phase-403 step 2) added the QoS DEPTHS -- `NROS_ENTITY_DECLARED_DEPTHS`
# and, load-bearing beside it, `NROS_ENTITY_UNDECLARED_DEPTH_COUNT`. Bumps on
# the same argument again: depth is a MULTIPLIER on the type bound, so a reader
# that took an absent list for "every endpoint is depth 0" would size an arena
# an order of magnitude short.
set(NROS_ENTITY_INVENTORY_SCHEMA_SUPPORTED 3 CACHE INTERNAL
    "phase-403 W9: the nros_entity_inventory fragment schema this tree reads")

# nros_entity_inventory_knobs_file(<out_var>)
#
# Where the composed, image-wide answer is written, and where a consumer reads
# it. ONE path, because the writer and the reader are in different files, run at
# different points of one configure, and a second spelling is how a derived
# value silently stops arriving.
#
# `CMAKE_BINARY_DIR` and not a per-package dir: the answer is a property of the
# IMAGE. It sits beside `message_bound_knobs.cmake`, its sibling.
function(nros_entity_inventory_knobs_file _out_var)
    set(${_out_var} "${CMAKE_BINARY_DIR}/nros/entity_inventory.cmake" PARENT_SCOPE)
endfunction()

# nros_entity_inventory_metadata_file(<out_var>)
#
# The input: the file `nano_ros_node_register` rewrites on every call
# (`_nros_metadata_emit`). Spelled once here so a reader never hand-derives it.
function(nros_entity_inventory_metadata_file _out_var)
    set(${_out_var} "${CMAKE_BINARY_DIR}/nros-metadata.json" PARENT_SCOPE)
endfunction()

# nros_entity_inventory_seed_knobs_file(<path>)
#
# Write a "nothing composed yet" fragment, for a reader that runs BEFORE the
# entry lane in the same configure.
#
# Exactly the mechanical reason `nros_message_bounds_seed_knobs_file` exists: a
# consumer registers this path with `CMAKE_CONFIGURE_DEPENDS`, and a ninja input
# that does not exist and has no rule producing it is a hard `missing and no
# known rule to make it` at LOAD, before any rule runs. Seeding makes the
# dependency well-formed on the first configure; the real answer overwrites it
# later in that same configure.
#
# Issue 0991 -- "and the next build picks it up" used to stand here, and was
# false. That registration never makes `build.ninja` stale, because
# `build.ninja` is written after this file. `nros_reconfigure_on_change` at
# `nano_ros_entry()` is what actually re-runs cmake.
#
# Does NOT overwrite an existing file: the point is that it may already hold an
# answer.
function(nros_entity_inventory_seed_knobs_file _path)
    if(EXISTS "${_path}")
        return()
    endif()
    get_filename_component(_dir "${_path}" DIRECTORY)
    file(MAKE_DIRECTORY "${_dir}")
    file(WRITE "${_path}"
        "# GENERATED by nros (phase-403 W9, issue 0965). Do not edit.\n"
        "#\n"
        "# Placeholder: no entity inventory had been composed when this configure\n"
        "# first needed one. It is rewritten with the real answer by\n"
        "# nros_derive_entity_inventory_knobs(); that rewrite arms a re-configure\n"
        "# (issue 0991) so the readers that already ran this pass see it.\n"
        "set(NROS_ENTITY_INVENTORY_SCHEMA_VERSION ${NROS_ENTITY_INVENTORY_SCHEMA_SUPPORTED})\n"
        "set(NROS_ENTITY_INVENTORY_STATUS \"refused\")\n"
        "set(NROS_ENTITY_INVENTORY_REASON \"no entity inventory composed yet\")\n")
endfunction()

# _nros_entity_publish(<name> <value>)
#
# Set a result BOTH locally and in the caller's scope. A macro, not a function,
# for the reason `_nros_bounds_publish` is one: a macro runs in the CALLER's
# scope, so its `PARENT_SCOPE` is the caller's parent, which is what the name
# promises. Publishing to only one of the two scopes is the shape that reads as
# working while half the values are empty.
macro(_nros_entity_publish _name _value)
    set(${_name} "${_value}")
    set(${_name} "${_value}" PARENT_SCOPE)
endmacro()

# nros_derive_entity_inventory_knobs(...)  -- see the header comment.
function(nros_derive_entity_inventory_knobs)
    cmake_parse_arguments(_E "QUIET" "CLI;METADATA;MODEL;OUTPUT_FILE" "" ${ARGN})

    set(_metadata "${_E_METADATA}")
    if(NOT _metadata)
        nros_entity_inventory_metadata_file(_metadata)
    endif()
    set(_output "${_E_OUTPUT_FILE}")
    if(NOT _output)
        nros_entity_inventory_knobs_file(_output)
    endif()

    # ---- Nothing derived until proven otherwise -------------------------
    # Every out-variable starts UNSET, in BOTH scopes. A knob that cannot be
    # derived must be ABSENT, so a consumer either reads a number this function
    # computed or reads nothing; and a refusal after a successful call must not
    # leave the previous numbers standing, which is what clearing this frame's
    # copy is for (a function inherits the caller's variables through the scope
    # chain).
    _nros_entity_publish(NROS_ENTITY_INVENTORY_STATUS "refused")
    _nros_entity_publish(NROS_ENTITY_INVENTORY_REASON "")
    _nros_entity_publish(NROS_ENTITY_INVENTORY_COMPONENT_COUNT 0)
    foreach(_v NROS_DERIVED_EXECUTOR_MAX_CBS NROS_DERIVED_EXECUTOR_ACTION_CLIENTS
               NROS_DERIVED_MAX_SUBSCRIBERS NROS_DERIVED_RMW_SUBSCRIBER_SLOTS
               NROS_DERIVED_MAX_PUBLISHERS NROS_DERIVED_MAX_QUERYABLES
               NROS_DERIVED_EXECUTOR_MAX_NODES
               NROS_ENTITY_INVENTORY_ENTITY_TOTAL
               NROS_ENTITY_DECLARED_DEPTH_STATUS NROS_ENTITY_DECLARED_DEPTH_REASON
               NROS_ENTITY_DECLARED_DEPTHS NROS_ENTITY_DECLARED_DEPTH_COUNT
               NROS_ENTITY_UNDECLARED_DEPTH_COUNT)
        unset(${_v})
        unset(${_v} PARENT_SCOPE)
    endforeach()

    if(NOT _E_CLI OR NOT EXISTS "${_E_CLI}")
        set(_why "the `nros` CLI was not available to this configure")
        _nros_entity_publish(NROS_ENTITY_INVENTORY_REASON "${_why}")
        nros_entity_inventory_seed_knobs_file("${_output}")
        return()
    endif()

    # A MISSING metadata file is a refusal, not a fatal: a configure that has
    # registered no component yet is the state every build was in before this
    # wave, and it is the state a pure-Rust or launch-only image stays in.
    if(NOT EXISTS "${_metadata}")
        set(_why
            "no component metadata at ${_metadata} -- nothing in this image called nano_ros_node_register()")
        _nros_entity_publish(NROS_ENTITY_INVENTORY_REASON "${_why}")
        nros_entity_inventory_seed_knobs_file("${_output}")
        return()
    endif()

    # The verb renders the fragment. It is the ONE derivation: the counting
    # rule, the per-kind slot cost and the refusal all live in
    # `nros_cli_core::entity_inventory`, and this file only reads what it wrote.
    # A second count in cmake is how two green tools come to disagree.
    # MODEL is optional and passed only when the file EXISTS. An entry that
    # names no bringup resolves no model, and a path that is empty or absent
    # must not reach the verb: `--model <missing>` is an error there, and the
    # absence of a contract is a normal state, not a broken configure.
    set(_model_arg "")
    if(_E_MODEL AND EXISTS "${_E_MODEL}")
        set(_model_arg --model "${_E_MODEL}")
    endif()
    execute_process(
        COMMAND "${_E_CLI}" ws entity-inventory
                --metadata "${_metadata}"
                ${_model_arg}
                --output-cmake "${_output}"
                --output-json "${CMAKE_BINARY_DIR}/nros/entity_inventory.json"
        OUTPUT_VARIABLE _out
        ERROR_VARIABLE _err
        RESULT_VARIABLE _rc
        OUTPUT_STRIP_TRAILING_WHITESPACE)
    if(NOT _rc EQUAL 0)
        # A non-zero exit is a BROKEN DECLARATION -- an unknown entity kind, a
        # component claiming NONE beside real entities -- not an absent one, and
        # the CLI's message names the component. That is a configuration error
        # and it is fatal, the same way a malformed message-bound fragment is.
        message(FATAL_ERROR
            "nros: the entity declaration in this image is not readable.\n"
            "  ${_err}\n"
            "  Fix the `ENTITIES` argument of the named nano_ros_node_register().")
    endif()

    if(NOT EXISTS "${_output}")
        set(_why "`nros ws entity-inventory` wrote no fragment")
        _nros_entity_publish(NROS_ENTITY_INVENTORY_REASON "${_why}")
        nros_entity_inventory_seed_knobs_file("${_output}")
        return()
    endif()

    unset(NROS_ENTITY_INVENTORY_SCHEMA_VERSION)
    include("${_output}")
    if(NOT DEFINED NROS_ENTITY_INVENTORY_SCHEMA_VERSION)
        message(FATAL_ERROR
            "nros: ${_output} sets no NROS_ENTITY_INVENTORY_SCHEMA_VERSION.\n"
            "  Either it is not an entity-inventory fragment, or it predates the "
            "schema. Rebuild the `nros` CLI: `./scripts/bootstrap.sh` "
            "(contributors: `just setup-cli`).")
    endif()
    if(NOT NROS_ENTITY_INVENTORY_SCHEMA_VERSION EQUAL
       NROS_ENTITY_INVENTORY_SCHEMA_SUPPORTED)
        message(FATAL_ERROR
            "nros: ${_output} states entity-inventory schema version "
            "${NROS_ENTITY_INVENTORY_SCHEMA_VERSION}; this reader understands "
            "${NROS_ENTITY_INVENTORY_SCHEMA_SUPPORTED}.\n"
            "  Refusing rather than reading fields that may have moved.\n"
            "  Rebuild the `nros` CLI so the producer and the reader come from "
            "one tree: `./scripts/bootstrap.sh` (contributors: "
            "`just setup-cli`).")
    endif()

    # Republish everything the fragment set. `include()` inside a function keeps
    # its `set()`s in THIS frame, so without this the caller sees nothing.
    _nros_entity_publish(NROS_ENTITY_INVENTORY_STATUS "${NROS_ENTITY_INVENTORY_STATUS}")
    _nros_entity_publish(NROS_ENTITY_INVENTORY_REASON "${NROS_ENTITY_INVENTORY_REASON}")
    _nros_entity_publish(NROS_ENTITY_INVENTORY_COMPONENT_COUNT
        "${NROS_ENTITY_INVENTORY_COMPONENT_COUNT}")
    if(DEFINED NROS_ENTITY_INVENTORY_ENTITY_TOTAL)
        _nros_entity_publish(NROS_ENTITY_INVENTORY_ENTITY_TOTAL
            "${NROS_ENTITY_INVENTORY_ENTITY_TOTAL}")
    endif()
    if(DEFINED NROS_DERIVED_EXECUTOR_MAX_CBS)
        _nros_entity_publish(NROS_DERIVED_EXECUTOR_MAX_CBS
            "${NROS_DERIVED_EXECUTOR_MAX_CBS}")
    endif()
    # Issue 0900 — how many of those slots the arena budgets at the ACTION
    # entry size. Published beside MAX_CBS because the two are only meaningful
    # together: build.rs clamps this to that.
    if(DEFINED NROS_DERIVED_EXECUTOR_ACTION_CLIENTS)
        _nros_entity_publish(NROS_DERIVED_EXECUTOR_ACTION_CLIENTS
            "${NROS_DERIVED_EXECUTOR_ACTION_CLIENTS}")
    endif()
    # phase-412 W1 -- the SESSION pools, republished on the same terms: present
    # only when the whole image declared, absent otherwise, so a consumer reads
    # a derived value or reads nothing.
    foreach(_pool NROS_DERIVED_MAX_SUBSCRIBERS NROS_DERIVED_RMW_SUBSCRIBER_SLOTS
                  NROS_DERIVED_MAX_PUBLISHERS NROS_DERIVED_MAX_QUERYABLES
                  NROS_DERIVED_EXECUTOR_MAX_NODES)
        if(DEFINED ${_pool})
            _nros_entity_publish(${_pool} "${${_pool}}")
        endif()
    endforeach()
    foreach(_kind PUBLISHER SUBSCRIPTION TIMER SERVICE_SERVER SERVICE_CLIENT
                  ACTION_SERVER ACTION_CLIENT GUARD_CONDITION)
        if(DEFINED NROS_ENTITY_COUNT_${_kind})
            _nros_entity_publish(NROS_ENTITY_COUNT_${_kind}
                "${NROS_ENTITY_COUNT_${_kind}}")
        endif()
    endforeach()
    # phase-403 step 1 -- the JOIN KEY. `nros_derive_message_bound_knobs`
    # includes the fragment itself rather than reading these, so republishing
    # them buys a caller that wants to inspect the set, not the join. Both views
    # travel: SUBSCRIBED is the payload-class population, RECEIVED is the wider
    # set the arena needs.
    foreach(_view SUBSCRIBED RECEIVED)
        foreach(_field TYPES_STATUS TYPES_REASON TYPES TYPE_COUNTS ENTITY_COUNT)
            if(DEFINED NROS_ENTITY_${_view}_${_field})
                _nros_entity_publish(NROS_ENTITY_${_view}_${_field}
                    "${NROS_ENTITY_${_view}_${_field}}")
            endif()
        endforeach()
    endforeach()
    # phase-403 step 2 -- the declared QoS DEPTHS. The UNDECLARED count travels
    # beside the list and is the one a size consumer reads FIRST: a list over
    # the endpoints that happened to be annotated is a subset of the image, and
    # sizing from a subset is the under-report this module exists to prevent.
    foreach(_field DECLARED_DEPTH_STATUS DECLARED_DEPTH_REASON DECLARED_DEPTHS
                   DECLARED_DEPTH_COUNT UNDECLARED_DEPTH_COUNT)
        if(DEFINED NROS_ENTITY_${_field})
            _nros_entity_publish(NROS_ENTITY_${_field} "${NROS_ENTITY_${_field}}")
        endif()
    endforeach()

    if(_E_QUIET)
        return()
    endif()
    if(NROS_ENTITY_INVENTORY_STATUS STREQUAL "derived")
        message(STATUS
            "nros: entity inventory DERIVED from "
            "${NROS_ENTITY_INVENTORY_COMPONENT_COUNT} components -- "
            "${NROS_ENTITY_INVENTORY_ENTITY_TOTAL} entities, "
            "${NROS_DERIVED_EXECUTOR_MAX_CBS} executor callback slots "
            "-> NROS_EXECUTOR_MAX_CBS, "
            "${NROS_DERIVED_EXECUTOR_ACTION_CLIENTS} of them action-sized "
            "-> NROS_EXECUTOR_ACTION_CLIENTS")
        if(DEFINED NROS_ENTITY_COUNT_PUBLISHER AND
           NROS_ENTITY_COUNT_PUBLISHER GREATER 0)
            message(STATUS
                "nros:   ${NROS_ENTITY_COUNT_PUBLISHER} of them are publishers, "
                "which claim no callback slot")
        endif()
    else()
        message(STATUS
            "nros: entity sizing not available this configure -- "
            "NROS_EXECUTOR_MAX_CBS keeps its configured value.\n"
            "  ${NROS_ENTITY_INVENTORY_REASON}")
    endif()
endfunction()

# -----------------------------------------------------------------------------
# `cmake -P` entry point.
#
# Running the derivation without configuring a project is what makes it
# TESTABLE, and it is how the message-bound reader beside it is tested
# (`tests/cmake-message-bounds-tests.sh`):
#
#   cmake -DNROS_ENTITY_CLI=<nros> -DNROS_ENTITY_METADATA=<meta.json> \
#         [-DNROS_ENTITY_OUTPUT=out.cmake] \
#         -P cmake/NanoRosEntityInventory.cmake
#
# `CMAKE_BINARY_DIR` in script mode is the invocation CWD, which is what makes
# the default output path land somewhere writable.
# -----------------------------------------------------------------------------
if(CMAKE_SCRIPT_MODE_FILE AND
   CMAKE_SCRIPT_MODE_FILE STREQUAL CMAKE_CURRENT_LIST_FILE)
    if(NOT DEFINED NROS_ENTITY_CLI OR NOT DEFINED NROS_ENTITY_METADATA)
        message(FATAL_ERROR
            "usage: cmake -DNROS_ENTITY_CLI=<nros> -DNROS_ENTITY_METADATA=<meta.json> "
            "[-DNROS_ENTITY_MODEL=<system_model.yaml>] [-DNROS_ENTITY_OUTPUT=path] "
            "-P cmake/NanoRosEntityInventory.cmake")
    endif()
    set(_args CLI "${NROS_ENTITY_CLI}" METADATA "${NROS_ENTITY_METADATA}")
    if(DEFINED NROS_ENTITY_MODEL)
        list(APPEND _args MODEL "${NROS_ENTITY_MODEL}")
    endif()
    if(DEFINED NROS_ENTITY_OUTPUT)
        list(APPEND _args OUTPUT_FILE "${NROS_ENTITY_OUTPUT}")
    endif()
    nros_derive_entity_inventory_knobs(${_args})
    message(STATUS "NROS_ENTITY_INVENTORY_STATUS=${NROS_ENTITY_INVENTORY_STATUS}")
    foreach(_v
        NROS_ENTITY_INVENTORY_COMPONENT_COUNT
        NROS_ENTITY_INVENTORY_ENTITY_TOTAL
        NROS_DERIVED_EXECUTOR_MAX_CBS
        NROS_DERIVED_EXECUTOR_ACTION_CLIENTS
        NROS_DERIVED_MAX_SUBSCRIBERS
        NROS_DERIVED_RMW_SUBSCRIBER_SLOTS
        NROS_DERIVED_MAX_PUBLISHERS
        NROS_DERIVED_MAX_QUERYABLES
        NROS_DERIVED_EXECUTOR_MAX_NODES
        NROS_ENTITY_COUNT_PUBLISHER
        NROS_ENTITY_COUNT_SUBSCRIPTION
        NROS_ENTITY_COUNT_TIMER
        NROS_ENTITY_COUNT_SERVICE_SERVER
        NROS_ENTITY_COUNT_SERVICE_CLIENT
        NROS_ENTITY_COUNT_ACTION_SERVER
        NROS_ENTITY_COUNT_ACTION_CLIENT
        NROS_ENTITY_COUNT_GUARD_CONDITION
        NROS_ENTITY_SUBSCRIBED_TYPES_STATUS
        NROS_ENTITY_SUBSCRIBED_TYPES
        NROS_ENTITY_SUBSCRIBED_TYPE_COUNTS
        NROS_ENTITY_SUBSCRIBED_ENTITY_COUNT
        NROS_ENTITY_RECEIVED_TYPES_STATUS
        NROS_ENTITY_RECEIVED_TYPES
        NROS_ENTITY_RECEIVED_TYPE_COUNTS
        NROS_ENTITY_RECEIVED_ENTITY_COUNT
        NROS_ENTITY_DECLARED_DEPTH_STATUS
        NROS_ENTITY_DECLARED_DEPTHS
        NROS_ENTITY_DECLARED_DEPTH_COUNT
        NROS_ENTITY_UNDECLARED_DEPTH_COUNT
        NROS_ENTITY_DECLARED_DEPTH_REASON)
        if(DEFINED ${_v})
            message(STATUS "${_v}=${${_v}}")
        endif()
    endforeach()
    if(NROS_ENTITY_INVENTORY_REASON)
        message(STATUS "NROS_ENTITY_INVENTORY_REASON=${NROS_ENTITY_INVENTORY_REASON}")
    endif()
endif()
