# phase-392 W5.c — deliver the ENTITY figures the RMW sizes its queryable table
# from to the cargo invocation this configure owns.
#
# Sibling of `NanoRosBoardFacts.cmake`, same carrier and the same reason: a
# workspace member's own `.cargo/config.toml` is never read, because Corrosion
# runs cargo from the workspace root (phase-349 W2.0), and `set(ENV{...})`
# reaches only the configure-time process, so a knob published that way lands in
# the C lane and not the cargo one (issue 0460). `corrosion_set_env_vars`
# attaches to the target's own build command.
#
# WHAT IS DIFFERENT FROM BOARD FACTS. Board facts answer a question about the
# BOARD, of which exactly one is active per configure. This answers a question
# about the resolved SystemModel, of which a workspace can hold SEVERAL — one
# per entry. There is only ONE runtime staticlib per configure and every entry
# links it, so the compile-time table must satisfy the largest declaration:
# entries ACCUMULATE here (union of the infrastructure flags, max of the
# application counts) and the union is applied once.
#
# Accumulation is safe in that order because `nros_synth_runtime_umbrella` runs
# AFTER the SUBDIRS loop that processes the entries (NanoRosWorkspace.cmake) —
# the same ordering `nros-metadata.json` already depends on.

include_guard(GLOBAL)

include("${CMAKE_CURRENT_LIST_DIR}/NanoRosCorrosionEnv.cmake")

# nros_record_entity_facts(<model-path>)
#
# Ask `nros ws entity-facts` about ONE entry's model and fold the answer into
# this configure's accumulated view.
#
# Deliberately soft on failure, exactly like `nros_resolve_board_facts`: a model
# that is not there yet, a CLI that has not been built, an entry addressed the
# `MODEL` way at a path that does not exist — all mean "this configure has no
# entity facts to carry", which is the state every build was in before this
# wave. Nothing here is a configuration error, so nothing here is fatal.
function(nros_record_entity_facts _model)
    if(_model STREQUAL "" OR NOT EXISTS "${_model}")
        return()
    endif()
    if(NOT DEFINED _NANO_ROS_CODEGEN_TOOL OR NOT EXISTS "${_NANO_ROS_CODEGEN_TOOL}")
        return()
    endif()

    # One run per distinct model — several entries share a bringup, and the
    # workspaces that do (workspaces/c has 7) would otherwise pay the verb once
    # per entry for the same answer.
    string(MAKE_C_IDENTIFIER "NROS_ENTITY_FACTS_MEMO__${_model}" _memo)
    get_property(_seen GLOBAL PROPERTY ${_memo})
    if(_seen)
        return()
    endif()
    set_property(GLOBAL PROPERTY ${_memo} TRUE)

    execute_process(
        COMMAND "${_NANO_ROS_CODEGEN_TOOL}" ws entity-facts --model "${_model}"
        OUTPUT_VARIABLE _out
        ERROR_VARIABLE _err
        RESULT_VARIABLE _rc
        OUTPUT_STRIP_TRAILING_WHITESPACE)
    if(NOT _rc EQUAL 0)
        string(REGEX REPLACE "\n+" " " _why "${_err}")
        string(SUBSTRING "${_why}" 0 200 _why)
        message(STATUS "nano-ros: entity facts NOT read from ${_model} — ${_why}")
        return()
    endif()

    set_property(GLOBAL PROPERTY NROS_ENTITY_FACTS_SEEN TRUE)

    string(REPLACE "\n" ";" _lines "${_out}")
    # An entry whose model describes no wiring says NOTHING about the
    # application count (the verb abstains rather than reporting a zero it
    # cannot support). One such entry makes the whole configure's application
    # count unknown: the shared staticlib has to hold the largest, and an
    # unknown is not smaller than anything.
    set(_saw_servers FALSE)
    foreach(_line IN LISTS _lines)
        if(_line MATCHES "^NROS_DECLARED_INFRA_QUERYABLES=(.*)$")
            set(_infra "${CMAKE_MATCH_1}")
            if(_infra MATCHES "param")
                set_property(GLOBAL PROPERTY NROS_ENTITY_INFRA_PARAM TRUE)
            endif()
            if(_infra MATCHES "lifecycle")
                set_property(GLOBAL PROPERTY NROS_ENTITY_INFRA_LIFECYCLE TRUE)
            endif()
        elseif(_line MATCHES "^NROS_DECLARED_SERVICE_SERVERS=([0-9]+)$")
            set(_saw_servers TRUE)
            get_property(_have GLOBAL PROPERTY NROS_ENTITY_SERVERS_MAX)
            if(NOT _have OR CMAKE_MATCH_1 GREATER _have)
                set_property(GLOBAL PROPERTY NROS_ENTITY_SERVERS_MAX "${CMAKE_MATCH_1}")
            endif()
        endif()
    endforeach()
    if(NOT _saw_servers)
        set_property(GLOBAL PROPERTY NROS_ENTITY_SERVERS_UNKNOWN TRUE)
    endif()
endfunction()

# nros_entity_facts_env_deferred(<target>)
#
# Schedule `nros_entity_facts_env` for the END of the top-level scope, once per
# target. Use this from anywhere that imports a Corrosion crate; call the
# immediate form only if you can prove every `nano_ros_add_entry()` has already
# run, which almost nothing can.
#
# WHY DEFERRED (phase-392 W5.g). The facts are accumulated by
# `nros_record_entity_facts`, which runs inside `nano_ros_add_entry()` — and an
# entry is declared LAST in a configure by design ("the first point guaranteed
# to be AFTER every nano_ros_node_register()"). Every caller that applies the env
# inline therefore reads an EMPTY accumulator. Traced on
# `examples/workspaces/mixed`: the consumer logged `seen=<empty>` before all
# seven producers, so the mechanism had never delivered anything anywhere.
#
# WHY EVERY CORROSION TARGET AND NOT JUST THE UMBRELLA (phase-392 W5.g follow-up).
# `zpico-sys` is compiled once per CARGO ROOT, and a workspace has two: the
# synthesised umbrella (`nros_ws_runtime`) and the repo root that
# `nros_cpp-static` / `nros_c-static` import from. Measured on `mixed`: 6
# `zpico-sys` units, and only the 1 under the umbrella could ever see the env.
# A pure-C/C++ workspace is worse — `nros_synth_runtime_umbrella` returns early
# for it, so the umbrella call site does not exist and NO unit was reachable.
#
# The DEFER target is the TOP-LEVEL scope, for the reason
# `_nano_ros_support_schedule_flush` states one module over: deferring to the
# CURRENT directory fires at the end of whichever package called first, which is
# the bug rather than a smaller version of it.
# The target list travels through a GLOBAL property and the deferred call takes
# NO arguments, which is the shape `_nano_ros_support_flush` uses one module
# over. That is not a style preference: passing the name as a deferred CALL
# argument was tried first and the callee received an EMPTY string, so the
# mapper resolved nothing and `corrosion_set_env_vars` was invoked with one
# argument ("incorrect arguments for function named"). A global carries the
# value across the scope boundary intact.
function(nros_entity_facts_env_deferred _target)
    get_property(_queued GLOBAL PROPERTY NROS_ENTITY_FACTS_TARGETS)
    if("${_target}" IN_LIST _queued)
        return()
    endif()
    set_property(GLOBAL APPEND PROPERTY NROS_ENTITY_FACTS_TARGETS "${_target}")
    get_property(_scheduled GLOBAL PROPERTY NROS_ENTITY_FACTS_FLUSH_SCHEDULED)
    if(_scheduled)
        return()
    endif()
    set_property(GLOBAL PROPERTY NROS_ENTITY_FACTS_FLUSH_SCHEDULED TRUE)
    cmake_language(DEFER DIRECTORY "${CMAKE_SOURCE_DIR}"
        CALL _nros_entity_facts_flush)
endfunction()

function(_nros_entity_facts_flush)
    get_property(_targets GLOBAL PROPERTY NROS_ENTITY_FACTS_TARGETS)
    foreach(_t IN LISTS _targets)
        if(TARGET "${_t}")
            nros_entity_facts_env("${_t}")
        endif()
    endforeach()
endfunction()

# nros_entity_facts_env(<target>)
#
# Attach this configure's accumulated entity facts to a Corrosion target's cargo
# invocation. Called once, after every entry has been processed.
function(nros_entity_facts_env _target)
    get_property(_seen GLOBAL PROPERTY NROS_ENTITY_FACTS_SEEN)
    if(NOT _seen)
        # Not a warning: a pure-C workspace with no LAUNCH entry, or a
        # configure whose models are not resolved yet, has always sized the
        # table from the backend's own default and still does.
        return()
    endif()

    get_property(_param GLOBAL PROPERTY NROS_ENTITY_INFRA_PARAM)
    get_property(_lc GLOBAL PROPERTY NROS_ENTITY_INFRA_LIFECYCLE)
    if(_param AND _lc)
        set(_infra "param+lifecycle")
    elseif(_param)
        set(_infra "param")
    elseif(_lc)
        set(_infra "lifecycle")
    else()
        set(_infra "none")
    endif()
    set(_env "NROS_DECLARED_INFRA_QUERYABLES=${_infra}")

    get_property(_unknown GLOBAL PROPERTY NROS_ENTITY_SERVERS_UNKNOWN)
    get_property(_max GLOBAL PROPERTY NROS_ENTITY_SERVERS_MAX)
    if(NOT _unknown AND NOT _max STREQUAL "")
        list(APPEND _env "NROS_DECLARED_SERVICE_SERVERS=${_max}")
        set(_app "${_max} declared service server(s)")
    else()
        # issue 0973 — say what a reader can DO about it. "No model here
        # describes wiring" is true and unactionable: it reads as a resolver
        # fault, and three consumers were written against it on that reading.
        # Endpoint wiring is AUTHORED, so the line names the file that would
        # answer the question. One spelling: this is the existing status line
        # extended, not a second diagnostic beside it.
        set(_app "application count undeclared — no model here describes wiring")
        string(APPEND _app "; to declare it, author")
        string(APPEND _app " <bringup>/launch/<stem>.contract.yaml beside")
        string(APPEND _app " <stem>.launch.xml (RFC-0060)")
    endif()

    if(NOT COMMAND corrosion_set_env_vars)
        message(FATAL_ERROR "nros_entity_facts_env(${_target}): Corrosion not loaded")
    endif()
    # issue 0657 — attach to the target the cargo command actually READS.
    nros_corrosion_env_target("${_target}" _target)
    corrosion_set_env_vars(${_target} ${_env})
    message(STATUS
        "nano-ros: queryable table sized from the declaration — "
        "infrastructure ${_infra}, ${_app} (phase-392 W5)")
endfunction()
