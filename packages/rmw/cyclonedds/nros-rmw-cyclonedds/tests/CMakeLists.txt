# Phase 117.3: minimal smoke harness — verifies that the backend's
# register entry point links and returns NROS_RMW_RET_OK against a
# stub `nros_rmw_cffi_register` symbol.
#
# 117.12 swaps the stub for a real link against nros-rmw-cffi + adds
# pub/sub interop tests against a stock `rmw_cyclonedds` peer.

add_executable(nros_rmw_cyclonedds_register_smoke register_smoke.cpp)
target_link_libraries(nros_rmw_cyclonedds_register_smoke
    PRIVATE
        nros_rmw_cyclonedds
)
target_include_directories(nros_rmw_cyclonedds_register_smoke
    PRIVATE
        ${NROS_RMW_CFFI_DIR}
)

# Prepend the in-tree Cyclone install dir so ld.so picks our
# pinned `libddsc.so` over any `ros-humble-cyclonedds` already on
# LD_LIBRARY_PATH (sourced by .envrc).
set(_nros_rmw_test_env
    "LD_LIBRARY_PATH=${CMAKE_PREFIX_PATH}/lib:$ENV{LD_LIBRARY_PATH}")

add_test(
    NAME nros_rmw_cyclonedds_register_smoke
    COMMAND nros_rmw_cyclonedds_register_smoke
)
set_tests_properties(nros_rmw_cyclonedds_register_smoke PROPERTIES
    ENVIRONMENT "${_nros_rmw_test_env}")

# Phase 117.4: session_open / session_close round-trip.
add_executable(nros_rmw_cyclonedds_session_smoke session_smoke.cpp)
target_link_libraries(nros_rmw_cyclonedds_session_smoke
    PRIVATE
        nros_rmw_cyclonedds
)
target_include_directories(nros_rmw_cyclonedds_session_smoke
    PRIVATE
        ${NROS_RMW_CFFI_DIR}
)
add_test(
    NAME nros_rmw_cyclonedds_session_smoke
    COMMAND nros_rmw_cyclonedds_session_smoke
)

# Phase 117.2 / 117.5: IDL → descriptor → registry → topic round-trip.
# In the in-tree build the type-support helper hasn't been installed
# yet; load it from the source tree directly.
include("${CMAKE_CURRENT_SOURCE_DIR}/../cmake/NrosRmwCycloneddsTypeSupport.cmake")

nros_rmw_cyclonedds_idlc_compile(_test_string_sources
    IDL_FILE  "${CMAKE_CURRENT_SOURCE_DIR}/types/test_string.idl"
    OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/types"
    TYPE_NAME "nros_test::msg::TestString"
)

add_executable(nros_rmw_cyclonedds_codegen_smoke
    codegen_smoke.cpp
    ${_test_string_sources}
)
target_link_libraries(nros_rmw_cyclonedds_codegen_smoke
    PRIVATE
        nros_rmw_cyclonedds
)
target_include_directories(nros_rmw_cyclonedds_codegen_smoke
    PRIVATE
        ${NROS_RMW_CFFI_DIR}
        "${CMAKE_CURRENT_BINARY_DIR}/types"
)
add_test(
    NAME nros_rmw_cyclonedds_codegen_smoke
    COMMAND nros_rmw_cyclonedds_codegen_smoke
)

# Phase 117.6 publisher / subscriber entity-creation smoke. Reuses
# the test_string descriptor from the codegen smoke above.
add_executable(nros_rmw_cyclonedds_pubsub_smoke
    pubsub_smoke.cpp
    ${_test_string_sources}
)
target_link_libraries(nros_rmw_cyclonedds_pubsub_smoke
    PRIVATE
        nros_rmw_cyclonedds
)
target_include_directories(nros_rmw_cyclonedds_pubsub_smoke
    PRIVATE
        ${NROS_RMW_CFFI_DIR}
        "${CMAKE_CURRENT_BINARY_DIR}/types"
)
add_test(
    NAME nros_rmw_cyclonedds_pubsub_smoke
    COMMAND nros_rmw_cyclonedds_pubsub_smoke
)

# Issue 0780 — polled status events, provoked for real (a 50 ms deadline and a
# publisher that stops). Reuses the test_string descriptor above.
add_executable(nros_rmw_cyclonedds_status_events
    status_events.cpp
    ${_test_string_sources}
)
target_link_libraries(nros_rmw_cyclonedds_status_events
    PRIVATE
        nros_rmw_cyclonedds
)
target_include_directories(nros_rmw_cyclonedds_status_events
    PRIVATE
        ${NROS_RMW_CFFI_DIR}
        "${CMAKE_CURRENT_BINARY_DIR}/types"
)
add_test(
    NAME nros_rmw_cyclonedds_status_events
    COMMAND nros_rmw_cyclonedds_status_events
)

# phase-393 W2 — matched counts and the publisher GID. Asserts the count MOVES
# when a reader appears, which a stub returning 0 could not fake.
add_executable(nros_rmw_cyclonedds_graph_counts
    graph_counts.cpp
    ${_test_string_sources}
)
target_link_libraries(nros_rmw_cyclonedds_graph_counts PRIVATE nros_rmw_cyclonedds)
target_include_directories(nros_rmw_cyclonedds_graph_counts
    PRIVATE
        ${NROS_RMW_CFFI_DIR}
        "${CMAKE_CURRENT_BINARY_DIR}/types"
)
add_test(
    NAME nros_rmw_cyclonedds_graph_counts
    COMMAND nros_rmw_cyclonedds_graph_counts
)

# Issue 0823 — the QoS read-back. Pre-loads the out-struct with values the
# entity cannot have, so an echoing or zeroing implementation is visible.
add_executable(nros_rmw_cyclonedds_actual_qos
    actual_qos.cpp
    ${_test_string_sources}
)
target_link_libraries(nros_rmw_cyclonedds_actual_qos
    PRIVATE
        nros_rmw_cyclonedds
)
target_include_directories(nros_rmw_cyclonedds_actual_qos
    PRIVATE
        ${NROS_RMW_CFFI_DIR}
        "${CMAKE_CURRENT_BINARY_DIR}/types"
)
add_test(
    NAME nros_rmw_cyclonedds_actual_qos
    COMMAND nros_rmw_cyclonedds_actual_qos
)

# Issue 0800 — the Cyclone `set_log_severity` body, asserted by reading the mask
# back rather than trusting the return code. No session or router needed.
add_executable(nros_rmw_cyclonedds_log_severity
    log_severity.cpp
)
target_link_libraries(nros_rmw_cyclonedds_log_severity
    PRIVATE
        nros_rmw_cyclonedds
)
target_include_directories(nros_rmw_cyclonedds_log_severity
    PRIVATE
        ${NROS_RMW_CFFI_DIR}
)
add_test(
    NAME nros_rmw_cyclonedds_log_severity
    COMMAND nros_rmw_cyclonedds_log_severity
)

# Phase 117.6.B raw-CDR data round-trip via publish_raw / take_serialized.
add_executable(nros_rmw_cyclonedds_data_roundtrip
    data_roundtrip.cpp
    ${_test_string_sources}
)
target_link_libraries(nros_rmw_cyclonedds_data_roundtrip
    PRIVATE
        nros_rmw_cyclonedds
)
target_include_directories(nros_rmw_cyclonedds_data_roundtrip
    PRIVATE
        ${NROS_RMW_CFFI_DIR}
        "${CMAKE_CURRENT_BINARY_DIR}/types"
)
add_test(
    NAME nros_rmw_cyclonedds_data_roundtrip
    COMMAND nros_rmw_cyclonedds_data_roundtrip
)

# Issue 0971 — the batch drain reports the count it earned, and the NEXT take
# reports why it stopped. First coverage `take_sequence` has in this backend.
add_executable(nros_rmw_cyclonedds_take_sequence_pending_status
    take_sequence_pending_status.cpp
    ${_test_string_sources}
)
target_link_libraries(nros_rmw_cyclonedds_take_sequence_pending_status
    PRIVATE
        nros_rmw_cyclonedds
)
target_include_directories(nros_rmw_cyclonedds_take_sequence_pending_status
    PRIVATE
        ${NROS_RMW_CFFI_DIR}
        "${CMAKE_CURRENT_BINARY_DIR}/types"
)
add_test(
    NAME nros_rmw_cyclonedds_take_sequence_pending_status
    COMMAND nros_rmw_cyclonedds_take_sequence_pending_status
)

# Phase 117.X.3 service test fixtures: drive `.srv` through the
# 117.X.1 codegen helper so the typed IDLs include the in-payload
# `cdds_request_header_t` (rmw_writer_guid + rmw_sequence_number)
# layout. The two registered descriptor type names are
# `nros_test::srv::dds_::AddTwoInts_Request_` and
# `nros_test::srv::dds_::AddTwoInts_Response_`; the backend looks
# them up via `<type_name>_Request_` / `_Response_` suffix from
# the consumer-supplied `nros_test::srv::dds_::AddTwoInts`.
if(NROS_RMW_CYCLONEDDS_MSG_TO_IDL)
    nros_rmw_cyclonedds_generate_from_msg(_addtwoints_sources
        PKG_NAME nros_test
        PKG_DIR  "${CMAKE_CURRENT_SOURCE_DIR}/types/nros_test"
        INTERFACES srv/AddTwoInts.srv srv/SumSeq.srv
    )
else()
    set(_addtwoints_sources "")
endif()
if(_addtwoints_sources STREQUAL "")
    message(STATUS
        "rosidl_adapter unavailable: skipping service_smoke / "
        "service_roundtrip / service_concurrent (Phase 117.X.3 "
        "needs the codegen helper for the typed IDL).")
endif()
if(_addtwoints_sources)
    add_executable(nros_rmw_cyclonedds_service_smoke
        service_smoke.cpp
        ${_addtwoints_sources}
    )
    target_link_libraries(nros_rmw_cyclonedds_service_smoke
        PRIVATE
            nros_rmw_cyclonedds
    )
    target_include_directories(nros_rmw_cyclonedds_service_smoke
        PRIVATE
            ${NROS_RMW_CFFI_DIR}
    )
    add_test(
        NAME nros_rmw_cyclonedds_service_smoke
        COMMAND nros_rmw_cyclonedds_service_smoke
    )

    # Phase 117.7 service request/reply data round-trip — Phase
    # 117.X.3 reshapes wire to include cdds_request_header_t.
    # Issue 0969 — the codec in isolation. NOT a ctest: it is a measuring
    # instrument, and a bench with a pass/fail verdict becomes a flaky test on a
    # loaded machine. Build it, run it by hand, read ns/msg.
    # `sertype_min.{hpp,cpp}` lives HERE, not in src/, since issue 0976 retired
    # its last production user. It reconstructs the CDR round trip this bench
    # exists to price, so it is a measurement fixture for a path the backend no
    # longer takes — keeping it beside the bench says that, and stops it being
    # read as live code.
    add_executable(nros_rmw_cyclonedds_codec_bench
        codec_bench.cpp
        sertype_min.cpp
        ${_addtwoints_sources}
    )
    target_link_libraries(nros_rmw_cyclonedds_codec_bench
        PRIVATE
            nros_rmw_cyclonedds
            pthread
    )
    target_include_directories(nros_rmw_cyclonedds_codec_bench
        PRIVATE
            ${NROS_RMW_CFFI_DIR}
    )
    # The bench calls the cdrstream helpers directly, so it needs the same
    # semi-internal ddsi/ddsrt roots `sertype_min.hpp` compiles against — the
    # backend has them PRIVATE, so they do not propagate here.
    if(NROS_CYCLONEDDS_PROVENANCE STREQUAL "source")
        target_include_directories(nros_rmw_cyclonedds_codec_bench PRIVATE
            "${CYCLONEDDS_SOURCE_DIR}/src/core/ddsi/include"
            "${CYCLONEDDS_SOURCE_DIR}/src/core/ddsc/include"
            "${CYCLONEDDS_SOURCE_DIR}/src/ddsrt/include"
            "${NROS_CYCLONEDDS_SOURCE_BUILD_DIR}/src/ddsrt/include")
    endif()

    add_executable(nros_rmw_cyclonedds_service_roundtrip
        service_roundtrip.cpp
        ${_addtwoints_sources}
    )
    target_link_libraries(nros_rmw_cyclonedds_service_roundtrip
        PRIVATE
            nros_rmw_cyclonedds
            pthread
    )
    target_include_directories(nros_rmw_cyclonedds_service_roundtrip
        PRIVATE
            ${NROS_RMW_CFFI_DIR}
    )
    add_test(
        NAME nros_rmw_cyclonedds_service_roundtrip
        COMMAND nros_rmw_cyclonedds_service_roundtrip
    )

    # Issue 0778 — two requests from ONE client in flight at once. Runs by
    # default (unlike `service_concurrent`, whose flake is cross-PARTICIPANT
    # SEDP jitter): this is a single client and a single server on one
    # participant, so there is no second discovery to race.
    add_executable(nros_rmw_cyclonedds_service_two_outstanding
        service_two_outstanding.cpp
        ${_addtwoints_sources}
    )
    target_link_libraries(nros_rmw_cyclonedds_service_two_outstanding
        PRIVATE
            nros_rmw_cyclonedds
            pthread
    )
    target_include_directories(nros_rmw_cyclonedds_service_two_outstanding
        PRIVATE
            ${NROS_RMW_CFFI_DIR}
    )
    add_test(
        NAME nros_rmw_cyclonedds_service_two_outstanding
        COMMAND nros_rmw_cyclonedds_service_two_outstanding
    )

    # phase-428 W13.c — `service_server_is_available` from the DDS matched
    # sets: false with no server, true once a server on a SECOND participant
    # matches both halves, false again after it is destroyed. Polled, not
    # slept, so cross-participant SEDP jitter costs time rather than a red.
    add_executable(nros_rmw_cyclonedds_service_server_available
        service_server_available.cpp
        ${_addtwoints_sources}
    )
    target_link_libraries(nros_rmw_cyclonedds_service_server_available
        PRIVATE
            nros_rmw_cyclonedds
            pthread
    )
    target_include_directories(nros_rmw_cyclonedds_service_server_available
        PRIVATE
            ${NROS_RMW_CFFI_DIR}
    )
    add_test(
        NAME nros_rmw_cyclonedds_service_server_available
        COMMAND nros_rmw_cyclonedds_service_server_available
    )

    # Phase 117.7.B concurrent-call correlation. With Phase 117.X.3
    # this now goes through the typed-IDL header path; the
    # AddTwoInts type must be registered.
    #
    # **Disabled by default in CI** because of a structural flake in
    # Cyclone 0.10.5: cross-participant SEDP discovery jitter
    # consistently times out one of the two clients' last call. The
    # (writer_guid, seq) filter logic is functionally validated by
    # `service_roundtrip` (single-client multi-call) and the unit
    # test in `mangling_test`. Re-enable manually with
    # `-DNROS_RMW_CYCLONEDDS_RUN_SERVICE_CONCURRENT=ON` once the
    # underlying Cyclone behaviour is understood.
    add_executable(nros_rmw_cyclonedds_service_concurrent
        service_concurrent.cpp
        ${_addtwoints_sources}
    )
    target_link_libraries(nros_rmw_cyclonedds_service_concurrent
        PRIVATE
            nros_rmw_cyclonedds
            pthread
    )
    target_include_directories(nros_rmw_cyclonedds_service_concurrent
        PRIVATE
            ${NROS_RMW_CFFI_DIR}
    )
    option(NROS_RMW_CYCLONEDDS_RUN_SERVICE_CONCURRENT
           "Run the cross-client correlation test (flaky on Cyclone 0.10.5)"
           OFF)
    if(NROS_RMW_CYCLONEDDS_RUN_SERVICE_CONCURRENT)
        add_test(
            NAME nros_rmw_cyclonedds_service_concurrent
            COMMAND nros_rmw_cyclonedds_service_concurrent
        )
    endif()
endif()

# Phase 212.K.7.4.b — dynamic-type bridge smoke. Calls
# `nros_cyclonedds_build_descriptor_from_schema` directly with a
# primitive-only schema and verifies the returned descriptor passes
# `dds_create_topic`. No idlc / no codegen path — proves the
# synthesised descriptor is well-formed enough for Cyclone.
add_executable(nros_rmw_cyclonedds_dynamic_bridge_smoke
    dynamic_bridge_smoke.cpp
)
target_link_libraries(nros_rmw_cyclonedds_dynamic_bridge_smoke
    PRIVATE
        nros_rmw_cyclonedds
)
add_test(
    NAME nros_rmw_cyclonedds_dynamic_bridge_smoke
    COMMAND nros_rmw_cyclonedds_dynamic_bridge_smoke
)
set_tests_properties(nros_rmw_cyclonedds_dynamic_bridge_smoke
    PROPERTIES ENVIRONMENT "${_nros_rmw_test_env}")

# phase-303 W1c (#0267) — ROS-edition wire extensibility (FINAL vs APPENDABLE /
# DDS_OP_DLC) of generated descriptors.
add_executable(nros_rmw_cyclonedds_appendable_extensibility
    appendable_extensibility.cpp
)
target_link_libraries(nros_rmw_cyclonedds_appendable_extensibility
    PRIVATE
        nros_rmw_cyclonedds
)
add_test(
    NAME nros_rmw_cyclonedds_appendable_extensibility
    COMMAND nros_rmw_cyclonedds_appendable_extensibility
)
set_tests_properties(nros_rmw_cyclonedds_appendable_extensibility
    PROPERTIES ENVIRONMENT "${_nros_rmw_test_env}")

# Phase 212.K.7.4.c — op-word audit for sequence/array/bsq of nested
# structs plus the EXT 3-word emission fix. Asserts exact `m_ops`
# values then hands the descriptor to live Cyclone for
# `dds_stream_countops` acceptance.
add_executable(nros_rmw_cyclonedds_dynamic_bridge_seq_nested
    dynamic_bridge_seq_nested.cpp
)
target_link_libraries(nros_rmw_cyclonedds_dynamic_bridge_seq_nested
    PRIVATE
        nros_rmw_cyclonedds
)
add_test(
    NAME nros_rmw_cyclonedds_dynamic_bridge_seq_nested
    COMMAND nros_rmw_cyclonedds_dynamic_bridge_seq_nested
)
set_tests_properties(nros_rmw_cyclonedds_dynamic_bridge_seq_nested
    PROPERTIES ENVIRONMENT "${_nros_rmw_test_env}")

# Phase 212.K.7.4.d — typed-sample publisher round-trip for the
# `_FeedbackMessage_` wire-format adapter. Builds a Fibonacci-shaped
# FeedbackMessage descriptor via the dynamic bridge, publishes a
# hand-crafted nano-ros runtime payload, asserts the subscriber takes
# back the same UUID + sequence body.
add_executable(nros_rmw_cyclonedds_feedback_roundtrip
    feedback_roundtrip.cpp
)
target_link_libraries(nros_rmw_cyclonedds_feedback_roundtrip
    PRIVATE
        nros_rmw_cyclonedds
)
target_include_directories(nros_rmw_cyclonedds_feedback_roundtrip
    PRIVATE
        ${NROS_RMW_CFFI_DIR}
)
add_test(
    NAME nros_rmw_cyclonedds_feedback_roundtrip
    COMMAND nros_rmw_cyclonedds_feedback_roundtrip
)
set_tests_properties(nros_rmw_cyclonedds_feedback_roundtrip
    PROPERTIES ENVIRONMENT "${_nros_rmw_test_env}")

# Phase 117.X.2: topic-prefix helper unit test (no DDS interaction).
add_executable(nros_rmw_cyclonedds_topic_prefix_test
    topic_prefix_test.cpp
)
target_include_directories(nros_rmw_cyclonedds_topic_prefix_test
    PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/../src"
)
add_test(
    NAME nros_rmw_cyclonedds_topic_prefix_test
    COMMAND nros_rmw_cyclonedds_topic_prefix_test
)

# Phase 117.X.1: .msg → mangled IDL → idlc → registry pipeline.
# Drives the new helper through a synthetic std_msgs package; if
# the host lacks /opt/ros/humble's rosidl_adapter the helper emits
# a soft warning at configure time and we skip the test.
if(NROS_RMW_CYCLONEDDS_MSG_TO_IDL)
    nros_rmw_cyclonedds_generate_from_msg(_from_msg_sources
        PKG_NAME std_msgs
        PKG_DIR  "${CMAKE_CURRENT_SOURCE_DIR}/types/std_msgs"
        INTERFACES msg/String.msg
    )
    add_executable(nros_rmw_cyclonedds_from_msg_smoke
        from_msg_smoke.cpp
        ${_from_msg_sources}
    )
    target_link_libraries(nros_rmw_cyclonedds_from_msg_smoke
        PRIVATE
            nros_rmw_cyclonedds
    )
    target_include_directories(nros_rmw_cyclonedds_from_msg_smoke
        PRIVATE
            ${NROS_RMW_CFFI_DIR}
    )
    add_test(
        NAME nros_rmw_cyclonedds_from_msg_smoke
        COMMAND nros_rmw_cyclonedds_from_msg_smoke
    )
    set_tests_properties(nros_rmw_cyclonedds_from_msg_smoke
        PROPERTIES ENVIRONMENT "${_nros_rmw_test_env}")

    # Phase 117.12.A: nano-ros pub/sub ↔ stock rmw_cyclonedds_cpp
    # round-trip. Two binaries (publisher + subscriber) drive the
    # codegen-emitted std_msgs::msg::dds_::String_ descriptor; a
    # bash harness pairs each with a `ros2 topic echo|pub` call
    # over /opt/ros/humble. Skips cleanly when ros2 isn't on PATH.
    add_executable(nros_rmw_cyclonedds_ros2_pub
        ros2_pub.cpp
        ${_from_msg_sources}
    )
    target_link_libraries(nros_rmw_cyclonedds_ros2_pub
        PRIVATE
            nros_rmw_cyclonedds
    )
    target_include_directories(nros_rmw_cyclonedds_ros2_pub
        PRIVATE
            ${NROS_RMW_CFFI_DIR}
    )

    add_executable(nros_rmw_cyclonedds_ros2_sub
        ros2_sub.cpp
        ${_from_msg_sources}
    )
    target_link_libraries(nros_rmw_cyclonedds_ros2_sub
        PRIVATE
            nros_rmw_cyclonedds
    )
    target_include_directories(nros_rmw_cyclonedds_ros2_sub
        PRIVATE
            ${NROS_RMW_CFFI_DIR}
    )

    add_test(
        NAME nros_rmw_cyclonedds_ros2_pubsub_e2e
        COMMAND bash "${CMAKE_CURRENT_SOURCE_DIR}/ros2_pubsub_e2e.sh"
    )
    set_tests_properties(nros_rmw_cyclonedds_ros2_pubsub_e2e
        PROPERTIES
            ENVIRONMENT
                "NROS_RMW_CYCLONEDDS_PUB_BIN=$<TARGET_FILE:nros_rmw_cyclonedds_ros2_pub>;NROS_RMW_CYCLONEDDS_SUB_BIN=$<TARGET_FILE:nros_rmw_cyclonedds_ros2_sub>;${_nros_rmw_test_env}"
    )

    # Phase 117.12.B: nano-ros service server ↔ stock `ros2 service
    # call` round-trip on `example_interfaces/srv/AddTwoInts`. The
    # codegen helper emits the stock-shape mangled type
    # `example_interfaces::srv::dds_::AddTwoInts_{Request,Response}_`
    # so the wire matches `rmw_cyclonedds_cpp` byte-for-byte.
    nros_rmw_cyclonedds_generate_from_msg(_example_addtwoints_sources
        PKG_NAME example_interfaces
        PKG_DIR  "${CMAKE_CURRENT_SOURCE_DIR}/types/example_interfaces"
        INTERFACES srv/AddTwoInts.srv
    )
    if(_example_addtwoints_sources)
        add_executable(nros_rmw_cyclonedds_ros2_srv_server
            ros2_srv_server.cpp
            ${_example_addtwoints_sources}
        )
        target_link_libraries(nros_rmw_cyclonedds_ros2_srv_server
            PRIVATE
                nros_rmw_cyclonedds
        )
        target_include_directories(nros_rmw_cyclonedds_ros2_srv_server
            PRIVATE
                ${NROS_RMW_CFFI_DIR}
        )
        add_executable(nros_rmw_cyclonedds_ros2_srv_client
            ros2_srv_client.cpp
            ${_example_addtwoints_sources}
        )
        target_link_libraries(nros_rmw_cyclonedds_ros2_srv_client
            PRIVATE
                nros_rmw_cyclonedds
        )
        target_include_directories(nros_rmw_cyclonedds_ros2_srv_client
            PRIVATE
                ${NROS_RMW_CFFI_DIR}
        )
        add_test(
            NAME nros_rmw_cyclonedds_ros2_srv_e2e
            COMMAND bash "${CMAKE_CURRENT_SOURCE_DIR}/ros2_srv_e2e.sh"
        )
        set_tests_properties(nros_rmw_cyclonedds_ros2_srv_e2e
            PROPERTIES
                ENVIRONMENT
                    "NROS_RMW_CYCLONEDDS_SRV_SERVER_BIN=$<TARGET_FILE:nros_rmw_cyclonedds_ros2_srv_server>;NROS_RMW_CYCLONEDDS_SRV_CLIENT_BIN=$<TARGET_FILE:nros_rmw_cyclonedds_ros2_srv_client>;${_nros_rmw_test_env}"
        )
    endif()

    # Phase 117.X.4: codegen pipeline produces stock-RMW-shape type
    # names. Reuses the std_msgs::msg::String_ + nros_test::srv::
    # AddTwoInts_{Request,Response}_ descriptors registered by
    # _from_msg_sources and _addtwoints_sources.
    if(_addtwoints_sources)
        add_executable(nros_rmw_cyclonedds_mangling_test
            mangling_test.cpp
            ${_from_msg_sources}
            ${_addtwoints_sources}
        )
        target_link_libraries(nros_rmw_cyclonedds_mangling_test
            PRIVATE
                nros_rmw_cyclonedds
        )
        add_test(
            NAME nros_rmw_cyclonedds_mangling_test
            COMMAND nros_rmw_cyclonedds_mangling_test
        )
        set_tests_properties(nros_rmw_cyclonedds_mangling_test
            PROPERTIES ENVIRONMENT "${_nros_rmw_test_env}")
    endif()
endif()

# Apply the LD_LIBRARY_PATH prefix to every test added above.
set_tests_properties(
    nros_rmw_cyclonedds_session_smoke
    nros_rmw_cyclonedds_codegen_smoke
    nros_rmw_cyclonedds_pubsub_smoke
    nros_rmw_cyclonedds_data_roundtrip
    nros_rmw_cyclonedds_take_sequence_pending_status
    PROPERTIES
        ENVIRONMENT "${_nros_rmw_test_env}"
)
if(_addtwoints_sources)
    set_tests_properties(
        nros_rmw_cyclonedds_service_smoke
        nros_rmw_cyclonedds_service_roundtrip
        nros_rmw_cyclonedds_service_server_available
        PROPERTIES
            ENVIRONMENT "${_nros_rmw_test_env}"
    )
    if(NROS_RMW_CYCLONEDDS_RUN_SERVICE_CONCURRENT)
        set_tests_properties(nros_rmw_cyclonedds_service_concurrent
            PROPERTIES ENVIRONMENT "${_nros_rmw_test_env}")
    endif()
endif()

# Concurrency test for the striped addrset locks (nano-ros issue 0496).
#
# C, not C++ like its siblings: ddsi_domaingv.h has `typedef struct
# os_sockWaitset *os_sockWaitset`, legal C and a conflicting declaration in C++.
#
# Calls cyclone's INTERNAL addrset API (q_addrset.h is not installed), so it can
# only be built when cyclone was self-provisioned from source and the `ddsc`
# target is in this build. Inheriting ddsc's INCLUDE_DIRECTORIES is what makes
# the internal + generated headers reachable without restating any of them here.
# When cyclone came from an installed package there is nothing to include
# against, and the test is skipped LOUDLY rather than silently dropped.
if(TARGET ddsc)
    add_executable(nros_rmw_cyclonedds_addrset_striped_lock_concurrency
        addrset_striped_lock_concurrency.c
    )
    target_link_libraries(nros_rmw_cyclonedds_addrset_striped_lock_concurrency
        PRIVATE
            ddsc
            pthread
    )
    target_include_directories(nros_rmw_cyclonedds_addrset_striped_lock_concurrency
        PRIVATE
            $<TARGET_PROPERTY:ddsc,INCLUDE_DIRECTORIES>
    )
    add_test(
        NAME nros_rmw_cyclonedds_addrset_striped_lock_concurrency
        COMMAND nros_rmw_cyclonedds_addrset_striped_lock_concurrency
    )
    # The failure mode under test is a deadlock, so cap it: the test's own
    # watchdog should fire first and explain itself, and this is the backstop if
    # the hang happens somewhere the watchdog cannot see.
    set_tests_properties(nros_rmw_cyclonedds_addrset_striped_lock_concurrency
        PROPERTIES
            TIMEOUT 300
            ENVIRONMENT "${_nros_rmw_test_env}"
    )
else()
    message(STATUS
        "nano-ros: SKIPPING nros_rmw_cyclonedds_addrset_striped_lock_concurrency — "
        "no in-build `ddsc` target, so cyclone's internal addrset headers are not "
        "reachable (issue 0496 regression cover is NOT running in this configuration)")
endif()

# Issue 1011 — the ONLY C++11 translation unit in the tree, deliberately.
#
# The Zephyr lane builds this backend with `-std=c++11`, where a class carrying
# default member initialisers is not an aggregate (C++14 relaxed that). Two such
# initialisers on `NrosCdrBlob` broke `NrosCdrBlob{data, len}` in publisher.cpp
# and service.cpp, failing six zephyr cyclonedds leaves while every other lane —
# all C++14 or later — stayed green.
#
# Pinning C++11 here is the whole point: nothing else in a normal `just check`
# compiles this header at that standard, so without this target a re-added
# initialiser is invisible until a Zephyr fixture build fails. Header-only, so
# it links the backend only to inherit CycloneDDS' include dirs — the header
# pulls in `dds/dds.h`, so it is not standalone.
#
# Only THIS target is C++11; the backend it links stays on its own standard,
# which is the point: the test proves the header is consumable at C++11 without
# dragging the rest of the tree back to it.
add_executable(nros_rmw_cyclonedds_sertype_sample_cxx11 sertype_sample_cxx11.cpp)
set_target_properties(nros_rmw_cyclonedds_sertype_sample_cxx11 PROPERTIES
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF)
target_link_libraries(nros_rmw_cyclonedds_sertype_sample_cxx11
    PRIVATE nros_rmw_cyclonedds)
target_include_directories(nros_rmw_cyclonedds_sertype_sample_cxx11
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src)
# `nros_sertype.hpp` reaches INTERNAL Cyclone headers (`dds/ddsi/ddsi_sertype.h`),
# which the backend adds PRIVATE — so they do not propagate to a consumer and
# have to be mirrored here. Same condition the backend guards them with.
if(NROS_CYCLONEDDS_PROVENANCE STREQUAL "source")
    target_include_directories(nros_rmw_cyclonedds_sertype_sample_cxx11 PRIVATE
        "${CYCLONEDDS_SOURCE_DIR}/src/core/ddsi/include"
        "${CYCLONEDDS_SOURCE_DIR}/src/core/ddsc/include"
        "${CYCLONEDDS_SOURCE_DIR}/src/ddsrt/include"
        "${NROS_CYCLONEDDS_SOURCE_BUILD_DIR}/src/ddsrt/include")
endif()

add_test(
    NAME nros_rmw_cyclonedds_sertype_sample_cxx11
    COMMAND nros_rmw_cyclonedds_sertype_sample_cxx11
)

# phase-206 W1 — an override must not cost the baked baseline.
#
# Composes `cyclone_config.hpp`'s baseline with a user fragment that states only
# <Discovery>, brings a domain up on it, and reads the RESOLVED settings back out
# of Cyclone's own config dump. Carries its own negative control (the user
# fragment alone, which is what the old three-way ternary passed), so the
# assertions cannot be satisfied by Cyclone's defaults.
#
# Defines NROS_RMW_CYCLONEDDS_TEST_BASELINE via the source, so the host build
# composes the REAL baseline string rather than a copy that could drift from it.
add_executable(nros_rmw_cyclonedds_config_compose cyclone_config_compose.cpp)
target_link_libraries(nros_rmw_cyclonedds_config_compose
    PRIVATE
        nros_rmw_cyclonedds
)
target_include_directories(nros_rmw_cyclonedds_config_compose
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../src
        ${NROS_RMW_CFFI_DIR}
)
add_test(
    NAME nros_rmw_cyclonedds_config_compose
    COMMAND nros_rmw_cyclonedds_config_compose
)
set_tests_properties(nros_rmw_cyclonedds_config_compose PROPERTIES
    ENVIRONMENT "${_nros_rmw_test_env}")

# phase-206 W2 — the bringup's own config is baked and reaches the backend.
#
# The test drives the REAL generator (`nros_bake_rmw_user_config`) against a
# fixture laid out exactly as a bringup would be — `<bringup>/rmw/cyclonedds.xml`
# — so a change to the generator's output shape fails here rather than only on
# an embedded image nobody builds locally.
include(${CMAKE_CURRENT_SOURCE_DIR}/../../../../../cmake/NanoRosRmwUserConfig.cmake)
set(_w2_bringup ${CMAKE_CURRENT_BINARY_DIR}/w2_bringup)
file(MAKE_DIRECTORY ${_w2_bringup}/rmw)
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/fixtures/bringup_cyclonedds.xml
    ${_w2_bringup}/rmw/cyclonedds.xml
    COPYONLY)
nros_bake_rmw_user_config(
    BRINGUP ${_w2_bringup}
    BACKEND cyclonedds
    OUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/w2_gen
    RESULT  _w2_header)
if(NOT _w2_header)
    message(FATAL_ERROR
        "phase-206 W2: the bake produced no header for a fixture that exists — "
        "the test would then assert against an empty config and pass vacuously.")
endif()

add_executable(nros_rmw_cyclonedds_user_config cyclone_user_config.cpp)
target_link_libraries(nros_rmw_cyclonedds_user_config PRIVATE nros_rmw_cyclonedds)
target_include_directories(nros_rmw_cyclonedds_user_config
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../src
        ${CMAKE_CURRENT_BINARY_DIR}/w2_gen   # the generated header goes FIRST-class
        ${NROS_RMW_CFFI_DIR}
)
# The fixture path, so the test compares the baked bytes against the file they
# came from rather than against a copy of itself.
target_compile_definitions(nros_rmw_cyclonedds_user_config
    PRIVATE NROS_W2_FIXTURE_PATH="${_w2_bringup}/rmw/cyclonedds.xml")
add_test(NAME nros_rmw_cyclonedds_user_config COMMAND nros_rmw_cyclonedds_user_config)
set_tests_properties(nros_rmw_cyclonedds_user_config PROPERTIES
    ENVIRONMENT "${_nros_rmw_test_env}")
