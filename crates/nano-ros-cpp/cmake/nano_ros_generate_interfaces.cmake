# nano_ros_generate_interfaces.cmake
#
# CMake macro for generating C++ message and service types from .msg and .srv files.
#
# Usage:
#   find_package(nano_ros_cpp REQUIRED)
#
#   nano_ros_generate_interfaces(${PROJECT_NAME}
#     "msg/MyMessage.msg"
#     "srv/MyService.srv"
#     DEPENDENCIES std_msgs geometry_msgs
#   )
#
#   target_link_libraries(my_target
#     nano_ros_cpp::nano_ros_cpp
#     ${PROJECT_NAME}::msg  # For messages
#     ${PROJECT_NAME}::srv  # For services
#   )

# Find Python interpreter
find_package(Python3 COMPONENTS Interpreter REQUIRED)

# Get the path to the generator script
get_filename_component(NANO_ROS_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" DIRECTORY)
set(NANO_ROS_GENERATOR_SCRIPT "${NANO_ROS_CMAKE_DIR}/../scripts/nano_ros_generate_cpp.py")

# Helper function to convert CamelCase to snake_case
function(_nano_ros_to_snake_case INPUT OUTPUT_VAR)
    string(REGEX REPLACE "([A-Z])" "_\\1" RESULT ${INPUT})
    string(TOLOWER ${RESULT} RESULT)
    string(REGEX REPLACE "^_" "" RESULT ${RESULT})
    set(${OUTPUT_VAR} ${RESULT} PARENT_SCOPE)
endfunction()

# Main interface generation function
function(nano_ros_generate_interfaces TARGET_NAME)
    # Parse arguments
    cmake_parse_arguments(ARG "" "" "DEPENDENCIES" ${ARGN})

    # Remaining arguments are interface files (.msg and .srv)
    set(INTERFACE_FILES ${ARG_UNPARSED_ARGUMENTS})

    if(NOT INTERFACE_FILES)
        message(FATAL_ERROR "nano_ros_generate_interfaces: No interface files specified")
    endif()

    # Output directory for generated files
    set(OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/nano_ros_generated")

    # Separate message and service files
    set(MSG_FILES "")
    set(SRV_FILES "")

    foreach(INTERFACE_FILE ${INTERFACE_FILES})
        get_filename_component(FILE_EXT ${INTERFACE_FILE} EXT)
        if(FILE_EXT STREQUAL ".msg")
            list(APPEND MSG_FILES ${INTERFACE_FILE})
        elseif(FILE_EXT STREQUAL ".srv")
            list(APPEND SRV_FILES ${INTERFACE_FILE})
        else()
            message(WARNING "nano_ros_generate_interfaces: Unknown file type: ${INTERFACE_FILE}")
        endif()
    endforeach()

    # =========================================================================
    # Process message files
    # =========================================================================
    set(MSG_GENERATED_HEADERS "")
    if(MSG_FILES)
        set(MSG_OUTPUT_DIR "${OUTPUT_DIR}/${TARGET_NAME}/msg")
        file(MAKE_DIRECTORY ${MSG_OUTPUT_DIR})

        foreach(MSG_FILE ${MSG_FILES})
            # Get absolute path
            if(NOT IS_ABSOLUTE ${MSG_FILE})
                set(MSG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/${MSG_FILE}")
            endif()

            if(NOT EXISTS ${MSG_FILE})
                message(FATAL_ERROR "nano_ros_generate_interfaces: Message file not found: ${MSG_FILE}")
            endif()

            get_filename_component(MSG_NAME ${MSG_FILE} NAME_WE)
            _nano_ros_to_snake_case(${MSG_NAME} HEADER_NAME)
            set(HEADER_NAME "${HEADER_NAME}.hpp")
            set(OUTPUT_HEADER "${MSG_OUTPUT_DIR}/${HEADER_NAME}")
            list(APPEND MSG_GENERATED_HEADERS ${OUTPUT_HEADER})

            add_custom_command(
                OUTPUT ${OUTPUT_HEADER}
                COMMAND ${Python3_EXECUTABLE}
                    ${NANO_ROS_GENERATOR_SCRIPT}
                    ${MSG_FILE}
                    ${OUTPUT_DIR}
                    --package ${TARGET_NAME}
                DEPENDS ${MSG_FILE} ${NANO_ROS_GENERATOR_SCRIPT}
                COMMENT "Generating C++ message: ${TARGET_NAME}/msg/${MSG_NAME}"
                VERBATIM
            )
        endforeach()

        # Create custom target for message generation
        add_custom_target(${TARGET_NAME}_msg_gen DEPENDS ${MSG_GENERATED_HEADERS})

        # Create INTERFACE library for messages
        add_library(${TARGET_NAME}_msg INTERFACE)
        add_library(${TARGET_NAME}::msg ALIAS ${TARGET_NAME}_msg)
        add_dependencies(${TARGET_NAME}_msg ${TARGET_NAME}_msg_gen)

        target_include_directories(${TARGET_NAME}_msg
            INTERFACE $<BUILD_INTERFACE:${OUTPUT_DIR}>
        )

        if(TARGET nano_ros_cpp::nano_ros_cpp)
            target_link_libraries(${TARGET_NAME}_msg INTERFACE nano_ros_cpp::nano_ros_cpp)
        endif()
    endif()

    # =========================================================================
    # Process service files
    # =========================================================================
    set(SRV_GENERATED_HEADERS "")
    if(SRV_FILES)
        set(SRV_OUTPUT_DIR "${OUTPUT_DIR}/${TARGET_NAME}/srv")
        file(MAKE_DIRECTORY ${SRV_OUTPUT_DIR})

        foreach(SRV_FILE ${SRV_FILES})
            # Get absolute path
            if(NOT IS_ABSOLUTE ${SRV_FILE})
                set(SRV_FILE "${CMAKE_CURRENT_SOURCE_DIR}/${SRV_FILE}")
            endif()

            if(NOT EXISTS ${SRV_FILE})
                message(FATAL_ERROR "nano_ros_generate_interfaces: Service file not found: ${SRV_FILE}")
            endif()

            get_filename_component(SRV_NAME ${SRV_FILE} NAME_WE)
            _nano_ros_to_snake_case(${SRV_NAME} HEADER_NAME)
            set(HEADER_NAME "${HEADER_NAME}.hpp")
            set(OUTPUT_HEADER "${SRV_OUTPUT_DIR}/${HEADER_NAME}")
            list(APPEND SRV_GENERATED_HEADERS ${OUTPUT_HEADER})

            add_custom_command(
                OUTPUT ${OUTPUT_HEADER}
                COMMAND ${Python3_EXECUTABLE}
                    ${NANO_ROS_GENERATOR_SCRIPT}
                    ${SRV_FILE}
                    ${OUTPUT_DIR}
                    --package ${TARGET_NAME}
                DEPENDS ${SRV_FILE} ${NANO_ROS_GENERATOR_SCRIPT}
                COMMENT "Generating C++ service: ${TARGET_NAME}/srv/${SRV_NAME}"
                VERBATIM
            )
        endforeach()

        # Create custom target for service generation
        add_custom_target(${TARGET_NAME}_srv_gen DEPENDS ${SRV_GENERATED_HEADERS})

        # Create INTERFACE library for services
        add_library(${TARGET_NAME}_srv INTERFACE)
        add_library(${TARGET_NAME}::srv ALIAS ${TARGET_NAME}_srv)
        add_dependencies(${TARGET_NAME}_srv ${TARGET_NAME}_srv_gen)

        target_include_directories(${TARGET_NAME}_srv
            INTERFACE $<BUILD_INTERFACE:${OUTPUT_DIR}>
        )

        if(TARGET nano_ros_cpp::nano_ros_cpp)
            target_link_libraries(${TARGET_NAME}_srv INTERFACE nano_ros_cpp::nano_ros_cpp)
        endif()
    endif()

    # =========================================================================
    # Handle dependencies (for both msg and srv)
    # =========================================================================
    foreach(DEP ${ARG_DEPENDENCIES})
        if(TARGET ${DEP}::msg)
            if(MSG_FILES)
                target_link_libraries(${TARGET_NAME}_msg INTERFACE ${DEP}::msg)
            endif()
            if(SRV_FILES)
                target_link_libraries(${TARGET_NAME}_srv INTERFACE ${DEP}::msg)
            endif()
        elseif(TARGET nano_ros_${DEP})
            # Standard messages bundled with nano_ros_cpp
            if(MSG_FILES)
                target_link_libraries(${TARGET_NAME}_msg INTERFACE nano_ros_${DEP})
            endif()
            if(SRV_FILES)
                target_link_libraries(${TARGET_NAME}_srv INTERFACE nano_ros_${DEP})
            endif()
        else()
            message(WARNING "nano_ros_generate_interfaces: Dependency '${DEP}' not found")
        endif()
    endforeach()

    # Store the lists for other CMake code to use
    set(${TARGET_NAME}_MSG_HEADERS ${MSG_GENERATED_HEADERS} PARENT_SCOPE)
    set(${TARGET_NAME}_SRV_HEADERS ${SRV_GENERATED_HEADERS} PARENT_SCOPE)

endfunction()
