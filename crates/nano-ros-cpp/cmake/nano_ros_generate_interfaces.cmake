# nano_ros_generate_interfaces.cmake
#
# CMake macro for generating C++ message types from .msg files.
#
# Usage:
#   find_package(nano_ros_cpp REQUIRED)
#
#   nano_ros_generate_interfaces(${PROJECT_NAME}
#     "msg/MyMessage.msg"
#     "msg/OtherMessage.msg"
#     DEPENDENCIES std_msgs geometry_msgs
#   )
#
#   target_link_libraries(my_target
#     nano_ros_cpp::nano_ros_cpp
#     ${PROJECT_NAME}::msg
#   )

# Find Python interpreter
find_package(Python3 COMPONENTS Interpreter REQUIRED)

# Get the path to the generator script
get_filename_component(NANO_ROS_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" DIRECTORY)
set(NANO_ROS_GENERATOR_SCRIPT "${NANO_ROS_CMAKE_DIR}/../scripts/nano_ros_generate_cpp.py")

# Main interface generation function
function(nano_ros_generate_interfaces TARGET_NAME)
    # Parse arguments
    cmake_parse_arguments(ARG "" "" "DEPENDENCIES" ${ARGN})

    # Remaining arguments are message files
    set(MSG_FILES ${ARG_UNPARSED_ARGUMENTS})

    if(NOT MSG_FILES)
        message(FATAL_ERROR "nano_ros_generate_interfaces: No message files specified")
    endif()

    # Output directory for generated files
    set(OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/nano_ros_generated")
    set(MSG_OUTPUT_DIR "${OUTPUT_DIR}/${TARGET_NAME}/msg")

    # Create output directory
    file(MAKE_DIRECTORY ${MSG_OUTPUT_DIR})

    # Collect generated header files
    set(GENERATED_HEADERS "")

    foreach(MSG_FILE ${MSG_FILES})
        # Get absolute path to message file
        if(NOT IS_ABSOLUTE ${MSG_FILE})
            set(MSG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/${MSG_FILE}")
        endif()

        # Check that file exists
        if(NOT EXISTS ${MSG_FILE})
            message(FATAL_ERROR "nano_ros_generate_interfaces: Message file not found: ${MSG_FILE}")
        endif()

        # Get message name from filename
        get_filename_component(MSG_NAME ${MSG_FILE} NAME_WE)

        # Convert MessageName to message_name.hpp
        string(REGEX REPLACE "([A-Z])" "_\\1" HEADER_NAME ${MSG_NAME})
        string(TOLOWER ${HEADER_NAME} HEADER_NAME)
        string(REGEX REPLACE "^_" "" HEADER_NAME ${HEADER_NAME})
        set(HEADER_NAME "${HEADER_NAME}.hpp")

        set(OUTPUT_HEADER "${MSG_OUTPUT_DIR}/${HEADER_NAME}")
        list(APPEND GENERATED_HEADERS ${OUTPUT_HEADER})

        # Add custom command to generate the header
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

    # Create a custom target for the generated headers
    add_custom_target(${TARGET_NAME}_msg_gen
        DEPENDS ${GENERATED_HEADERS}
    )

    # Create INTERFACE library for the generated messages
    add_library(${TARGET_NAME}_msg INTERFACE)
    add_library(${TARGET_NAME}::msg ALIAS ${TARGET_NAME}_msg)

    # Add dependency on generation target
    add_dependencies(${TARGET_NAME}_msg ${TARGET_NAME}_msg_gen)

    # Set include directories
    target_include_directories(${TARGET_NAME}_msg
        INTERFACE
            $<BUILD_INTERFACE:${OUTPUT_DIR}>
    )

    # Link to nano_ros_cpp for CdrWriter/CdrReader
    if(TARGET nano_ros_cpp::nano_ros_cpp)
        target_link_libraries(${TARGET_NAME}_msg
            INTERFACE nano_ros_cpp::nano_ros_cpp
        )
    endif()

    # Handle dependencies (other message packages)
    foreach(DEP ${ARG_DEPENDENCIES})
        if(TARGET ${DEP}::msg)
            target_link_libraries(${TARGET_NAME}_msg
                INTERFACE ${DEP}::msg
            )
        elseif(TARGET nano_ros_${DEP})
            # Standard messages bundled with nano_ros_cpp
            target_link_libraries(${TARGET_NAME}_msg
                INTERFACE nano_ros_${DEP}
            )
        else()
            message(WARNING "nano_ros_generate_interfaces: Dependency '${DEP}' not found")
        endif()
    endforeach()

    # Store the list of generated headers for other CMake code to use
    set(${TARGET_NAME}_MSG_HEADERS ${GENERATED_HEADERS} PARENT_SCOPE)

endfunction()
