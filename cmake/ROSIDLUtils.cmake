#
# Add the include directories and libraries from an interface generation target
# in the current project and a specific type support to another target.
#
# It also adds target dependencies to `target` to ensure the interface
# generation happens before.
#
# This is used in the same way as target_link_libraries, ie.
#
# rosidl_target_link_interfaces(<target> <|PRIVATE|PUBLIC|INTERFACE> <interface>...)
#
function(rosidl_target_link_interfaces target)
    set(options PUBLIC PRIVATE INTERFACE)
    set(one_value_args )
    set(multi_value_args )
    cmake_parse_arguments(PARSED "${options}"
        "${one_value_args}" "${multi_value_args}" ${ARGN})

    message("target: ${target}")
    message("PARSED_PUBLIC: ${PARSED_PUBLIC}")
    message("PARSED_PRIVATE: ${PARSED_PRIVATE}")
    message("PARSED_INTERFACE: ${PARSED_INTERFACE}")
    message("PARSED_UNPARSED_ARGUMENTS: ${PARSED_UNPARSED_ARGUMENTS}")

    if(NOT TARGET ${target})
        message(FATAL_ERROR "rosidl_target_link_interfaces() the first argument '${target}' must be a valid target name")
    endif()
    if(NOT PARSED_UNPARSED_ARGUMENTS)
        message(FATAL_ERROR "rosidl_target_link_interfaces() nothing was passed to link to")
    endif()

    # Hardcode this to be rosidl_typesupport_cpp
    set(typesupport_name rosidl_typesupport_cpp)

    foreach(interface_target ${PARSED_UNPARSED_ARGUMENTS})
        if(NOT TARGET ${interface_target})
            message(FATAL_ERROR "rosidl_target_link_interfaces() the argn '${interface_target}' must be a valid target name")
        endif()

        set(typesupport_target "${interface_target}__${typesupport_name}")


        if(NOT TARGET ${typesupport_target})
            message(FATAL_ERROR
                "rosidl_target_interfaces() the second argument '${interface_target}' "
                "concatenated with the third argument '${typesupport_name}' "
                "using double underscores must be a valid target name")
        endif()

        set(visibility "")
        if(${PARSED_PUBLIC})
            set(visibility "PUBLIC")
        elseif(${PARSED_PRIVATE})
            set(visibility "PRIVATE")
        elseif(${PARSED_INTERFACE})
            set(visibility "INTERFACE")
        endif()

        add_dependencies(${target} ${interface_target})
        get_target_property(include_directories ${typesupport_target} INTERFACE_INCLUDE_DIRECTORIES)
        target_include_directories(${target} PUBLIC ${include_directories})
        target_link_libraries(${target} ${visibility} ${typesupport_target})
    endforeach()
endfunction()
