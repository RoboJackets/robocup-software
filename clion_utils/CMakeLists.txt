cmake_minimum_required(VERSION 3.14)
project(ros2_master)


# Add subdirectories to the project so clion can do clangd things
function(colcon_add_subdirectories)
    include(ExternalProject)

    cmake_parse_arguments(PARSE_ARGV 0 "ARG" "" "BUILD_BASE;BASE_PATHS" "")

    message("search criteria: ${ARGV}")

    execute_process(COMMAND colcon list
            --paths-only
            --base-paths ${ARG_BASE_PATHS}
            --topological-order
            ${ARG_UNPARSED_ARGUMENTS}
            OUTPUT_VARIABLE paths)

    string(STRIP "${paths}" paths)
    string(REPLACE "\n" ";" paths "${paths}")

    MESSAGE("colcon shows paths ${paths}")

    foreach (path IN LISTS paths)
        message("...examining ${path}")

        execute_process(
                COMMAND colcon info --paths "${path}"
                OUTPUT_VARIABLE package_info
                WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        )

        if (NOT "${package_info}" MATCHES "type:[ \t]+(cmake|ros.ament_cmake|ros.cmake)")
            message("skipping non-cmake project")
        elseif (NOT "${package_info}" MATCHES "name:[ \t]+([^ \r\n\t]*)")
            message(WARNING "could not identify package at ${path}")
        else ()
            set(name "${CMAKE_MATCH_1}")
            message("...adding package ${name} from path ${path}")

            string(REGEX MATCH "build: ([A-Za-z0-9_ ]*)" _ "${package_info}")
            set(raw_dependencies_str "${CMAKE_MATCH_1}")

            # Convert the space-delimited list to a string
            string(REPLACE " " ";" raw_dependencies_list ${raw_dependencies_str})

            message("\n>>>> processed_packages: ${processed_packages}")
            message(">>>> raw_dependencies_list: ${raw_dependencies_list}")
            unset(dependencies)
            foreach (dependency IN LISTS raw_dependencies_list)
                if (${dependency} IN_LIST processed_packages)
                    list(APPEND dependencies ${dependency})
                endif()
            endforeach ()

            message(">>>> dependencies: ${dependencies}")

            get_filename_component(BUILD_PATH "${name}" ABSOLUTE BASE_DIR "${ARG_BUILD_BASE}")

            ExternalProject_Add(${name}
                    SOURCE_DIR ${path}
                    PREFIX ${name}
                    INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/${name}
                    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> -DCMAKE_PREFIX_PATH=${CMAKE_CURRENT_BINARY_DIR}
                    DEPENDS "${dependencies}"
                    )

            # Add to processed_packages
            list(APPEND processed_packages ${name})
        endif ()
    endforeach ()
endfunction()

set(ROS2_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../")

colcon_add_subdirectories(
        BUILD_BASE "${PROJECT_SOURCE_DIR}/../build"
        BASE_PATHS "${ROS2_SRC_DIR}/robocup-software/"
        --packages-select
        rj_robocup_cmake_utils
        rj_robocup_protobuf
        geometry2d
        geometry2d_msgs
)