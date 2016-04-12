# wrapper around add_executable for adding mbed executables
function(rj_add_mbed_executable name)
    set(binfile ${PROJECT_SOURCE_DIR}/run/${name}.bin)

    add_executable(${name} ${ARGN})
    _rj_configure_mbed_binary(${name})

    # the final product is the .bin file, not the elf one.  We hide this away in the build dir
    set_target_properties(${name} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

    # custom target for creating a .bin file from an elf binary
    add_custom_target(${name}-bin
        arm-none-eabi-objcopy -O binary ${name} ${binfile} # todo: rename binfile
        DEPENDS ${name}
        COMMENT "objcopying to make mbed-compatible executable"
    )
    set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES ${binfile})

    add_custom_target(${name}-prog
        COMMAND ${MBED_COPY_SCRIPT} ${binfile}
        DEPENDS ${name}-bin
        COMMENT "Copying the robot's binary over to the mbed"
    )
endfunction(rj_add_mbed_executable)


function(rj_add_mbed_library name)
    add_library(${name} ${ARGN})
    _rj_configure_mbed_binary(${name})
endfunction(rj_add_mbed_library)


function(rj_add_external_mbed_library)
    set(oneValueArgs NAME HG_REPO HG_TAG)
    set(multiValueArgs SRCS INCLUDE_DIRS)
    cmake_parse_arguments(arg "" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    # Specify source repository
    set(extproj ${arg_NAME}_ext_proj)
    ExternalProject_Add(${extproj}
        HG_REPOSITORY ${arg_HG_REPO}
        HG_TAG ${arg_HG_TAG}
        CONFIGURE_COMMAND   ""
        BUILD_COMMAND       ""
        INSTALL_COMMAND     ""
        UPDATE_COMMAND      ""
    )
    set_target_properties(${extproj} PROPERTIES EXCLUDE_FROM_ALL TRUE)

    # Get the download directory
    ExternalProject_Get_Property(${extproj} SOURCE_DIR)

    # convert source files to absolute paths
    set(lib_srcs "")
    foreach(src ${arg_SRCS})
        list(APPEND lib_srcs ${SOURCE_DIR}/${src})
    endforeach()

    # convert include dirs to absolute paths
    set(${arg_LIBNAME}_INCLUDES "")
    foreach(include ${arg_INCLUDE_DIRS})
        list(APPEND ${arg_LIBNAME}_INCLUDES ${SOURCE_DIR}/${include})
    endforeach()

    # Specify that each source file depends on the external project
    foreach(src ${lib_srcs})
        add_custom_command(
            OUTPUT ${src}
            DEPENDS ${extproj}
        )
    endforeach()

    rj_add_mbed_library(${arg_NAME} ${lib_srcs})
    target_include_directories(${arg_NAME} PUBLIC ${${arg_LIBNAME}_INCLUDES})
endfunction(rj_add_external_mbed_library)

function(rj_mbed_env)
    set(CMAKE_CXX_FLAGS ${MBED_CMAKE_CXX_FLAGS} PARENT_SCOPE)
    set(CMAKE_C_FLAGS ${MBED_CMAKE_C_FLAGS} PARENT_SCOPE)
    set(CMAKE_CXX_FLAGS ${MBED_CMAKE_CXX_FLAGS} PARENT_SCOPE)
    set(CMAKE_EXE_LINKER_FLAGS ${MBED_CMAKE_EXE_LINKER_FLAGS} PARENT_SCOPE)

    # Include the arm toolchain for gcc
    set(CMAKE_TOOLCHAIN_FILE ${ARM_TOOLCHAIN_FILE} PARENT_SCOPE)
    include(${ARM_TOOLCHAIN_FILE})
endfunction(mbed_env)

function(_rj_configure_mbed_binary name)
    # only build this if specifically instructed
    set_target_properties(${name} PROPERTIES EXCLUDE_FROM_ALL TRUE)

    # link and include official mbed libs
    target_include_directories(${name} PUBLIC ${MBED_INCLUDE_DIR})
    add_dependencies(${name} mbed_libraries)
endfunction()
