# this file came from https://github.com/uwearzt/mbed-cmake
# and was modified by Justin Buchanan
# The end result of this file is the 'robot2015' target
# building this target ouputs an MBED-runnable file at run/robot2015.bin, which can be copied onto the MBED

# set variables to the names of the directories for each section of code
set(DRIVERS_ROOT_DIR drivers)
set(MODULES_ROOT_DIR modules)
set(UTILS_ROOT_DIR utils)
set(USR_LIB_DIR lib)
set(USR_CONFIG_DIR config)

# set variables for each subdirectory of the above sections of the firmware
set(DRIVERS ${DRIVERS_ROOT_DIR})
set(MODULES ${MODULES_ROOT_DIR})
set(UTILS ${UTILS_ROOT_DIR})

list(APPEND ${DRIVERS} adc cc1101 cc1201 cc1201/cfg dma ds2411 fpga radio)
# list(APPEND ${DRIVERS} adc cc1101 cc1201 cc1201/cfg dma ds2411 fpga radio speaker)
list(APPEND ${MODULES} CommLink CommModule console system)
list(APPEND ${UTILS} assert firmware-check logger rtos-mgmt watchdog)
set(SUB_DIRS_LIST ${DRIVERS} ${MODULES} ${UTILS} ${USR_CONFIG_DIR})

# generate the list of all directories that are needed
message("\n===== Adding firmware directories =====")
set(total_subdirs 0)
foreach(sub_dirs ${SUB_DIRS_LIST})
    # get the name of the directory root for each specific section of firmware
    list(GET sub_dirs 0 root_dir_name)
    list(REMOVE_ITEM sub_dirs root_dir_name)
    message(STATUS "adding '${root_dir_name}'")
    include_directories(${root_dir_name})
    MATH( EXPR total_subdirs "${total_subdirs} + 1")

    # loop through all subdirectories for each section
    foreach(sub_dir_name ${${sub_dirs}})
        message(STATUS "       '${root_dir_name}/${sub_dir_name}'")
        include_directories(${root_dir_name}/${sub_dir_name})
        MATH( EXPR total_subdirs "${total_subdirs} + 1")
    endforeach()
    message(STATUS "")
endforeach()
message("done. ${total_subdirs} total directories added.\n=======================================\n")
unset(total_subdirs)

# set the arm toolchain
message("\n======== Adding ARM toolchain =========")
set(CMAKE_TOOLCHAIN_FILE arm_toolchain.cmake)
include(lib/arm_toolchain.cmake)
message("done.\n=======================================\n")

# suppress -rdynamic flag
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "")

# set the target processor and include settings for cmake
set(MBED_TARGET LPC1768)
#set(MBED_TARGET LPC11U24)
#set(MBED_TARGET RBLAB_NRF51822)
set(PREBUILD_DEPS)

include(lib/arm_mbed.cmake)

#all official libraries pulled with/by the build tools
include(ExternalProject)
ExternalProject_Add(mbed_lib_build_tools
    GIT_REPOSITORY https://github.com/mbedmicro/mbed.git
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
    UPDATE_COMMAND ""
)
set(PREBUILD_DEPS ${PREBUILD_DEPS} mbed_lib_build_tools)

#pull DIO breakout
if(${BUILD_MCP23017} STREQUAL "true")
    ExternalProject_Add(mcp23017
        HG_REPOSITORY https://developer.mbed.org/users/Nurbol/code/MCP23017/
        HG_TAG default
        CONFIGURE_COMMAND ""
        BUILD_COMMAND ""
        INSTALL_COMMAND ""
        UPDATE_COMMAND ""
    )
    set(PREBUILD_DEPS ${PREBUILD_DEPS} mcp23017)
endif()

# don't get the external project unless it's required
set_target_properties(mbed_lib_build_tools PROPERTIES EXCLUDE_FROM_ALL TRUE)

# firmware source files
file(GLOB_RECURSE robot2015_SRC
    "*.cpp"
)

message("\n======== Finding source files =========")
foreach(full_path_file ${robot2015_SRC})
    get_filename_component(filename ${full_path_file} NAME)
    message(STATUS "found '${filename}'")
endforeach(full_path_file)
unset(listLen)
list(LENGTH robot2015_SRC listLen)
message("done. ${listLen} total source files found.\n=======================================\n")

add_executable(robot2015_elf ${robot2015_SRC} ${CMAKE_BINARY_DIR}/common/git_version.cpp ${MBED_OBJECTS})
target_link_libraries(robot2015_elf ${MBED_LIBS})

# the final product is the .bin file, not the elf one.  We hide this away in the build dir
set_target_properties(robot2015_elf PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

# only build robot firmware if specifically instructed
set_target_properties(robot2015_elf PROPERTIES EXCLUDE_FROM_ALL TRUE)

#pre-build target will pull and compile mbed libs
#this is added as a dependency for standard builds, forcing the python script to
#run at the correct time
add_custom_target(pre-build
    COMMAND python2 ${PY_TOOLS_DIR}/workspace_tools/build.py --mcu=${PY_MCU_OPT} --tool=${PY_TOOLCHAIN_OPT} ${PY_LIBS}
    DEPENDS ${PREBUILD_DEPS})
set_target_properties(pre-build PROPERTIES EXCLUDE_FROM_ALL TRUE)

# tell CMake that the obj files all come from the ExternalProject
# otherwise it'll complain that the files can't be found
foreach(mbed_obj ${MBED_OBJECTS})
    add_custom_command(
        OUTPUT ${mbed_obj}
	DEPENDS pre-build
        COMMAND "")
endforeach()

# custom target for creating a .bin file from an elf binary
add_custom_target(robot2015
    arm-none-eabi-objcopy -O binary robot2015_elf robot2015.bin
    COMMAND cp ${CMAKE_CURRENT_BINARY_DIR}/robot2015.bin ${PROJECT_SOURCE_DIR}/run/
    DEPENDS robot2015_elf
    COMMENT "objcopying to make MBED-compatible executable"
)
set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES ${PROJECT_SOURCE_DIR}/run/robot2015.bin)

add_custom_target(robot2015-prog
    arm-none-eabi-objcopy -O binary robot2015_elf robot2015.bin
    COMMAND ${PROJECT_SOURCE_DIR}/util/robot2015-prog.sh ${PROJECT_SOURCE_DIR}/run/robot2015.bin
    DEPENDS robot2015
    COMMENT "Copies the files created by robot2015 into a single mbed"
)


# See common/CMakeLists for more info on how this works
add_custom_command(
    OUTPUT ${CMAKE_BINARY_DIR}/common/git_version.cpp
    DEPENDS git_version
)

message("========== System Overview ============")
message("=======================================")
message("Project Name:\t\t\t${CMAKE_PROJECT_NAME}")
message("CMake Version:\t\t\t${CMAKE_VERSION}")
message("Compilation System:\t\t${CMAKE_SYSTEM}")
message("Compilation System Type:\t${CMAKE_SYSTEM_NAME}")
message("Compilation Processor:\t\t${CMAKE_SYSTEM_PROCESSOR}")
message("=======================================\n")
