###############################################################################
################################ LOCAL VARIABLES ##############################
###############################################################################
# Device specifics
set(DEV_PARTNUM xc3s100etq144-4)

# names are limited to 8.3 filename types since we need on-board access to this during runtime
# lookup the limitations of an 8.3 filename before changing the output bitfile's name if that's ever needed
set(OUTPUT_FILENAME_BASE rj-fpga)
set(TOP_LEVEL_NAME robocup)

 # These are set with the above
set(BIN_OUT_FILENAME ${OUTPUT_FILENAME_BASE}.bin)
set(NIB_OUT_FILENAME ${OUTPUT_FILENAME_BASE}.nib)
set(BIT_OUT_FILENAME ${TOP_LEVEL_NAME}.bit)
set(UCF_IN_FILENAME  ${TOP_LEVEL_NAME}.ucf)

# This is the root path to anything Xilinx related
set(XILINX_SYSTEM_ROOT /opt/Xilinx)


###############################################################################
########################## FIND PATH TO ISE WEBPACK ###########################
###############################################################################
# look for installed Xilinx tools in /opt/Xilinx
message(STATUS "Searching for Xilinx tools in '${XILINX_SYSTEM_ROOT}'")
file(GLOB XILINX_VERSION_DIRS
    ${XILINX_SYSTEM_ROOT}/*
)

# Pick out the folders that look like a floating point formatted value
string(REGEX MATCHALL
    "${XILINX_SYSTEM_ROOT}/(([0-9]+)?[\\.]).*"
    XILINX_VERSION_DIRS
    "${XILINX_VERSION_DIRS}"
)

# Remove anything that isn't a directory
foreach(TMPVAR ${XILINX_VERSION_DIRS})
    if(NOT((IS_DIRECTORY ${TMPVAR}) AND (EXISTS ${TMPVAR})))
        list(REMOVE_ITEM XILINX_VERSION_DIRS ${TMPVAR})
    endif()
endforeach()

# Find the highester version 
set(XILINX_ROOT "0.0")
foreach(TMPVAR ${XILINX_VERSION_DIRS})
    get_filename_component(VERSION_FOUND ${TMPVAR} NAME)

    if(${VERSION_FOUND} VERSION_GREATER ${XILINX_ROOT})
        set(XILINX_ROOT ${TMPVAR})
    endif()
endforeach()

# log a warning if no Xilinx versions could be found, otherwise set the directory to the tools
list(LENGTH XILINX_VERSION_DIRS AVAILABLE_XILINX_VERSIONS_COUNT)

# show what versions were found and what version we're using
if(AVAILABLE_XILINX_VERSIONS_COUNT EQUAL 0)
    message(WARNING
        "Unable to find Xilinx tools.  Install the ISE WebPACK (14.7) to \
        '${XILINX_SYSTEM_ROOT}' to synthesize Verilog for the FPGA. \
        http://www.xilinx.com/support/download/index.html/content/xilinx/en/downloadNav/design-tools.html.")
else()
    # print out some version info
    if(${AVAILABLE_XILINX_VERSIONS_COUNT} EQUAL 1)
        message(STATUS "${AVAILABLE_XILINX_VERSIONS_COUNT} version of Xilinx tools found:")
    else()
        message(STATUS "${AVAILABLE_XILINX_VERSIONS_COUNT} versions of Xilinx tools found:")
    endif()
    
    foreach(VERSION_NUM ${XILINX_VERSION_DIRS})
        message(STATUS "  Found '${VERSION_NUM}'")
    endforeach()

    list(GET XILINX_VERSION_DIRS 0 XILINX_VERSION_DIR)
    set(XILINX_TOOLS_DIR "${XILINX_ROOT}/ISE_DS/ISE/bin/lin64")

    message(STATUS "Using '${XILINX_ROOT}'\n")
endif()


###############################################################################
################################ FPGA TARGETS #################################
###############################################################################
# Gather a list of all of the project sources
file(GLOB_RECURSE verilog_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/*.v)
# Make sure the constraint file is part of the project file
list(APPEND verilog_SRC ${CMAKE_CURRENT_SOURCE_DIR}/${UCF_IN_FILENAME})
list(LENGTH verilog_SRC NUM_SRCS)

set(NEED_TO_SYNTHESIZE FALSE)
set(COMPRESS_BINARY FALSE)

add_custom_target(copy_src)
foreach(srcFile ${verilog_SRC})
    get_filename_component(SRC_BASENAME ${srcFile} NAME)
    set(srcBuildFile ${CMAKE_CURRENT_BINARY_DIR}/${SRC_BASENAME})

    add_custom_command(
        TARGET copy_src PRE_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${srcFile} ${srcBuildFile}
        COMMENT "Checking for newer versions of ${SRC_BASENAME}"
        depends ${CMAKE_CURRENT_BINARY_DIR}/git_version.vh
    )

    # check if there is a new version for each of the source files - set a flag if anything has changed
    if(${srcFile} IS_NEWER_THAN ${srcBuildFile})
        set(NEED_TO_SYNTHESIZE TRUE)
    endif()
endforeach()

# target for synthesizing the Verilog to a bitstream file that can be used to configure the fpga
add_custom_target(fpga2015bit
    ${XILINX_TOOLS_DIR}/xflow
        -g CMAKE_SCRIPT
        -wd ${CMAKE_CURRENT_BINARY_DIR}
        -p  ${DEV_PARTNUM}
        -synth xst_verilog
        -implement fast_runtime
        # -implement balanced.opt
        # -implement high_effort.opt
        -config bitgen
        ${TOP_LEVEL_NAME}
    COMMENT "Using Xilinx xflow to generate the FPGA's bitstream from ${NUM_SRCS} Verilog files"
    DEPENDS copy_src
)

# The bitstream file contains extra header information that we don't need when configuring the fpga, so we convert to the binary format
# which can be used for configuration directly.
# The -b flag instructs promgen to swap the bit order, which is required for slave serial mode.  See this forum post for more info:
# http://dbaspot.com/arch/340642-spartan3e-slave-serial-daisy-chain-print.html
set(PROGMEN_FLAGS)
if(${COMPRESS_BINARY})
    set(PROGMEN_FLAGS -z ${PROGMEN_FLAGS})
endif()
add_custom_target(fpga2015bin
    ${XILINX_TOOLS_DIR}/promgen ${PROGMEN_FLAGS} -w -b -p bin -u 0 ${CMAKE_CURRENT_BINARY_DIR}/${BIT_OUT_FILENAME} -o ${CMAKE_CURRENT_BINARY_DIR}/${BIN_OUT_FILENAME}
    COMMENT "Creating ${BIN_OUT_FILENAME} binary from the ${BIT_OUT_FILENAME} bitstream"
    DEPENDS fpga2015bit
)

# only synthesize if we need to
set(FPGA_TARGET_DEPENDS copy_src)
if(${NEED_TO_SYNTHESIZE})
    list(APPEND FPGA_TARGET_DEPENDS fpga2015bin)
else()
    message(STATUS "No new or modified source files found for synthesis")
endif()

# MBED tries to load the most recent .bin file as its main program, so we can't name our fpga file with a .bin extension
add_custom_target(fpga2015
    cp ${CMAKE_CURRENT_BINARY_DIR}/${BIN_OUT_FILENAME} ${PROJECT_SOURCE_DIR}/run/${NIB_OUT_FILENAME}
    COMMENT "Copying ${BIN_OUT_FILENAME} to ${NIB_OUT_FILENAME} b/c *.bin is a reserved extension on the mbed"
    DEPENDS ${FPGA_TARGET_DEPENDS} ${CMAKE_CURRENT_BINARY_DIR}/git_version.vh
)
set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES ${PROJECT_SOURCE_DIR}/run/${NIB_OUT_FILENAME})

# target to copy the output fpga bitstream to the mbed
add_custom_target(fpga2015-prog
    COMMAND sudo ${MBED_COPY_SCRIPT} ${PROJECT_SOURCE_DIR}/run/${NIB_OUT_FILENAME}
    COMMENT COMMENT "Copying ${NIB_OUT_FILENAME} (bitstream file) to the mbed"
    DEPENDS fpga2015
)


###############################################################################
###################### PROVIDE GIT HASH FOR SYNTHESIS #########################
###############################################################################
# Write git_version.vh into the build dir containing the first 8 characters of
# the current git hash and a boolean for whether or not the repo is "dirty"
execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse HEAD
    OUTPUT_VARIABLE GIT_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
string(SUBSTRING ${GIT_HASH} 0 8 GIT_HASH_8)

execute_process(
    COMMAND ${GIT_EXECUTABLE} diff --quiet
    RESULT_VARIABLE GIT_DIRTY
)

execute_process(
    COMMAND date
    OUTPUT_VARIABLE _TIMESTAMP
)

file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/git_version.vh
    "/*  Git version information - auto-generated by CMake during synthesis.\n"
    " *  \n"
    " *  Device:\t${DEV_PARTNUM}\n"
    " *  Timestamp:\t${_TIMESTAMP}"
    " */  \n\n"
    "`define GIT_VERSION_HASH\t160'h${GIT_HASH}\n"
    "`define GIT_VERSION_HASH_32\t32'h${GIT_HASH_8}\n"
    "`define GIT_VERSION_DIRTY\t1'b${GIT_DIRTY}\n"
)
