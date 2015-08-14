###############################################################################
################################ LOCAL VARIABLES ##############################
###############################################################################
# Device specifics
set(DEV_PARTNUM xc3s100etq144-4)

# names are limited to 8.3 filename types since we need on-board access to this during runtime
# lookup the limitations of an 8.3 filename before changing the output bitfile's name if that's ever needed
set(OUTPUT_FILENAME_BASE rj-fpga)

 # These are set with the above
set(BIT_OUT_FILENAME ${OUTPUT_FILENAME_BASE}.nib)
set(XILINX_PROJ_FILENAME ${OUTPUT_FILENAME_BASE}.prj)

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
# Move the constrain file to where we'll call the Xilinx command from
add_custom_target(copy_ucf
    ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/robocup.ucf ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Copying UCF file to xflow working directory"
)

# Find all of the verilog files and create a project file that we can pass into xflow
file(GLOB_RECURSE verilog_SRC "*.v" ${CMAKE_CURRENT_SOURCE_DIR}/src)
# Make sure the constraint file is part of the project file
list(APPEND verilog_SRC ${CMAKE_CURRENT_SOURCE_DIR}/robocup.ucf)
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/${XILINX_PROJ_FILENAME} "EOF")
#foreach(srcFile ${verilog_SRC})
    file(APPEND ${CMAKE_CURRENT_BINARY_DIR}/${XILINX_PROJ_FILENAME} "verilog work \"robocup.v\"\r\n")
#endforeach()

# target for synthesizing the Verilog to a bitstream file that can be used to configure the fpga
add_custom_target(fpga2015bit
    ${XILINX_TOOLS_DIR}/xflow
        -g CMAKE_SCRIPT
        -wd ${CMAKE_CURRENT_BINARY_DIR}
        -p  ${DEV_PARTNUM}
        -synth xst_verilog
        -implement fast_runtime
        -config bitgen
        #${CMAKE_CURRENT_BINARY_DIR}/${XILINX_PROJ_FILENAME}
        ${CMAKE_CURRENT_SOURCE_DIR}/src/robocup.v
    COMMENT "Using Xilinx xflow to generate the FPGA's bitstream from Verilog files"
    DEPENDS copy_ucf
)

# The bitstream file contains extra header information that we don't need when configuring the fpga, so we convert to the binary format
# which can be used for configuration directly.
# The -b flag instructs promgen to swap the bit order, which is required for slave serial mode.  See this forum post for more info:
# http://dbaspot.com/arch/340642-spartan3e-slave-serial-daisy-chain-print.html
add_custom_target(fpga2015bin
    ${XILINX_TOOLS_DIR}/promgen
        -w -b -p bin -u 0 ${CMAKE_CURRENT_BINARY_DIR}/robocup.bit -o ${CMAKE_CURRENT_BINARY_DIR}/robocup.bin
    DEPENDS fpga2015bit
    COMMENT "Creating bin file from bitstream"
)

# MBED tries to load the most recent .bin file as its main program, so we can't name our fpga file with a .bin extension
add_custom_target(fpga2015nib
    cp ${CMAKE_CURRENT_BINARY_DIR}/robocup.bin ${PROJECT_SOURCE_DIR}/run/${BIT_OUT_FILENAME}
    COMMENT "Copying '.bin' to '.nib' b/c '.bin' is a reserved extension on the mbed."
    DEPENDS fpga2015bin
)
add_custom_target(fpga2015  # An alias back to the fpga2015bin target
    DEPENDS fpga2015nib
)
set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES ${PROJECT_SOURCE_DIR}/run/${BIT_OUT_FILENAME})

# target to copy the output fpga bitstream to the mbed
add_custom_target(fpga2015-prog
    COMMAND ${MBED_COPY_SCRIPT} ${PROJECT_SOURCE_DIR}/run/${BIT_OUT_FILENAME}
    DEPENDS fpga2015nib
    COMMENT COMMENT "Copying the FPGA's bitfile over to the mbed"
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
    " *  Device:\t\t${DEV_PARTNUM}\n"
    " *  Timestamp:\t${_TIMESTAMP}"
    " */  \n\n"
    "`define GIT_VERSION_HASH 8'h${GIT_HASH_8}\n"
    "`define GIT_VERSION_DIRTY 1'b${GIT_DIRTY}\n"
)
