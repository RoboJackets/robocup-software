
# look for installed Xilinx tools in /opt/Xilinx
message(STATUS "Searching for Xilinx tools in /opt/Xilinx")
file(GLOB XILINX_VERSION_DIRS
    /opt/Xilinx/*
)
set(XILINX_VERSION_DIRS ${XILINX_VERSION_DIRS}) # convert XILINX_VERSION_DIRS from a string to a list


# log a warning if no Xilinx versions could be found, otherwise set the directory to the tools
list(LENGTH XILINX_VERSION_DIRS AVAILABLE_XILINX_VERSIONS_COUNT)
if(AVAILABLE_XILINX_VERSIONS_COUNT EQUAL 0)
    message(WARNING "Unable to find Xilinx tools.  Install the ISE WebPack to /opt/Xilinx to synthesize Verilog for the fpga")
else()
    # sort the available versions and take the last one (the highest version)
    list(SORT XILINX_VERSION_DIRS)
    list(GET XILINX_VERSION_DIRS -1 XILINX_VERSION_DIR)
    set(XILINX_TOOLS_DIR "${XILINX_VERSION_DIR}/ISE_DS/ISE/bin/lin64")

    message(STATUS "Found Xilinx tools at ${XILINX_TOOLS_DIR}")
endif()


add_custom_target(copy_ucf
    ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/robocup.ucf ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Copying ucf file to xflow working directory")

# target for synthesizing the Verilog to a bitstream file that can be used to configure the fpga
add_custom_target(fpga2015bit
    ${XILINX_TOOLS_DIR}/xflow
        -wd ${CMAKE_CURRENT_BINARY_DIR}
        -p xc3s100etq144-4
        -synth xst_verilog
        -implement fast_runtime
        -config bitgen ${CMAKE_CURRENT_SOURCE_DIR}/robocup.v
    COMMENT "Using xflow to generate fpga bitstream from Verilog files"
    DEPENDS copy_ucf)

add_custom_target(fpga2015bin
    ${XILINX_TOOLS_DIR}/promgen
        -w -b -p bin -u 0 ${CMAKE_CURRENT_BINARY_DIR}/robocup.bit -o ${CMAKE_CURRENT_BINARY_DIR}/robocup.bin
    DEPENDS fpga2015bit
    COMMENT "Creating bin file from bitstream"
)

add_custom_target(fpga2015nib
    cp ${CMAKE_CURRENT_BINARY_DIR}/robocup.bin ${CMAKE_CURRENT_BINARY_DIR}/robocup.nib
    COMMENT "Copying .bin to .nib b/c .nib is a reserved extension on MBED"
    DEPENDS fpga2015bin
)

# target to copy the output fpga bitstream to the mbed
add_custom_target(fpga2015-prog
    COMMAND ${PROJECT_SOURCE_DIR}/util/mbed-copy.sh ${CMAKE_CURRENT_BINARY_DIR}/robocup.nib
    DEPENDS fpga2015nib
    COMMENT "Copies the fpga bitstream file to the mbed"
)
