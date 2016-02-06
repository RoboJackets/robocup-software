# include all sub directories
add_subdirectory(src-ctrl)
add_subdirectory(src-kckr)

# only include the FPGA things if we're running something
# unix related
if(${UNIX})
    add_subdirectory(src-fpga)
else()
    message(WARNING "Skipping Xilinx configurations")
endif()

add_subdirectory(hw-test)
