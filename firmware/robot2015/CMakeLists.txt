# include all sub directories
add_subdirectory(src-ctrl)
add_subdirectory(src-kckr)
add_subdirectory(hw-test)

# only include the FPGA things if we're running something
# unix related, but not something apple related
if( UNIX AND NOT APPLE )
    add_subdirectory(src-fpga)
else()
    message(WARNING "Skipping Xilinx configurations")
endif()
