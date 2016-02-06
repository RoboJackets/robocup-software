# include all sub directories
add_subdirectory(src-ctrl)
add_subdirectory(src-kckr)

# only include the FPGA things if we're running something
# unix related, but not something apple related
if(${UNIX} AND NOT ${APPLE})
    add_subdirectory(src-fpga)
else()
    message(WARNING "Skipping Xilinx configurations")
endif()

add_subdirectory(hw-test)
