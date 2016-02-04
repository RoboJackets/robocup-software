# include all sub directories
add_subdirectory(src-ctrl)
add_subdirectory(src-kckr)

if( CMAKE_HOST_UNIX )
    add_subdirectory(src-fpga)
else()
    message(STATUS "Skipping Xilinx configurations")
endif()

add_subdirectory(hw-test)
