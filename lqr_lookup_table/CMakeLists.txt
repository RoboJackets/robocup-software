
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Program that generates a lookup table (large array of floats) for LQR gain
# matrices based on the Robot2015ModelParams in RobotModel.cpp.  Note: this is
# put here rather than in the cpu/motion_control folder because any CMake
# commands in or under the cpu directory are compiled for the MBED, not the host
# computer.
include_directories(../firmware/robot2015/cpu/motion_control)
set(LOOKUP_TABLE_GEN_SRC
    "../firmware/robot2015/cpu/motion_control/RobotModel.cpp"
    "../firmware/robot2015/cpu/motion_control/TunableRobotParameters.cpp"
    "LqrLookupTableGenerator.cpp")
add_executable(lqr-lookup-table-gen ${LOOKUP_TABLE_GEN_SRC})

# only build lookup table generator if needed
set_target_properties(lqr-lookup-table-gen PROPERTIES EXCLUDE_FROM_ALL TRUE)

# put the binary under the /build directory instead of /run
set_target_properties(lqr-lookup-table-gen PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
