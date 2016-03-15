
include(../mbed/mbed_executable.cmake)

# set the ARM toolchain
include(${ARM_TOOLCHAIN_FILE})

# Set compiler and linker flags
set(CMAKE_CXX_FLAGS ${MBED_CMAKE_CXX_FLAGS})
set(CMAKE_C_FLAGS ${MBED_CMAKE_C_FLAGS})
set(CMAKE_EXE_LINKER_FLAGS ${MBED_CMAKE_EXE_LINKER_FLAGS})

# firmware source files
set(base2015_SRC "main.cpp")

rj_add_mbed_executable(base2015 ${base2015_SRC})
target_link_libraries(base2015 common2015)
