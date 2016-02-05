# This sets up MBED all MBED libraries, then sets up the robot and base station
# firmware, which depend on the MBED libs

# Set the mbed target processor and settings for cmake
# set(MBED_TARGET lpc1768)

set(MBED_USE_RTOS true)
set(MBED_USE_ETH false)
set(MBED_USE_USB false)
set(MBED_USE_DSP false)
set(MBED_USE_RPC true)

# The name of the toolchain we're using.  Note: we don't actually set the CMake
# toolchain here because it would require that all files compiled under this
# CMakeLists file to be compiled for the MBED, which we don't want.  We want out
# tests and other things to be compiled for the host computer.
set(MBED_TOOLCHAIN TOOLCHAIN_GCC_ARM)

# set the path to a bash script for moving files to the mbed's USB storage
if( CMAKE_HOST_APPLE )
    set(MBED_COPY_SCRIPT ${PROJECT_SOURCE_DIR}/util/mbed/mbed-copy-osx.sh)
else()
    set(MBED_COPY_SCRIPT ${PROJECT_SOURCE_DIR}/util/mbed/mbed-copy.sh)
endif()

# sets many variables and paths related to building for the MBED
include(${CMAKE_CURRENT_LIST_DIR}/mbed/mbed_util.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/mbed/arm_mbed.cmake)

# Note: the arm_mbed.cmake script exports flags that need to be set in each CMake
# file used to build MBED code like so:
# set(CMAKE_CXX_FLAGS ${MBED_CMAKE_CXX_FLAGS})
# set(CMAKE_C_FLAGS ${MBED_CMAKE_C_FLAGS})
# set(CMAKE_EXE_LINKER_FLAGS ${MBED_CMAKE_EXE_LINKER_FLAGS})

# enable C++ exceptions
set(MBED_CMAKE_CXX_FLAGS "${MBED_CMAKE_CXX_FLAGS} -fexceptions")
# enable C++11
set(MBED_CMAKE_CXX_FLAGS "${MBED_CMAKE_CXX_FLAGS} -std=c++11")
# Enable logging for the common2015 library
set(MBED_CMAKE_CXX_FLAGS "${MBED_CMAKE_CXX_FLAGS} -DRJ_LOGGING_EN")
# ignore formatting flags for printf
set(MBED_CMAKE_CXX_FLAGS "${MBED_CMAKE_CXX_FLAGS} -Wno-format")
# generate debugging information for gdb
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")

# suppress -rdynamic flag
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "")

# Add a test runner target "test-firmware"
set(FIRMWARE_TEST_SRC
    "common2015/ExampleTest.cpp"
)

# test firmware target
add_executable(test-firmware ${FIRMWARE_TEST_SRC})
add_dependencies(test-firmware googletest)
target_link_libraries(test-firmware ${GTEST_LIBRARIES})

# header files
target_include_directories(test-firmware PUBLIC
    "common2015/drivers"
    "common2015/drivers/cc1101"
    "common2015/drivers/cc1201"
    "common2015/drivers/cc1201/ti"
    "common2015/drivers/cc1201/cfg/default"
    "common2015/drivers/rtos-i2c"
    "common2015/drivers/software-spi"
    "common2015/modules"
    "common2015/modules/CommLink"
    "common2015/modules/CommModule"
    "common2015/modules/Console"
    "common2015/utils/logger"
    "common2015/utils/assert"
    "common2015/utils/numparser"
    "common2015/utils/rtos-mgmt"
)

# Don't build the tests by default
set_target_properties(test-firmware PROPERTIES EXCLUDE_FROM_ALL TRUE)

# build robot and base station firmware and the library that they depend on
add_subdirectory(mbed)
add_subdirectory(common2015)
add_subdirectory(base2015)
add_subdirectory(robot2015)
