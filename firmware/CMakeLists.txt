# This sets up MBED all MBED libraries, then sets up the robot and base station
# firmware, which depend on the MBED libs

# Set the mbed target processor and settings for cmake
set(MBED_TARGET LPC1768)
set(MBED_USE_RTOS true)
set(MBED_USE_NET false)
set(MBED_USE_USB false)
set(MBED_USE_DSP false)

# The name of the toolchain we're using.  Note: we don't actually set the CMake
# toolchain here because it would require that all files compiled under this
# CMakeLists file to be compiled for the MBED, which we don't want.  We want out
# tests and other things to be compiled for the host computer.
set(MBED_TOOLCHAIN TOOLCHAIN_GCC_ARM)

# set the path to a bash script for moving files to the mbed's USB memory
#set(MBED_COPY_SCRIPT ${PROJECT_SOURCE_DIR}/util/mbed-copy.sh)
set(MBED_COPY_SCRIPT ${PROJECT_SOURCE_DIR}/util/mbed/mbed-load.sh)
# set(HW_TEST_SCRIPT ${PROJECT_SOURCE_DIR}/util/mbed/mbed-prgm.sh)
set(HW_TEST_SCRIPT ${MBED_COPY_SCRIPT})

# sets many variables and paths related to building for the MBED
set(MBED_ENV_SETUP_FILE ${CMAKE_CURRENT_LIST_DIR}/mbed/arm_mbed.cmake)
include(${MBED_ENV_SETUP_FILE})
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
#ignore formatting flags for printf
set(MBED_CMAKE_CXX_FLAGS "${MBED_CMAKE_CXX_FLAGS} -Wno-format")
# Use filenames relative to this directory for logging output paths
set(MBED_CMAKE_CXX_FLAGS "${MBED_CMAKE_CXX_FLAGS} -D__FILE_NAME__='\"$(subst ${CMAKE_CURRENT_LIST_DIR}/,,$(basename $(abspath $<)))\"'")

# suppress -rdynamic flag
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "")

# header files
include_directories("common2015/drivers" )
include_directories("common2015/drivers/CC1101")
include_directories("common2015/modules" )
include_directories("common2015/modules/CommLink")
include_directories("common2015/modules/CommModule")
include_directories("common2015/utils/logger")

# Add a test runner target "test-firmware"
set(FIRMWARE_TEST_SRC
    "common2015/ExampleTest.cpp"
)
add_executable(test-firmware ${FIRMWARE_TEST_SRC})
target_link_libraries(test-firmware ${GTEST_BOTH_LIBRARIES})
add_dependencies(test-firmware googletest)

# Don't build the tests by default
set_target_properties(test-firmware PROPERTIES EXCLUDE_FROM_ALL TRUE)

# build robot and base station firmware and the library that they depend on
add_subdirectory(mbed)
add_subdirectory(common2015)
add_subdirectory(base2015)
add_subdirectory(robot2015)
