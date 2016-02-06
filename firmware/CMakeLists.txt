# This sets up MBED all MBED libraries, then sets up the robot and base station
# firmware, which depend on the MBED libs
include(RemoveFlags)
include(ShowVars)

# remove the color flag since it will get set on anything with clang assert
# the default compiler (mostly apple computers). The arm compilers know
# nothing about this flag however.
show_vars()
remove_cxx_flag("-fcolor-diagnostics")
show_vars()

# sets many variables and paths related to building for the MBED
include(${CMAKE_CURRENT_LIST_DIR}/mbed/arm_mbed.cmake)

# set the path to a script for moving files to the mbed's USB storage
if( UNIX AND NOT APPLE )
    set(MBED_COPY_SCRIPT "${PROJECT_SOURCE_DIR}/util/mbed/mbed-copy.sh")
elseif( APPLE )
    set(MBED_COPY_SCRIPT "${PROJECT_SOURCE_DIR}/util/mbed/mbed-copy-osx.sh")
else()
    message(WARNING "unsupported host system for mbed-copy usage")
    set(MBED_COPY_SCRIPT "")
endif()

# Note: the arm_mbed.cmake script exports flags that need to be set in each CMake
# file used to build MBED code like so:
# set(CMAKE_CXX_FLAGS ${MBED_CMAKE_CXX_FLAGS})
# set(CMAKE_C_FLAGS ${MBED_CMAKE_C_FLAGS})
# set(CMAKE_EXE_LINKER_FLAGS ${MBED_CMAKE_EXE_LINKER_FLAGS})

# enable C++ exceptions
set(MBED_CMAKE_CXX_FLAGS "${MBED_CMAKE_CXX_FLAGS} -fexceptions")
# enable C++11
# set(MBED_CMAKE_CXX_FLAGS "${MBED_CMAKE_CXX_FLAGS} -std=c++11")
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
