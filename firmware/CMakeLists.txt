# This sets up MBED all MBED libraries, then sets up the robot and base station
# firmware, which depend on the MBED libs
include(RemoveFlags)

# remove the color flag since it will get set on anything with clang assert
# the default compiler (mostly apple computers). The arm compilers know
# nothing about this flag however.
remove_cxx_flag("-fcolor-diagnostics")

# sets many variables and paths related to building for the MBED
include(${CMAKE_CURRENT_LIST_DIR}/mbed/arm_mbed.cmake)

# set the path to a script for moving files to the mbed's USB storage
set(MBED_COPY_SCRIPT sudo ${PROJECT_SOURCE_DIR}/util/mbed/mbed-copy.py)

# Note: the arm_mbed.cmake script exports flags that need to be set in each CMake
# file used to build MBED code like so:
# set(CMAKE_CXX_FLAGS ${MBED_CMAKE_CXX_FLAGS})
# set(CMAKE_C_FLAGS ${MBED_CMAKE_C_FLAGS})
# set(CMAKE_EXE_LINKER_FLAGS ${MBED_CMAKE_EXE_LINKER_FLAGS})

# enable C++ exceptions
set(MBED_CMAKE_CXX_FLAGS "${MBED_CMAKE_CXX_FLAGS} -fexceptions")
# the mbed libraries are compiled with -fno-rtti, so we need to as well
set(MBED_CMAKE_CXX_FLAGS "${MBED_CMAKE_CXX_FLAGS} -fno-rtti")
# enable C++14
set(MBED_CMAKE_CXX_FLAGS "${MBED_CMAKE_CXX_FLAGS} -std=c++14")
# Enable logging for the common2015 library
set(MBED_CMAKE_CXX_FLAGS "${MBED_CMAKE_CXX_FLAGS} -DRJ_LOGGING_EN")
# ignore formatting flags for printf
set(MBED_CMAKE_CXX_FLAGS "${MBED_CMAKE_CXX_FLAGS} -Wno-format -Wno-switch -Wno-unused-parameter -Wno-missing-field-initializers")
# generate debugging information for gdb
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")

# suppress -rdynamic flag
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "")

# Add a test runner target "test-firmware"
file(GLOB FIRMWARE_TEST_SRC "common2015/testing/*.cpp")
add_executable(test-firmware ${FIRMWARE_TEST_SRC})
add_dependencies(test-firmware googletest)
target_link_libraries(test-firmware ${GTEST_LIBRARIES})

# Don't build the tests by default
set_target_properties(test-firmware PROPERTIES EXCLUDE_FROM_ALL TRUE)

# build robot and base station firmware and the library that they depend on
add_subdirectory(mbed)
add_subdirectory(common2015)
add_subdirectory(base2015)
add_subdirectory(robot2015)
