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

# sets many variables and paths related to building for the MBED
include(mbed/arm_mbed.cmake)

# suppress -rdynamic flag
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "")

# header files
include_directories("common2015/drivers" )
include_directories("common2015/drivers/CC1101")
include_directories("common2015/modules" )
include_directories("common2015/modules/CommLink")
include_directories("common2015/modules/CommModule")
include_directories("common2015/utils")
include_directories("common2015/utils/FirmwareCheck")
include_directories("common2015/utils/Logger")
include_directories("common2015/utils/RTOS_Management")
include_directories("common2015/utils/Watchdog")

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
