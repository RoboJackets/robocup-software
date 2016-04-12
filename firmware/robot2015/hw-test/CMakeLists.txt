
include(../../mbed/mbed_executable.cmake)


# Set compiler and linker flags
set(ROBOT2015_HW_TEST_INCLUDES ${CONFIG_ROOT_NAME} ${CMAKE_CURRENT_LIST_DIR} ../src-ctrl/drivers/ws2811)

# TODO: remove, these are set in rj_add_mbed_executable
# Set compiler and linker flags
include(${ARM_TOOLCHAIN_FILE})
rj_mbed_env()

function(rj_add_mbed_test_executable name)
    set(target ${name}-test)
    rj_add_mbed_executable(${target} ${name}-test.cpp ${asm_SRC} ${ARGN})
    target_link_libraries(${target} common2015)
    target_include_directories(${target} PUBLIC ${ROBOT2015_HW_TEST_INCLUDES})
endfunction(rj_add_mbed_test_executable)

rj_add_mbed_test_executable(i2c)
rj_add_mbed_test_executable(io-expander)
rj_add_mbed_test_executable(fpga)
rj_add_mbed_test_executable(piezo)
rj_add_mbed_test_executable(neopixel)
