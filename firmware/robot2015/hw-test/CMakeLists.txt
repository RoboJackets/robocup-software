
include(../../mbed/mbed_executable.cmake)


# HW_TEST_UNIT should always be set to whatever comes before '-test.cpp' in
# their respective filename that contains main().
if(HW_TEST_UNIT AND NOT(HW_TEST_UNIT MATCHES "default"))
    # Set compiler and linker flags
    set(ROBOT2015_HW_TEST_INCLUDES ${CONFIG_ROOT_NAME} ${CMAKE_CURRENT_LIST_DIR} ../src-ctrl/drivers/ws2811)
    set(TEST_UNIT_FILE "${HW_TEST_UNIT}-test.cpp")

    # list(APPEND TEST_UNIT_FILE ../src-ctrl/drivers/fpga/fpga.cpp)
    list(APPEND TEST_UNIT_FILE ../src-ctrl/drivers/ws2811/neostrip.cpp)

    string(SUBSTRING "${HW_TEST_UNIT}" 0 3 TEST_UNIT_BIN_FILE)
    set(TEST_UNIT_BIN_FILE "${TEST_UNIT_BIN_FILE}-test.bin")

    rj_add_mbed_executable(robot2015-test ${TEST_UNIT_FILE} ${asm_SRC})
    target_link_libraries(robot2015-test common2015)
    target_include_directories(robot2015-test PUBLIC ${ROBOT2015_HW_TEST_INCLUDES})

    # TODO: remove, these are set in rj_add_mbed_executable
    # Set compiler and linker flags
    include(${ARM_TOOLCHAIN_FILE})
    set(CMAKE_CXX_FLAGS         ${MBED_CMAKE_CXX_FLAGS}         )
    set(CMAKE_C_FLAGS           ${MBED_CMAKE_C_FLAGS}           )
    set(CMAKE_EXE_LINKER_FLAGS  ${MBED_CMAKE_EXE_LINKER_FLAGS}  )
endif()
