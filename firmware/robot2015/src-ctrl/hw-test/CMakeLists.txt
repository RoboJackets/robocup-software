# HW_TEST_UNIT should always be set to whatever comes before '-test.cpp' in their respective filename that contains main()
if(HW_TEST_UNIT AND NOT(HW_TEST_UNIT MATCHES "default"))
    set(ROBOT2015_HW_TEST_INCLUDES ${CONFIG_ROOT_NAME})
    set(TEST_UNIT_FILE "${HW_TEST_UNIT}-test.cpp")

    string(SUBSTRING "${HW_TEST_UNIT}" 0 3 TEST_UNIT_BIN_FILE)
    set(TEST_UNIT_BIN_FILE "${TEST_UNIT_BIN_FILE}-test.bin")

    add_executable(robot2015_test_ELF ${TEST_UNIT_FILE})

    # specify that we depend on the common2015 library and link to it
    add_dependencies(robot2015_test_ELF common2015)
    target_link_libraries(robot2015_test_ELF common2015)

    # include 
    target_include_directories(robot2015_test_ELF PUBLIC ${ROBOT2015_HW_TEST_INCLUDES})

    # the final product is the .bin file, not the elf one.  We hide this away in the build dir
    set_target_properties(robot2015_test_ELF PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

    # only build robot firmware if specifically instructed
    set_target_properties(robot2015_test_ELF PROPERTIES EXCLUDE_FROM_ALL TRUE)

    # custom target for creating a .bin file from an elf binary
    add_custom_target(robot2015-test
        arm-none-eabi-objcopy -O binary robot2015_test_ELF ${TEST_UNIT_BIN_FILE}
        COMMAND cp ${CMAKE_CURRENT_BINARY_DIR}/${TEST_UNIT_BIN_FILE} ${PROJECT_SOURCE_DIR}/run/
        DEPENDS robot2015_test_ELF
        COMMENT "Objcopying for making the ${HW_TEST_UNIT} hardware test"
    )
    set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES ${PROJECT_SOURCE_DIR}/run/${TEST_UNIT_BIN_FILE})

    add_custom_target(robot2015-test-prog
        arm-none-eabi-objcopy -O binary robot2015_test_ELF ${TEST_UNIT_BIN_FILE}
        COMMAND ${HW_TEST_SCRIPT} ${CMAKE_CURRENT_BINARY_DIR}/${TEST_UNIT_BIN_FILE}
        DEPENDS robot2015-test
        COMMENT "Copying ${HW_TEST_UNIT}'s test binary over to the mbed"
    )

endif()
