
enable_testing()

if (test)
    file(GLOB TEST_SRC "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
    add_executable(run_tests ${TEST_SRC})
    add_dependencies(run_tests googletest)
    include_directories("${PROJECT_BINARY_DIR}/test/src/googletest-build")
    target_link_libraries(run_tests gtest gtest_main)
    target_link_libraries(run_tests robocup)

    add_test(test1 run_tests)
endif()
