
enable_testing()


file(GLOB TEST_SRC "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
add_executable(run_tests ${TEST_SRC})
qt5_use_modules(run_tests Core Widgets Xml)
add_dependencies(run_tests googletest)
include_directories(${GTEST_INCLUDE_DIRS})
# include_directories(${GTEST_LIBS_DIR})
# include_directories("${PROJECT_BINARY_DIR}/test/src/googletest-build")
target_link_libraries(run_tests
    ${GTEST_LIBS_DIR}/libgtest.a
    ${GTEST_LIBS_DIR}/libgtest_main.a
)
target_link_libraries(run_tests robocup)

add_test(test1 run_tests)
