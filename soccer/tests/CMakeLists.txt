
# Add a test runner target "test-cpp" to run all tests in this directory
file(GLOB TEST_SRC "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
add_executable(test-soccer ${TEST_SRC})
target_link_libraries(test-soccer robocup)
qt5_use_modules(test-soccer Core Widgets Xml)
target_link_libraries(test-soccer ${GTEST_BOTH_LIBRARIES})
add_dependencies(test-soccer googletest)

# Don't build the tests by default
set_target_properties(test-soccer PROPERTIES EXCLUDE_FROM_ALL TRUE)
