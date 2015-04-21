
enable_testing()


file(GLOB TEST_SRC "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
add_executable(test-cpp ${TEST_SRC})
set_target_properties(test-cpp PROPERTIES EXCLUDE_FROM_ALL TRUE)
qt5_use_modules(test-cpp Core Widgets Xml)
add_dependencies(test-cpp googletest)
include_directories(${GTEST_INCLUDE_DIRS})
target_link_libraries(test-cpp
    ${GTEST_LIBS_DIR}/libgtest.a
    ${GTEST_LIBS_DIR}/libgtest_main.a
)
target_link_libraries(test-cpp robocup)

add_test(test1 test-cpp)
