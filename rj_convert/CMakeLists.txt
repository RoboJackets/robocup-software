# ======================================================================
# Preamble
# ======================================================================
cmake_minimum_required(VERSION 3.16)
project(rj_convert LANGUAGES CXX)

# ======================================================================
# Find package
# ======================================================================
find_package(rclcpp REQUIRED)

# ======================================================================
# Define Targets
# ======================================================================
add_library(rj_convert INTERFACE)

if(BUILD_TESTING)
  add_executable(test_rj_convert)
endif()

add_subdirectory(testing)

# ======================================================================
# Dependencies List
# ======================================================================
set(RJ_CONVERT_DEPS_SYSTEM_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})

set(RJ_CONVERT_DEPS_INCLUDE_DIRS include)

# ======================================================================
# Include and Linking
# ======================================================================
target_include_directories(rj_convert SYSTEM INTERFACE ${RJ_CONVERT_DEPS_SYSTEM_INCLUDE_DIRS})
target_include_directories(rj_convert INTERFACE ${RJ_CONVERT_DEPS_INCLUDE_DIRS})

ament_target_dependencies(rj_convert INTERFACE rclcpp)

# Add a test runner target "test_rj_convert" to run all tests in this directory
if(BUILD_TESTING)
  target_link_libraries(test_rj_convert PRIVATE rj_convert gtest gtest_main)
endif()

# ======================================================================
# Packaging
# ======================================================================
install(DIRECTORY include/ DESTINATION include/)

install(
  TARGETS rj_convert
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  install(TARGETS test_rj_convert DESTINATION lib/${CMAKE_PROJECT_NAME})
endif()
