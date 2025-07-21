# ======================================================================
# Preamble
# ======================================================================
cmake_minimum_required(VERSION 3.16)
project(rj_common LANGUAGES CXX)

# ======================================================================
# Find package
# ======================================================================
find_package(
  Qt5
  COMPONENTS Core Network Widgets
  REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rj_msgs REQUIRED)

# ======================================================================
# Define Targets
# ======================================================================
add_library(rj_common STATIC)

if(BUILD_TESTING)
  add_executable(test_rj_common)
endif()

# position-independent-code flag
target_compile_options(rj_common PRIVATE "-fPIC")

add_subdirectory(src)
add_subdirectory(testing)

# ======================================================================
# Dependencies List
# ======================================================================
set(RJ_COMMON_DEPS_SYSTEM_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})

set(RJ_COMMON_DEPS_SYSTEM_LIBRARIES Qt5::Core Qt5::Network Qt5::Widgets)

set(RJ_COMMON_DEPS_INCLUDE_DIRS include)

set(RJ_COMMON_DEPS_LIBRARIES geometry2d rj_constants rj_convert)

# ======================================================================
# Include and Linking
# ======================================================================
target_include_directories(rj_common SYSTEM PUBLIC ${RJ_COMMON_DEPS_SYSTEM_INCLUDE_DIRS})
target_include_directories(rj_common PUBLIC ${RJ_COMMON_DEPS_INCLUDE_DIRS})

target_link_libraries(rj_common PUBLIC ${RJ_COMMON_DEPS_SYSTEM_LIBRARIES}
                                       ${RJ_COMMON_DEPS_LIBRARIES})

ament_target_dependencies(
  rj_common
  PUBLIC
  rclcpp
  rclcpp_action
  rclcpp_components
  rj_msgs)

# Add a test runner target "test_rj_common" to run all tests in this directory
if(BUILD_TESTING)
  target_link_libraries(test_rj_common PRIVATE rj_common gtest gtest_main)
endif()

# ======================================================================
# Packaging
# ======================================================================
install(DIRECTORY include/ DESTINATION include/)

install(
  TARGETS rj_common
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  install(TARGETS test_rj_common DESTINATION lib/${CMAKE_PROJECT_NAME})
endif()
