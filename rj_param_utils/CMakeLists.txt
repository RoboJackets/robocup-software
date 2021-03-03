# ======================================================================
# Preamble
# ======================================================================
cmake_minimum_required(VERSION 3.16)
project(rj_param_utils)

# ======================================================================
# Find package
# ======================================================================
find_package(rclcpp REQUIRED)

# ======================================================================
# Define Targets
# ======================================================================
add_library(rj_param_utils STATIC)

add_subdirectory(src)

# ======================================================================
# Dependencies List
# ======================================================================
set(RJ_PARAM_UTILS_DEPS_INCLUDE_DIRS include)

# ======================================================================
# Include and Linking
# ======================================================================
target_include_directories(rj_param_utils PUBLIC ${RJ_PARAM_UTILS_DEPS_INCLUDE_DIRS})

target_link_libraries(rj_param_utils PUBLIC ${RJ_PARAM_UTILS_DEPS_LIBRARIES})

ament_target_dependencies(rj_param_utils PUBLIC rclcpp)

target_sources(rj_param_utils PRIVATE ${RJ_PARAM_UTILS_SRCS})

# ======================================================================
# Testing
# ======================================================================
if(BUILD_TESTING)
  enable_testing()

  add_library(rj_param_utils_testing STATIC)
  add_subdirectory(testing)
  target_link_libraries(rj_param_utils_testing PUBLIC rj_param_utils ${GTEST_LIBRARIES})
endif()

# position-independent-code flag
target_compile_options(rj_param_utils PRIVATE "-fPIC")

# ======================================================================
# Packaging
# ======================================================================
install(
  TARGETS rj_param_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
