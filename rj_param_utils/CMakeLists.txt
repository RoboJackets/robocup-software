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
add_executable(global_params)

add_subdirectory(src)

# ======================================================================
# Dependencies List
# ======================================================================
set(RJ_PARAM_UTILS_DEPS_INCLUDE_DIRS include)

# ======================================================================
# Include and Linking
# ======================================================================
target_include_directories(rj_param_utils PRIVATE ${RJ_PARAM_UTILS_DEPS_INCLUDE_DIRS})

target_link_libraries(rj_param_utils PRIVATE ${RJ_PARAM_UTILS_DEPS_LIBRARIES})

ament_target_dependencies(rj_param_utils PUBLIC rclcpp)

# ======================================================================
# Packaging
# ======================================================================
install(
  TARGETS global_params
  DESTINATION lib/${CMAKE_PROJECT_NAME})
