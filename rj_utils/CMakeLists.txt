# ======================================================================
# Preamble
# ======================================================================
cmake_minimum_required(VERSION 3.16)
project(rj_utils LANGUAGES CXX)

# ======================================================================
# Find package
# ======================================================================
find_package(rclcpp REQUIRED)
find_package(rj_msgs REQUIRED)

# ======================================================================
# Define Targets
# ======================================================================
add_library(rj_utils STATIC)

# position-independent-code flag
target_compile_options(rj_utils PRIVATE "-fPIC")

add_subdirectory(src)

# ======================================================================
# Dependencies List
# ======================================================================
set(RJ_UTILS_DEPS_INCLUDE_DIRS include)

set(RJ_UTILS_DEPS_LIBRARIES rj_protos)

# ======================================================================
# Include and Linking
# ======================================================================
target_include_directories(rj_utils PUBLIC ${RJ_UTILS_DEPS_INCLUDE_DIRS})
target_link_libraries(rj_utils PUBLIC ${RJ_UTILS_DEPS_LIBRARIES})
ament_target_dependencies(
  rj_utils
  PUBLIC
  rclcpp
  rj_msgs)

# ======================================================================
# Packaging
# ======================================================================
install(DIRECTORY include/ DESTINATION include/)

install(
  TARGETS rj_utils
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)
