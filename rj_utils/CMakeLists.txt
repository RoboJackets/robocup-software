#======================================================================
# Preamble
#======================================================================
cmake_minimum_required(VERSION 3.16)
project(rj_utils LANGUAGES CXX)

#======================================================================
# Find package
#======================================================================
find_package(rclcpp REQUIRED)

#======================================================================
# Define Targets
#======================================================================
add_library(rj_utils)

# position-independent-code flag
target_compile_options(rj_utils PRIVATE "-fPIC")

add_subdirectory(src)

#======================================================================
# Dependencies List
#======================================================================
set(RJ_UTILS_DEPS_INCLUDE_DIRS
    include)

set(RJ_UTILS_DEPS_LIBRARIES
    rj_protos)

#======================================================================
# Include and Linking
#======================================================================
target_include_directories(rj_utils PUBLIC ${RJ_UTILS_DEPS_INCLUDE_DIRS})
target_link_libraries(rj_utils PUBLIC ${RJ_UTILS_DEPS_LIBRARIES})
ament_target_dependencies(rj_utils PUBLIC rclcpp)

include(ROSIDLUtils)
rosidl_target_link_interfaces(rj_utils PUBLIC rj_msgs)

#======================================================================
# Packaging
#======================================================================
install(DIRECTORY include/
    DESTINATION include/)
