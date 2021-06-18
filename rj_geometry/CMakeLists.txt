# ======================================================================
# Preamble
# ======================================================================
cmake_minimum_required(VERSION 3.16)
project(geometry2d LANGUAGES CXX)

# ======================================================================
# Find package
# ======================================================================
find_package(Eigen3 REQUIRED)
find_package(
  Qt5
  COMPONENTS Core
  REQUIRED)
find_package(rj_geometry_msgs REQUIRED)

# ======================================================================
# Define Targets
# ======================================================================
add_library(geometry2d STATIC)

add_subdirectory(src)

# ======================================================================
# Dependencies List
# ======================================================================
set(GEOMETRY2D_DEPS_SYSTEM_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})

set(GEOMETRY2D_DEPS_SYSTEM_LIBRARIES Qt5::Core)

set(GEOMETRY2D_DEPS_INCLUDE_DIRS include)

set(GEOMETRY2D_DEPS_LIBRARIES rj_protos rj_constants rj_convert)

# ======================================================================
# Include and Linking
# ======================================================================
target_include_directories(geometry2d SYSTEM PUBLIC ${GEOMETRY2D_DEPS_SYSTEM_INCLUDE_DIRS})
target_include_directories(geometry2d PUBLIC ${GEOMETRY2D_DEPS_INCLUDE_DIRS})

target_link_libraries(geometry2d PUBLIC ${GEOMETRY2D_DEPS_SYSTEM_LIBRARIES}
                                        ${GEOMETRY2D_DEPS_LIBRARIES})

ament_target_dependencies(geometry2d PUBLIC rj_geometry_msgs)

# ======================================================================
# Testing
# ======================================================================
if(BUILD_TESTING)
  enable_testing()

  add_library(geometry2d_testing STATIC)
  add_subdirectory(testing)
  target_link_libraries(geometry2d_testing PUBLIC geometry2d)
endif()

# ======================================================================
# Packaging
# ======================================================================
install(DIRECTORY include/ DESTINATION include/)
