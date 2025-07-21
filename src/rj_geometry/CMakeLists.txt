# how to write/edit this file: https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Documentation.html

cmake_minimum_required(VERSION 3.16)
project(rj_geometry LANGUAGES CXX)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Enable useful warnings for GCC/Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(
  Qt5
  COMPONENTS Core
  REQUIRED)
find_package(rj_geometry_msgs REQUIRED)
find_package(rj_protos REQUIRED)
find_package(rj_convert REQUIRED)
find_package(rj_constants REQUIRED)
find_package(fmt REQUIRED)

# create library
add_library(${PROJECT_NAME} STATIC
    src/arc.cpp
    src/circle.cpp
    src/composite_shape.cpp
    src/line.cpp
    src/point.cpp
    src/polygon.cpp
    src/rect.cpp
    src/segment.cpp
    src/stadium_shape.cpp
    src/transform_matrix.cpp)

# add dependencies
target_link_libraries(${PROJECT_NAME} PUBLIC 
    rj_protos::rj_protos 
    rj_convert::rj_convert 
    rj_constants::rj_constants
    Qt5::Core
    fmt::fmt)


ament_target_dependencies(${PROJECT_NAME} PUBLIC
    Eigen3
    rj_geometry_msgs)


# required for ament
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(${PROJECT_NAME} PUBLIC ${EIGEN3_INCLUDE_DIRS})

install(
  DIRECTORY include/
  DESTINATION include/
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Export targets and dependencies for other packages
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rj_convert rj_protos rj_constants Eigen3 Qt5)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(testing)
endif()

# Register this package with ament
ament_package()
