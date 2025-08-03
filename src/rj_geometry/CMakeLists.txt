cmake_minimum_required(VERSION 3.8)
project(rj_geometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Qt5 COMPONENTS Core REQUIRED)
find_package(rj_protos REQUIRED)
find_package(rj_constants REQUIRED)
find_package(rj_convert REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rj_geometry_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/arc.cpp
  src/circle.cpp
  src/composite_shape.cpp
  src/line.cpp
  src/point.cpp
  src/polygon.cpp
  src/rect.cpp
  src/segment.cpp
  src/stadium_shape.cpp
  src/transform_matrix.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
  Eigen3
  Qt5
  rj_protos
  rj_constants
  rj_convert
  rj_geometry_msgs
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
  Eigen3::Eigen
  Qt5::Core
  ${CMAKE_DL_LIBS}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_dependencies(
  Eigen3
  Qt5
  rj_protos
  rj_constants
  rj_convert
  rj_geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  # TODO: Add tests from testing

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
