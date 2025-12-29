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
find_package(fmt REQUIRED)
find_package(rj_protos REQUIRED)
find_package(rj_constants REQUIRED)
find_package(rj_convert REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rj_geometry_msgs REQUIRED)

set(RJ_GEOMETRY_DEPS
  Eigen3
  fmt
  rj_protos
  rj_constants
  rj_convert
  rj_geometry_msgs
)

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
  Qt5
  ${RJ_GEOMETRY_DEPS}
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
  Eigen3::Eigen
  Qt5::Core
  fmt
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
  Qt5Core
  ${RJ_GEOMETRY_DEPS}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  set(RJ_GEOMETRY_TESTS
     arc_test
     circle_test
     composite_shape_test
     geometry_conversions_test
     line_test
     point_test
     pose_test
     rect_test
     segment_test
     transform_matrix_test
  )

  foreach(TEST IN LISTS RJ_GEOMETRY_TESTS)
    ament_add_gtest(${TEST} tests/${TEST}.cpp)
    target_include_directories(${TEST} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
    target_link_libraries(${TEST} ${PROJECT_NAME})
  endforeach()
endif()

ament_export_targets(${PROJECT_NAME})
ament_package()
