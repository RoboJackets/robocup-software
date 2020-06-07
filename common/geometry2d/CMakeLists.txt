find_package(Eigen3 REQUIRED)
find_package(Qt5 COMPONENTS Widgets Xml Core OpenGL Network Svg REQUIRED)

# Library and headers
add_library(geometry2d
  src/arc.cpp
  src/circle.cpp
  src/line.cpp
  src/rect.cpp
  src/transform_matrix.cpp
  src/composite_shape.cpp
  src/point.cpp
  src/polygon.cpp
  src/segment.cpp
)
target_include_directories(geometry2d PUBLIC ${COMMON_INCLUDE_DIR} ${EIGEN_INCLUDE_DIR})
target_link_libraries(geometry2d PUBLIC Qt5::Core proto_messages)

