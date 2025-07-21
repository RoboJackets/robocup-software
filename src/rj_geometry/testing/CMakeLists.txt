ament_add_gtest(rj_geometry_testing
    arc_test.cpp
    circle_test.cpp
    composite_shape_test.cpp
    geometry_conversions_test.cpp
    line_test.cpp
    point_test.cpp
    pose_test.cpp
    rect_test.cpp
    segment_test.cpp
    transform_matrix_test.cpp)

target_link_libraries(rj_geometry_testing ${PROJECT_NAME})