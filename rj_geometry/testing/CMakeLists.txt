# ======================================================================
# Set Testing Sources
# ======================================================================
set(GEOMETRY2D_TESTING_SRCS
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

# ======================================================================
# Add Sources
# ======================================================================
target_sources(geometry2d_testing PRIVATE ${GEOMETRY2D_TESTING_SRCS})
