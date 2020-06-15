#======================================================================
# Set Testing Sources
#======================================================================
set(GEOMETRY2D_TESTING_SRCS
    ArcTest.cpp
    CircleTest.cpp
    CompositeShapeTest.cpp
    LineTest.cpp
    PointTest.cpp
    PoseTest.cpp
    RectTest.cpp
    SegmentTest.cpp
    TransformMatrixTest.cpp)

#======================================================================
# Add Sources
#======================================================================
target_sources(geometry2d_testing PRIVATE ${GEOMETRY2D_TESTING_SRCS})
