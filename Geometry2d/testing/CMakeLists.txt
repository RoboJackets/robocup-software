#======================================================================
# Set Testing Sources
#======================================================================
set(GEOMETRY2D_TESTING_SRCS
    ArcTest.cpp
    CircleTest.cpp
    CompositeShapeTest.cpp
    LineTest.cpp
    PointTest.cpp
    RectTest.cpp
    SegmentTest.cpp)

#======================================================================
# Add Sources
#======================================================================
target_sources(geometry2d_testing PRIVATE ${GEOMETRY2D_TESTING_SRCS})
