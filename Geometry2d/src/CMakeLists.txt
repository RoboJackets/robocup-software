# ======================================================================
# Set Sources
# ======================================================================
set(GEOMETRY2D_SRCS
    # cmake-format: sortable
    Arc.cpp
    Circle.cpp
    CompositeShape.cpp
    Line.cpp
    Point.cpp
    Polygon.cpp
    Rect.cpp
    Segment.cpp
    TransformMatrix.cpp)

# ======================================================================
# Add sources
# ======================================================================
target_sources(geometry2d PRIVATE ${GEOMETRY2D_SRCS})
