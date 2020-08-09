# ======================================================================
# Set Sources
# ======================================================================
set(GEOMETRY2D_SRCS
    arc.cpp
    circle.cpp
    composite_shape.cpp
    line.cpp
    point.cpp
    polygon.cpp
    rect.cpp
    segment.cpp
    transform_matrix.cpp)

# ======================================================================
# Add sources
# ======================================================================
target_sources(geometry2d PRIVATE ${GEOMETRY2D_SRCS})
