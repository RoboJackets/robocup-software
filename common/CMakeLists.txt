#======================================================================
# Preamble
#======================================================================
cmake_minimum_required(VERSION 3.16)
project(common LANGUAGES CXX)

#======================================================================
# Find package
#======================================================================
find_package(Qt5 COMPONENTS Core Network Widgets REQUIRED)
find_package(ament_cmake REQUIRED)

#======================================================================
# Define Targets
#======================================================================
add_library(common STATIC)

# position-independent-code flag
target_compile_options(common PRIVATE "-fPIC")

#======================================================================
# Find package
#======================================================================

set(COMMON_SRC
    "Field_Dimensions.cpp"
    "Geometry2d/Arc.cpp"
    "Geometry2d/Circle.cpp"
    "Geometry2d/Line.cpp"
    "Geometry2d/Rect.cpp"
    "Geometry2d/TransformMatrix.cpp"
    "Geometry2d/CompositeShape.cpp"
    "Geometry2d/Point.cpp"
    "Geometry2d/Polygon.cpp"
    "Geometry2d/Segment.cpp"
    "multicast.cpp"
    "Utils.cpp"
    "RefereeEnums.cpp"
)


include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# build the 'common' static library (and include our protobuf messages in it)
include_directories(SYSTEM ${EIGEN_INCLUDE_DIR})
target_link_libraries(common PUBLIC rj_protos)

#======================================================================
# Add sources
#======================================================================
target_sources(common
    PRIVATE
    ${COMMON_SRC})

#======================================================================
# Include and Linking
#======================================================================
target_link_libraries(common PUBLIC Qt5::Core Qt5::Network Qt5::Widgets)

#======================================================================
# Packaging
#======================================================================

#======================================================================
# Ament packaging
#======================================================================
ament_export_libraries(common)
ament_package()
