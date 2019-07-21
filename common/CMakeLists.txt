
add_subdirectory(protobuf)
include_directories("${CMAKE_CURRENT_BINARY_DIR}/protobuf")

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
)


include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# positon-independent-code flag
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

# build the 'common' static library (and include our protobuf messages in it)
include_directories(SYSTEM ${EIGEN_INCLUDE_DIR})
add_library(common STATIC ${COMMON_SRC})
target_link_libraries(common proto_messages)

if(APPLE)
    # look for the homebrew-installed version of Qt5
    find_package(Qt5Widgets
        REQUIRED
        PATHS /usr/local/Cellar/qt/* /usr/local/Cellar/qt5/*)

    # this directory is included by default, but we mark it as SYSTEM so
    # warnings in external headers don't show up every time we build
    include_directories(SYSTEM /usr/local/include/)

else()
    find_package(Qt5Widgets REQUIRED)
endif()
qt5_use_modules(common Core Network Widgets)
