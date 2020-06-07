find_package(Qt5 COMPONENTS Widgets Xml Core OpenGL Network Svg REQUIRED)

# Library and headers
add_library(common field_dimensions.cpp utils.cpp)
target_include_directories(common PUBLIC ${COMMON_INCLUDE_DIR})
target_link_libraries(common Qt5::Core Qt5::Widgets geometry2d)
