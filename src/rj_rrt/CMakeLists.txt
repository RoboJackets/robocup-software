cmake_minimum_required(VERSION 3.8)
project(rj_rrt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(ament_cmake REQUIRED)
find_package(Flann REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Qt5 COMPONENTS Core Qml Quick Widgets REQUIRED)

# Enable Qt's automatic processing
set(CMAKE_AUTOMOC ON)   # Handle Q_OBJECT in headers automatically
set(CMAKE_AUTOUIC ON)   # Handle .ui → ui_*.h automatically
set(CMAKE_AUTORCC ON)   # Handle .qrc → resources automatically

set(RJ_RRT_SRC
    # CPP Source Files
    src/2dplane/2dplane.cpp
    src/2dplane/GridStateSpace.cpp
    src/2dplane/ObstacleGrid.cpp
)

set(RJ_RRT_DEPS
    Flann
    Eigen3
)

set(RJ_RRT_LIBS
    Eigen3::Eigen
    ${FLANN_LIBRARIES}
)

find_library(LZ4_LIBRARY NAMES lz4)
if(LZ4_LIBRARY)
    list(APPEND RJ_RRT_LIBS ${LZ4_LIBRARY})
endif()

add_library(${PROJECT_NAME} SHARED
    ${RJ_RRT_SRC}
)
set_target_properties(${PROJECT_NAME} PROPERTIES UNITY_BUILD ON)

add_executable(rrt_viewer
    # SRC Files
    src/rrt-viewer/main.cpp
    src/rrt-viewer/RRTWidget.cpp

    # QT MOC Headers
    include/rj_rrt/rrt-viewer/RRTWidget.hpp

    # QT QRC Files
    src/rrt-viewer/main.qrc
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_include_directories(rrt_viewer
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${CMAKE_CURRENT_BINARY_DIR}
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
    ${RJ_RRT_LIBS}
)

target_link_libraries(rrt_viewer
    PRIVATE
    ${PROJECT_NAME}
    Qt5::Core
    Qt5::Qml
    Qt5::Quick
    Qt5::Widgets
)

ament_target_dependencies(${PROJECT_NAME}
    PUBLIC
    ${RJ_RRT_DEPS}
)

ament_target_dependencies(rrt_viewer
    PUBLIC
    ${RJ_RRT_DEPS}
    Qt5
)

install(
    DIRECTORY include/
    DESTINATION include
)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(
    TARGETS rrt_viewer
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

ament_export_dependencies(
    ${RJ_RRT_DEPS}
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    set(RJ_RRT_TESTS
        BiRRTTest
        TreeTest 
    )

    foreach(TEST IN LISTS RJ_RRT_TESTS)
        ament_add_gtest(${TEST} tests/${TEST}.cpp)
        target_include_directories(${TEST} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
        target_link_libraries(${TEST} ${PROJECT_NAME})
    endforeach()
endif()

ament_export_targets(${PROJECT_NAME})
ament_package()
