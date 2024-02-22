# ======================================================================
# Preamble
# ======================================================================
cmake_minimum_required(VERSION 3.16)
project(soccer LANGUAGES CXX)

# ======================================================================
# Find package
# ======================================================================
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rj_msgs REQUIRED)
find_package(rj_drawing_msgs REQUIRED)
find_package(rj_geometry_msgs REQUIRED)

# Qt5
find_package(
        Qt5
        COMPONENTS Widgets
        Xml
        Core
        OpenGL
        Network
        Svg
        REQUIRED)
message(STATUS "Found Qt5: ${Qt5Widgets_DIR}")
add_definitions(-DQT_NO_KEYWORDS) # Remove QT slots/signals/emit keywords

# Google Protobuf
find_package(Protobuf REQUIRED)
include_directories(SYSTEM ${PROTOBUF_INCLUDE_DIR})

# Python
find_package(PythonInterp 3.2 REQUIRED)
find_package(PythonLibs 3.2 REQUIRED)

# Eigen - used for linear algebra
find_package(Eigen3 REQUIRED)

# SDL2
include(FindPkgConfig)
pkg_search_module(SDL2 REQUIRED sdl2)

# Pthread
find_package(Threads REQUIRED)

# Boost
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(
        Boost
        COMPONENTS system
        REQUIRED)

find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)

# ======================================================================
# QT build tool things
# ======================================================================
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# ======================================================================
# Define Targets
# ======================================================================
# Build stand-alone soccer dylib This is linked into soccer and our unit tests, as well as being a
# python module
add_library(robocup SHARED)

set_target_properties(robocup PROPERTIES PREFIX "")
set_target_properties(robocup PROPERTIES SUFFIX ".so")

add_executable(rj_vision_filter)

add_executable(soccer)
add_executable(log_viewer)
add_executable(internal_referee_node)
add_executable(external_referee_node)
add_executable(planner_node)
add_executable(control_node)
add_executable(sim_radio_node)
add_executable(soccer_mom_node)
add_executable(network_radio_node)
add_executable(manual_control_node)
add_executable(global_param_server_node)
add_executable(agent_action_client_node)

if(BUILD_TESTING)
  add_executable(test-soccer)
endif()

add_subdirectory(src)

# ======================================================================
# Dependencies List
# ======================================================================

# ---- robocup ----
set(ROBOCUP_DEPS_SYSTEM_INCLUDE_DIRS
        ${EIGEN_INCLUDE_DIRS}
        ${LIBUSB_1_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}
        ${RRT_INCLUDE_DIR}
        ${SDL2_INCLUDE_DIRS})

set(ROBOCUP_DEPS_SYSTEM_LIBRARIES
    ${Boost_LIBRARIES}
    ${Boost_SYSTEM_LIBRARY}
    fmt
    GL
    GLU
    glut
    ${PYTHON_LINK_DIRS}
    ${PYTHON_LIBRARIES}
    Qt5::Widgets
    Qt5::Xml
    Qt5::Core
    Qt5::OpenGL
    Qt5::Network
    Qt5::Svg
    spdlog
    Threads::Threads
    RRT
    spnav
    ${SDL2_LIBRARIES})

set(ROBOCUP_DEPS_INCLUDE_DIRS
        ${CMAKE_CURRENT_SOURCE_DIR}/src/soccer ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_BINARY_DIR}/robocup_autogen/include)

set(ROBOCUP_DEPS_LIBRARIES
    rj_common
    rj_param_utils
    rj_protos
    rc-fshare
    rj_param_utils
    rj_topic_utils
    rj_utils
    config_client)

# ---- vision_filter ----
set(RJ_VISION_FILTER_DEPS_SYSTEM_LIBRARIES spdlog)

set(RJ_VISION_FILTER_DEPS_LIBRARIES rj_param_utils robocup)

# ---- soccer ----
set(SOCCER_DEPS_SYSTEM_LIBRARIES
        Qt5::Widgets
        Qt5::Xml
        Qt5::Core
        Qt5::OpenGL
        Qt5::Network
        Qt5::Svg)

set(SOCCER_DEPS_LIBRARIES robocup)

# ---- log_viewer ----
set(LOG_VIEWER_DEPS_SYSTEM_LIBRARIES
        Qt5::Core
        Qt5::Widgets
        Qt5::OpenGL
        Qt5::Xml
        Qt5::Svg)

set(LOG_VIEWER_DEPS_LIBRARIES robocup)

set(REFEREE_NODE_DEPS_SYSTEM_LIBRARIES)
set(REFEREE_NODE_DEPS_LIBRARIES robocup)

set(RADIO_NODE_DEPS_SYSTEM_LIBRARIES)
set(RADIO_NODE_DEPS_LIBRARIES robocup)

set(SOCCER_MOM_NODE_DEPS_SYSTEM_LIBRARIES)
set(SOCCER_MOM_NODE_DEPS_LIBRARIES robocup)

set(CONTROL_NODE_DEPS_SYSTEM_LIBRARIES)
set(CONTROL_NODE_DEPS_LIBRARIES robocup)

set(PLANNER_NODE_DEPS_SYSTEM_LIBRARIES)
set(PLANNER_NODE_DEPS_LIBRARIES robocup)

set(MANUAL_CONTROL_NODE_DEPS_SYSTEM_LIBRARIES)
set(MANUAL_CONTROL_NODE_DEPS_LIBRARIES robocup)

set(GLOBAL_PARAM_SERVER_NODE_DEPS_SYSTEM_LIBRARIES)
set(GLOBAL_PARAM_SERVER_NODE_DEPS_LIBRARIES robocup)

set(AGENT_ACTION_CLIENT_NODE_DEPS_SYSTEM_LIBRARIES)
set(AGENT_ACTION_CLIENT_NODE_DEPS_LIBRARIES robocup)

# ======================================================================
# Include and Linking
# ======================================================================

# ---- Robocup ----
ament_target_dependencies(robocup PUBLIC ament_index_cpp)

target_include_directories(robocup SYSTEM PUBLIC ${ROBOCUP_DEPS_SYSTEM_INCLUDE_DIRS})
target_include_directories(robocup PUBLIC ${ROBOCUP_DEPS_INCLUDE_DIRS})
target_link_libraries(robocup PUBLIC ${ROBOCUP_DEPS_SYSTEM_LIBRARIES} ${ROBOCUP_DEPS_LIBRARIES})

ament_target_dependencies(robocup PUBLIC rj_geometry_msgs)
ament_target_dependencies(robocup PUBLIC rj_drawing_msgs)
ament_target_dependencies(robocup PUBLIC rj_msgs)

# ---- rj_vision_filter ----
target_link_libraries(rj_vision_filter PRIVATE robocup ${RJ_VISION_FILTER_DEPS_LIBRARIES} ${RJ_VISION_FILTER_DEPS_SYSTEM_LIBRARIES})
ament_target_dependencies(rj_vision_filter PUBLIC ament_index_cpp)

# ---- Soccer ----
target_link_libraries(soccer PUBLIC ${SOCCER_DEPS_SYSTEM_LIBRARIES} ${SOCCER_DEPS_LIBRARIES})

# ---- log_viewer ----
target_link_libraries(log_viewer PRIVATE ${LOG_VIEWER_DEPS_SYSTEM_LIBRARIES}
        ${LOG_VIEWER_DEPS_LIBRARIES})

# -- external_referee_node --
target_link_libraries(external_referee_node PRIVATE ${REFEREE_NODE_DEPS_SYSTEM_LIBRARIES}
        ${REFEREE_NODE_DEPS_LIBRARIES})

# -- internal_referee_node --
target_link_libraries(internal_referee_node PRIVATE ${REFEREE_NODE_DEPS_SYSTEM_LIBRARIES}
        ${REFEREE_NODE_DEPS_LIBRARIES})

# -- soccer_mom node --
target_link_libraries(soccer_mom_node PRIVATE ${SOCCER_MOM_NODE_DEPS_SYSTEM_LIBRARIES}
        ${SOCCER_MOM_NODE_DEPS_LIBRARIES})

# -- sim_radio_node --
target_link_libraries(sim_radio_node PRIVATE ${RADIO_NODE_DEPS_SYSTEM_LIBRARIES}
        ${RADIO_NODE_DEPS_LIBRARIES})

# -- network_radio_node --
target_link_libraries(network_radio_node PRIVATE ${RADIO_NODE_DEPS_SYSTEM_LIBRARIES}
        ${RADIO_NODE_DEPS_LIBRARIES})

# -- control_node --
target_link_libraries(control_node PRIVATE ${CONTROL_NODE_DEPS_SYSTEM_LIBRARIES}
        ${CONTROL_NODE_DEPS_LIBRARIES})

# -- planner_node --
target_link_libraries(planner_node PRIVATE ${PLANNER_NODE_DEPS_SYSTEM_LIBRARIES}
        ${PLANNER_NODE_DEPS_LIBRARIES})

# -- manual_control_node --
target_link_libraries(manual_control_node PRIVATE ${MANUAL_CONTROL_NODE_DEPS_SYSTEM_LIBRARIES}
        ${MANUAL_CONTROL_NODE_DEPS_LIBRARIES})

# -- global_param_server_node --
target_link_libraries(global_param_server_node PRIVATE ${GLOBAL_PARAM_SERVER_NODE_DEPS_SYSTEM_LIBRARIES}
        ${GLOBAL_PARAM_SERVER_NODE_DEPS_LIBRARIES})

# -- agent_action_client_node --
target_link_libraries(agent_action_client_node PRIVATE ${AGENT_ACTION_CLIENT_NODE_DEPS_SYSTEM_LIBRARIES}
    ${AGENT_ACTION_CLIENT_NODE_DEPS_LIBRARIES})

# ======================================================================
# Testing
# ======================================================================
# Add a test runner target "test-soccer" to run all tests in this directory
if(BUILD_TESTING)
  enable_testing()
  add_subdirectory(testing)

  target_link_libraries(test-soccer PRIVATE robocup)
  target_link_libraries(test-soccer PRIVATE Qt5::Core Qt5::Widgets Qt5::Xml)

  set(test_libs geometry2d_testing rj_param_utils_testing)
  foreach(test_lib ${test_libs})
    target_link_libraries(
            test-soccer
            PRIVATE
            "-Wl,--whole-archive -L$<TARGET_FILE_DIR:${test_lib}> -l${test_lib} -Wl,--no-whole-archive")
    target_link_libraries(test-soccer PRIVATE ${test_lib})
  endforeach()

  target_link_libraries(test-soccer PRIVATE gtest gtest_main)
endif()

# ======================================================================
# Packaging
# ======================================================================
install(
        TARGETS robocup
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(
        TARGETS soccer
        internal_referee_node
        external_referee_node
        rj_vision_filter
        planner_node
        control_node
        sim_radio_node
        soccer_mom_node
        network_radio_node
        manual_control_node
        global_param_server_node
        agent_action_client_node
        DESTINATION lib/${CMAKE_PROJECT_NAME})

install(
        FILES config/realShopField.xml
        DESTINATION share/${CMAKE_PROJECT_NAME}/config
        RENAME soccer-real.cfg)

install(
        FILES config/sim.xml
        DESTINATION share/${CMAKE_PROJECT_NAME}/config
        RENAME soccer-sim.cfg)

if(BUILD_TESTING)
  install(TARGETS test-soccer DESTINATION lib/${CMAKE_PROJECT_NAME})
endif()
