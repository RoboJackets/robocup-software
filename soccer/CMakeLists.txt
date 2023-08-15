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
find_package(rclcpp REQUIRED)
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
# Targets
# ======================================================================

# Common libraries used by everything in the stack
set(SOCCER_COMMON_SYSTEM_LIBS
        fmt
        spdlog
        Threads::Threads)
set(SOCCER_COMMON_LIBS
        rj_common
        rj_constants
        rj_param_utils
        rj_topic_utils
        rj_protos
        rj_utils
        config_client)
set(SOCCER_COMMON_AMENT_LIBS
        rclcpp
        rj_geometry_msgs
        rj_drawing_msgs
        rj_msgs
        ament_index_cpp)

add_library(soccer_common)
target_sources(soccer_common PRIVATE
        battery_profile.cpp
        debug_drawer.cpp
        global_params.cpp
        world_state.cpp
        robot_intent.cpp)
target_include_directories(soccer_common PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(soccer_common SYSTEM PUBLIC ${EIGEN_INCLUDE_DIRECTORIES})
ament_target_dependencies(soccer_common PUBLIC ${SOCCER_COMMON_AMENT_LIBS})
target_link_libraries(soccer_common PUBLIC
        ${SOCCER_COMMON_LIBS}
        ${SOCCER_COMMON_SYSTEM_LIBS}
        # needed for motion command, but this feels janky TODO(sid-parikh): make sure this is best
        planning_types)

# soccer directories
add_subdirectory(planning)
add_subdirectory(control)
add_subdirectory(joystick)
add_subdirectory(radio)
add_subdirectory(referee)
add_subdirectory(vision)
add_subdirectory(strategy)

# Soccer library
set(SOCCER_UIS
        ui/qt/ConfigFileTab.ui
        ui/qt/LogViewer.ui
        ui/qt/MainWindow.ui
        ui/qt/RefereeTab.ui
        ui/qt/RobotStatusWidget.ui)

set(SOCCER_RSRC ui/qt/main_icons.qrc)

# Library
add_library(soccer)
target_sources(soccer PRIVATE
        logger.cpp
        processor.cpp
        ros2_temp/autonomy_interface.cpp
        ros2_temp/debug_draw_interface.cpp
        ros2_temp/raw_vision_packet_sub.cpp
        ros2_temp/referee_sub.cpp
        ros2_temp/coach_sub.cpp
        ros2_temp/soccer_config_client.cpp)
target_link_libraries(soccer PUBLIC
        ${SOCCER_COMMON_LIBS}
        ${SOCCER_COMMON_SYSTEM_LIBS}
        # Radio is needed for PacketConvert
        radio
        soccer_common
        rc-fshare)
target_include_directories(soccer PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(soccer SYSTEM PUBLIC)
ament_target_dependencies(soccer PUBLIC ${SOCCER_COMMON_AMENT_LIBS})

# Soccer binary
add_executable(soccer_node
        main.cpp
        ui/battery_widget.cpp
        ui/field_view.cpp
        ui/main_window.cpp
        ui/protobuf_tree.cpp
        ui/referee_tab.cpp
        ui/robot_status_widget.cpp
        ui/robot_widget.cpp
        ui/sim_field_view.cpp
        ui/strip_chart.cpp
        ui/style_sheet_manager.cpp
        ${SOCCER_UIS}
        ${SOCCER_RSRC})

# Set the directory so AUTOUIC know where to find .ui files
set(UI_PATH ${CMAKE_CURRENT_SOURCE_DIR}/ui/qt)
set_target_properties(soccer_node PROPERTIES AUTOUIC_SEARCH_PATHS ${UI_PATH})

target_link_libraries(soccer_node PUBLIC
        soccer
        Qt5::Widgets
        Qt5::Xml
        Qt5::Core
        Qt5::OpenGL
        Qt5::Network
        Qt5::Svg)
target_include_directories(soccer_node PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_BINARY_DIR}/soccer_autogen/include)

# Test
if (BUILD_TESTING)
    add_executable(soccer_test
            battery_profile_test.cpp
            ball_test.cpp
            logger_test.cpp
            world_state_test.cpp
            game_state_test.cpp)
    target_link_libraries(soccer_test PUBLIC soccer ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES})
    add_test(NAME soccer_test COMMAND soccer_test)
endif ()

# Global parameter server node
add_executable(global_param_server_node
        global_param_server.cpp)
target_link_libraries(global_param_server_node PUBLIC
        ${SOCCER_COMMON_LIBS}
        ${SOCCER_COMMON_SYSTEM_LIBS}
        soccer_common)

# Log viewer node
#add_executable(log_viewer)
#
#target_sources(log_viewer PRIVATE
#        log_viewer.cpp
#        ui/qt/log_icons.qrc
#        ui/qt/log_icons.qrc)
#
#set_target_properties(log_viewer PROPERTIES AUTOUIC_SEARCH_PATHS ${UI_PATH})
#
#target_link_libraries(log_viewer PRIVATE
#        Qt5::Core
#        Qt5::Widgets
#        Qt5::OpenGL
#        Qt5::Xml
#        Qt5::Svg
#        soccer
#        ${PROTOBUF_LIBRARIES}
#)

# ======================================================================
# Packaging
# ======================================================================
install(
        TARGETS soccer
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(
        TARGETS soccer_node
        internal_referee_node
        external_referee_node
        vision_filter_node
        planning_node
        control_node
        sim_radio_node
        coach_node
        network_radio_node
        joystick_node
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