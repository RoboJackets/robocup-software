cmake_minimum_required(VERSION 3.8)
project(rj_ui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(Qt5 COMPONENTS Widgets Xml Core OpenGL Network Svg REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Boost REQUIRED)
find_package(Protobuf REQUIRED)
find_package(std_msgs REQUIRED)
find_package(fmt REQUIRED)
find_package(Flann REQUIRED)
find_package(rj_common REQUIRED)
find_package(rj_param_utils REQUIRED)
find_package(rj_protos REQUIRED)
find_package(rj_geometry REQUIRED)
find_package(rj_utils REQUIRED)
find_package(rj_planning REQUIRED)
find_package(rj_constants REQUIRED)
find_package(rj_convert REQUIRED)
find_package(rj_msgs REQUIRED)
find_package(rj_strategy REQUIRED)
find_package(rj_referee REQUIRED)
find_package(rj_config_client REQUIRED)
find_package(rj_topic_utils REQUIRED)

# Enable Qt's automatic processing
set(CMAKE_AUTOMOC ON)   # Handle Q_OBJECT in headers automatically
set(CMAKE_AUTOUIC ON)   # Handle .ui → ui_*.h automatically
set(CMAKE_AUTORCC ON)   # Handle .qrc → resources automatically

set(CMAKE_AUTOUIC_SEARCH_PATHS
  ${CMAKE_CURRENT_SOURCE_DIR}/qt
)

set(SRC_FILES
  src/battery_profile.cpp
  src/battery_widget.cpp
  src/field_view.cpp
  src/main_window.cpp
  src/processor.cpp
  src/referee_tab.cpp
  src/robot_status_widget.cpp
  src/robot_widget.cpp
  src/style_sheet_manager.cpp

  src/ros2_temp/autonomy_interface.cpp
  src/ros2_temp/debug_draw_interface.cpp
  src/ros2_temp/raw_vision_packet_sub.cpp
  src/ros2_temp/referee_sub.cpp
  src/ros2_temp/soccer_config_client.cpp

  # QT MOC Headers (need to be included here so qt generates MOC files properly)
  include/rj_ui/main_window.hpp
  include/rj_ui/referee_tab.hpp

  # QT UI Files
  qt/ConfigFileTab.ui
  qt/MainWindow.ui
  qt/RefereeTab.ui
  qt/RobotStatusWidget.ui

  # QT QRC Files
  qt/main_icons.qrc
)

set(RJ_UI_NODE_AUTOGEN_FILES
  include/rj_ui/main_window.hpp
  include/rj_ui/robot_status_widget.hpp
  qt/MainWindow.ui
  qt/RobotStatusWidget.ui
)

set(RJ_UI_DEPS
  rclcpp
  Qt5
  ament_index_cpp
  Boost
  Protobuf
  spdlog
  std_msgs
  fmt
  Flann
  rj_common
  rj_param_utils
  rj_protos
  rj_geometry
  rj_utils
  rj_planning
  rj_constants
  rj_convert
  rj_msgs
  rj_strategy
  rj_referee
  rj_config_client
  rj_topic_utils
)

add_library(${PROJECT_NAME} SHARED
  ${SRC_FILES}
)

add_executable(${PROJECT_NAME}_node
  src/main.cpp
  ${RJ_UI_NODE_AUTOGEN_FILES}
)

target_include_directories(${PROJECT_NAME}_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${CMAKE_CURRENT_BINARY_DIR}
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${CMAKE_CURRENT_BINARY_DIR}
)

target_link_libraries(${PROJECT_NAME}_node
  PRIVATE
  ${PROJECT_NAME}
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
  Boost::boost
  Qt5::Widgets
  Qt5::Xml
  Qt5::Core
  Qt5::OpenGL
  Qt5::Network
  Qt5::Svg
  protobuf::libprotobuf
  spdlog
  fmt
  ${FLANN_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}_node
  PUBLIC
  ${RJ_UI_DEPS}
)

ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
  ${RJ_UI_DEPS}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(battery_profile_test tests/battery_profile_test.cpp)
  target_include_directories(battery_profile_test PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_link_libraries(battery_profile_test ${PROJECT_NAME})
endif()

ament_package()
