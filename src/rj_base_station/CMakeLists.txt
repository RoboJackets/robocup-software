cmake_minimum_required(VERSION 3.8)
project(rj_base_station)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(WARNINGS_AS_ERRORS FALSE)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-comment)
endif()

add_subdirectory(external/RF24)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rj_msgs REQUIRED)
find_package(rj_constants REQUIRED)

find_library(GPIOD_LIBRARY gpiod REQUIRED)

set(RJ_BASE_STATION_DEPS
  rclcpp
  rclcpp_components
  rj_msgs
  rj_constants
)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE include)
ament_target_dependencies(${PROJECT_NAME} INTERFACE ${RJ_BASE_STATION_DEPS})
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(radio_node
  src/rf_radio_node.cpp
)

add_executable(alive_robots_node
  src/alive_robots_node.cpp
)

# Include RF24 Radio Library with the radio node
target_link_libraries(radio_node
  PUBLIC
  rf24
  pthread
  fmt
  ${CMAKE_DL_LIBS}
  ${GPIOD_LIBRARY}
)

target_include_directories(radio_node
  PUBLIC
  external/RF24
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(alive_robots_node
  PUBLIC
  pthread
  fmt
  ${CMAKE_DL_LIBS}
)

target_include_directories(alive_robots_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(radio_node
  PUBLIC
  ${RJ_BASE_STATION_DEPS}
)

ament_target_dependencies(alive_robots_node
  PUBLIC
  ${RJ_BASE_STATION_DEPS}
)

ament_export_dependencies(
  ${RJ_BASE_STATION_DEPS}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS radio_node alive_robots_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(message_packing_test tests/test_message_packing.cpp)
  target_include_directories(message_packing_test PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_link_libraries(message_packing_test ${PROJECT_NAME})
endif()

ament_export_include_directories(include)
ament_package()
