cmake_minimum_required(VERSION 3.8)
project(rj_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rj_msgs REQUIRED)
find_package(rj_convert REQUIRED)
find_package(rj_protos REQUIRED)
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
find_package(rj_benchmarking REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/conversions.cpp
  src/logging.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
  rclcpp
  rj_msgs
  rj_convert
  rj_protos
  rj_benchmarking
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
  Qt5::Core
  Qt5::Widgets
  ${CMAKE_DL_LIBS}
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_export_dependencies(rclcpp rj_msgs rj_convert rj_protos rj_benchmarking)

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
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(${PROJECT_NAME})
ament_package()
