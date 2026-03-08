cmake_minimum_required(VERSION 3.8)
project(rj_control_extensions)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rj_geometry REQUIRED)
find_package(rj_convert REQUIRED)
find_package(rj_control_msgs REQUIRED)
find_package(rj_protos REQUIRED)

set(RJ_CONTROL_EXTENSIONS_DEPS
  rclcpp
  std_msgs
  rj_geometry
  rj_convert
  rj_control_msgs
  rj_protos
)

set(RJ_CONTROL_EXTENSIONS_SRC
  src/action.cpp
  src/control_command.cpp
)

add_library(${PROJECT_NAME} SHARED
  ${RJ_CONTROL_EXTENSIONS_SRC}
)

ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
  ${RJ_CONTROL_EXTENSIONS_DEPS}
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
  ${CMAKE_DL_LIBS}
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_export_dependencies(
  ${RJ_CONTROL_EXTENSIONS_DEPS}
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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(${PROJECT_NAME})
ament_package()
