cmake_minimum_required(VERSION 3.8)
project(rj_benchmarking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rj_msgs REQUIRED)
find_package(rj_constants REQUIRED)

set(BENCHMARKING_DEPS
    fmt
    rclcpp
    spdlog
    rj_msgs
    rj_constants
)

set(BENCHMARKING_LIBS
    fmt
    spdlog
)

set(BENCHMARKING_SRC
    src/registry.cpp
    src/timer.cpp
    src/registry_publisher.cpp
)

# RJ Benchmarking Library
add_library(${PROJECT_NAME} SHARED
    ${BENCHMARKING_SRC}
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
    ${BENCHMARKING_LIBS}
)

ament_target_dependencies(${PROJECT_NAME}
    PUBLIC
    ${BENCHMARKING_DEPS}
)

# Benchmarking Node
add_executable(${PROJECT_NAME}_node
    src/main.cpp
)

target_include_directories(${PROJECT_NAME}_node
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_node
    PUBLIC
    ${PROJECT_NAME}
)

ament_target_dependencies(${PROJECT_NAME}_node
    PUBLIC
    ${BENCHMARKING_DEPS}
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
    TARGETS ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(
    ${BENCHMARKING_DEPS}
)

ament_export_include_directories(include)

ament_export_targets(${PROJECT_NAME})
ament_package()
