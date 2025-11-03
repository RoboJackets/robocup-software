cmake_minimum_required(VERSION 3.8)
project(rj_benchmarking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)

set(BENCHMARKING_DEPS
    fmt
    rclcpp
    spdlog
)

set(BENCHMARKING_LIBS
    fmt
    spdlog
)

set(BENCHMARKING_SRC
    src/benchmarking.cpp
    src/registry.cpp
    src/timer.cpp
)

add_executable(${PROJECT_NAME}_node
    ${BENCHMARKING_SRC}
    src/main.cpp
)

target_link_libraries(${PROJECT_NAME}_node PUBLIC ${BENCHMARKING_LIBS})

target_include_directories(${PROJECT_NAME}_node
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_node PUBLIC ${BENCHMARKING_DEPS})

install(
  DIRECTORY include/
  DESTINATION include
)

install(
    TARGETS ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
