cmake_minimum_required(VERSION 3.8)
project(rj_optimization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rj_common REQUIRED)
find_package(rj_geometry REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/gradient_ascent_1d.cpp
  src/nelder_mead_2d.cpp
  src/parallel_gradient_ascent_1d.cpp
  src/python_function_wrapper.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  ${PYTHON_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
  fmt
  spdlog
  ${PYTHON_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
  fmt
  spdlog
  PythonLibs
  rj_common
  rj_geometry
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

ament_export_dependencies(
  fmt
  spdlog
  PythonLibs
  rj_common
  rj_geometry
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

  find_package(ament_cmake_gtest REQUIRED)

  # Gradient Asceint 1D Test
  ament_add_gtest(gradient_ascient_1d_test tests/gradient_ascent_1d_test.cpp)
  target_include_directories(gradient_ascient_1d_test PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(gradient_ascient_1d_test ${PROJECT_NAME})

  # Nelder Mead 2D Test
  ament_add_gtest(nelder_mead_2d_test tests/nelder_mead_2d_test.cpp)
  target_include_directories(nelder_mead_2d_test PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(nelder_mead_2d_test ${PROJECT_NAME})

  # Parallel Gradient Ascent 1D Test
  ament_add_gtest(parallel_gradient_ascent_1d_test tests/parallel_gradient_ascent_1d_test.cpp)
  target_include_directories(parallel_gradient_ascent_1d_test PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(parallel_gradient_ascent_1d_test ${PROJECT_NAME})
endif()

ament_package()
