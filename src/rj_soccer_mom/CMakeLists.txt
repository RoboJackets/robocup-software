cmake_minimum_required(VERSION 3.8)
project(rj_soccer_mom)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rj_msgs REQUIRED)

# 2. Build the executable
add_executable(soccer_mom 
  src/soccer_mom.cpp
)

# 3. Include local headers
target_include_directories(soccer_mom PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>$<INSTALL_INTERFACE:include>
)

# 4. Link dependencies
ament_target_dependencies(soccer_mom
  rclcpp
  std_msgs
  rj_msgs
)

# 5. Install target so `ros2 run` can find it
install(TARGETS
  soccer_mom
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

# all this was gemini'ed