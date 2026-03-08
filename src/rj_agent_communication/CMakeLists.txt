cmake_minimum_required(VERSION 3.8)
project(rj_agent_communication)

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
find_package(rj_common REQUIRED)
find_package(rj_convert REQUIRED)
find_package(rj_agent_msgs REQUIRED)
find_package(rj_geometry REQUIRED)
find_package(rj_geometry_msgs REQUIRED)

set(RJ_AGENT_MSG_DEPS
  rj_common
  rj_convert
  rj_agent_msgs
  rj_geometry
  rj_geometry_msgs
)

# rj_agent_msg_extensions library
add_library(${PROJECT_NAME} SHARED
  src/communication.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
  ${RJ_AGENT_MSG_DEPS}
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

ament_export_targets(${PROJECT_NAME})
ament_package()
