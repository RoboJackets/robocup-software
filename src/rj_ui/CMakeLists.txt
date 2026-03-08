cmake_minimum_required(VERSION 3.8)
project(rj_ui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Build frontend
message("Building Frontend")
add_custom_target(build_frontend ALL
  COMMAND npm install
  COMMAND npm run build
  COMMAND neu build
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/rj-ui
)

message("Installing Neutralino App")
install(
  PROGRAMS rj-ui/dist/rj-ui/rj-ui-linux_x64
  DESTINATION share/${PROJECT_NAME}/bin
)
install(
  FILES rj-ui/dist/rj-ui/resources.neu
  DESTINATION share/${PROJECT_NAME}/bin
)

# Install launcher
message("Installing the Launcher")
install(
  PROGRAMS scripts/rj_ui
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
