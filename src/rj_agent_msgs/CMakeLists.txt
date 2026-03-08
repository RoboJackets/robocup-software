cmake_minimum_required(VERSION 3.8)
project(rj_agent_msgs)

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

find_package(rosidl_default_generators REQUIRED)
find_package(rj_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  msg/AgentRequest.msg
  msg/AgentResponse.msg
  msg/AgentResponseVariant.msg
  msg/AgentState.msg

  # Requests
  request/BallInTransitRequest.msg
  request/IncomingBallRequest.msg
  request/JoinWallRequest.msg
  request/LeaveWallRequest.msg
  request/PassRequest.msg
  request/PositionRequest.msg
  request/ResetScorerRequest.msg
  request/ScorerRequest.msg
  request/SeekerRequest.msg
  request/TestRequest.msg

  # Responses
  response/Acknowledge.msg
  response/JoinWallResponse.msg
  response/LeaveWallResponse.msg
  response/PassResponse.msg
  response/PositionResponse.msg
  response/ScorerResponse.msg
  response/TestResponse.msg

  # Services
  srv/AgentCommunication.srv

  DEPENDENCIES
  std_msgs
  builtin_interfaces
  rj_geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
