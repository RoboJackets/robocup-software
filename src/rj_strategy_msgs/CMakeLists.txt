cmake_minimum_required(VERSION 3.8)
project(rj_strategy_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}

  # Agent
  msg/AgentState.msg
  msg/AgentRequest.msg
  msg/AgentResponse.msg
  msg/AgentResponseVariant.msg

  # Coordinators
  msg/KickerPicker.msg
  msg/Marking.msg
  msg/Waller.msg

  # Agent Request Messages
  msg/request/BallInTransitRequest.msg
  msg/request/IncomingBallRequest.msg
  msg/request/JoinWallRequest.msg
  msg/request/LeaveWallRequest.msg
  msg/request/PassReceivedRequest.msg
  msg/request/PassRequest.msg
  msg/request/PositionRequest.msg
  msg/request/ResetScorerRequest.msg
  msg/request/ScorerRequest.msg
  msg/request/SeekerRequest.msg
  msg/request/TestRequest.msg

  # Agent Response Messages
  msg/response/Acknowledge.msg
  msg/response/JoinWallResponse.msg
  msg/response/LeaveWallResponse.msg
  msg/response/PassResponse.msg
  msg/response/PositionResponse.msg
  msg/response/ScorerResponse.msg
  msg/response/TestResponse.msg

  # Services
  srv/AgentCommunication.srv
  srv/KickerPicker.srv
  srv/Marking.srv
  srv/Waller.srv
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
