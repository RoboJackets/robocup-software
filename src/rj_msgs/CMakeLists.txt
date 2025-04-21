cmake_minimum_required(VERSION 3.8)
project(rj_msgs)

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
  # Actions
  action/RobotMove.action

  # Messages
  msg/AgentState.msg
  msg/AliveRobots.msg
  msg/BallState.msg
  msg/BallPlacement.msg
  msg/DetectionBall.msg
  msg/DetectionFrame.msg
  msg/DetectionRobot.msg
  msg/FieldDimensions.msg
  msg/FieldOrientation.msg
  msg/GameSettings.msg
  msg/Goalie.msg
  msg/ManipulatorSetpoint.msg
  msg/MatchState.msg
  msg/MotionSetpoint.msg

  msg/LinearMotionInstant.msg
  msg/MotionCommand.msg

  msg/RawProtobuf.msg
  msg/RobotIntent.msg
  msg/RobotInstant.msg
  msg/RobotPlacement.msg
  msg/RobotState.msg
  msg/RobotStatus.msg
  msg/PlayState.msg
  msg/TeamColor.msg
  msg/TeamInfo.msg
  msg/Trajectory.msg
  msg/WorldState.msg
  msg/OverridePosition.msg

  # Communication
  msg/AgentRequest.msg
  msg/AgentResponse.msg
  msg/AgentResponseVariant.msg

  # Coordinators
  msg/KickerPicker.msg
  msg/Marking.msg

  # Agent Request Messages
  request/JoinWallRequest.msg
  request/TestRequest.msg
  request/PassRequest.msg
  request/ScorerRequest.msg
  request/BallInTransitRequest.msg
  request/SeekerRequest.msg
  request/PositionRequest.msg
  request/LeaveWallRequest.msg
  request/ResetScorerRequest.msg
  request/IncomingBallRequest.msg

  # Agent Response Messages
  response/ScorerResponse.msg
  response/LeaveWallResponse.msg
  response/PositionResponse.msg
  response/TestResponse.msg
  response/PassResponse.msg
  response/Acknowledge.msg
  response/JoinWallResponse.msg

  # Services
  srv/AgentCommunication.srv
  srv/KickerPicker.srv
  srv/Marking.srv
  srv/ListJoysticks.srv
  srv/PlanHypotheticalPath.srv
  srv/QuickCommands.srv
  srv/QuickRestart.srv
  srv/SetFieldDimensions.srv
  srv/SetGameSettings.srv
  srv/SimPlacement.srv
  srv/SetManual.srv

  DEPENDENCIES
  std_msgs
  builtin_interfaces
  rj_geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
