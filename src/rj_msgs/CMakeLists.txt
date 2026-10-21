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

  # UI
  msg/ui_msgs/BallPlacement.msg
  msg/ui_msgs/OverridePosition.msg
  msg/ui_msgs/RobotPlacement.msg
  msg/ui_msgs/SimPlacement.srv

  # Config
  msg/config_msgs/FieldDimensions.msg
  msg/config_msgs/GameSettings.msg
  msg/config_msgs/SetFieldDimensions.srv
  msg/config_msgs/SetGameSettings.srv

  # Joystick
  msg/joystick_msgs/ManipulatorSetpoint.msg
  msg/joystick_msgs/ListJoysticks.srv
  msg/joystick_msgs/SetManual.srv

  # Benchmarking
  msg/benchmarking_msgs/Latency.msg

  # Vision
  msg/vision_msgs/BallState.msg
  msg/vision_msgs/RobotState.msg
  msg/vision_msgs/DetectionBall.msg
  msg/vision_msgs/DetectionFrame.msg
  msg/vision_msgs/DetectionRobot.msg
  msg/vision_msgs/RawProtobuf.msg

  # Motion
  msg/motion_msgs/MotionSetpoint.msg
  msg/motion_msgs/LinearMotionInstant.msg
  msg/motion_msgs/MotionCommand.msg
  msg/motion_msgs/RobotInstant.msg
  msg/motion_msgs/Trajectory.msg

  # Radio
  msg/radio_msgs/AliveRobots.msg
  msg/radio_msgs/RobotStatus.msg

  # Planning
  msg/planning_msgs/RobotIntent.msg
  msg/planning_msgs/PlanHypotheticalPath.srv

  # Referee
  msg/referee_msgs/PlayState.msg
  msg/referee_msgs/TeamColor.msg
  msg/referee_msgs/TeamInfo.msg
  msg/referee_msgs/WorldState.msg
  msg/referee_msgs/GoalieID.msg
  msg/referee_msgs/MatchState.msg
  msg/referee_msgs/QuickCommands.srv

  # Strategy
  msg/strategy_msgs/agent/AgentState.msg
  msg/strategy_msgs/agent/AgentRequest.msg
  msg/strategy_msgs/agent/AgentResponse.msg
  msg/strategy_msgs/agent/AgentResponseVariant.msg
  msg/strategy_msgs/agent/AgentCommunication.srv

  # Coordinators
  msg/strategy_msgs/coordinators/KickerPicker.msg
  msg/strategy_msgs/coordinators/Marking.msg
  msg/strategy_msgs/coordinators/Waller.msg
  msg/strategy_msgs/coordinators/KickerPicker.srv
  msg/strategy_msgs/coordinators/Marking.srv
  msg/strategy_msgs/coordinators/Waller.srv

  # Agent Request Messages
  msg/strategy_msgs/request/JoinWallRequest.msg
  msg/strategy_msgs/request/TestRequest.msg
  msg/strategy_msgs/request/PassRequest.msg
  msg/strategy_msgs/request/ScorerRequest.msg
  msg/strategy_msgs/request/BallInTransitRequest.msg
  msg/strategy_msgs/request/SeekerRequest.msg
  msg/strategy_msgs/request/PositionRequest.msg
  msg/strategy_msgs/request/LeaveWallRequest.msg
  msg/strategy_msgs/request/ResetScorerRequest.msg
  msg/strategy_msgs/request/IncomingBallRequest.msg
  msg/strategy_msgs/request/PassReceivedRequest.msg

  # Agent Response Messages
  msg/strategy_msgs/response/ScorerResponse.msg
  msg/strategy_msgs/response/LeaveWallResponse.msg
  msg/strategy_msgs/response/PositionResponse.msg
  msg/strategy_msgs/response/TestResponse.msg
  msg/strategy_msgs/response/PassResponse.msg
  msg/strategy_msgs/response/Acknowledge.msg
  msg/strategy_msgs/response/JoinWallResponse.msg

  
  DEPENDENCIES
  std_msgs
  builtin_interfaces
  rj_geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
