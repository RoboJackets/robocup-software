# ======================================================================
# Preamble
# ======================================================================
cmake_minimum_required(VERSION 3.16)
project(rj_msgs)

# ======================================================================
# Find package
# ======================================================================
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rj_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# ======================================================================
# Define Targets
# ======================================================================
# This creates the target rj_msgs__rosidl_typesupport_cpp
rosidl_generate_interfaces(
  rj_msgs
  # Actions
  action/RobotMove.action

  # Messages
  msg/AgentState.msg
  msg/BallState.msg
  msg/BallPlacement.msg
  msg/CoachState.msg
  msg/DetectionBall.msg
  msg/DetectionFrame.msg
  msg/DetectionRobot.msg
  msg/FieldDimensions.msg
  msg/FieldOrientation.msg
  msg/GameSettings.msg
  msg/GlobalOverride.msg
  msg/Goalie.msg
  msg/ManipulatorSetpoint.msg
  msg/MatchState.msg
  msg/MotionSetpoint.msg

  msg/LinearMotionInstant.msg
  msg/EmptyMotionCommand.msg
  msg/GoalieIdleMotionCommand.msg
  msg/PathTargetMotionCommand.msg
  msg/WorldVelMotionCommand.msg
  msg/PivotMotionCommand.msg
  msg/SettleMotionCommand.msg
  msg/CollectMotionCommand.msg
  msg/LineKickMotionCommand.msg
  msg/InterceptMotionCommand.msg
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
  msg/PositionAssignment.msg
<<<<<<< HEAD
  msg/PositionAck.msg
  
=======

  # Communication
  msg/AgentRequest.msg
  msg/AgentResponse.msg
  msg/AgentResponseVariant.msg

  # Agent Request Messages
  request/PassRequest.msg
  request/PositionRequest.msg
  request/TestRequest.msg

  # Agent Response Messages
  response/Acknowledge.msg
  response/PassResponse.msg
  response/PositionResponse.msg
  response/TestResponse.msg

>>>>>>> 756fb2c4083511c580fb2e9bd9ec827a013b0a57
  # Services
  srv/AgentCommunication.srv
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
  rj_geometry_msgs)

# ======================================================================
# ROS2 packaging
# ======================================================================
ament_export_dependencies(rosidl_default_runtime)
ament_package()
