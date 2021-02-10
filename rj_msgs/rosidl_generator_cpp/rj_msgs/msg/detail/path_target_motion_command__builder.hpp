// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rj_msgs:msg/PathTargetMotionCommand.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__PATH_TARGET_MOTION_COMMAND__BUILDER_HPP_
#define RJ_MSGS__MSG__DETAIL__PATH_TARGET_MOTION_COMMAND__BUILDER_HPP_

#include "rj_msgs/msg/detail/path_target_motion_command__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rj_msgs
{

namespace msg
{

namespace builder
{

class Init_PathTargetMotionCommand_override_face_point
{
public:
  explicit Init_PathTargetMotionCommand_override_face_point(::rj_msgs::msg::PathTargetMotionCommand & msg)
  : msg_(msg)
  {}
  ::rj_msgs::msg::PathTargetMotionCommand override_face_point(::rj_msgs::msg::PathTargetMotionCommand::_override_face_point_type arg)
  {
    msg_.override_face_point = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rj_msgs::msg::PathTargetMotionCommand msg_;
};

class Init_PathTargetMotionCommand_override_angle
{
public:
  explicit Init_PathTargetMotionCommand_override_angle(::rj_msgs::msg::PathTargetMotionCommand & msg)
  : msg_(msg)
  {}
  Init_PathTargetMotionCommand_override_face_point override_angle(::rj_msgs::msg::PathTargetMotionCommand::_override_angle_type arg)
  {
    msg_.override_angle = std::move(arg);
    return Init_PathTargetMotionCommand_override_face_point(msg_);
  }

private:
  ::rj_msgs::msg::PathTargetMotionCommand msg_;
};

class Init_PathTargetMotionCommand_target
{
public:
  Init_PathTargetMotionCommand_target()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PathTargetMotionCommand_override_angle target(::rj_msgs::msg::PathTargetMotionCommand::_target_type arg)
  {
    msg_.target = std::move(arg);
    return Init_PathTargetMotionCommand_override_angle(msg_);
  }

private:
  ::rj_msgs::msg::PathTargetMotionCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rj_msgs::msg::PathTargetMotionCommand>()
{
  return rj_msgs::msg::builder::Init_PathTargetMotionCommand_target();
}

}  // namespace rj_msgs

#endif  // RJ_MSGS__MSG__DETAIL__PATH_TARGET_MOTION_COMMAND__BUILDER_HPP_
