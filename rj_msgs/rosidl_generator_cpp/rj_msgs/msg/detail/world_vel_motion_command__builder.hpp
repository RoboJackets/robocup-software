// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rj_msgs:msg/WorldVelMotionCommand.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__WORLD_VEL_MOTION_COMMAND__BUILDER_HPP_
#define RJ_MSGS__MSG__DETAIL__WORLD_VEL_MOTION_COMMAND__BUILDER_HPP_

#include "rj_msgs/msg/detail/world_vel_motion_command__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rj_msgs
{

namespace msg
{

namespace builder
{

class Init_WorldVelMotionCommand_world_vel
{
public:
  Init_WorldVelMotionCommand_world_vel()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rj_msgs::msg::WorldVelMotionCommand world_vel(::rj_msgs::msg::WorldVelMotionCommand::_world_vel_type arg)
  {
    msg_.world_vel = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rj_msgs::msg::WorldVelMotionCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rj_msgs::msg::WorldVelMotionCommand>()
{
  return rj_msgs::msg::builder::Init_WorldVelMotionCommand_world_vel();
}

}  // namespace rj_msgs

#endif  // RJ_MSGS__MSG__DETAIL__WORLD_VEL_MOTION_COMMAND__BUILDER_HPP_
