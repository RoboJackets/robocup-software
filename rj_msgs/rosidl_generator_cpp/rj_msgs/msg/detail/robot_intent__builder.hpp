// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rj_msgs:msg/RobotIntent.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__ROBOT_INTENT__BUILDER_HPP_
#define RJ_MSGS__MSG__DETAIL__ROBOT_INTENT__BUILDER_HPP_

#include "rj_msgs/msg/detail/robot_intent__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rj_msgs
{

namespace msg
{

namespace builder
{

class Init_RobotIntent_priority
{
public:
  explicit Init_RobotIntent_priority(::rj_msgs::msg::RobotIntent & msg)
  : msg_(msg)
  {}
  ::rj_msgs::msg::RobotIntent priority(::rj_msgs::msg::RobotIntent::_priority_type arg)
  {
    msg_.priority = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rj_msgs::msg::RobotIntent msg_;
};

class Init_RobotIntent_is_active
{
public:
  explicit Init_RobotIntent_is_active(::rj_msgs::msg::RobotIntent & msg)
  : msg_(msg)
  {}
  Init_RobotIntent_priority is_active(::rj_msgs::msg::RobotIntent::_is_active_type arg)
  {
    msg_.is_active = std::move(arg);
    return Init_RobotIntent_priority(msg_);
  }

private:
  ::rj_msgs::msg::RobotIntent msg_;
};

class Init_RobotIntent_dribbler_speed
{
public:
  explicit Init_RobotIntent_dribbler_speed(::rj_msgs::msg::RobotIntent & msg)
  : msg_(msg)
  {}
  Init_RobotIntent_is_active dribbler_speed(::rj_msgs::msg::RobotIntent::_dribbler_speed_type arg)
  {
    msg_.dribbler_speed = std::move(arg);
    return Init_RobotIntent_is_active(msg_);
  }

private:
  ::rj_msgs::msg::RobotIntent msg_;
};

class Init_RobotIntent_kick_speed
{
public:
  explicit Init_RobotIntent_kick_speed(::rj_msgs::msg::RobotIntent & msg)
  : msg_(msg)
  {}
  Init_RobotIntent_dribbler_speed kick_speed(::rj_msgs::msg::RobotIntent::_kick_speed_type arg)
  {
    msg_.kick_speed = std::move(arg);
    return Init_RobotIntent_dribbler_speed(msg_);
  }

private:
  ::rj_msgs::msg::RobotIntent msg_;
};

class Init_RobotIntent_trigger_mode
{
public:
  explicit Init_RobotIntent_trigger_mode(::rj_msgs::msg::RobotIntent & msg)
  : msg_(msg)
  {}
  Init_RobotIntent_kick_speed trigger_mode(::rj_msgs::msg::RobotIntent::_trigger_mode_type arg)
  {
    msg_.trigger_mode = std::move(arg);
    return Init_RobotIntent_kick_speed(msg_);
  }

private:
  ::rj_msgs::msg::RobotIntent msg_;
};

class Init_RobotIntent_shoot_mode
{
public:
  explicit Init_RobotIntent_shoot_mode(::rj_msgs::msg::RobotIntent & msg)
  : msg_(msg)
  {}
  Init_RobotIntent_trigger_mode shoot_mode(::rj_msgs::msg::RobotIntent::_shoot_mode_type arg)
  {
    msg_.shoot_mode = std::move(arg);
    return Init_RobotIntent_trigger_mode(msg_);
  }

private:
  ::rj_msgs::msg::RobotIntent msg_;
};

class Init_RobotIntent_local_obstacles
{
public:
  explicit Init_RobotIntent_local_obstacles(::rj_msgs::msg::RobotIntent & msg)
  : msg_(msg)
  {}
  Init_RobotIntent_shoot_mode local_obstacles(::rj_msgs::msg::RobotIntent::_local_obstacles_type arg)
  {
    msg_.local_obstacles = std::move(arg);
    return Init_RobotIntent_shoot_mode(msg_);
  }

private:
  ::rj_msgs::msg::RobotIntent msg_;
};

class Init_RobotIntent_motion_command
{
public:
  Init_RobotIntent_motion_command()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotIntent_local_obstacles motion_command(::rj_msgs::msg::RobotIntent::_motion_command_type arg)
  {
    msg_.motion_command = std::move(arg);
    return Init_RobotIntent_local_obstacles(msg_);
  }

private:
  ::rj_msgs::msg::RobotIntent msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rj_msgs::msg::RobotIntent>()
{
  return rj_msgs::msg::builder::Init_RobotIntent_motion_command();
}

}  // namespace rj_msgs

#endif  // RJ_MSGS__MSG__DETAIL__ROBOT_INTENT__BUILDER_HPP_
