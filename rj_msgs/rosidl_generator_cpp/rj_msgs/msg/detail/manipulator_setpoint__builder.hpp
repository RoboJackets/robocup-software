// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rj_msgs:msg/ManipulatorSetpoint.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__MANIPULATOR_SETPOINT__BUILDER_HPP_
#define RJ_MSGS__MSG__DETAIL__MANIPULATOR_SETPOINT__BUILDER_HPP_

#include "rj_msgs/msg/detail/manipulator_setpoint__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rj_msgs
{

namespace msg
{

namespace builder
{

class Init_ManipulatorSetpoint_dribbler_speed
{
public:
  explicit Init_ManipulatorSetpoint_dribbler_speed(::rj_msgs::msg::ManipulatorSetpoint & msg)
  : msg_(msg)
  {}
  ::rj_msgs::msg::ManipulatorSetpoint dribbler_speed(::rj_msgs::msg::ManipulatorSetpoint::_dribbler_speed_type arg)
  {
    msg_.dribbler_speed = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rj_msgs::msg::ManipulatorSetpoint msg_;
};

class Init_ManipulatorSetpoint_kick_strength
{
public:
  explicit Init_ManipulatorSetpoint_kick_strength(::rj_msgs::msg::ManipulatorSetpoint & msg)
  : msg_(msg)
  {}
  Init_ManipulatorSetpoint_dribbler_speed kick_strength(::rj_msgs::msg::ManipulatorSetpoint::_kick_strength_type arg)
  {
    msg_.kick_strength = std::move(arg);
    return Init_ManipulatorSetpoint_dribbler_speed(msg_);
  }

private:
  ::rj_msgs::msg::ManipulatorSetpoint msg_;
};

class Init_ManipulatorSetpoint_trigger_mode
{
public:
  explicit Init_ManipulatorSetpoint_trigger_mode(::rj_msgs::msg::ManipulatorSetpoint & msg)
  : msg_(msg)
  {}
  Init_ManipulatorSetpoint_kick_strength trigger_mode(::rj_msgs::msg::ManipulatorSetpoint::_trigger_mode_type arg)
  {
    msg_.trigger_mode = std::move(arg);
    return Init_ManipulatorSetpoint_kick_strength(msg_);
  }

private:
  ::rj_msgs::msg::ManipulatorSetpoint msg_;
};

class Init_ManipulatorSetpoint_shoot_mode
{
public:
  Init_ManipulatorSetpoint_shoot_mode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ManipulatorSetpoint_trigger_mode shoot_mode(::rj_msgs::msg::ManipulatorSetpoint::_shoot_mode_type arg)
  {
    msg_.shoot_mode = std::move(arg);
    return Init_ManipulatorSetpoint_trigger_mode(msg_);
  }

private:
  ::rj_msgs::msg::ManipulatorSetpoint msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rj_msgs::msg::ManipulatorSetpoint>()
{
  return rj_msgs::msg::builder::Init_ManipulatorSetpoint_shoot_mode();
}

}  // namespace rj_msgs

#endif  // RJ_MSGS__MSG__DETAIL__MANIPULATOR_SETPOINT__BUILDER_HPP_
