// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rj_msgs:msg/MotionSetpoint.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__MOTION_SETPOINT__BUILDER_HPP_
#define RJ_MSGS__MSG__DETAIL__MOTION_SETPOINT__BUILDER_HPP_

#include "rj_msgs/msg/detail/motion_setpoint__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rj_msgs
{

namespace msg
{

namespace builder
{

class Init_MotionSetpoint_velocity_z_radps
{
public:
  explicit Init_MotionSetpoint_velocity_z_radps(::rj_msgs::msg::MotionSetpoint & msg)
  : msg_(msg)
  {}
  ::rj_msgs::msg::MotionSetpoint velocity_z_radps(::rj_msgs::msg::MotionSetpoint::_velocity_z_radps_type arg)
  {
    msg_.velocity_z_radps = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rj_msgs::msg::MotionSetpoint msg_;
};

class Init_MotionSetpoint_velocity_y_mps
{
public:
  explicit Init_MotionSetpoint_velocity_y_mps(::rj_msgs::msg::MotionSetpoint & msg)
  : msg_(msg)
  {}
  Init_MotionSetpoint_velocity_z_radps velocity_y_mps(::rj_msgs::msg::MotionSetpoint::_velocity_y_mps_type arg)
  {
    msg_.velocity_y_mps = std::move(arg);
    return Init_MotionSetpoint_velocity_z_radps(msg_);
  }

private:
  ::rj_msgs::msg::MotionSetpoint msg_;
};

class Init_MotionSetpoint_velocity_x_mps
{
public:
  Init_MotionSetpoint_velocity_x_mps()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotionSetpoint_velocity_y_mps velocity_x_mps(::rj_msgs::msg::MotionSetpoint::_velocity_x_mps_type arg)
  {
    msg_.velocity_x_mps = std::move(arg);
    return Init_MotionSetpoint_velocity_y_mps(msg_);
  }

private:
  ::rj_msgs::msg::MotionSetpoint msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rj_msgs::msg::MotionSetpoint>()
{
  return rj_msgs::msg::builder::Init_MotionSetpoint_velocity_x_mps();
}

}  // namespace rj_msgs

#endif  // RJ_MSGS__MSG__DETAIL__MOTION_SETPOINT__BUILDER_HPP_
