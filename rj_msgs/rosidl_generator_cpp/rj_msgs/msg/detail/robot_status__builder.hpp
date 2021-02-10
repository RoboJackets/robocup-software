// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rj_msgs:msg/RobotStatus.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__ROBOT_STATUS__BUILDER_HPP_
#define RJ_MSGS__MSG__DETAIL__ROBOT_STATUS__BUILDER_HPP_

#include "rj_msgs/msg/detail/robot_status__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rj_msgs
{

namespace msg
{

namespace builder
{

class Init_RobotStatus_encoder_deltas
{
public:
  explicit Init_RobotStatus_encoder_deltas(::rj_msgs::msg::RobotStatus & msg)
  : msg_(msg)
  {}
  ::rj_msgs::msg::RobotStatus encoder_deltas(::rj_msgs::msg::RobotStatus::_encoder_deltas_type arg)
  {
    msg_.encoder_deltas = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rj_msgs::msg::RobotStatus msg_;
};

class Init_RobotStatus_fpga_error
{
public:
  explicit Init_RobotStatus_fpga_error(::rj_msgs::msg::RobotStatus & msg)
  : msg_(msg)
  {}
  Init_RobotStatus_encoder_deltas fpga_error(::rj_msgs::msg::RobotStatus::_fpga_error_type arg)
  {
    msg_.fpga_error = std::move(arg);
    return Init_RobotStatus_encoder_deltas(msg_);
  }

private:
  ::rj_msgs::msg::RobotStatus msg_;
};

class Init_RobotStatus_kicker_healthy
{
public:
  explicit Init_RobotStatus_kicker_healthy(::rj_msgs::msg::RobotStatus & msg)
  : msg_(msg)
  {}
  Init_RobotStatus_fpga_error kicker_healthy(::rj_msgs::msg::RobotStatus::_kicker_healthy_type arg)
  {
    msg_.kicker_healthy = std::move(arg);
    return Init_RobotStatus_fpga_error(msg_);
  }

private:
  ::rj_msgs::msg::RobotStatus msg_;
};

class Init_RobotStatus_kicker_charged
{
public:
  explicit Init_RobotStatus_kicker_charged(::rj_msgs::msg::RobotStatus & msg)
  : msg_(msg)
  {}
  Init_RobotStatus_kicker_healthy kicker_charged(::rj_msgs::msg::RobotStatus::_kicker_charged_type arg)
  {
    msg_.kicker_charged = std::move(arg);
    return Init_RobotStatus_kicker_healthy(msg_);
  }

private:
  ::rj_msgs::msg::RobotStatus msg_;
};

class Init_RobotStatus_has_ball_sense
{
public:
  explicit Init_RobotStatus_has_ball_sense(::rj_msgs::msg::RobotStatus & msg)
  : msg_(msg)
  {}
  Init_RobotStatus_kicker_charged has_ball_sense(::rj_msgs::msg::RobotStatus::_has_ball_sense_type arg)
  {
    msg_.has_ball_sense = std::move(arg);
    return Init_RobotStatus_kicker_charged(msg_);
  }

private:
  ::rj_msgs::msg::RobotStatus msg_;
};

class Init_RobotStatus_motor_errors
{
public:
  explicit Init_RobotStatus_motor_errors(::rj_msgs::msg::RobotStatus & msg)
  : msg_(msg)
  {}
  Init_RobotStatus_has_ball_sense motor_errors(::rj_msgs::msg::RobotStatus::_motor_errors_type arg)
  {
    msg_.motor_errors = std::move(arg);
    return Init_RobotStatus_has_ball_sense(msg_);
  }

private:
  ::rj_msgs::msg::RobotStatus msg_;
};

class Init_RobotStatus_battery_voltage
{
public:
  explicit Init_RobotStatus_battery_voltage(::rj_msgs::msg::RobotStatus & msg)
  : msg_(msg)
  {}
  Init_RobotStatus_motor_errors battery_voltage(::rj_msgs::msg::RobotStatus::_battery_voltage_type arg)
  {
    msg_.battery_voltage = std::move(arg);
    return Init_RobotStatus_motor_errors(msg_);
  }

private:
  ::rj_msgs::msg::RobotStatus msg_;
};

class Init_RobotStatus_robot_id
{
public:
  Init_RobotStatus_robot_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotStatus_battery_voltage robot_id(::rj_msgs::msg::RobotStatus::_robot_id_type arg)
  {
    msg_.robot_id = std::move(arg);
    return Init_RobotStatus_battery_voltage(msg_);
  }

private:
  ::rj_msgs::msg::RobotStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rj_msgs::msg::RobotStatus>()
{
  return rj_msgs::msg::builder::Init_RobotStatus_robot_id();
}

}  // namespace rj_msgs

#endif  // RJ_MSGS__MSG__DETAIL__ROBOT_STATUS__BUILDER_HPP_
