// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rj_msgs:msg/RobotInstant.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__ROBOT_INSTANT__BUILDER_HPP_
#define RJ_MSGS__MSG__DETAIL__ROBOT_INSTANT__BUILDER_HPP_

#include "rj_msgs/msg/detail/robot_instant__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rj_msgs
{

namespace msg
{

namespace builder
{

class Init_RobotInstant_velocity
{
public:
  explicit Init_RobotInstant_velocity(::rj_msgs::msg::RobotInstant & msg)
  : msg_(msg)
  {}
  ::rj_msgs::msg::RobotInstant velocity(::rj_msgs::msg::RobotInstant::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rj_msgs::msg::RobotInstant msg_;
};

class Init_RobotInstant_pose
{
public:
  explicit Init_RobotInstant_pose(::rj_msgs::msg::RobotInstant & msg)
  : msg_(msg)
  {}
  Init_RobotInstant_velocity pose(::rj_msgs::msg::RobotInstant::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_RobotInstant_velocity(msg_);
  }

private:
  ::rj_msgs::msg::RobotInstant msg_;
};

class Init_RobotInstant_stamp
{
public:
  Init_RobotInstant_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotInstant_pose stamp(::rj_msgs::msg::RobotInstant::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_RobotInstant_pose(msg_);
  }

private:
  ::rj_msgs::msg::RobotInstant msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rj_msgs::msg::RobotInstant>()
{
  return rj_msgs::msg::builder::Init_RobotInstant_stamp();
}

}  // namespace rj_msgs

#endif  // RJ_MSGS__MSG__DETAIL__ROBOT_INSTANT__BUILDER_HPP_
