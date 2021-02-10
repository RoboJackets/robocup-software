// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rj_msgs:msg/RobotState.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__ROBOT_STATE__BUILDER_HPP_
#define RJ_MSGS__MSG__DETAIL__ROBOT_STATE__BUILDER_HPP_

#include "rj_msgs/msg/detail/robot_state__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rj_msgs
{

namespace msg
{

namespace builder
{

class Init_RobotState_visible
{
public:
  explicit Init_RobotState_visible(::rj_msgs::msg::RobotState & msg)
  : msg_(msg)
  {}
  ::rj_msgs::msg::RobotState visible(::rj_msgs::msg::RobotState::_visible_type arg)
  {
    msg_.visible = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rj_msgs::msg::RobotState msg_;
};

class Init_RobotState_velocity
{
public:
  explicit Init_RobotState_velocity(::rj_msgs::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_visible velocity(::rj_msgs::msg::RobotState::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_RobotState_visible(msg_);
  }

private:
  ::rj_msgs::msg::RobotState msg_;
};

class Init_RobotState_pose
{
public:
  explicit Init_RobotState_pose(::rj_msgs::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_velocity pose(::rj_msgs::msg::RobotState::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_RobotState_velocity(msg_);
  }

private:
  ::rj_msgs::msg::RobotState msg_;
};

class Init_RobotState_stamp
{
public:
  Init_RobotState_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotState_pose stamp(::rj_msgs::msg::RobotState::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_RobotState_pose(msg_);
  }

private:
  ::rj_msgs::msg::RobotState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rj_msgs::msg::RobotState>()
{
  return rj_msgs::msg::builder::Init_RobotState_stamp();
}

}  // namespace rj_msgs

#endif  // RJ_MSGS__MSG__DETAIL__ROBOT_STATE__BUILDER_HPP_
