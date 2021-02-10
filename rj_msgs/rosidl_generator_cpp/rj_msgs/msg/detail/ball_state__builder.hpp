// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rj_msgs:msg/BallState.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__BALL_STATE__BUILDER_HPP_
#define RJ_MSGS__MSG__DETAIL__BALL_STATE__BUILDER_HPP_

#include "rj_msgs/msg/detail/ball_state__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rj_msgs
{

namespace msg
{

namespace builder
{

class Init_BallState_visible
{
public:
  explicit Init_BallState_visible(::rj_msgs::msg::BallState & msg)
  : msg_(msg)
  {}
  ::rj_msgs::msg::BallState visible(::rj_msgs::msg::BallState::_visible_type arg)
  {
    msg_.visible = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rj_msgs::msg::BallState msg_;
};

class Init_BallState_velocity
{
public:
  explicit Init_BallState_velocity(::rj_msgs::msg::BallState & msg)
  : msg_(msg)
  {}
  Init_BallState_visible velocity(::rj_msgs::msg::BallState::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_BallState_visible(msg_);
  }

private:
  ::rj_msgs::msg::BallState msg_;
};

class Init_BallState_position
{
public:
  explicit Init_BallState_position(::rj_msgs::msg::BallState & msg)
  : msg_(msg)
  {}
  Init_BallState_velocity position(::rj_msgs::msg::BallState::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_BallState_velocity(msg_);
  }

private:
  ::rj_msgs::msg::BallState msg_;
};

class Init_BallState_stamp
{
public:
  Init_BallState_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BallState_position stamp(::rj_msgs::msg::BallState::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_BallState_position(msg_);
  }

private:
  ::rj_msgs::msg::BallState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rj_msgs::msg::BallState>()
{
  return rj_msgs::msg::builder::Init_BallState_stamp();
}

}  // namespace rj_msgs

#endif  // RJ_MSGS__MSG__DETAIL__BALL_STATE__BUILDER_HPP_
