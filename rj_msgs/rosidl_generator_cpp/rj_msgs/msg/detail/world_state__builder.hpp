// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rj_msgs:msg/WorldState.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__WORLD_STATE__BUILDER_HPP_
#define RJ_MSGS__MSG__DETAIL__WORLD_STATE__BUILDER_HPP_

#include "rj_msgs/msg/detail/world_state__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rj_msgs
{

namespace msg
{

namespace builder
{

class Init_WorldState_ball
{
public:
  explicit Init_WorldState_ball(::rj_msgs::msg::WorldState & msg)
  : msg_(msg)
  {}
  ::rj_msgs::msg::WorldState ball(::rj_msgs::msg::WorldState::_ball_type arg)
  {
    msg_.ball = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rj_msgs::msg::WorldState msg_;
};

class Init_WorldState_our_robots
{
public:
  explicit Init_WorldState_our_robots(::rj_msgs::msg::WorldState & msg)
  : msg_(msg)
  {}
  Init_WorldState_ball our_robots(::rj_msgs::msg::WorldState::_our_robots_type arg)
  {
    msg_.our_robots = std::move(arg);
    return Init_WorldState_ball(msg_);
  }

private:
  ::rj_msgs::msg::WorldState msg_;
};

class Init_WorldState_their_robots
{
public:
  explicit Init_WorldState_their_robots(::rj_msgs::msg::WorldState & msg)
  : msg_(msg)
  {}
  Init_WorldState_our_robots their_robots(::rj_msgs::msg::WorldState::_their_robots_type arg)
  {
    msg_.their_robots = std::move(arg);
    return Init_WorldState_our_robots(msg_);
  }

private:
  ::rj_msgs::msg::WorldState msg_;
};

class Init_WorldState_last_update_time
{
public:
  Init_WorldState_last_update_time()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WorldState_their_robots last_update_time(::rj_msgs::msg::WorldState::_last_update_time_type arg)
  {
    msg_.last_update_time = std::move(arg);
    return Init_WorldState_their_robots(msg_);
  }

private:
  ::rj_msgs::msg::WorldState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rj_msgs::msg::WorldState>()
{
  return rj_msgs::msg::builder::Init_WorldState_last_update_time();
}

}  // namespace rj_msgs

#endif  // RJ_MSGS__MSG__DETAIL__WORLD_STATE__BUILDER_HPP_
