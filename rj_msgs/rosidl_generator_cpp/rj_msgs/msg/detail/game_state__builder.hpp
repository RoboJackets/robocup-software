// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rj_msgs:msg/GameState.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__GAME_STATE__BUILDER_HPP_
#define RJ_MSGS__MSG__DETAIL__GAME_STATE__BUILDER_HPP_

#include "rj_msgs/msg/detail/game_state__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rj_msgs
{

namespace msg
{

namespace builder
{

class Init_GameState_placement_point
{
public:
  explicit Init_GameState_placement_point(::rj_msgs::msg::GameState & msg)
  : msg_(msg)
  {}
  ::rj_msgs::msg::GameState placement_point(::rj_msgs::msg::GameState::_placement_point_type arg)
  {
    msg_.placement_point = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rj_msgs::msg::GameState msg_;
};

class Init_GameState_stage_time_left
{
public:
  explicit Init_GameState_stage_time_left(::rj_msgs::msg::GameState & msg)
  : msg_(msg)
  {}
  Init_GameState_placement_point stage_time_left(::rj_msgs::msg::GameState::_stage_time_left_type arg)
  {
    msg_.stage_time_left = std::move(arg);
    return Init_GameState_placement_point(msg_);
  }

private:
  ::rj_msgs::msg::GameState msg_;
};

class Init_GameState_our_restart
{
public:
  explicit Init_GameState_our_restart(::rj_msgs::msg::GameState & msg)
  : msg_(msg)
  {}
  Init_GameState_stage_time_left our_restart(::rj_msgs::msg::GameState::_our_restart_type arg)
  {
    msg_.our_restart = std::move(arg);
    return Init_GameState_stage_time_left(msg_);
  }

private:
  ::rj_msgs::msg::GameState msg_;
};

class Init_GameState_restart
{
public:
  explicit Init_GameState_restart(::rj_msgs::msg::GameState & msg)
  : msg_(msg)
  {}
  Init_GameState_our_restart restart(::rj_msgs::msg::GameState::_restart_type arg)
  {
    msg_.restart = std::move(arg);
    return Init_GameState_our_restart(msg_);
  }

private:
  ::rj_msgs::msg::GameState msg_;
};

class Init_GameState_state
{
public:
  explicit Init_GameState_state(::rj_msgs::msg::GameState & msg)
  : msg_(msg)
  {}
  Init_GameState_restart state(::rj_msgs::msg::GameState::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_GameState_restart(msg_);
  }

private:
  ::rj_msgs::msg::GameState msg_;
};

class Init_GameState_period
{
public:
  Init_GameState_period()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GameState_state period(::rj_msgs::msg::GameState::_period_type arg)
  {
    msg_.period = std::move(arg);
    return Init_GameState_state(msg_);
  }

private:
  ::rj_msgs::msg::GameState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rj_msgs::msg::GameState>()
{
  return rj_msgs::msg::builder::Init_GameState_period();
}

}  // namespace rj_msgs

#endif  // RJ_MSGS__MSG__DETAIL__GAME_STATE__BUILDER_HPP_
