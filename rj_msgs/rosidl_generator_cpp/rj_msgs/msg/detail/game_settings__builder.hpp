// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rj_msgs:msg/GameSettings.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__GAME_SETTINGS__BUILDER_HPP_
#define RJ_MSGS__MSG__DETAIL__GAME_SETTINGS__BUILDER_HPP_

#include "rj_msgs/msg/detail/game_settings__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rj_msgs
{

namespace msg
{

namespace builder
{

class Init_GameSettings_paused
{
public:
  explicit Init_GameSettings_paused(::rj_msgs::msg::GameSettings & msg)
  : msg_(msg)
  {}
  ::rj_msgs::msg::GameSettings paused(::rj_msgs::msg::GameSettings::_paused_type arg)
  {
    msg_.paused = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rj_msgs::msg::GameSettings msg_;
};

class Init_GameSettings_use_their_half
{
public:
  explicit Init_GameSettings_use_their_half(::rj_msgs::msg::GameSettings & msg)
  : msg_(msg)
  {}
  Init_GameSettings_paused use_their_half(::rj_msgs::msg::GameSettings::_use_their_half_type arg)
  {
    msg_.use_their_half = std::move(arg);
    return Init_GameSettings_paused(msg_);
  }

private:
  ::rj_msgs::msg::GameSettings msg_;
};

class Init_GameSettings_use_our_half
{
public:
  explicit Init_GameSettings_use_our_half(::rj_msgs::msg::GameSettings & msg)
  : msg_(msg)
  {}
  Init_GameSettings_use_their_half use_our_half(::rj_msgs::msg::GameSettings::_use_our_half_type arg)
  {
    msg_.use_our_half = std::move(arg);
    return Init_GameSettings_use_their_half(msg_);
  }

private:
  ::rj_msgs::msg::GameSettings msg_;
};

class Init_GameSettings_defend_plus_x
{
public:
  explicit Init_GameSettings_defend_plus_x(::rj_msgs::msg::GameSettings & msg)
  : msg_(msg)
  {}
  Init_GameSettings_use_our_half defend_plus_x(::rj_msgs::msg::GameSettings::_defend_plus_x_type arg)
  {
    msg_.defend_plus_x = std::move(arg);
    return Init_GameSettings_use_our_half(msg_);
  }

private:
  ::rj_msgs::msg::GameSettings msg_;
};

class Init_GameSettings_request_goalie_id
{
public:
  explicit Init_GameSettings_request_goalie_id(::rj_msgs::msg::GameSettings & msg)
  : msg_(msg)
  {}
  Init_GameSettings_defend_plus_x request_goalie_id(::rj_msgs::msg::GameSettings::_request_goalie_id_type arg)
  {
    msg_.request_goalie_id = std::move(arg);
    return Init_GameSettings_defend_plus_x(msg_);
  }

private:
  ::rj_msgs::msg::GameSettings msg_;
};

class Init_GameSettings_request_blue_team
{
public:
  explicit Init_GameSettings_request_blue_team(::rj_msgs::msg::GameSettings & msg)
  : msg_(msg)
  {}
  Init_GameSettings_request_goalie_id request_blue_team(::rj_msgs::msg::GameSettings::_request_blue_team_type arg)
  {
    msg_.request_blue_team = std::move(arg);
    return Init_GameSettings_request_goalie_id(msg_);
  }

private:
  ::rj_msgs::msg::GameSettings msg_;
};

class Init_GameSettings_simulation
{
public:
  Init_GameSettings_simulation()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GameSettings_request_blue_team simulation(::rj_msgs::msg::GameSettings::_simulation_type arg)
  {
    msg_.simulation = std::move(arg);
    return Init_GameSettings_request_blue_team(msg_);
  }

private:
  ::rj_msgs::msg::GameSettings msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rj_msgs::msg::GameSettings>()
{
  return rj_msgs::msg::builder::Init_GameSettings_simulation();
}

}  // namespace rj_msgs

#endif  // RJ_MSGS__MSG__DETAIL__GAME_SETTINGS__BUILDER_HPP_
