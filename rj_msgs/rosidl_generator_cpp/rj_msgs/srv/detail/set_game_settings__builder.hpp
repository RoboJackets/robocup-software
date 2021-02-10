// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rj_msgs:srv/SetGameSettings.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__SRV__DETAIL__SET_GAME_SETTINGS__BUILDER_HPP_
#define RJ_MSGS__SRV__DETAIL__SET_GAME_SETTINGS__BUILDER_HPP_

#include "rj_msgs/srv/detail/set_game_settings__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rj_msgs
{

namespace srv
{

namespace builder
{

class Init_SetGameSettings_Request_game_settings
{
public:
  Init_SetGameSettings_Request_game_settings()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rj_msgs::srv::SetGameSettings_Request game_settings(::rj_msgs::srv::SetGameSettings_Request::_game_settings_type arg)
  {
    msg_.game_settings = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rj_msgs::srv::SetGameSettings_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rj_msgs::srv::SetGameSettings_Request>()
{
  return rj_msgs::srv::builder::Init_SetGameSettings_Request_game_settings();
}

}  // namespace rj_msgs


namespace rj_msgs
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rj_msgs::srv::SetGameSettings_Response>()
{
  return ::rj_msgs::srv::SetGameSettings_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace rj_msgs

#endif  // RJ_MSGS__SRV__DETAIL__SET_GAME_SETTINGS__BUILDER_HPP_
