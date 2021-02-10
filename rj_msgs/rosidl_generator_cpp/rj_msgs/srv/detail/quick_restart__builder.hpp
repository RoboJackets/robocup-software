// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rj_msgs:srv/QuickRestart.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__SRV__DETAIL__QUICK_RESTART__BUILDER_HPP_
#define RJ_MSGS__SRV__DETAIL__QUICK_RESTART__BUILDER_HPP_

#include "rj_msgs/srv/detail/quick_restart__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rj_msgs
{

namespace srv
{

namespace builder
{

class Init_QuickRestart_Request_blue_team
{
public:
  explicit Init_QuickRestart_Request_blue_team(::rj_msgs::srv::QuickRestart_Request & msg)
  : msg_(msg)
  {}
  ::rj_msgs::srv::QuickRestart_Request blue_team(::rj_msgs::srv::QuickRestart_Request::_blue_team_type arg)
  {
    msg_.blue_team = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rj_msgs::srv::QuickRestart_Request msg_;
};

class Init_QuickRestart_Request_restart
{
public:
  Init_QuickRestart_Request_restart()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_QuickRestart_Request_blue_team restart(::rj_msgs::srv::QuickRestart_Request::_restart_type arg)
  {
    msg_.restart = std::move(arg);
    return Init_QuickRestart_Request_blue_team(msg_);
  }

private:
  ::rj_msgs::srv::QuickRestart_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rj_msgs::srv::QuickRestart_Request>()
{
  return rj_msgs::srv::builder::Init_QuickRestart_Request_restart();
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
auto build<::rj_msgs::srv::QuickRestart_Response>()
{
  return ::rj_msgs::srv::QuickRestart_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace rj_msgs

#endif  // RJ_MSGS__SRV__DETAIL__QUICK_RESTART__BUILDER_HPP_
