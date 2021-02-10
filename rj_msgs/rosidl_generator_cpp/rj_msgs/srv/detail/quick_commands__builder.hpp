// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rj_msgs:srv/QuickCommands.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__SRV__DETAIL__QUICK_COMMANDS__BUILDER_HPP_
#define RJ_MSGS__SRV__DETAIL__QUICK_COMMANDS__BUILDER_HPP_

#include "rj_msgs/srv/detail/quick_commands__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rj_msgs
{

namespace srv
{

namespace builder
{

class Init_QuickCommands_Request_state
{
public:
  Init_QuickCommands_Request_state()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rj_msgs::srv::QuickCommands_Request state(::rj_msgs::srv::QuickCommands_Request::_state_type arg)
  {
    msg_.state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rj_msgs::srv::QuickCommands_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rj_msgs::srv::QuickCommands_Request>()
{
  return rj_msgs::srv::builder::Init_QuickCommands_Request_state();
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
auto build<::rj_msgs::srv::QuickCommands_Response>()
{
  return ::rj_msgs::srv::QuickCommands_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace rj_msgs

#endif  // RJ_MSGS__SRV__DETAIL__QUICK_COMMANDS__BUILDER_HPP_
