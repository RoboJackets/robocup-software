// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rj_msgs:msg/TeamColor.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__TEAM_COLOR__BUILDER_HPP_
#define RJ_MSGS__MSG__DETAIL__TEAM_COLOR__BUILDER_HPP_

#include "rj_msgs/msg/detail/team_color__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rj_msgs
{

namespace msg
{

namespace builder
{

class Init_TeamColor_is_blue
{
public:
  Init_TeamColor_is_blue()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rj_msgs::msg::TeamColor is_blue(::rj_msgs::msg::TeamColor::_is_blue_type arg)
  {
    msg_.is_blue = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rj_msgs::msg::TeamColor msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rj_msgs::msg::TeamColor>()
{
  return rj_msgs::msg::builder::Init_TeamColor_is_blue();
}

}  // namespace rj_msgs

#endif  // RJ_MSGS__MSG__DETAIL__TEAM_COLOR__BUILDER_HPP_
