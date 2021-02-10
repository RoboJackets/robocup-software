// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rj_msgs:msg/Goalie.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__GOALIE__BUILDER_HPP_
#define RJ_MSGS__MSG__DETAIL__GOALIE__BUILDER_HPP_

#include "rj_msgs/msg/detail/goalie__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rj_msgs
{

namespace msg
{

namespace builder
{

class Init_Goalie_goalie_id
{
public:
  Init_Goalie_goalie_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rj_msgs::msg::Goalie goalie_id(::rj_msgs::msg::Goalie::_goalie_id_type arg)
  {
    msg_.goalie_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rj_msgs::msg::Goalie msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rj_msgs::msg::Goalie>()
{
  return rj_msgs::msg::builder::Init_Goalie_goalie_id();
}

}  // namespace rj_msgs

#endif  // RJ_MSGS__MSG__DETAIL__GOALIE__BUILDER_HPP_
