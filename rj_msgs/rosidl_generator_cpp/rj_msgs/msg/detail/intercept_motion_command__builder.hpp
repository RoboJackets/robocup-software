// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rj_msgs:msg/InterceptMotionCommand.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__INTERCEPT_MOTION_COMMAND__BUILDER_HPP_
#define RJ_MSGS__MSG__DETAIL__INTERCEPT_MOTION_COMMAND__BUILDER_HPP_

#include "rj_msgs/msg/detail/intercept_motion_command__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rj_msgs
{

namespace msg
{

namespace builder
{

class Init_InterceptMotionCommand_target
{
public:
  Init_InterceptMotionCommand_target()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rj_msgs::msg::InterceptMotionCommand target(::rj_msgs::msg::InterceptMotionCommand::_target_type arg)
  {
    msg_.target = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rj_msgs::msg::InterceptMotionCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rj_msgs::msg::InterceptMotionCommand>()
{
  return rj_msgs::msg::builder::Init_InterceptMotionCommand_target();
}

}  // namespace rj_msgs

#endif  // RJ_MSGS__MSG__DETAIL__INTERCEPT_MOTION_COMMAND__BUILDER_HPP_
