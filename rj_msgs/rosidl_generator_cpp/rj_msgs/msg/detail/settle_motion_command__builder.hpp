// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rj_msgs:msg/SettleMotionCommand.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__SETTLE_MOTION_COMMAND__BUILDER_HPP_
#define RJ_MSGS__MSG__DETAIL__SETTLE_MOTION_COMMAND__BUILDER_HPP_

#include "rj_msgs/msg/detail/settle_motion_command__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rj_msgs
{

namespace msg
{

namespace builder
{

class Init_SettleMotionCommand_maybe_target
{
public:
  Init_SettleMotionCommand_maybe_target()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rj_msgs::msg::SettleMotionCommand maybe_target(::rj_msgs::msg::SettleMotionCommand::_maybe_target_type arg)
  {
    msg_.maybe_target = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rj_msgs::msg::SettleMotionCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rj_msgs::msg::SettleMotionCommand>()
{
  return rj_msgs::msg::builder::Init_SettleMotionCommand_maybe_target();
}

}  // namespace rj_msgs

#endif  // RJ_MSGS__MSG__DETAIL__SETTLE_MOTION_COMMAND__BUILDER_HPP_
