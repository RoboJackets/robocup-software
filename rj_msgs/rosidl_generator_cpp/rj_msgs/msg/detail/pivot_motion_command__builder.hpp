// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rj_msgs:msg/PivotMotionCommand.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__PIVOT_MOTION_COMMAND__BUILDER_HPP_
#define RJ_MSGS__MSG__DETAIL__PIVOT_MOTION_COMMAND__BUILDER_HPP_

#include "rj_msgs/msg/detail/pivot_motion_command__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rj_msgs
{

namespace msg
{

namespace builder
{

class Init_PivotMotionCommand_pivot_target
{
public:
  explicit Init_PivotMotionCommand_pivot_target(::rj_msgs::msg::PivotMotionCommand & msg)
  : msg_(msg)
  {}
  ::rj_msgs::msg::PivotMotionCommand pivot_target(::rj_msgs::msg::PivotMotionCommand::_pivot_target_type arg)
  {
    msg_.pivot_target = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rj_msgs::msg::PivotMotionCommand msg_;
};

class Init_PivotMotionCommand_pivot_point
{
public:
  Init_PivotMotionCommand_pivot_point()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PivotMotionCommand_pivot_target pivot_point(::rj_msgs::msg::PivotMotionCommand::_pivot_point_type arg)
  {
    msg_.pivot_point = std::move(arg);
    return Init_PivotMotionCommand_pivot_target(msg_);
  }

private:
  ::rj_msgs::msg::PivotMotionCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rj_msgs::msg::PivotMotionCommand>()
{
  return rj_msgs::msg::builder::Init_PivotMotionCommand_pivot_point();
}

}  // namespace rj_msgs

#endif  // RJ_MSGS__MSG__DETAIL__PIVOT_MOTION_COMMAND__BUILDER_HPP_
