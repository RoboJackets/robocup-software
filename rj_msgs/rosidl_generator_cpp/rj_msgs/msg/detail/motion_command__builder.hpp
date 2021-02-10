// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rj_msgs:msg/MotionCommand.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__MOTION_COMMAND__BUILDER_HPP_
#define RJ_MSGS__MSG__DETAIL__MOTION_COMMAND__BUILDER_HPP_

#include "rj_msgs/msg/detail/motion_command__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rj_msgs
{

namespace msg
{

namespace builder
{

class Init_MotionCommand_intercept_command
{
public:
  explicit Init_MotionCommand_intercept_command(::rj_msgs::msg::MotionCommand & msg)
  : msg_(msg)
  {}
  ::rj_msgs::msg::MotionCommand intercept_command(::rj_msgs::msg::MotionCommand::_intercept_command_type arg)
  {
    msg_.intercept_command = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rj_msgs::msg::MotionCommand msg_;
};

class Init_MotionCommand_line_kick_command
{
public:
  explicit Init_MotionCommand_line_kick_command(::rj_msgs::msg::MotionCommand & msg)
  : msg_(msg)
  {}
  Init_MotionCommand_intercept_command line_kick_command(::rj_msgs::msg::MotionCommand::_line_kick_command_type arg)
  {
    msg_.line_kick_command = std::move(arg);
    return Init_MotionCommand_intercept_command(msg_);
  }

private:
  ::rj_msgs::msg::MotionCommand msg_;
};

class Init_MotionCommand_collect_command
{
public:
  explicit Init_MotionCommand_collect_command(::rj_msgs::msg::MotionCommand & msg)
  : msg_(msg)
  {}
  Init_MotionCommand_line_kick_command collect_command(::rj_msgs::msg::MotionCommand::_collect_command_type arg)
  {
    msg_.collect_command = std::move(arg);
    return Init_MotionCommand_line_kick_command(msg_);
  }

private:
  ::rj_msgs::msg::MotionCommand msg_;
};

class Init_MotionCommand_settle_command
{
public:
  explicit Init_MotionCommand_settle_command(::rj_msgs::msg::MotionCommand & msg)
  : msg_(msg)
  {}
  Init_MotionCommand_collect_command settle_command(::rj_msgs::msg::MotionCommand::_settle_command_type arg)
  {
    msg_.settle_command = std::move(arg);
    return Init_MotionCommand_collect_command(msg_);
  }

private:
  ::rj_msgs::msg::MotionCommand msg_;
};

class Init_MotionCommand_pivot_command
{
public:
  explicit Init_MotionCommand_pivot_command(::rj_msgs::msg::MotionCommand & msg)
  : msg_(msg)
  {}
  Init_MotionCommand_settle_command pivot_command(::rj_msgs::msg::MotionCommand::_pivot_command_type arg)
  {
    msg_.pivot_command = std::move(arg);
    return Init_MotionCommand_settle_command(msg_);
  }

private:
  ::rj_msgs::msg::MotionCommand msg_;
};

class Init_MotionCommand_world_vel_command
{
public:
  explicit Init_MotionCommand_world_vel_command(::rj_msgs::msg::MotionCommand & msg)
  : msg_(msg)
  {}
  Init_MotionCommand_pivot_command world_vel_command(::rj_msgs::msg::MotionCommand::_world_vel_command_type arg)
  {
    msg_.world_vel_command = std::move(arg);
    return Init_MotionCommand_pivot_command(msg_);
  }

private:
  ::rj_msgs::msg::MotionCommand msg_;
};

class Init_MotionCommand_path_target_command
{
public:
  explicit Init_MotionCommand_path_target_command(::rj_msgs::msg::MotionCommand & msg)
  : msg_(msg)
  {}
  Init_MotionCommand_world_vel_command path_target_command(::rj_msgs::msg::MotionCommand::_path_target_command_type arg)
  {
    msg_.path_target_command = std::move(arg);
    return Init_MotionCommand_world_vel_command(msg_);
  }

private:
  ::rj_msgs::msg::MotionCommand msg_;
};

class Init_MotionCommand_empty_command
{
public:
  Init_MotionCommand_empty_command()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotionCommand_path_target_command empty_command(::rj_msgs::msg::MotionCommand::_empty_command_type arg)
  {
    msg_.empty_command = std::move(arg);
    return Init_MotionCommand_path_target_command(msg_);
  }

private:
  ::rj_msgs::msg::MotionCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rj_msgs::msg::MotionCommand>()
{
  return rj_msgs::msg::builder::Init_MotionCommand_empty_command();
}

}  // namespace rj_msgs

#endif  // RJ_MSGS__MSG__DETAIL__MOTION_COMMAND__BUILDER_HPP_
