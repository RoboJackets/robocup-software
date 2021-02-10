// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rj_msgs:msg/Trajectory.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__TRAJECTORY__BUILDER_HPP_
#define RJ_MSGS__MSG__DETAIL__TRAJECTORY__BUILDER_HPP_

#include "rj_msgs/msg/detail/trajectory__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rj_msgs
{

namespace msg
{

namespace builder
{

class Init_Trajectory_instants
{
public:
  explicit Init_Trajectory_instants(::rj_msgs::msg::Trajectory & msg)
  : msg_(msg)
  {}
  ::rj_msgs::msg::Trajectory instants(::rj_msgs::msg::Trajectory::_instants_type arg)
  {
    msg_.instants = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rj_msgs::msg::Trajectory msg_;
};

class Init_Trajectory_stamp
{
public:
  Init_Trajectory_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Trajectory_instants stamp(::rj_msgs::msg::Trajectory::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_Trajectory_instants(msg_);
  }

private:
  ::rj_msgs::msg::Trajectory msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rj_msgs::msg::Trajectory>()
{
  return rj_msgs::msg::builder::Init_Trajectory_stamp();
}

}  // namespace rj_msgs

#endif  // RJ_MSGS__MSG__DETAIL__TRAJECTORY__BUILDER_HPP_
