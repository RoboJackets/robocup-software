// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rj_msgs:msg/LinearMotionInstant.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__LINEAR_MOTION_INSTANT__BUILDER_HPP_
#define RJ_MSGS__MSG__DETAIL__LINEAR_MOTION_INSTANT__BUILDER_HPP_

#include "rj_msgs/msg/detail/linear_motion_instant__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rj_msgs
{

namespace msg
{

namespace builder
{

class Init_LinearMotionInstant_velocity
{
public:
  explicit Init_LinearMotionInstant_velocity(::rj_msgs::msg::LinearMotionInstant & msg)
  : msg_(msg)
  {}
  ::rj_msgs::msg::LinearMotionInstant velocity(::rj_msgs::msg::LinearMotionInstant::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rj_msgs::msg::LinearMotionInstant msg_;
};

class Init_LinearMotionInstant_position
{
public:
  Init_LinearMotionInstant_position()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LinearMotionInstant_velocity position(::rj_msgs::msg::LinearMotionInstant::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_LinearMotionInstant_velocity(msg_);
  }

private:
  ::rj_msgs::msg::LinearMotionInstant msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rj_msgs::msg::LinearMotionInstant>()
{
  return rj_msgs::msg::builder::Init_LinearMotionInstant_position();
}

}  // namespace rj_msgs

#endif  // RJ_MSGS__MSG__DETAIL__LINEAR_MOTION_INSTANT__BUILDER_HPP_
