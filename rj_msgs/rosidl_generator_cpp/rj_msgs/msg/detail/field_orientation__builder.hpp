// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rj_msgs:msg/FieldOrientation.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__FIELD_ORIENTATION__BUILDER_HPP_
#define RJ_MSGS__MSG__DETAIL__FIELD_ORIENTATION__BUILDER_HPP_

#include "rj_msgs/msg/detail/field_orientation__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rj_msgs
{

namespace msg
{

namespace builder
{

class Init_FieldOrientation_defend_plus_x
{
public:
  Init_FieldOrientation_defend_plus_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rj_msgs::msg::FieldOrientation defend_plus_x(::rj_msgs::msg::FieldOrientation::_defend_plus_x_type arg)
  {
    msg_.defend_plus_x = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rj_msgs::msg::FieldOrientation msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rj_msgs::msg::FieldOrientation>()
{
  return rj_msgs::msg::builder::Init_FieldOrientation_defend_plus_x();
}

}  // namespace rj_msgs

#endif  // RJ_MSGS__MSG__DETAIL__FIELD_ORIENTATION__BUILDER_HPP_
