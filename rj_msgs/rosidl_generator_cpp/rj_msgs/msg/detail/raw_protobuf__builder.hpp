// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rj_msgs:msg/RawProtobuf.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__RAW_PROTOBUF__BUILDER_HPP_
#define RJ_MSGS__MSG__DETAIL__RAW_PROTOBUF__BUILDER_HPP_

#include "rj_msgs/msg/detail/raw_protobuf__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rj_msgs
{

namespace msg
{

namespace builder
{

class Init_RawProtobuf_data
{
public:
  Init_RawProtobuf_data()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rj_msgs::msg::RawProtobuf data(::rj_msgs::msg::RawProtobuf::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rj_msgs::msg::RawProtobuf msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rj_msgs::msg::RawProtobuf>()
{
  return rj_msgs::msg::builder::Init_RawProtobuf_data();
}

}  // namespace rj_msgs

#endif  // RJ_MSGS__MSG__DETAIL__RAW_PROTOBUF__BUILDER_HPP_
