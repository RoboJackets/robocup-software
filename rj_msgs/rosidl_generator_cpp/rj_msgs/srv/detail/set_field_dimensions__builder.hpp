// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rj_msgs:srv/SetFieldDimensions.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__SRV__DETAIL__SET_FIELD_DIMENSIONS__BUILDER_HPP_
#define RJ_MSGS__SRV__DETAIL__SET_FIELD_DIMENSIONS__BUILDER_HPP_

#include "rj_msgs/srv/detail/set_field_dimensions__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rj_msgs
{

namespace srv
{

namespace builder
{

class Init_SetFieldDimensions_Request_field_dimensions
{
public:
  Init_SetFieldDimensions_Request_field_dimensions()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rj_msgs::srv::SetFieldDimensions_Request field_dimensions(::rj_msgs::srv::SetFieldDimensions_Request::_field_dimensions_type arg)
  {
    msg_.field_dimensions = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rj_msgs::srv::SetFieldDimensions_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rj_msgs::srv::SetFieldDimensions_Request>()
{
  return rj_msgs::srv::builder::Init_SetFieldDimensions_Request_field_dimensions();
}

}  // namespace rj_msgs


namespace rj_msgs
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rj_msgs::srv::SetFieldDimensions_Response>()
{
  return ::rj_msgs::srv::SetFieldDimensions_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace rj_msgs

#endif  // RJ_MSGS__SRV__DETAIL__SET_FIELD_DIMENSIONS__BUILDER_HPP_
