// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rj_msgs:msg/RobotStatus.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__ROBOT_STATUS__TRAITS_HPP_
#define RJ_MSGS__MSG__DETAIL__ROBOT_STATUS__TRAITS_HPP_

#include "rj_msgs/msg/detail/robot_status__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rj_msgs::msg::RobotStatus>()
{
  return "rj_msgs::msg::RobotStatus";
}

template<>
inline const char * name<rj_msgs::msg::RobotStatus>()
{
  return "rj_msgs/msg/RobotStatus";
}

template<>
struct has_fixed_size<rj_msgs::msg::RobotStatus>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rj_msgs::msg::RobotStatus>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rj_msgs::msg::RobotStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RJ_MSGS__MSG__DETAIL__ROBOT_STATUS__TRAITS_HPP_
