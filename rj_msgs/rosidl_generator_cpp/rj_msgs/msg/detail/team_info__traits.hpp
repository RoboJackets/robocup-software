// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rj_msgs:msg/TeamInfo.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__TEAM_INFO__TRAITS_HPP_
#define RJ_MSGS__MSG__DETAIL__TEAM_INFO__TRAITS_HPP_

#include "rj_msgs/msg/detail/team_info__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'remaining_timeout_time'
#include "builtin_interfaces/msg/detail/duration__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rj_msgs::msg::TeamInfo>()
{
  return "rj_msgs::msg::TeamInfo";
}

template<>
inline const char * name<rj_msgs::msg::TeamInfo>()
{
  return "rj_msgs/msg/TeamInfo";
}

template<>
struct has_fixed_size<rj_msgs::msg::TeamInfo>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rj_msgs::msg::TeamInfo>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rj_msgs::msg::TeamInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RJ_MSGS__MSG__DETAIL__TEAM_INFO__TRAITS_HPP_
