// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rj_msgs:msg/CollectMotionCommand.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__COLLECT_MOTION_COMMAND__TRAITS_HPP_
#define RJ_MSGS__MSG__DETAIL__COLLECT_MOTION_COMMAND__TRAITS_HPP_

#include "rj_msgs/msg/detail/collect_motion_command__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rj_msgs::msg::CollectMotionCommand>()
{
  return "rj_msgs::msg::CollectMotionCommand";
}

template<>
inline const char * name<rj_msgs::msg::CollectMotionCommand>()
{
  return "rj_msgs/msg/CollectMotionCommand";
}

template<>
struct has_fixed_size<rj_msgs::msg::CollectMotionCommand>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rj_msgs::msg::CollectMotionCommand>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rj_msgs::msg::CollectMotionCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RJ_MSGS__MSG__DETAIL__COLLECT_MOTION_COMMAND__TRAITS_HPP_
