// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rj_msgs:msg/LineKickMotionCommand.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__LINE_KICK_MOTION_COMMAND__TRAITS_HPP_
#define RJ_MSGS__MSG__DETAIL__LINE_KICK_MOTION_COMMAND__TRAITS_HPP_

#include "rj_msgs/msg/detail/line_kick_motion_command__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'target'
#include "rj_geometry_msgs/msg/detail/point__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rj_msgs::msg::LineKickMotionCommand>()
{
  return "rj_msgs::msg::LineKickMotionCommand";
}

template<>
inline const char * name<rj_msgs::msg::LineKickMotionCommand>()
{
  return "rj_msgs/msg/LineKickMotionCommand";
}

template<>
struct has_fixed_size<rj_msgs::msg::LineKickMotionCommand>
  : std::integral_constant<bool, has_fixed_size<rj_geometry_msgs::msg::Point>::value> {};

template<>
struct has_bounded_size<rj_msgs::msg::LineKickMotionCommand>
  : std::integral_constant<bool, has_bounded_size<rj_geometry_msgs::msg::Point>::value> {};

template<>
struct is_message<rj_msgs::msg::LineKickMotionCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RJ_MSGS__MSG__DETAIL__LINE_KICK_MOTION_COMMAND__TRAITS_HPP_
