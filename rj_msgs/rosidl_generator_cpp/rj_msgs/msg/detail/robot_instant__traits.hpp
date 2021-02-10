// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rj_msgs:msg/RobotInstant.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__ROBOT_INSTANT__TRAITS_HPP_
#define RJ_MSGS__MSG__DETAIL__ROBOT_INSTANT__TRAITS_HPP_

#include "rj_msgs/msg/detail/robot_instant__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"
// Member 'pose'
#include "rj_geometry_msgs/msg/detail/pose__traits.hpp"
// Member 'velocity'
#include "rj_geometry_msgs/msg/detail/twist__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rj_msgs::msg::RobotInstant>()
{
  return "rj_msgs::msg::RobotInstant";
}

template<>
inline const char * name<rj_msgs::msg::RobotInstant>()
{
  return "rj_msgs/msg/RobotInstant";
}

template<>
struct has_fixed_size<rj_msgs::msg::RobotInstant>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value && has_fixed_size<rj_geometry_msgs::msg::Pose>::value && has_fixed_size<rj_geometry_msgs::msg::Twist>::value> {};

template<>
struct has_bounded_size<rj_msgs::msg::RobotInstant>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value && has_bounded_size<rj_geometry_msgs::msg::Pose>::value && has_bounded_size<rj_geometry_msgs::msg::Twist>::value> {};

template<>
struct is_message<rj_msgs::msg::RobotInstant>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RJ_MSGS__MSG__DETAIL__ROBOT_INSTANT__TRAITS_HPP_
