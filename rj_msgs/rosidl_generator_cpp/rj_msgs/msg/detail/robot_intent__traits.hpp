// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rj_msgs:msg/RobotIntent.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__ROBOT_INTENT__TRAITS_HPP_
#define RJ_MSGS__MSG__DETAIL__ROBOT_INTENT__TRAITS_HPP_

#include "rj_msgs/msg/detail/robot_intent__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'motion_command'
#include "rj_msgs/msg/detail/motion_command__traits.hpp"
// Member 'local_obstacles'
#include "rj_geometry_msgs/msg/detail/shape_set__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rj_msgs::msg::RobotIntent>()
{
  return "rj_msgs::msg::RobotIntent";
}

template<>
inline const char * name<rj_msgs::msg::RobotIntent>()
{
  return "rj_msgs/msg/RobotIntent";
}

template<>
struct has_fixed_size<rj_msgs::msg::RobotIntent>
  : std::integral_constant<bool, has_fixed_size<rj_geometry_msgs::msg::ShapeSet>::value && has_fixed_size<rj_msgs::msg::MotionCommand>::value> {};

template<>
struct has_bounded_size<rj_msgs::msg::RobotIntent>
  : std::integral_constant<bool, has_bounded_size<rj_geometry_msgs::msg::ShapeSet>::value && has_bounded_size<rj_msgs::msg::MotionCommand>::value> {};

template<>
struct is_message<rj_msgs::msg::RobotIntent>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RJ_MSGS__MSG__DETAIL__ROBOT_INTENT__TRAITS_HPP_
