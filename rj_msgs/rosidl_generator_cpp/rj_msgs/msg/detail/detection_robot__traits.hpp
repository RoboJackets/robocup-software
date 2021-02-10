// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rj_msgs:msg/DetectionRobot.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__DETECTION_ROBOT__TRAITS_HPP_
#define RJ_MSGS__MSG__DETAIL__DETECTION_ROBOT__TRAITS_HPP_

#include "rj_msgs/msg/detail/detection_robot__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rj_msgs::msg::DetectionRobot>()
{
  return "rj_msgs::msg::DetectionRobot";
}

template<>
inline const char * name<rj_msgs::msg::DetectionRobot>()
{
  return "rj_msgs/msg/DetectionRobot";
}

template<>
struct has_fixed_size<rj_msgs::msg::DetectionRobot>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rj_msgs::msg::DetectionRobot>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rj_msgs::msg::DetectionRobot>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RJ_MSGS__MSG__DETAIL__DETECTION_ROBOT__TRAITS_HPP_
