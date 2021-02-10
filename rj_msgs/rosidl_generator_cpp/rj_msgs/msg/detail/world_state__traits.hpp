// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rj_msgs:msg/WorldState.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__WORLD_STATE__TRAITS_HPP_
#define RJ_MSGS__MSG__DETAIL__WORLD_STATE__TRAITS_HPP_

#include "rj_msgs/msg/detail/world_state__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'last_update_time'
#include "builtin_interfaces/msg/detail/time__traits.hpp"
// Member 'ball'
#include "rj_msgs/msg/detail/ball_state__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rj_msgs::msg::WorldState>()
{
  return "rj_msgs::msg::WorldState";
}

template<>
inline const char * name<rj_msgs::msg::WorldState>()
{
  return "rj_msgs/msg/WorldState";
}

template<>
struct has_fixed_size<rj_msgs::msg::WorldState>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rj_msgs::msg::WorldState>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rj_msgs::msg::WorldState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RJ_MSGS__MSG__DETAIL__WORLD_STATE__TRAITS_HPP_
