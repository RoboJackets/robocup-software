// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rj_msgs:srv/SetGameSettings.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__SRV__DETAIL__SET_GAME_SETTINGS__TRAITS_HPP_
#define RJ_MSGS__SRV__DETAIL__SET_GAME_SETTINGS__TRAITS_HPP_

#include "rj_msgs/srv/detail/set_game_settings__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'game_settings'
#include "rj_msgs/msg/detail/game_settings__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rj_msgs::srv::SetGameSettings_Request>()
{
  return "rj_msgs::srv::SetGameSettings_Request";
}

template<>
inline const char * name<rj_msgs::srv::SetGameSettings_Request>()
{
  return "rj_msgs/srv/SetGameSettings_Request";
}

template<>
struct has_fixed_size<rj_msgs::srv::SetGameSettings_Request>
  : std::integral_constant<bool, has_fixed_size<rj_msgs::msg::GameSettings>::value> {};

template<>
struct has_bounded_size<rj_msgs::srv::SetGameSettings_Request>
  : std::integral_constant<bool, has_bounded_size<rj_msgs::msg::GameSettings>::value> {};

template<>
struct is_message<rj_msgs::srv::SetGameSettings_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rj_msgs::srv::SetGameSettings_Response>()
{
  return "rj_msgs::srv::SetGameSettings_Response";
}

template<>
inline const char * name<rj_msgs::srv::SetGameSettings_Response>()
{
  return "rj_msgs/srv/SetGameSettings_Response";
}

template<>
struct has_fixed_size<rj_msgs::srv::SetGameSettings_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rj_msgs::srv::SetGameSettings_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rj_msgs::srv::SetGameSettings_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rj_msgs::srv::SetGameSettings>()
{
  return "rj_msgs::srv::SetGameSettings";
}

template<>
inline const char * name<rj_msgs::srv::SetGameSettings>()
{
  return "rj_msgs/srv/SetGameSettings";
}

template<>
struct has_fixed_size<rj_msgs::srv::SetGameSettings>
  : std::integral_constant<
    bool,
    has_fixed_size<rj_msgs::srv::SetGameSettings_Request>::value &&
    has_fixed_size<rj_msgs::srv::SetGameSettings_Response>::value
  >
{
};

template<>
struct has_bounded_size<rj_msgs::srv::SetGameSettings>
  : std::integral_constant<
    bool,
    has_bounded_size<rj_msgs::srv::SetGameSettings_Request>::value &&
    has_bounded_size<rj_msgs::srv::SetGameSettings_Response>::value
  >
{
};

template<>
struct is_service<rj_msgs::srv::SetGameSettings>
  : std::true_type
{
};

template<>
struct is_service_request<rj_msgs::srv::SetGameSettings_Request>
  : std::true_type
{
};

template<>
struct is_service_response<rj_msgs::srv::SetGameSettings_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // RJ_MSGS__SRV__DETAIL__SET_GAME_SETTINGS__TRAITS_HPP_
