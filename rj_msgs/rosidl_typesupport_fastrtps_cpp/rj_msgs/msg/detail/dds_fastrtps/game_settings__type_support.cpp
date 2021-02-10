// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from rj_msgs:msg/GameSettings.idl
// generated code does not contain a copyright notice
#include "rj_msgs/msg/detail/game_settings__rosidl_typesupport_fastrtps_cpp.hpp"
#include "rj_msgs/msg/detail/game_settings__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace rj_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rj_msgs
cdr_serialize(
  const rj_msgs::msg::GameSettings & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: simulation
  cdr << (ros_message.simulation ? true : false);
  // Member: request_blue_team
  cdr << (ros_message.request_blue_team ? true : false);
  // Member: request_goalie_id
  cdr << ros_message.request_goalie_id;
  // Member: defend_plus_x
  cdr << (ros_message.defend_plus_x ? true : false);
  // Member: use_our_half
  cdr << (ros_message.use_our_half ? true : false);
  // Member: use_their_half
  cdr << (ros_message.use_their_half ? true : false);
  // Member: paused
  cdr << (ros_message.paused ? true : false);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rj_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  rj_msgs::msg::GameSettings & ros_message)
{
  // Member: simulation
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.simulation = tmp ? true : false;
  }

  // Member: request_blue_team
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.request_blue_team = tmp ? true : false;
  }

  // Member: request_goalie_id
  cdr >> ros_message.request_goalie_id;

  // Member: defend_plus_x
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.defend_plus_x = tmp ? true : false;
  }

  // Member: use_our_half
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.use_our_half = tmp ? true : false;
  }

  // Member: use_their_half
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.use_their_half = tmp ? true : false;
  }

  // Member: paused
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.paused = tmp ? true : false;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rj_msgs
get_serialized_size(
  const rj_msgs::msg::GameSettings & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: simulation
  {
    size_t item_size = sizeof(ros_message.simulation);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: request_blue_team
  {
    size_t item_size = sizeof(ros_message.request_blue_team);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: request_goalie_id
  {
    size_t item_size = sizeof(ros_message.request_goalie_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: defend_plus_x
  {
    size_t item_size = sizeof(ros_message.defend_plus_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: use_our_half
  {
    size_t item_size = sizeof(ros_message.use_our_half);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: use_their_half
  {
    size_t item_size = sizeof(ros_message.use_their_half);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: paused
  {
    size_t item_size = sizeof(ros_message.paused);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rj_msgs
max_serialized_size_GameSettings(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: simulation
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: request_blue_team
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: request_goalie_id
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: defend_plus_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: use_our_half
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: use_their_half
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: paused
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static bool _GameSettings__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const rj_msgs::msg::GameSettings *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _GameSettings__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<rj_msgs::msg::GameSettings *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _GameSettings__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const rj_msgs::msg::GameSettings *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _GameSettings__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_GameSettings(full_bounded, 0);
}

static message_type_support_callbacks_t _GameSettings__callbacks = {
  "rj_msgs::msg",
  "GameSettings",
  _GameSettings__cdr_serialize,
  _GameSettings__cdr_deserialize,
  _GameSettings__get_serialized_size,
  _GameSettings__max_serialized_size
};

static rosidl_message_type_support_t _GameSettings__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_GameSettings__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace rj_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_rj_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<rj_msgs::msg::GameSettings>()
{
  return &rj_msgs::msg::typesupport_fastrtps_cpp::_GameSettings__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, rj_msgs, msg, GameSettings)() {
  return &rj_msgs::msg::typesupport_fastrtps_cpp::_GameSettings__handle;
}

#ifdef __cplusplus
}
#endif
