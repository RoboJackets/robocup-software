// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from rj_msgs:msg/GameSettings.idl
// generated code does not contain a copyright notice
#include "rj_msgs/msg/detail/game_settings__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rj_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "rj_msgs/msg/detail/game_settings__struct.h"
#include "rj_msgs/msg/detail/game_settings__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _GameSettings__ros_msg_type = rj_msgs__msg__GameSettings;

static bool _GameSettings__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _GameSettings__ros_msg_type * ros_message = static_cast<const _GameSettings__ros_msg_type *>(untyped_ros_message);
  // Field name: simulation
  {
    cdr << (ros_message->simulation ? true : false);
  }

  // Field name: request_blue_team
  {
    cdr << (ros_message->request_blue_team ? true : false);
  }

  // Field name: request_goalie_id
  {
    cdr << ros_message->request_goalie_id;
  }

  // Field name: defend_plus_x
  {
    cdr << (ros_message->defend_plus_x ? true : false);
  }

  // Field name: use_our_half
  {
    cdr << (ros_message->use_our_half ? true : false);
  }

  // Field name: use_their_half
  {
    cdr << (ros_message->use_their_half ? true : false);
  }

  // Field name: paused
  {
    cdr << (ros_message->paused ? true : false);
  }

  return true;
}

static bool _GameSettings__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _GameSettings__ros_msg_type * ros_message = static_cast<_GameSettings__ros_msg_type *>(untyped_ros_message);
  // Field name: simulation
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->simulation = tmp ? true : false;
  }

  // Field name: request_blue_team
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->request_blue_team = tmp ? true : false;
  }

  // Field name: request_goalie_id
  {
    cdr >> ros_message->request_goalie_id;
  }

  // Field name: defend_plus_x
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->defend_plus_x = tmp ? true : false;
  }

  // Field name: use_our_half
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->use_our_half = tmp ? true : false;
  }

  // Field name: use_their_half
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->use_their_half = tmp ? true : false;
  }

  // Field name: paused
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->paused = tmp ? true : false;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rj_msgs
size_t get_serialized_size_rj_msgs__msg__GameSettings(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _GameSettings__ros_msg_type * ros_message = static_cast<const _GameSettings__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name simulation
  {
    size_t item_size = sizeof(ros_message->simulation);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name request_blue_team
  {
    size_t item_size = sizeof(ros_message->request_blue_team);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name request_goalie_id
  {
    size_t item_size = sizeof(ros_message->request_goalie_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name defend_plus_x
  {
    size_t item_size = sizeof(ros_message->defend_plus_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name use_our_half
  {
    size_t item_size = sizeof(ros_message->use_our_half);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name use_their_half
  {
    size_t item_size = sizeof(ros_message->use_their_half);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name paused
  {
    size_t item_size = sizeof(ros_message->paused);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _GameSettings__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_rj_msgs__msg__GameSettings(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rj_msgs
size_t max_serialized_size_rj_msgs__msg__GameSettings(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: simulation
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: request_blue_team
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: request_goalie_id
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: defend_plus_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: use_our_half
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: use_their_half
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: paused
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static size_t _GameSettings__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_rj_msgs__msg__GameSettings(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_GameSettings = {
  "rj_msgs::msg",
  "GameSettings",
  _GameSettings__cdr_serialize,
  _GameSettings__cdr_deserialize,
  _GameSettings__get_serialized_size,
  _GameSettings__max_serialized_size
};

static rosidl_message_type_support_t _GameSettings__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_GameSettings,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rj_msgs, msg, GameSettings)() {
  return &_GameSettings__type_support;
}

#if defined(__cplusplus)
}
#endif
