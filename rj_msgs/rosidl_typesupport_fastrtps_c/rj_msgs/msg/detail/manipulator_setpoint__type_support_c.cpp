// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from rj_msgs:msg/ManipulatorSetpoint.idl
// generated code does not contain a copyright notice
#include "rj_msgs/msg/detail/manipulator_setpoint__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rj_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "rj_msgs/msg/detail/manipulator_setpoint__struct.h"
#include "rj_msgs/msg/detail/manipulator_setpoint__functions.h"
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


using _ManipulatorSetpoint__ros_msg_type = rj_msgs__msg__ManipulatorSetpoint;

static bool _ManipulatorSetpoint__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _ManipulatorSetpoint__ros_msg_type * ros_message = static_cast<const _ManipulatorSetpoint__ros_msg_type *>(untyped_ros_message);
  // Field name: shoot_mode
  {
    cdr << ros_message->shoot_mode;
  }

  // Field name: trigger_mode
  {
    cdr << ros_message->trigger_mode;
  }

  // Field name: kick_strength
  {
    cdr << ros_message->kick_strength;
  }

  // Field name: dribbler_speed
  {
    cdr << ros_message->dribbler_speed;
  }

  return true;
}

static bool _ManipulatorSetpoint__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _ManipulatorSetpoint__ros_msg_type * ros_message = static_cast<_ManipulatorSetpoint__ros_msg_type *>(untyped_ros_message);
  // Field name: shoot_mode
  {
    cdr >> ros_message->shoot_mode;
  }

  // Field name: trigger_mode
  {
    cdr >> ros_message->trigger_mode;
  }

  // Field name: kick_strength
  {
    cdr >> ros_message->kick_strength;
  }

  // Field name: dribbler_speed
  {
    cdr >> ros_message->dribbler_speed;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rj_msgs
size_t get_serialized_size_rj_msgs__msg__ManipulatorSetpoint(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ManipulatorSetpoint__ros_msg_type * ros_message = static_cast<const _ManipulatorSetpoint__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name shoot_mode
  {
    size_t item_size = sizeof(ros_message->shoot_mode);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name trigger_mode
  {
    size_t item_size = sizeof(ros_message->trigger_mode);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kick_strength
  {
    size_t item_size = sizeof(ros_message->kick_strength);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name dribbler_speed
  {
    size_t item_size = sizeof(ros_message->dribbler_speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _ManipulatorSetpoint__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_rj_msgs__msg__ManipulatorSetpoint(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rj_msgs
size_t max_serialized_size_rj_msgs__msg__ManipulatorSetpoint(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: shoot_mode
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: trigger_mode
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: kick_strength
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: dribbler_speed
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _ManipulatorSetpoint__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_rj_msgs__msg__ManipulatorSetpoint(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_ManipulatorSetpoint = {
  "rj_msgs::msg",
  "ManipulatorSetpoint",
  _ManipulatorSetpoint__cdr_serialize,
  _ManipulatorSetpoint__cdr_deserialize,
  _ManipulatorSetpoint__get_serialized_size,
  _ManipulatorSetpoint__max_serialized_size
};

static rosidl_message_type_support_t _ManipulatorSetpoint__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ManipulatorSetpoint,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rj_msgs, msg, ManipulatorSetpoint)() {
  return &_ManipulatorSetpoint__type_support;
}

#if defined(__cplusplus)
}
#endif
