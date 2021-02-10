// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from rj_msgs:msg/Goalie.idl
// generated code does not contain a copyright notice
#include "rj_msgs/msg/detail/goalie__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rj_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "rj_msgs/msg/detail/goalie__struct.h"
#include "rj_msgs/msg/detail/goalie__functions.h"
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


using _Goalie__ros_msg_type = rj_msgs__msg__Goalie;

static bool _Goalie__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Goalie__ros_msg_type * ros_message = static_cast<const _Goalie__ros_msg_type *>(untyped_ros_message);
  // Field name: goalie_id
  {
    cdr << ros_message->goalie_id;
  }

  return true;
}

static bool _Goalie__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Goalie__ros_msg_type * ros_message = static_cast<_Goalie__ros_msg_type *>(untyped_ros_message);
  // Field name: goalie_id
  {
    cdr >> ros_message->goalie_id;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rj_msgs
size_t get_serialized_size_rj_msgs__msg__Goalie(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Goalie__ros_msg_type * ros_message = static_cast<const _Goalie__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name goalie_id
  {
    size_t item_size = sizeof(ros_message->goalie_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _Goalie__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_rj_msgs__msg__Goalie(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rj_msgs
size_t max_serialized_size_rj_msgs__msg__Goalie(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: goalie_id
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static size_t _Goalie__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_rj_msgs__msg__Goalie(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_Goalie = {
  "rj_msgs::msg",
  "Goalie",
  _Goalie__cdr_serialize,
  _Goalie__cdr_deserialize,
  _Goalie__get_serialized_size,
  _Goalie__max_serialized_size
};

static rosidl_message_type_support_t _Goalie__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Goalie,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rj_msgs, msg, Goalie)() {
  return &_Goalie__type_support;
}

#if defined(__cplusplus)
}
#endif
