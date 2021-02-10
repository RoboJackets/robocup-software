// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from rj_msgs:msg/WorldVelMotionCommand.idl
// generated code does not contain a copyright notice
#include "rj_msgs/msg/detail/world_vel_motion_command__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rj_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "rj_msgs/msg/detail/world_vel_motion_command__struct.h"
#include "rj_msgs/msg/detail/world_vel_motion_command__functions.h"
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

#include "rj_geometry_msgs/msg/detail/point__functions.h"  // world_vel

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_rj_msgs
size_t get_serialized_size_rj_geometry_msgs__msg__Point(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_rj_msgs
size_t max_serialized_size_rj_geometry_msgs__msg__Point(
  bool & full_bounded,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_rj_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rj_geometry_msgs, msg, Point)();


using _WorldVelMotionCommand__ros_msg_type = rj_msgs__msg__WorldVelMotionCommand;

static bool _WorldVelMotionCommand__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _WorldVelMotionCommand__ros_msg_type * ros_message = static_cast<const _WorldVelMotionCommand__ros_msg_type *>(untyped_ros_message);
  // Field name: world_vel
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, rj_geometry_msgs, msg, Point
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->world_vel, cdr))
    {
      return false;
    }
  }

  return true;
}

static bool _WorldVelMotionCommand__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _WorldVelMotionCommand__ros_msg_type * ros_message = static_cast<_WorldVelMotionCommand__ros_msg_type *>(untyped_ros_message);
  // Field name: world_vel
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, rj_geometry_msgs, msg, Point
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->world_vel))
    {
      return false;
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rj_msgs
size_t get_serialized_size_rj_msgs__msg__WorldVelMotionCommand(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _WorldVelMotionCommand__ros_msg_type * ros_message = static_cast<const _WorldVelMotionCommand__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name world_vel

  current_alignment += get_serialized_size_rj_geometry_msgs__msg__Point(
    &(ros_message->world_vel), current_alignment);

  return current_alignment - initial_alignment;
}

static uint32_t _WorldVelMotionCommand__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_rj_msgs__msg__WorldVelMotionCommand(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rj_msgs
size_t max_serialized_size_rj_msgs__msg__WorldVelMotionCommand(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: world_vel
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_rj_geometry_msgs__msg__Point(
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static size_t _WorldVelMotionCommand__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_rj_msgs__msg__WorldVelMotionCommand(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_WorldVelMotionCommand = {
  "rj_msgs::msg",
  "WorldVelMotionCommand",
  _WorldVelMotionCommand__cdr_serialize,
  _WorldVelMotionCommand__cdr_deserialize,
  _WorldVelMotionCommand__get_serialized_size,
  _WorldVelMotionCommand__max_serialized_size
};

static rosidl_message_type_support_t _WorldVelMotionCommand__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_WorldVelMotionCommand,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rj_msgs, msg, WorldVelMotionCommand)() {
  return &_WorldVelMotionCommand__type_support;
}

#if defined(__cplusplus)
}
#endif
