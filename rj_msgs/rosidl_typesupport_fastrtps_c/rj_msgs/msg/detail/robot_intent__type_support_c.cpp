// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from rj_msgs:msg/RobotIntent.idl
// generated code does not contain a copyright notice
#include "rj_msgs/msg/detail/robot_intent__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rj_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "rj_msgs/msg/detail/robot_intent__struct.h"
#include "rj_msgs/msg/detail/robot_intent__functions.h"
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

#include "rj_geometry_msgs/msg/detail/shape_set__functions.h"  // local_obstacles
#include "rj_msgs/msg/detail/motion_command__functions.h"  // motion_command

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_rj_msgs
size_t get_serialized_size_rj_geometry_msgs__msg__ShapeSet(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_rj_msgs
size_t max_serialized_size_rj_geometry_msgs__msg__ShapeSet(
  bool & full_bounded,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_rj_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rj_geometry_msgs, msg, ShapeSet)();
size_t get_serialized_size_rj_msgs__msg__MotionCommand(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_rj_msgs__msg__MotionCommand(
  bool & full_bounded,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rj_msgs, msg, MotionCommand)();


using _RobotIntent__ros_msg_type = rj_msgs__msg__RobotIntent;

static bool _RobotIntent__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _RobotIntent__ros_msg_type * ros_message = static_cast<const _RobotIntent__ros_msg_type *>(untyped_ros_message);
  // Field name: motion_command
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, rj_msgs, msg, MotionCommand
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->motion_command, cdr))
    {
      return false;
    }
  }

  // Field name: local_obstacles
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, rj_geometry_msgs, msg, ShapeSet
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->local_obstacles, cdr))
    {
      return false;
    }
  }

  // Field name: shoot_mode
  {
    cdr << ros_message->shoot_mode;
  }

  // Field name: trigger_mode
  {
    cdr << ros_message->trigger_mode;
  }

  // Field name: kick_speed
  {
    cdr << ros_message->kick_speed;
  }

  // Field name: dribbler_speed
  {
    cdr << ros_message->dribbler_speed;
  }

  // Field name: is_active
  {
    cdr << (ros_message->is_active ? true : false);
  }

  // Field name: priority
  {
    cdr << ros_message->priority;
  }

  return true;
}

static bool _RobotIntent__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _RobotIntent__ros_msg_type * ros_message = static_cast<_RobotIntent__ros_msg_type *>(untyped_ros_message);
  // Field name: motion_command
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, rj_msgs, msg, MotionCommand
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->motion_command))
    {
      return false;
    }
  }

  // Field name: local_obstacles
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, rj_geometry_msgs, msg, ShapeSet
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->local_obstacles))
    {
      return false;
    }
  }

  // Field name: shoot_mode
  {
    cdr >> ros_message->shoot_mode;
  }

  // Field name: trigger_mode
  {
    cdr >> ros_message->trigger_mode;
  }

  // Field name: kick_speed
  {
    cdr >> ros_message->kick_speed;
  }

  // Field name: dribbler_speed
  {
    cdr >> ros_message->dribbler_speed;
  }

  // Field name: is_active
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->is_active = tmp ? true : false;
  }

  // Field name: priority
  {
    cdr >> ros_message->priority;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rj_msgs
size_t get_serialized_size_rj_msgs__msg__RobotIntent(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _RobotIntent__ros_msg_type * ros_message = static_cast<const _RobotIntent__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name motion_command

  current_alignment += get_serialized_size_rj_msgs__msg__MotionCommand(
    &(ros_message->motion_command), current_alignment);
  // field.name local_obstacles

  current_alignment += get_serialized_size_rj_geometry_msgs__msg__ShapeSet(
    &(ros_message->local_obstacles), current_alignment);
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
  // field.name kick_speed
  {
    size_t item_size = sizeof(ros_message->kick_speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name dribbler_speed
  {
    size_t item_size = sizeof(ros_message->dribbler_speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name is_active
  {
    size_t item_size = sizeof(ros_message->is_active);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name priority
  {
    size_t item_size = sizeof(ros_message->priority);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _RobotIntent__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_rj_msgs__msg__RobotIntent(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rj_msgs
size_t max_serialized_size_rj_msgs__msg__RobotIntent(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: motion_command
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_rj_msgs__msg__MotionCommand(
        full_bounded, current_alignment);
    }
  }
  // member: local_obstacles
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_rj_geometry_msgs__msg__ShapeSet(
        full_bounded, current_alignment);
    }
  }
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
  // member: kick_speed
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: dribbler_speed
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: is_active
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: priority
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static size_t _RobotIntent__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_rj_msgs__msg__RobotIntent(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_RobotIntent = {
  "rj_msgs::msg",
  "RobotIntent",
  _RobotIntent__cdr_serialize,
  _RobotIntent__cdr_deserialize,
  _RobotIntent__get_serialized_size,
  _RobotIntent__max_serialized_size
};

static rosidl_message_type_support_t _RobotIntent__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_RobotIntent,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rj_msgs, msg, RobotIntent)() {
  return &_RobotIntent__type_support;
}

#if defined(__cplusplus)
}
#endif
