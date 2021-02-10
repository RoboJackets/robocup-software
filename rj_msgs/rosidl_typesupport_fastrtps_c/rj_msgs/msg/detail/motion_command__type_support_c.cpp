// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from rj_msgs:msg/MotionCommand.idl
// generated code does not contain a copyright notice
#include "rj_msgs/msg/detail/motion_command__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rj_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "rj_msgs/msg/detail/motion_command__struct.h"
#include "rj_msgs/msg/detail/motion_command__functions.h"
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

#include "rj_msgs/msg/detail/collect_motion_command__functions.h"  // collect_command
#include "rj_msgs/msg/detail/empty_motion_command__functions.h"  // empty_command
#include "rj_msgs/msg/detail/intercept_motion_command__functions.h"  // intercept_command
#include "rj_msgs/msg/detail/line_kick_motion_command__functions.h"  // line_kick_command
#include "rj_msgs/msg/detail/path_target_motion_command__functions.h"  // path_target_command
#include "rj_msgs/msg/detail/pivot_motion_command__functions.h"  // pivot_command
#include "rj_msgs/msg/detail/settle_motion_command__functions.h"  // settle_command
#include "rj_msgs/msg/detail/world_vel_motion_command__functions.h"  // world_vel_command

// forward declare type support functions
size_t get_serialized_size_rj_msgs__msg__CollectMotionCommand(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_rj_msgs__msg__CollectMotionCommand(
  bool & full_bounded,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rj_msgs, msg, CollectMotionCommand)();
size_t get_serialized_size_rj_msgs__msg__EmptyMotionCommand(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_rj_msgs__msg__EmptyMotionCommand(
  bool & full_bounded,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rj_msgs, msg, EmptyMotionCommand)();
size_t get_serialized_size_rj_msgs__msg__InterceptMotionCommand(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_rj_msgs__msg__InterceptMotionCommand(
  bool & full_bounded,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rj_msgs, msg, InterceptMotionCommand)();
size_t get_serialized_size_rj_msgs__msg__LineKickMotionCommand(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_rj_msgs__msg__LineKickMotionCommand(
  bool & full_bounded,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rj_msgs, msg, LineKickMotionCommand)();
size_t get_serialized_size_rj_msgs__msg__PathTargetMotionCommand(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_rj_msgs__msg__PathTargetMotionCommand(
  bool & full_bounded,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rj_msgs, msg, PathTargetMotionCommand)();
size_t get_serialized_size_rj_msgs__msg__PivotMotionCommand(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_rj_msgs__msg__PivotMotionCommand(
  bool & full_bounded,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rj_msgs, msg, PivotMotionCommand)();
size_t get_serialized_size_rj_msgs__msg__SettleMotionCommand(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_rj_msgs__msg__SettleMotionCommand(
  bool & full_bounded,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rj_msgs, msg, SettleMotionCommand)();
size_t get_serialized_size_rj_msgs__msg__WorldVelMotionCommand(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_rj_msgs__msg__WorldVelMotionCommand(
  bool & full_bounded,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rj_msgs, msg, WorldVelMotionCommand)();


using _MotionCommand__ros_msg_type = rj_msgs__msg__MotionCommand;

static bool _MotionCommand__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _MotionCommand__ros_msg_type * ros_message = static_cast<const _MotionCommand__ros_msg_type *>(untyped_ros_message);
  // Field name: empty_command
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, rj_msgs, msg, EmptyMotionCommand
      )()->data);
    size_t size = ros_message->empty_command.size;
    auto array_ptr = ros_message->empty_command.data;
    if (size > 1) {
      fprintf(stderr, "array size exceeds upper bound\n");
      return false;
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_serialize(
          &array_ptr[i], cdr))
      {
        return false;
      }
    }
  }

  // Field name: path_target_command
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, rj_msgs, msg, PathTargetMotionCommand
      )()->data);
    size_t size = ros_message->path_target_command.size;
    auto array_ptr = ros_message->path_target_command.data;
    if (size > 1) {
      fprintf(stderr, "array size exceeds upper bound\n");
      return false;
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_serialize(
          &array_ptr[i], cdr))
      {
        return false;
      }
    }
  }

  // Field name: world_vel_command
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, rj_msgs, msg, WorldVelMotionCommand
      )()->data);
    size_t size = ros_message->world_vel_command.size;
    auto array_ptr = ros_message->world_vel_command.data;
    if (size > 1) {
      fprintf(stderr, "array size exceeds upper bound\n");
      return false;
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_serialize(
          &array_ptr[i], cdr))
      {
        return false;
      }
    }
  }

  // Field name: pivot_command
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, rj_msgs, msg, PivotMotionCommand
      )()->data);
    size_t size = ros_message->pivot_command.size;
    auto array_ptr = ros_message->pivot_command.data;
    if (size > 1) {
      fprintf(stderr, "array size exceeds upper bound\n");
      return false;
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_serialize(
          &array_ptr[i], cdr))
      {
        return false;
      }
    }
  }

  // Field name: settle_command
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, rj_msgs, msg, SettleMotionCommand
      )()->data);
    size_t size = ros_message->settle_command.size;
    auto array_ptr = ros_message->settle_command.data;
    if (size > 1) {
      fprintf(stderr, "array size exceeds upper bound\n");
      return false;
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_serialize(
          &array_ptr[i], cdr))
      {
        return false;
      }
    }
  }

  // Field name: collect_command
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, rj_msgs, msg, CollectMotionCommand
      )()->data);
    size_t size = ros_message->collect_command.size;
    auto array_ptr = ros_message->collect_command.data;
    if (size > 1) {
      fprintf(stderr, "array size exceeds upper bound\n");
      return false;
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_serialize(
          &array_ptr[i], cdr))
      {
        return false;
      }
    }
  }

  // Field name: line_kick_command
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, rj_msgs, msg, LineKickMotionCommand
      )()->data);
    size_t size = ros_message->line_kick_command.size;
    auto array_ptr = ros_message->line_kick_command.data;
    if (size > 1) {
      fprintf(stderr, "array size exceeds upper bound\n");
      return false;
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_serialize(
          &array_ptr[i], cdr))
      {
        return false;
      }
    }
  }

  // Field name: intercept_command
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, rj_msgs, msg, InterceptMotionCommand
      )()->data);
    size_t size = ros_message->intercept_command.size;
    auto array_ptr = ros_message->intercept_command.data;
    if (size > 1) {
      fprintf(stderr, "array size exceeds upper bound\n");
      return false;
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_serialize(
          &array_ptr[i], cdr))
      {
        return false;
      }
    }
  }

  return true;
}

static bool _MotionCommand__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _MotionCommand__ros_msg_type * ros_message = static_cast<_MotionCommand__ros_msg_type *>(untyped_ros_message);
  // Field name: empty_command
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, rj_msgs, msg, EmptyMotionCommand
      )()->data);
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->empty_command.data) {
      rj_msgs__msg__EmptyMotionCommand__Sequence__fini(&ros_message->empty_command);
    }
    if (!rj_msgs__msg__EmptyMotionCommand__Sequence__init(&ros_message->empty_command, size)) {
      return "failed to create array for field 'empty_command'";
    }
    auto array_ptr = ros_message->empty_command.data;
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_deserialize(
          cdr, &array_ptr[i]))
      {
        return false;
      }
    }
  }

  // Field name: path_target_command
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, rj_msgs, msg, PathTargetMotionCommand
      )()->data);
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->path_target_command.data) {
      rj_msgs__msg__PathTargetMotionCommand__Sequence__fini(&ros_message->path_target_command);
    }
    if (!rj_msgs__msg__PathTargetMotionCommand__Sequence__init(&ros_message->path_target_command, size)) {
      return "failed to create array for field 'path_target_command'";
    }
    auto array_ptr = ros_message->path_target_command.data;
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_deserialize(
          cdr, &array_ptr[i]))
      {
        return false;
      }
    }
  }

  // Field name: world_vel_command
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, rj_msgs, msg, WorldVelMotionCommand
      )()->data);
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->world_vel_command.data) {
      rj_msgs__msg__WorldVelMotionCommand__Sequence__fini(&ros_message->world_vel_command);
    }
    if (!rj_msgs__msg__WorldVelMotionCommand__Sequence__init(&ros_message->world_vel_command, size)) {
      return "failed to create array for field 'world_vel_command'";
    }
    auto array_ptr = ros_message->world_vel_command.data;
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_deserialize(
          cdr, &array_ptr[i]))
      {
        return false;
      }
    }
  }

  // Field name: pivot_command
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, rj_msgs, msg, PivotMotionCommand
      )()->data);
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->pivot_command.data) {
      rj_msgs__msg__PivotMotionCommand__Sequence__fini(&ros_message->pivot_command);
    }
    if (!rj_msgs__msg__PivotMotionCommand__Sequence__init(&ros_message->pivot_command, size)) {
      return "failed to create array for field 'pivot_command'";
    }
    auto array_ptr = ros_message->pivot_command.data;
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_deserialize(
          cdr, &array_ptr[i]))
      {
        return false;
      }
    }
  }

  // Field name: settle_command
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, rj_msgs, msg, SettleMotionCommand
      )()->data);
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->settle_command.data) {
      rj_msgs__msg__SettleMotionCommand__Sequence__fini(&ros_message->settle_command);
    }
    if (!rj_msgs__msg__SettleMotionCommand__Sequence__init(&ros_message->settle_command, size)) {
      return "failed to create array for field 'settle_command'";
    }
    auto array_ptr = ros_message->settle_command.data;
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_deserialize(
          cdr, &array_ptr[i]))
      {
        return false;
      }
    }
  }

  // Field name: collect_command
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, rj_msgs, msg, CollectMotionCommand
      )()->data);
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->collect_command.data) {
      rj_msgs__msg__CollectMotionCommand__Sequence__fini(&ros_message->collect_command);
    }
    if (!rj_msgs__msg__CollectMotionCommand__Sequence__init(&ros_message->collect_command, size)) {
      return "failed to create array for field 'collect_command'";
    }
    auto array_ptr = ros_message->collect_command.data;
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_deserialize(
          cdr, &array_ptr[i]))
      {
        return false;
      }
    }
  }

  // Field name: line_kick_command
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, rj_msgs, msg, LineKickMotionCommand
      )()->data);
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->line_kick_command.data) {
      rj_msgs__msg__LineKickMotionCommand__Sequence__fini(&ros_message->line_kick_command);
    }
    if (!rj_msgs__msg__LineKickMotionCommand__Sequence__init(&ros_message->line_kick_command, size)) {
      return "failed to create array for field 'line_kick_command'";
    }
    auto array_ptr = ros_message->line_kick_command.data;
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_deserialize(
          cdr, &array_ptr[i]))
      {
        return false;
      }
    }
  }

  // Field name: intercept_command
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, rj_msgs, msg, InterceptMotionCommand
      )()->data);
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->intercept_command.data) {
      rj_msgs__msg__InterceptMotionCommand__Sequence__fini(&ros_message->intercept_command);
    }
    if (!rj_msgs__msg__InterceptMotionCommand__Sequence__init(&ros_message->intercept_command, size)) {
      return "failed to create array for field 'intercept_command'";
    }
    auto array_ptr = ros_message->intercept_command.data;
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_deserialize(
          cdr, &array_ptr[i]))
      {
        return false;
      }
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rj_msgs
size_t get_serialized_size_rj_msgs__msg__MotionCommand(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _MotionCommand__ros_msg_type * ros_message = static_cast<const _MotionCommand__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name empty_command
  {
    size_t array_size = ros_message->empty_command.size;
    auto array_ptr = ros_message->empty_command.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_rj_msgs__msg__EmptyMotionCommand(
        &array_ptr[index], current_alignment);
    }
  }
  // field.name path_target_command
  {
    size_t array_size = ros_message->path_target_command.size;
    auto array_ptr = ros_message->path_target_command.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_rj_msgs__msg__PathTargetMotionCommand(
        &array_ptr[index], current_alignment);
    }
  }
  // field.name world_vel_command
  {
    size_t array_size = ros_message->world_vel_command.size;
    auto array_ptr = ros_message->world_vel_command.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_rj_msgs__msg__WorldVelMotionCommand(
        &array_ptr[index], current_alignment);
    }
  }
  // field.name pivot_command
  {
    size_t array_size = ros_message->pivot_command.size;
    auto array_ptr = ros_message->pivot_command.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_rj_msgs__msg__PivotMotionCommand(
        &array_ptr[index], current_alignment);
    }
  }
  // field.name settle_command
  {
    size_t array_size = ros_message->settle_command.size;
    auto array_ptr = ros_message->settle_command.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_rj_msgs__msg__SettleMotionCommand(
        &array_ptr[index], current_alignment);
    }
  }
  // field.name collect_command
  {
    size_t array_size = ros_message->collect_command.size;
    auto array_ptr = ros_message->collect_command.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_rj_msgs__msg__CollectMotionCommand(
        &array_ptr[index], current_alignment);
    }
  }
  // field.name line_kick_command
  {
    size_t array_size = ros_message->line_kick_command.size;
    auto array_ptr = ros_message->line_kick_command.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_rj_msgs__msg__LineKickMotionCommand(
        &array_ptr[index], current_alignment);
    }
  }
  // field.name intercept_command
  {
    size_t array_size = ros_message->intercept_command.size;
    auto array_ptr = ros_message->intercept_command.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_rj_msgs__msg__InterceptMotionCommand(
        &array_ptr[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static uint32_t _MotionCommand__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_rj_msgs__msg__MotionCommand(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rj_msgs
size_t max_serialized_size_rj_msgs__msg__MotionCommand(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: empty_command
  {
    size_t array_size = 1;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_rj_msgs__msg__EmptyMotionCommand(
        full_bounded, current_alignment);
    }
  }
  // member: path_target_command
  {
    size_t array_size = 1;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_rj_msgs__msg__PathTargetMotionCommand(
        full_bounded, current_alignment);
    }
  }
  // member: world_vel_command
  {
    size_t array_size = 1;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_rj_msgs__msg__WorldVelMotionCommand(
        full_bounded, current_alignment);
    }
  }
  // member: pivot_command
  {
    size_t array_size = 1;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_rj_msgs__msg__PivotMotionCommand(
        full_bounded, current_alignment);
    }
  }
  // member: settle_command
  {
    size_t array_size = 1;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_rj_msgs__msg__SettleMotionCommand(
        full_bounded, current_alignment);
    }
  }
  // member: collect_command
  {
    size_t array_size = 1;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_rj_msgs__msg__CollectMotionCommand(
        full_bounded, current_alignment);
    }
  }
  // member: line_kick_command
  {
    size_t array_size = 1;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_rj_msgs__msg__LineKickMotionCommand(
        full_bounded, current_alignment);
    }
  }
  // member: intercept_command
  {
    size_t array_size = 1;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_rj_msgs__msg__InterceptMotionCommand(
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static size_t _MotionCommand__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_rj_msgs__msg__MotionCommand(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_MotionCommand = {
  "rj_msgs::msg",
  "MotionCommand",
  _MotionCommand__cdr_serialize,
  _MotionCommand__cdr_deserialize,
  _MotionCommand__get_serialized_size,
  _MotionCommand__max_serialized_size
};

static rosidl_message_type_support_t _MotionCommand__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_MotionCommand,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rj_msgs, msg, MotionCommand)() {
  return &_MotionCommand__type_support;
}

#if defined(__cplusplus)
}
#endif
