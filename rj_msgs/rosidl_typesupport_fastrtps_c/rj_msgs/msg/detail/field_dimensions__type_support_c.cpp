// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from rj_msgs:msg/FieldDimensions.idl
// generated code does not contain a copyright notice
#include "rj_msgs/msg/detail/field_dimensions__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rj_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "rj_msgs/msg/detail/field_dimensions__struct.h"
#include "rj_msgs/msg/detail/field_dimensions__functions.h"
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


using _FieldDimensions__ros_msg_type = rj_msgs__msg__FieldDimensions;

static bool _FieldDimensions__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _FieldDimensions__ros_msg_type * ros_message = static_cast<const _FieldDimensions__ros_msg_type *>(untyped_ros_message);
  // Field name: length
  {
    cdr << ros_message->length;
  }

  // Field name: width
  {
    cdr << ros_message->width;
  }

  // Field name: border
  {
    cdr << ros_message->border;
  }

  // Field name: line_width
  {
    cdr << ros_message->line_width;
  }

  // Field name: goal_width
  {
    cdr << ros_message->goal_width;
  }

  // Field name: goal_depth
  {
    cdr << ros_message->goal_depth;
  }

  // Field name: goal_height
  {
    cdr << ros_message->goal_height;
  }

  // Field name: penalty_short_dist
  {
    cdr << ros_message->penalty_short_dist;
  }

  // Field name: penalty_long_dist
  {
    cdr << ros_message->penalty_long_dist;
  }

  // Field name: center_radius
  {
    cdr << ros_message->center_radius;
  }

  // Field name: center_diameter
  {
    cdr << ros_message->center_diameter;
  }

  // Field name: goal_flat
  {
    cdr << ros_message->goal_flat;
  }

  // Field name: floor_length
  {
    cdr << ros_message->floor_length;
  }

  // Field name: floor_width
  {
    cdr << ros_message->floor_width;
  }

  return true;
}

static bool _FieldDimensions__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _FieldDimensions__ros_msg_type * ros_message = static_cast<_FieldDimensions__ros_msg_type *>(untyped_ros_message);
  // Field name: length
  {
    cdr >> ros_message->length;
  }

  // Field name: width
  {
    cdr >> ros_message->width;
  }

  // Field name: border
  {
    cdr >> ros_message->border;
  }

  // Field name: line_width
  {
    cdr >> ros_message->line_width;
  }

  // Field name: goal_width
  {
    cdr >> ros_message->goal_width;
  }

  // Field name: goal_depth
  {
    cdr >> ros_message->goal_depth;
  }

  // Field name: goal_height
  {
    cdr >> ros_message->goal_height;
  }

  // Field name: penalty_short_dist
  {
    cdr >> ros_message->penalty_short_dist;
  }

  // Field name: penalty_long_dist
  {
    cdr >> ros_message->penalty_long_dist;
  }

  // Field name: center_radius
  {
    cdr >> ros_message->center_radius;
  }

  // Field name: center_diameter
  {
    cdr >> ros_message->center_diameter;
  }

  // Field name: goal_flat
  {
    cdr >> ros_message->goal_flat;
  }

  // Field name: floor_length
  {
    cdr >> ros_message->floor_length;
  }

  // Field name: floor_width
  {
    cdr >> ros_message->floor_width;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rj_msgs
size_t get_serialized_size_rj_msgs__msg__FieldDimensions(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _FieldDimensions__ros_msg_type * ros_message = static_cast<const _FieldDimensions__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name length
  {
    size_t item_size = sizeof(ros_message->length);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name width
  {
    size_t item_size = sizeof(ros_message->width);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name border
  {
    size_t item_size = sizeof(ros_message->border);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name line_width
  {
    size_t item_size = sizeof(ros_message->line_width);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name goal_width
  {
    size_t item_size = sizeof(ros_message->goal_width);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name goal_depth
  {
    size_t item_size = sizeof(ros_message->goal_depth);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name goal_height
  {
    size_t item_size = sizeof(ros_message->goal_height);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name penalty_short_dist
  {
    size_t item_size = sizeof(ros_message->penalty_short_dist);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name penalty_long_dist
  {
    size_t item_size = sizeof(ros_message->penalty_long_dist);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name center_radius
  {
    size_t item_size = sizeof(ros_message->center_radius);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name center_diameter
  {
    size_t item_size = sizeof(ros_message->center_diameter);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name goal_flat
  {
    size_t item_size = sizeof(ros_message->goal_flat);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name floor_length
  {
    size_t item_size = sizeof(ros_message->floor_length);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name floor_width
  {
    size_t item_size = sizeof(ros_message->floor_width);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _FieldDimensions__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_rj_msgs__msg__FieldDimensions(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rj_msgs
size_t max_serialized_size_rj_msgs__msg__FieldDimensions(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: length
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: width
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: border
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: line_width
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: goal_width
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: goal_depth
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: goal_height
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: penalty_short_dist
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: penalty_long_dist
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: center_radius
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: center_diameter
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: goal_flat
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: floor_length
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: floor_width
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _FieldDimensions__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_rj_msgs__msg__FieldDimensions(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_FieldDimensions = {
  "rj_msgs::msg",
  "FieldDimensions",
  _FieldDimensions__cdr_serialize,
  _FieldDimensions__cdr_deserialize,
  _FieldDimensions__get_serialized_size,
  _FieldDimensions__max_serialized_size
};

static rosidl_message_type_support_t _FieldDimensions__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_FieldDimensions,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rj_msgs, msg, FieldDimensions)() {
  return &_FieldDimensions__type_support;
}

#if defined(__cplusplus)
}
#endif
