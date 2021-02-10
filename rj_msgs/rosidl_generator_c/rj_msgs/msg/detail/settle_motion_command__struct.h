// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rj_msgs:msg/SettleMotionCommand.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__SETTLE_MOTION_COMMAND__STRUCT_H_
#define RJ_MSGS__MSG__DETAIL__SETTLE_MOTION_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'maybe_target'
#include "rj_geometry_msgs/msg/detail/point__struct.h"

// constants for array fields with an upper bound
// maybe_target
enum
{
  rj_msgs__msg__SettleMotionCommand__maybe_target__MAX_SIZE = 1
};

// Struct defined in msg/SettleMotionCommand in the package rj_msgs.
typedef struct rj_msgs__msg__SettleMotionCommand
{
  rj_geometry_msgs__msg__Point__Sequence maybe_target;
} rj_msgs__msg__SettleMotionCommand;

// Struct for a sequence of rj_msgs__msg__SettleMotionCommand.
typedef struct rj_msgs__msg__SettleMotionCommand__Sequence
{
  rj_msgs__msg__SettleMotionCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rj_msgs__msg__SettleMotionCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RJ_MSGS__MSG__DETAIL__SETTLE_MOTION_COMMAND__STRUCT_H_
