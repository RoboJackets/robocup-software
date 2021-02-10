// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rj_msgs:msg/InterceptMotionCommand.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__INTERCEPT_MOTION_COMMAND__STRUCT_H_
#define RJ_MSGS__MSG__DETAIL__INTERCEPT_MOTION_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'target'
#include "rj_geometry_msgs/msg/detail/point__struct.h"

// Struct defined in msg/InterceptMotionCommand in the package rj_msgs.
typedef struct rj_msgs__msg__InterceptMotionCommand
{
  rj_geometry_msgs__msg__Point target;
} rj_msgs__msg__InterceptMotionCommand;

// Struct for a sequence of rj_msgs__msg__InterceptMotionCommand.
typedef struct rj_msgs__msg__InterceptMotionCommand__Sequence
{
  rj_msgs__msg__InterceptMotionCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rj_msgs__msg__InterceptMotionCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RJ_MSGS__MSG__DETAIL__INTERCEPT_MOTION_COMMAND__STRUCT_H_
