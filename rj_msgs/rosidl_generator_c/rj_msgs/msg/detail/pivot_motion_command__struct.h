// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rj_msgs:msg/PivotMotionCommand.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__PIVOT_MOTION_COMMAND__STRUCT_H_
#define RJ_MSGS__MSG__DETAIL__PIVOT_MOTION_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'pivot_point'
// Member 'pivot_target'
#include "rj_geometry_msgs/msg/detail/point__struct.h"

// Struct defined in msg/PivotMotionCommand in the package rj_msgs.
typedef struct rj_msgs__msg__PivotMotionCommand
{
  rj_geometry_msgs__msg__Point pivot_point;
  rj_geometry_msgs__msg__Point pivot_target;
} rj_msgs__msg__PivotMotionCommand;

// Struct for a sequence of rj_msgs__msg__PivotMotionCommand.
typedef struct rj_msgs__msg__PivotMotionCommand__Sequence
{
  rj_msgs__msg__PivotMotionCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rj_msgs__msg__PivotMotionCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RJ_MSGS__MSG__DETAIL__PIVOT_MOTION_COMMAND__STRUCT_H_
