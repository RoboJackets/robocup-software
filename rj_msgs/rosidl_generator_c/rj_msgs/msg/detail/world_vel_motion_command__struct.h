// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rj_msgs:msg/WorldVelMotionCommand.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__WORLD_VEL_MOTION_COMMAND__STRUCT_H_
#define RJ_MSGS__MSG__DETAIL__WORLD_VEL_MOTION_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'world_vel'
#include "rj_geometry_msgs/msg/detail/point__struct.h"

// Struct defined in msg/WorldVelMotionCommand in the package rj_msgs.
typedef struct rj_msgs__msg__WorldVelMotionCommand
{
  rj_geometry_msgs__msg__Point world_vel;
} rj_msgs__msg__WorldVelMotionCommand;

// Struct for a sequence of rj_msgs__msg__WorldVelMotionCommand.
typedef struct rj_msgs__msg__WorldVelMotionCommand__Sequence
{
  rj_msgs__msg__WorldVelMotionCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rj_msgs__msg__WorldVelMotionCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RJ_MSGS__MSG__DETAIL__WORLD_VEL_MOTION_COMMAND__STRUCT_H_
