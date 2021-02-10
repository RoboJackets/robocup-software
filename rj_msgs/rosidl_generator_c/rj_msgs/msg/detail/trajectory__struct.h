// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rj_msgs:msg/Trajectory.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__TRAJECTORY__STRUCT_H_
#define RJ_MSGS__MSG__DETAIL__TRAJECTORY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"
// Member 'instants'
#include "rj_msgs/msg/detail/robot_instant__struct.h"

// Struct defined in msg/Trajectory in the package rj_msgs.
typedef struct rj_msgs__msg__Trajectory
{
  builtin_interfaces__msg__Time stamp;
  rj_msgs__msg__RobotInstant__Sequence instants;
} rj_msgs__msg__Trajectory;

// Struct for a sequence of rj_msgs__msg__Trajectory.
typedef struct rj_msgs__msg__Trajectory__Sequence
{
  rj_msgs__msg__Trajectory * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rj_msgs__msg__Trajectory__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RJ_MSGS__MSG__DETAIL__TRAJECTORY__STRUCT_H_
