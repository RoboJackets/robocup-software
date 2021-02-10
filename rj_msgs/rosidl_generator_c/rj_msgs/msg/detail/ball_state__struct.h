// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rj_msgs:msg/BallState.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__BALL_STATE__STRUCT_H_
#define RJ_MSGS__MSG__DETAIL__BALL_STATE__STRUCT_H_

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
// Member 'position'
// Member 'velocity'
#include "rj_geometry_msgs/msg/detail/point__struct.h"

// Struct defined in msg/BallState in the package rj_msgs.
typedef struct rj_msgs__msg__BallState
{
  builtin_interfaces__msg__Time stamp;
  rj_geometry_msgs__msg__Point position;
  rj_geometry_msgs__msg__Point velocity;
  bool visible;
} rj_msgs__msg__BallState;

// Struct for a sequence of rj_msgs__msg__BallState.
typedef struct rj_msgs__msg__BallState__Sequence
{
  rj_msgs__msg__BallState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rj_msgs__msg__BallState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RJ_MSGS__MSG__DETAIL__BALL_STATE__STRUCT_H_
