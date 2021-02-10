// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rj_msgs:msg/WorldState.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__WORLD_STATE__STRUCT_H_
#define RJ_MSGS__MSG__DETAIL__WORLD_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'last_update_time'
#include "builtin_interfaces/msg/detail/time__struct.h"
// Member 'their_robots'
// Member 'our_robots'
#include "rj_msgs/msg/detail/robot_state__struct.h"
// Member 'ball'
#include "rj_msgs/msg/detail/ball_state__struct.h"

// Struct defined in msg/WorldState in the package rj_msgs.
typedef struct rj_msgs__msg__WorldState
{
  builtin_interfaces__msg__Time last_update_time;
  rj_msgs__msg__RobotState__Sequence their_robots;
  rj_msgs__msg__RobotState__Sequence our_robots;
  rj_msgs__msg__BallState ball;
} rj_msgs__msg__WorldState;

// Struct for a sequence of rj_msgs__msg__WorldState.
typedef struct rj_msgs__msg__WorldState__Sequence
{
  rj_msgs__msg__WorldState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rj_msgs__msg__WorldState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RJ_MSGS__MSG__DETAIL__WORLD_STATE__STRUCT_H_
