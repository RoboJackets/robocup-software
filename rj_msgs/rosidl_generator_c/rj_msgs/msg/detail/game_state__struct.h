// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rj_msgs:msg/GameState.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__GAME_STATE__STRUCT_H_
#define RJ_MSGS__MSG__DETAIL__GAME_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'stage_time_left'
#include "builtin_interfaces/msg/detail/duration__struct.h"
// Member 'placement_point'
#include "rj_geometry_msgs/msg/detail/point__struct.h"

// Struct defined in msg/GameState in the package rj_msgs.
typedef struct rj_msgs__msg__GameState
{
  uint8_t period;
  uint8_t state;
  uint8_t restart;
  bool our_restart;
  builtin_interfaces__msg__Duration stage_time_left;
  rj_geometry_msgs__msg__Point placement_point;
} rj_msgs__msg__GameState;

// Struct for a sequence of rj_msgs__msg__GameState.
typedef struct rj_msgs__msg__GameState__Sequence
{
  rj_msgs__msg__GameState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rj_msgs__msg__GameState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RJ_MSGS__MSG__DETAIL__GAME_STATE__STRUCT_H_
