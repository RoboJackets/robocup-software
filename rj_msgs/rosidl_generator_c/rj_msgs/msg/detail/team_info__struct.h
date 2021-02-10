// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rj_msgs:msg/TeamInfo.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__TEAM_INFO__STRUCT_H_
#define RJ_MSGS__MSG__DETAIL__TEAM_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'name'
#include "rosidl_runtime_c/string.h"
// Member 'yellow_card_remaining_times'
// Member 'remaining_timeout_time'
#include "builtin_interfaces/msg/detail/duration__struct.h"

// Struct defined in msg/TeamInfo in the package rj_msgs.
typedef struct rj_msgs__msg__TeamInfo
{
  rosidl_runtime_c__String name;
  int32_t score;
  uint64_t num_red_cards;
  uint64_t num_yellow_cards;
  builtin_interfaces__msg__Duration__Sequence yellow_card_remaining_times;
  uint64_t timeouts_left;
  builtin_interfaces__msg__Duration remaining_timeout_time;
  uint8_t goalie_id;
} rj_msgs__msg__TeamInfo;

// Struct for a sequence of rj_msgs__msg__TeamInfo.
typedef struct rj_msgs__msg__TeamInfo__Sequence
{
  rj_msgs__msg__TeamInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rj_msgs__msg__TeamInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RJ_MSGS__MSG__DETAIL__TEAM_INFO__STRUCT_H_
