// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rj_msgs:msg/GameSettings.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__GAME_SETTINGS__STRUCT_H_
#define RJ_MSGS__MSG__DETAIL__GAME_SETTINGS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/GameSettings in the package rj_msgs.
typedef struct rj_msgs__msg__GameSettings
{
  bool simulation;
  bool request_blue_team;
  int32_t request_goalie_id;
  bool defend_plus_x;
  bool use_our_half;
  bool use_their_half;
  bool paused;
} rj_msgs__msg__GameSettings;

// Struct for a sequence of rj_msgs__msg__GameSettings.
typedef struct rj_msgs__msg__GameSettings__Sequence
{
  rj_msgs__msg__GameSettings * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rj_msgs__msg__GameSettings__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RJ_MSGS__MSG__DETAIL__GAME_SETTINGS__STRUCT_H_
