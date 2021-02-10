// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rj_msgs:msg/TeamColor.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__TEAM_COLOR__STRUCT_H_
#define RJ_MSGS__MSG__DETAIL__TEAM_COLOR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/TeamColor in the package rj_msgs.
typedef struct rj_msgs__msg__TeamColor
{
  bool is_blue;
} rj_msgs__msg__TeamColor;

// Struct for a sequence of rj_msgs__msg__TeamColor.
typedef struct rj_msgs__msg__TeamColor__Sequence
{
  rj_msgs__msg__TeamColor * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rj_msgs__msg__TeamColor__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RJ_MSGS__MSG__DETAIL__TEAM_COLOR__STRUCT_H_
