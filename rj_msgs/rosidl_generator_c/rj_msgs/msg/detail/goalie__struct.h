// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rj_msgs:msg/Goalie.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__GOALIE__STRUCT_H_
#define RJ_MSGS__MSG__DETAIL__GOALIE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/Goalie in the package rj_msgs.
typedef struct rj_msgs__msg__Goalie
{
  uint8_t goalie_id;
} rj_msgs__msg__Goalie;

// Struct for a sequence of rj_msgs__msg__Goalie.
typedef struct rj_msgs__msg__Goalie__Sequence
{
  rj_msgs__msg__Goalie * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rj_msgs__msg__Goalie__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RJ_MSGS__MSG__DETAIL__GOALIE__STRUCT_H_
