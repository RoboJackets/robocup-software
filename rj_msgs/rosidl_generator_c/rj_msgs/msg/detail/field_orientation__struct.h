// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rj_msgs:msg/FieldOrientation.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__FIELD_ORIENTATION__STRUCT_H_
#define RJ_MSGS__MSG__DETAIL__FIELD_ORIENTATION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/FieldOrientation in the package rj_msgs.
typedef struct rj_msgs__msg__FieldOrientation
{
  bool defend_plus_x;
} rj_msgs__msg__FieldOrientation;

// Struct for a sequence of rj_msgs__msg__FieldOrientation.
typedef struct rj_msgs__msg__FieldOrientation__Sequence
{
  rj_msgs__msg__FieldOrientation * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rj_msgs__msg__FieldOrientation__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RJ_MSGS__MSG__DETAIL__FIELD_ORIENTATION__STRUCT_H_
