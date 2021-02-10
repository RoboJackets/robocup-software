// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rj_msgs:msg/FieldDimensions.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__FIELD_DIMENSIONS__STRUCT_H_
#define RJ_MSGS__MSG__DETAIL__FIELD_DIMENSIONS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/FieldDimensions in the package rj_msgs.
typedef struct rj_msgs__msg__FieldDimensions
{
  float length;
  float width;
  float border;
  float line_width;
  float goal_width;
  float goal_depth;
  float goal_height;
  float penalty_short_dist;
  float penalty_long_dist;
  float center_radius;
  float center_diameter;
  float goal_flat;
  float floor_length;
  float floor_width;
} rj_msgs__msg__FieldDimensions;

// Struct for a sequence of rj_msgs__msg__FieldDimensions.
typedef struct rj_msgs__msg__FieldDimensions__Sequence
{
  rj_msgs__msg__FieldDimensions * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rj_msgs__msg__FieldDimensions__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RJ_MSGS__MSG__DETAIL__FIELD_DIMENSIONS__STRUCT_H_
