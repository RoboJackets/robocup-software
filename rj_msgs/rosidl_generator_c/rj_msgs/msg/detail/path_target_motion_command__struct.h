// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rj_msgs:msg/PathTargetMotionCommand.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__PATH_TARGET_MOTION_COMMAND__STRUCT_H_
#define RJ_MSGS__MSG__DETAIL__PATH_TARGET_MOTION_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'target'
#include "rj_msgs/msg/detail/linear_motion_instant__struct.h"
// Member 'override_angle'
#include "rosidl_runtime_c/primitives_sequence.h"
// Member 'override_face_point'
#include "rj_geometry_msgs/msg/detail/point__struct.h"

// constants for array fields with an upper bound
// override_angle
enum
{
  rj_msgs__msg__PathTargetMotionCommand__override_angle__MAX_SIZE = 1
};
// override_face_point
enum
{
  rj_msgs__msg__PathTargetMotionCommand__override_face_point__MAX_SIZE = 1
};

// Struct defined in msg/PathTargetMotionCommand in the package rj_msgs.
typedef struct rj_msgs__msg__PathTargetMotionCommand
{
  rj_msgs__msg__LinearMotionInstant target;
  rosidl_runtime_c__double__Sequence override_angle;
  rj_geometry_msgs__msg__Point__Sequence override_face_point;
} rj_msgs__msg__PathTargetMotionCommand;

// Struct for a sequence of rj_msgs__msg__PathTargetMotionCommand.
typedef struct rj_msgs__msg__PathTargetMotionCommand__Sequence
{
  rj_msgs__msg__PathTargetMotionCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rj_msgs__msg__PathTargetMotionCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RJ_MSGS__MSG__DETAIL__PATH_TARGET_MOTION_COMMAND__STRUCT_H_
