// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rj_msgs:msg/DetectionFrame.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__DETECTION_FRAME__STRUCT_H_
#define RJ_MSGS__MSG__DETAIL__DETECTION_FRAME__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 't_capture'
// Member 't_sent'
// Member 't_received'
#include "builtin_interfaces/msg/detail/time__struct.h"
// Member 'balls'
#include "rj_msgs/msg/detail/detection_ball__struct.h"
// Member 'robots_yellow'
// Member 'robots_blue'
#include "rj_msgs/msg/detail/detection_robot__struct.h"

// Struct defined in msg/DetectionFrame in the package rj_msgs.
typedef struct rj_msgs__msg__DetectionFrame
{
  uint32_t frame_number;
  builtin_interfaces__msg__Time t_capture;
  builtin_interfaces__msg__Time t_sent;
  builtin_interfaces__msg__Time t_received;
  uint32_t camera_id;
  rj_msgs__msg__DetectionBall__Sequence balls;
  rj_msgs__msg__DetectionRobot__Sequence robots_yellow;
  rj_msgs__msg__DetectionRobot__Sequence robots_blue;
} rj_msgs__msg__DetectionFrame;

// Struct for a sequence of rj_msgs__msg__DetectionFrame.
typedef struct rj_msgs__msg__DetectionFrame__Sequence
{
  rj_msgs__msg__DetectionFrame * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rj_msgs__msg__DetectionFrame__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RJ_MSGS__MSG__DETAIL__DETECTION_FRAME__STRUCT_H_
