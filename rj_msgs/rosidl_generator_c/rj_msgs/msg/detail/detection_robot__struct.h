// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rj_msgs:msg/DetectionRobot.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__DETECTION_ROBOT__STRUCT_H_
#define RJ_MSGS__MSG__DETAIL__DETECTION_ROBOT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/DetectionRobot in the package rj_msgs.
typedef struct rj_msgs__msg__DetectionRobot
{
  float confidence;
  uint32_t robot_id;
  float x;
  float y;
  float orientation;
  float pixel_x;
  float pixel_y;
  float height;
} rj_msgs__msg__DetectionRobot;

// Struct for a sequence of rj_msgs__msg__DetectionRobot.
typedef struct rj_msgs__msg__DetectionRobot__Sequence
{
  rj_msgs__msg__DetectionRobot * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rj_msgs__msg__DetectionRobot__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RJ_MSGS__MSG__DETAIL__DETECTION_ROBOT__STRUCT_H_
