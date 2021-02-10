// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rj_msgs:msg/RawProtobuf.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__RAW_PROTOBUF__STRUCT_H_
#define RJ_MSGS__MSG__DETAIL__RAW_PROTOBUF__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'data'
#include "rosidl_runtime_c/primitives_sequence.h"

// Struct defined in msg/RawProtobuf in the package rj_msgs.
typedef struct rj_msgs__msg__RawProtobuf
{
  rosidl_runtime_c__octet__Sequence data;
} rj_msgs__msg__RawProtobuf;

// Struct for a sequence of rj_msgs__msg__RawProtobuf.
typedef struct rj_msgs__msg__RawProtobuf__Sequence
{
  rj_msgs__msg__RawProtobuf * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rj_msgs__msg__RawProtobuf__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RJ_MSGS__MSG__DETAIL__RAW_PROTOBUF__STRUCT_H_
