// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rj_msgs:srv/SetFieldDimensions.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__SRV__DETAIL__SET_FIELD_DIMENSIONS__STRUCT_H_
#define RJ_MSGS__SRV__DETAIL__SET_FIELD_DIMENSIONS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'field_dimensions'
#include "rj_msgs/msg/detail/field_dimensions__struct.h"

// Struct defined in srv/SetFieldDimensions in the package rj_msgs.
typedef struct rj_msgs__srv__SetFieldDimensions_Request
{
  rj_msgs__msg__FieldDimensions field_dimensions;
} rj_msgs__srv__SetFieldDimensions_Request;

// Struct for a sequence of rj_msgs__srv__SetFieldDimensions_Request.
typedef struct rj_msgs__srv__SetFieldDimensions_Request__Sequence
{
  rj_msgs__srv__SetFieldDimensions_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rj_msgs__srv__SetFieldDimensions_Request__Sequence;


// Constants defined in the message

// Struct defined in srv/SetFieldDimensions in the package rj_msgs.
typedef struct rj_msgs__srv__SetFieldDimensions_Response
{
  uint8_t structure_needs_at_least_one_member;
} rj_msgs__srv__SetFieldDimensions_Response;

// Struct for a sequence of rj_msgs__srv__SetFieldDimensions_Response.
typedef struct rj_msgs__srv__SetFieldDimensions_Response__Sequence
{
  rj_msgs__srv__SetFieldDimensions_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rj_msgs__srv__SetFieldDimensions_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RJ_MSGS__SRV__DETAIL__SET_FIELD_DIMENSIONS__STRUCT_H_
