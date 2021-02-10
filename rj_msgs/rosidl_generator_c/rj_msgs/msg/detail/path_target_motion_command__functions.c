// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rj_msgs:msg/PathTargetMotionCommand.idl
// generated code does not contain a copyright notice
#include "rj_msgs/msg/detail/path_target_motion_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `target`
#include "rj_msgs/msg/detail/linear_motion_instant__functions.h"
// Member `override_angle`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `override_face_point`
#include "rj_geometry_msgs/msg/detail/point__functions.h"

bool
rj_msgs__msg__PathTargetMotionCommand__init(rj_msgs__msg__PathTargetMotionCommand * msg)
{
  if (!msg) {
    return false;
  }
  // target
  if (!rj_msgs__msg__LinearMotionInstant__init(&msg->target)) {
    rj_msgs__msg__PathTargetMotionCommand__fini(msg);
    return false;
  }
  // override_angle
  if (!rosidl_runtime_c__double__Sequence__init(&msg->override_angle, 0)) {
    rj_msgs__msg__PathTargetMotionCommand__fini(msg);
    return false;
  }
  // override_face_point
  if (!rj_geometry_msgs__msg__Point__Sequence__init(&msg->override_face_point, 0)) {
    rj_msgs__msg__PathTargetMotionCommand__fini(msg);
    return false;
  }
  return true;
}

void
rj_msgs__msg__PathTargetMotionCommand__fini(rj_msgs__msg__PathTargetMotionCommand * msg)
{
  if (!msg) {
    return;
  }
  // target
  rj_msgs__msg__LinearMotionInstant__fini(&msg->target);
  // override_angle
  rosidl_runtime_c__double__Sequence__fini(&msg->override_angle);
  // override_face_point
  rj_geometry_msgs__msg__Point__Sequence__fini(&msg->override_face_point);
}

rj_msgs__msg__PathTargetMotionCommand *
rj_msgs__msg__PathTargetMotionCommand__create()
{
  rj_msgs__msg__PathTargetMotionCommand * msg = (rj_msgs__msg__PathTargetMotionCommand *)malloc(sizeof(rj_msgs__msg__PathTargetMotionCommand));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rj_msgs__msg__PathTargetMotionCommand));
  bool success = rj_msgs__msg__PathTargetMotionCommand__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
rj_msgs__msg__PathTargetMotionCommand__destroy(rj_msgs__msg__PathTargetMotionCommand * msg)
{
  if (msg) {
    rj_msgs__msg__PathTargetMotionCommand__fini(msg);
  }
  free(msg);
}


bool
rj_msgs__msg__PathTargetMotionCommand__Sequence__init(rj_msgs__msg__PathTargetMotionCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rj_msgs__msg__PathTargetMotionCommand * data = NULL;
  if (size) {
    data = (rj_msgs__msg__PathTargetMotionCommand *)calloc(size, sizeof(rj_msgs__msg__PathTargetMotionCommand));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rj_msgs__msg__PathTargetMotionCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rj_msgs__msg__PathTargetMotionCommand__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
rj_msgs__msg__PathTargetMotionCommand__Sequence__fini(rj_msgs__msg__PathTargetMotionCommand__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rj_msgs__msg__PathTargetMotionCommand__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

rj_msgs__msg__PathTargetMotionCommand__Sequence *
rj_msgs__msg__PathTargetMotionCommand__Sequence__create(size_t size)
{
  rj_msgs__msg__PathTargetMotionCommand__Sequence * array = (rj_msgs__msg__PathTargetMotionCommand__Sequence *)malloc(sizeof(rj_msgs__msg__PathTargetMotionCommand__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = rj_msgs__msg__PathTargetMotionCommand__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
rj_msgs__msg__PathTargetMotionCommand__Sequence__destroy(rj_msgs__msg__PathTargetMotionCommand__Sequence * array)
{
  if (array) {
    rj_msgs__msg__PathTargetMotionCommand__Sequence__fini(array);
  }
  free(array);
}
