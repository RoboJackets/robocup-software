// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rj_msgs:msg/DetectionFrame.idl
// generated code does not contain a copyright notice
#include "rj_msgs/msg/detail/detection_frame__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `t_capture`
// Member `t_sent`
// Member `t_received`
#include "builtin_interfaces/msg/detail/time__functions.h"
// Member `balls`
#include "rj_msgs/msg/detail/detection_ball__functions.h"
// Member `robots_yellow`
// Member `robots_blue`
#include "rj_msgs/msg/detail/detection_robot__functions.h"

bool
rj_msgs__msg__DetectionFrame__init(rj_msgs__msg__DetectionFrame * msg)
{
  if (!msg) {
    return false;
  }
  // frame_number
  // t_capture
  if (!builtin_interfaces__msg__Time__init(&msg->t_capture)) {
    rj_msgs__msg__DetectionFrame__fini(msg);
    return false;
  }
  // t_sent
  if (!builtin_interfaces__msg__Time__init(&msg->t_sent)) {
    rj_msgs__msg__DetectionFrame__fini(msg);
    return false;
  }
  // t_received
  if (!builtin_interfaces__msg__Time__init(&msg->t_received)) {
    rj_msgs__msg__DetectionFrame__fini(msg);
    return false;
  }
  // camera_id
  // balls
  if (!rj_msgs__msg__DetectionBall__Sequence__init(&msg->balls, 0)) {
    rj_msgs__msg__DetectionFrame__fini(msg);
    return false;
  }
  // robots_yellow
  if (!rj_msgs__msg__DetectionRobot__Sequence__init(&msg->robots_yellow, 0)) {
    rj_msgs__msg__DetectionFrame__fini(msg);
    return false;
  }
  // robots_blue
  if (!rj_msgs__msg__DetectionRobot__Sequence__init(&msg->robots_blue, 0)) {
    rj_msgs__msg__DetectionFrame__fini(msg);
    return false;
  }
  return true;
}

void
rj_msgs__msg__DetectionFrame__fini(rj_msgs__msg__DetectionFrame * msg)
{
  if (!msg) {
    return;
  }
  // frame_number
  // t_capture
  builtin_interfaces__msg__Time__fini(&msg->t_capture);
  // t_sent
  builtin_interfaces__msg__Time__fini(&msg->t_sent);
  // t_received
  builtin_interfaces__msg__Time__fini(&msg->t_received);
  // camera_id
  // balls
  rj_msgs__msg__DetectionBall__Sequence__fini(&msg->balls);
  // robots_yellow
  rj_msgs__msg__DetectionRobot__Sequence__fini(&msg->robots_yellow);
  // robots_blue
  rj_msgs__msg__DetectionRobot__Sequence__fini(&msg->robots_blue);
}

rj_msgs__msg__DetectionFrame *
rj_msgs__msg__DetectionFrame__create()
{
  rj_msgs__msg__DetectionFrame * msg = (rj_msgs__msg__DetectionFrame *)malloc(sizeof(rj_msgs__msg__DetectionFrame));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rj_msgs__msg__DetectionFrame));
  bool success = rj_msgs__msg__DetectionFrame__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
rj_msgs__msg__DetectionFrame__destroy(rj_msgs__msg__DetectionFrame * msg)
{
  if (msg) {
    rj_msgs__msg__DetectionFrame__fini(msg);
  }
  free(msg);
}


bool
rj_msgs__msg__DetectionFrame__Sequence__init(rj_msgs__msg__DetectionFrame__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rj_msgs__msg__DetectionFrame * data = NULL;
  if (size) {
    data = (rj_msgs__msg__DetectionFrame *)calloc(size, sizeof(rj_msgs__msg__DetectionFrame));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rj_msgs__msg__DetectionFrame__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rj_msgs__msg__DetectionFrame__fini(&data[i - 1]);
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
rj_msgs__msg__DetectionFrame__Sequence__fini(rj_msgs__msg__DetectionFrame__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rj_msgs__msg__DetectionFrame__fini(&array->data[i]);
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

rj_msgs__msg__DetectionFrame__Sequence *
rj_msgs__msg__DetectionFrame__Sequence__create(size_t size)
{
  rj_msgs__msg__DetectionFrame__Sequence * array = (rj_msgs__msg__DetectionFrame__Sequence *)malloc(sizeof(rj_msgs__msg__DetectionFrame__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = rj_msgs__msg__DetectionFrame__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
rj_msgs__msg__DetectionFrame__Sequence__destroy(rj_msgs__msg__DetectionFrame__Sequence * array)
{
  if (array) {
    rj_msgs__msg__DetectionFrame__Sequence__fini(array);
  }
  free(array);
}
