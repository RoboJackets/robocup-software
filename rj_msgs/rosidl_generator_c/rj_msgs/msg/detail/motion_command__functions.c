// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rj_msgs:msg/MotionCommand.idl
// generated code does not contain a copyright notice
#include "rj_msgs/msg/detail/motion_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `empty_command`
#include "rj_msgs/msg/detail/empty_motion_command__functions.h"
// Member `path_target_command`
#include "rj_msgs/msg/detail/path_target_motion_command__functions.h"
// Member `world_vel_command`
#include "rj_msgs/msg/detail/world_vel_motion_command__functions.h"
// Member `pivot_command`
#include "rj_msgs/msg/detail/pivot_motion_command__functions.h"
// Member `settle_command`
#include "rj_msgs/msg/detail/settle_motion_command__functions.h"
// Member `collect_command`
#include "rj_msgs/msg/detail/collect_motion_command__functions.h"
// Member `line_kick_command`
#include "rj_msgs/msg/detail/line_kick_motion_command__functions.h"
// Member `intercept_command`
#include "rj_msgs/msg/detail/intercept_motion_command__functions.h"

bool
rj_msgs__msg__MotionCommand__init(rj_msgs__msg__MotionCommand * msg)
{
  if (!msg) {
    return false;
  }
  // empty_command
  if (!rj_msgs__msg__EmptyMotionCommand__Sequence__init(&msg->empty_command, 0)) {
    rj_msgs__msg__MotionCommand__fini(msg);
    return false;
  }
  // path_target_command
  if (!rj_msgs__msg__PathTargetMotionCommand__Sequence__init(&msg->path_target_command, 0)) {
    rj_msgs__msg__MotionCommand__fini(msg);
    return false;
  }
  // world_vel_command
  if (!rj_msgs__msg__WorldVelMotionCommand__Sequence__init(&msg->world_vel_command, 0)) {
    rj_msgs__msg__MotionCommand__fini(msg);
    return false;
  }
  // pivot_command
  if (!rj_msgs__msg__PivotMotionCommand__Sequence__init(&msg->pivot_command, 0)) {
    rj_msgs__msg__MotionCommand__fini(msg);
    return false;
  }
  // settle_command
  if (!rj_msgs__msg__SettleMotionCommand__Sequence__init(&msg->settle_command, 0)) {
    rj_msgs__msg__MotionCommand__fini(msg);
    return false;
  }
  // collect_command
  if (!rj_msgs__msg__CollectMotionCommand__Sequence__init(&msg->collect_command, 0)) {
    rj_msgs__msg__MotionCommand__fini(msg);
    return false;
  }
  // line_kick_command
  if (!rj_msgs__msg__LineKickMotionCommand__Sequence__init(&msg->line_kick_command, 0)) {
    rj_msgs__msg__MotionCommand__fini(msg);
    return false;
  }
  // intercept_command
  if (!rj_msgs__msg__InterceptMotionCommand__Sequence__init(&msg->intercept_command, 0)) {
    rj_msgs__msg__MotionCommand__fini(msg);
    return false;
  }
  return true;
}

void
rj_msgs__msg__MotionCommand__fini(rj_msgs__msg__MotionCommand * msg)
{
  if (!msg) {
    return;
  }
  // empty_command
  rj_msgs__msg__EmptyMotionCommand__Sequence__fini(&msg->empty_command);
  // path_target_command
  rj_msgs__msg__PathTargetMotionCommand__Sequence__fini(&msg->path_target_command);
  // world_vel_command
  rj_msgs__msg__WorldVelMotionCommand__Sequence__fini(&msg->world_vel_command);
  // pivot_command
  rj_msgs__msg__PivotMotionCommand__Sequence__fini(&msg->pivot_command);
  // settle_command
  rj_msgs__msg__SettleMotionCommand__Sequence__fini(&msg->settle_command);
  // collect_command
  rj_msgs__msg__CollectMotionCommand__Sequence__fini(&msg->collect_command);
  // line_kick_command
  rj_msgs__msg__LineKickMotionCommand__Sequence__fini(&msg->line_kick_command);
  // intercept_command
  rj_msgs__msg__InterceptMotionCommand__Sequence__fini(&msg->intercept_command);
}

rj_msgs__msg__MotionCommand *
rj_msgs__msg__MotionCommand__create()
{
  rj_msgs__msg__MotionCommand * msg = (rj_msgs__msg__MotionCommand *)malloc(sizeof(rj_msgs__msg__MotionCommand));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rj_msgs__msg__MotionCommand));
  bool success = rj_msgs__msg__MotionCommand__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
rj_msgs__msg__MotionCommand__destroy(rj_msgs__msg__MotionCommand * msg)
{
  if (msg) {
    rj_msgs__msg__MotionCommand__fini(msg);
  }
  free(msg);
}


bool
rj_msgs__msg__MotionCommand__Sequence__init(rj_msgs__msg__MotionCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rj_msgs__msg__MotionCommand * data = NULL;
  if (size) {
    data = (rj_msgs__msg__MotionCommand *)calloc(size, sizeof(rj_msgs__msg__MotionCommand));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rj_msgs__msg__MotionCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rj_msgs__msg__MotionCommand__fini(&data[i - 1]);
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
rj_msgs__msg__MotionCommand__Sequence__fini(rj_msgs__msg__MotionCommand__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rj_msgs__msg__MotionCommand__fini(&array->data[i]);
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

rj_msgs__msg__MotionCommand__Sequence *
rj_msgs__msg__MotionCommand__Sequence__create(size_t size)
{
  rj_msgs__msg__MotionCommand__Sequence * array = (rj_msgs__msg__MotionCommand__Sequence *)malloc(sizeof(rj_msgs__msg__MotionCommand__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = rj_msgs__msg__MotionCommand__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
rj_msgs__msg__MotionCommand__Sequence__destroy(rj_msgs__msg__MotionCommand__Sequence * array)
{
  if (array) {
    rj_msgs__msg__MotionCommand__Sequence__fini(array);
  }
  free(array);
}
