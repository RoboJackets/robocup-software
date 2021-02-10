// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rj_msgs:msg/WorldState.idl
// generated code does not contain a copyright notice
#include "rj_msgs/msg/detail/world_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `last_update_time`
#include "builtin_interfaces/msg/detail/time__functions.h"
// Member `their_robots`
// Member `our_robots`
#include "rj_msgs/msg/detail/robot_state__functions.h"
// Member `ball`
#include "rj_msgs/msg/detail/ball_state__functions.h"

bool
rj_msgs__msg__WorldState__init(rj_msgs__msg__WorldState * msg)
{
  if (!msg) {
    return false;
  }
  // last_update_time
  if (!builtin_interfaces__msg__Time__init(&msg->last_update_time)) {
    rj_msgs__msg__WorldState__fini(msg);
    return false;
  }
  // their_robots
  if (!rj_msgs__msg__RobotState__Sequence__init(&msg->their_robots, 0)) {
    rj_msgs__msg__WorldState__fini(msg);
    return false;
  }
  // our_robots
  if (!rj_msgs__msg__RobotState__Sequence__init(&msg->our_robots, 0)) {
    rj_msgs__msg__WorldState__fini(msg);
    return false;
  }
  // ball
  if (!rj_msgs__msg__BallState__init(&msg->ball)) {
    rj_msgs__msg__WorldState__fini(msg);
    return false;
  }
  return true;
}

void
rj_msgs__msg__WorldState__fini(rj_msgs__msg__WorldState * msg)
{
  if (!msg) {
    return;
  }
  // last_update_time
  builtin_interfaces__msg__Time__fini(&msg->last_update_time);
  // their_robots
  rj_msgs__msg__RobotState__Sequence__fini(&msg->their_robots);
  // our_robots
  rj_msgs__msg__RobotState__Sequence__fini(&msg->our_robots);
  // ball
  rj_msgs__msg__BallState__fini(&msg->ball);
}

rj_msgs__msg__WorldState *
rj_msgs__msg__WorldState__create()
{
  rj_msgs__msg__WorldState * msg = (rj_msgs__msg__WorldState *)malloc(sizeof(rj_msgs__msg__WorldState));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rj_msgs__msg__WorldState));
  bool success = rj_msgs__msg__WorldState__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
rj_msgs__msg__WorldState__destroy(rj_msgs__msg__WorldState * msg)
{
  if (msg) {
    rj_msgs__msg__WorldState__fini(msg);
  }
  free(msg);
}


bool
rj_msgs__msg__WorldState__Sequence__init(rj_msgs__msg__WorldState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rj_msgs__msg__WorldState * data = NULL;
  if (size) {
    data = (rj_msgs__msg__WorldState *)calloc(size, sizeof(rj_msgs__msg__WorldState));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rj_msgs__msg__WorldState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rj_msgs__msg__WorldState__fini(&data[i - 1]);
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
rj_msgs__msg__WorldState__Sequence__fini(rj_msgs__msg__WorldState__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rj_msgs__msg__WorldState__fini(&array->data[i]);
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

rj_msgs__msg__WorldState__Sequence *
rj_msgs__msg__WorldState__Sequence__create(size_t size)
{
  rj_msgs__msg__WorldState__Sequence * array = (rj_msgs__msg__WorldState__Sequence *)malloc(sizeof(rj_msgs__msg__WorldState__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = rj_msgs__msg__WorldState__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
rj_msgs__msg__WorldState__Sequence__destroy(rj_msgs__msg__WorldState__Sequence * array)
{
  if (array) {
    rj_msgs__msg__WorldState__Sequence__fini(array);
  }
  free(array);
}
