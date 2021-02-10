// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rj_msgs:srv/SetFieldDimensions.idl
// generated code does not contain a copyright notice
#include "rj_msgs/srv/detail/set_field_dimensions__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

// Include directives for member types
// Member `field_dimensions`
#include "rj_msgs/msg/detail/field_dimensions__functions.h"

bool
rj_msgs__srv__SetFieldDimensions_Request__init(rj_msgs__srv__SetFieldDimensions_Request * msg)
{
  if (!msg) {
    return false;
  }
  // field_dimensions
  if (!rj_msgs__msg__FieldDimensions__init(&msg->field_dimensions)) {
    rj_msgs__srv__SetFieldDimensions_Request__fini(msg);
    return false;
  }
  return true;
}

void
rj_msgs__srv__SetFieldDimensions_Request__fini(rj_msgs__srv__SetFieldDimensions_Request * msg)
{
  if (!msg) {
    return;
  }
  // field_dimensions
  rj_msgs__msg__FieldDimensions__fini(&msg->field_dimensions);
}

rj_msgs__srv__SetFieldDimensions_Request *
rj_msgs__srv__SetFieldDimensions_Request__create()
{
  rj_msgs__srv__SetFieldDimensions_Request * msg = (rj_msgs__srv__SetFieldDimensions_Request *)malloc(sizeof(rj_msgs__srv__SetFieldDimensions_Request));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rj_msgs__srv__SetFieldDimensions_Request));
  bool success = rj_msgs__srv__SetFieldDimensions_Request__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
rj_msgs__srv__SetFieldDimensions_Request__destroy(rj_msgs__srv__SetFieldDimensions_Request * msg)
{
  if (msg) {
    rj_msgs__srv__SetFieldDimensions_Request__fini(msg);
  }
  free(msg);
}


bool
rj_msgs__srv__SetFieldDimensions_Request__Sequence__init(rj_msgs__srv__SetFieldDimensions_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rj_msgs__srv__SetFieldDimensions_Request * data = NULL;
  if (size) {
    data = (rj_msgs__srv__SetFieldDimensions_Request *)calloc(size, sizeof(rj_msgs__srv__SetFieldDimensions_Request));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rj_msgs__srv__SetFieldDimensions_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rj_msgs__srv__SetFieldDimensions_Request__fini(&data[i - 1]);
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
rj_msgs__srv__SetFieldDimensions_Request__Sequence__fini(rj_msgs__srv__SetFieldDimensions_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rj_msgs__srv__SetFieldDimensions_Request__fini(&array->data[i]);
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

rj_msgs__srv__SetFieldDimensions_Request__Sequence *
rj_msgs__srv__SetFieldDimensions_Request__Sequence__create(size_t size)
{
  rj_msgs__srv__SetFieldDimensions_Request__Sequence * array = (rj_msgs__srv__SetFieldDimensions_Request__Sequence *)malloc(sizeof(rj_msgs__srv__SetFieldDimensions_Request__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = rj_msgs__srv__SetFieldDimensions_Request__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
rj_msgs__srv__SetFieldDimensions_Request__Sequence__destroy(rj_msgs__srv__SetFieldDimensions_Request__Sequence * array)
{
  if (array) {
    rj_msgs__srv__SetFieldDimensions_Request__Sequence__fini(array);
  }
  free(array);
}


bool
rj_msgs__srv__SetFieldDimensions_Response__init(rj_msgs__srv__SetFieldDimensions_Response * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
rj_msgs__srv__SetFieldDimensions_Response__fini(rj_msgs__srv__SetFieldDimensions_Response * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

rj_msgs__srv__SetFieldDimensions_Response *
rj_msgs__srv__SetFieldDimensions_Response__create()
{
  rj_msgs__srv__SetFieldDimensions_Response * msg = (rj_msgs__srv__SetFieldDimensions_Response *)malloc(sizeof(rj_msgs__srv__SetFieldDimensions_Response));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rj_msgs__srv__SetFieldDimensions_Response));
  bool success = rj_msgs__srv__SetFieldDimensions_Response__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
rj_msgs__srv__SetFieldDimensions_Response__destroy(rj_msgs__srv__SetFieldDimensions_Response * msg)
{
  if (msg) {
    rj_msgs__srv__SetFieldDimensions_Response__fini(msg);
  }
  free(msg);
}


bool
rj_msgs__srv__SetFieldDimensions_Response__Sequence__init(rj_msgs__srv__SetFieldDimensions_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rj_msgs__srv__SetFieldDimensions_Response * data = NULL;
  if (size) {
    data = (rj_msgs__srv__SetFieldDimensions_Response *)calloc(size, sizeof(rj_msgs__srv__SetFieldDimensions_Response));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rj_msgs__srv__SetFieldDimensions_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rj_msgs__srv__SetFieldDimensions_Response__fini(&data[i - 1]);
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
rj_msgs__srv__SetFieldDimensions_Response__Sequence__fini(rj_msgs__srv__SetFieldDimensions_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rj_msgs__srv__SetFieldDimensions_Response__fini(&array->data[i]);
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

rj_msgs__srv__SetFieldDimensions_Response__Sequence *
rj_msgs__srv__SetFieldDimensions_Response__Sequence__create(size_t size)
{
  rj_msgs__srv__SetFieldDimensions_Response__Sequence * array = (rj_msgs__srv__SetFieldDimensions_Response__Sequence *)malloc(sizeof(rj_msgs__srv__SetFieldDimensions_Response__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = rj_msgs__srv__SetFieldDimensions_Response__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
rj_msgs__srv__SetFieldDimensions_Response__Sequence__destroy(rj_msgs__srv__SetFieldDimensions_Response__Sequence * array)
{
  if (array) {
    rj_msgs__srv__SetFieldDimensions_Response__Sequence__fini(array);
  }
  free(array);
}
