// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from rj_msgs:msg/MotionCommand.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "rj_msgs/msg/detail/motion_command__struct.h"
#include "rj_msgs/msg/detail/motion_command__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

// Nested array functions includes
#include "rj_msgs/msg/detail/collect_motion_command__functions.h"
#include "rj_msgs/msg/detail/empty_motion_command__functions.h"
#include "rj_msgs/msg/detail/intercept_motion_command__functions.h"
#include "rj_msgs/msg/detail/line_kick_motion_command__functions.h"
#include "rj_msgs/msg/detail/path_target_motion_command__functions.h"
#include "rj_msgs/msg/detail/pivot_motion_command__functions.h"
#include "rj_msgs/msg/detail/settle_motion_command__functions.h"
#include "rj_msgs/msg/detail/world_vel_motion_command__functions.h"
// end nested array functions include
bool rj_msgs__msg__empty_motion_command__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * rj_msgs__msg__empty_motion_command__convert_to_py(void * raw_ros_message);
bool rj_msgs__msg__path_target_motion_command__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * rj_msgs__msg__path_target_motion_command__convert_to_py(void * raw_ros_message);
bool rj_msgs__msg__world_vel_motion_command__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * rj_msgs__msg__world_vel_motion_command__convert_to_py(void * raw_ros_message);
bool rj_msgs__msg__pivot_motion_command__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * rj_msgs__msg__pivot_motion_command__convert_to_py(void * raw_ros_message);
bool rj_msgs__msg__settle_motion_command__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * rj_msgs__msg__settle_motion_command__convert_to_py(void * raw_ros_message);
bool rj_msgs__msg__collect_motion_command__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * rj_msgs__msg__collect_motion_command__convert_to_py(void * raw_ros_message);
bool rj_msgs__msg__line_kick_motion_command__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * rj_msgs__msg__line_kick_motion_command__convert_to_py(void * raw_ros_message);
bool rj_msgs__msg__intercept_motion_command__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * rj_msgs__msg__intercept_motion_command__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool rj_msgs__msg__motion_command__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[42];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("rj_msgs.msg._motion_command.MotionCommand", full_classname_dest, 41) == 0);
  }
  rj_msgs__msg__MotionCommand * ros_message = _ros_message;
  {  // empty_command
    PyObject * field = PyObject_GetAttrString(_pymsg, "empty_command");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'empty_command'");
    if (!seq_field) {
      Py_DECREF(field);
      return false;
    }
    Py_ssize_t size = PySequence_Size(field);
    if (-1 == size) {
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    if (!rj_msgs__msg__EmptyMotionCommand__Sequence__init(&(ros_message->empty_command), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create rj_msgs__msg__EmptyMotionCommand__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    rj_msgs__msg__EmptyMotionCommand * dest = ros_message->empty_command.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!rj_msgs__msg__empty_motion_command__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }
  {  // path_target_command
    PyObject * field = PyObject_GetAttrString(_pymsg, "path_target_command");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'path_target_command'");
    if (!seq_field) {
      Py_DECREF(field);
      return false;
    }
    Py_ssize_t size = PySequence_Size(field);
    if (-1 == size) {
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    if (!rj_msgs__msg__PathTargetMotionCommand__Sequence__init(&(ros_message->path_target_command), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create rj_msgs__msg__PathTargetMotionCommand__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    rj_msgs__msg__PathTargetMotionCommand * dest = ros_message->path_target_command.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!rj_msgs__msg__path_target_motion_command__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }
  {  // world_vel_command
    PyObject * field = PyObject_GetAttrString(_pymsg, "world_vel_command");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'world_vel_command'");
    if (!seq_field) {
      Py_DECREF(field);
      return false;
    }
    Py_ssize_t size = PySequence_Size(field);
    if (-1 == size) {
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    if (!rj_msgs__msg__WorldVelMotionCommand__Sequence__init(&(ros_message->world_vel_command), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create rj_msgs__msg__WorldVelMotionCommand__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    rj_msgs__msg__WorldVelMotionCommand * dest = ros_message->world_vel_command.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!rj_msgs__msg__world_vel_motion_command__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }
  {  // pivot_command
    PyObject * field = PyObject_GetAttrString(_pymsg, "pivot_command");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'pivot_command'");
    if (!seq_field) {
      Py_DECREF(field);
      return false;
    }
    Py_ssize_t size = PySequence_Size(field);
    if (-1 == size) {
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    if (!rj_msgs__msg__PivotMotionCommand__Sequence__init(&(ros_message->pivot_command), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create rj_msgs__msg__PivotMotionCommand__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    rj_msgs__msg__PivotMotionCommand * dest = ros_message->pivot_command.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!rj_msgs__msg__pivot_motion_command__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }
  {  // settle_command
    PyObject * field = PyObject_GetAttrString(_pymsg, "settle_command");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'settle_command'");
    if (!seq_field) {
      Py_DECREF(field);
      return false;
    }
    Py_ssize_t size = PySequence_Size(field);
    if (-1 == size) {
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    if (!rj_msgs__msg__SettleMotionCommand__Sequence__init(&(ros_message->settle_command), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create rj_msgs__msg__SettleMotionCommand__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    rj_msgs__msg__SettleMotionCommand * dest = ros_message->settle_command.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!rj_msgs__msg__settle_motion_command__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }
  {  // collect_command
    PyObject * field = PyObject_GetAttrString(_pymsg, "collect_command");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'collect_command'");
    if (!seq_field) {
      Py_DECREF(field);
      return false;
    }
    Py_ssize_t size = PySequence_Size(field);
    if (-1 == size) {
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    if (!rj_msgs__msg__CollectMotionCommand__Sequence__init(&(ros_message->collect_command), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create rj_msgs__msg__CollectMotionCommand__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    rj_msgs__msg__CollectMotionCommand * dest = ros_message->collect_command.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!rj_msgs__msg__collect_motion_command__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }
  {  // line_kick_command
    PyObject * field = PyObject_GetAttrString(_pymsg, "line_kick_command");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'line_kick_command'");
    if (!seq_field) {
      Py_DECREF(field);
      return false;
    }
    Py_ssize_t size = PySequence_Size(field);
    if (-1 == size) {
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    if (!rj_msgs__msg__LineKickMotionCommand__Sequence__init(&(ros_message->line_kick_command), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create rj_msgs__msg__LineKickMotionCommand__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    rj_msgs__msg__LineKickMotionCommand * dest = ros_message->line_kick_command.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!rj_msgs__msg__line_kick_motion_command__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }
  {  // intercept_command
    PyObject * field = PyObject_GetAttrString(_pymsg, "intercept_command");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'intercept_command'");
    if (!seq_field) {
      Py_DECREF(field);
      return false;
    }
    Py_ssize_t size = PySequence_Size(field);
    if (-1 == size) {
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    if (!rj_msgs__msg__InterceptMotionCommand__Sequence__init(&(ros_message->intercept_command), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create rj_msgs__msg__InterceptMotionCommand__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    rj_msgs__msg__InterceptMotionCommand * dest = ros_message->intercept_command.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!rj_msgs__msg__intercept_motion_command__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * rj_msgs__msg__motion_command__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of MotionCommand */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("rj_msgs.msg._motion_command");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "MotionCommand");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  rj_msgs__msg__MotionCommand * ros_message = (rj_msgs__msg__MotionCommand *)raw_ros_message;
  {  // empty_command
    PyObject * field = NULL;
    size_t size = ros_message->empty_command.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    rj_msgs__msg__EmptyMotionCommand * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->empty_command.data[i]);
      PyObject * pyitem = rj_msgs__msg__empty_motion_command__convert_to_py(item);
      if (!pyitem) {
        Py_DECREF(field);
        return NULL;
      }
      int rc = PyList_SetItem(field, i, pyitem);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "empty_command", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // path_target_command
    PyObject * field = NULL;
    size_t size = ros_message->path_target_command.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    rj_msgs__msg__PathTargetMotionCommand * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->path_target_command.data[i]);
      PyObject * pyitem = rj_msgs__msg__path_target_motion_command__convert_to_py(item);
      if (!pyitem) {
        Py_DECREF(field);
        return NULL;
      }
      int rc = PyList_SetItem(field, i, pyitem);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "path_target_command", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // world_vel_command
    PyObject * field = NULL;
    size_t size = ros_message->world_vel_command.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    rj_msgs__msg__WorldVelMotionCommand * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->world_vel_command.data[i]);
      PyObject * pyitem = rj_msgs__msg__world_vel_motion_command__convert_to_py(item);
      if (!pyitem) {
        Py_DECREF(field);
        return NULL;
      }
      int rc = PyList_SetItem(field, i, pyitem);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "world_vel_command", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pivot_command
    PyObject * field = NULL;
    size_t size = ros_message->pivot_command.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    rj_msgs__msg__PivotMotionCommand * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->pivot_command.data[i]);
      PyObject * pyitem = rj_msgs__msg__pivot_motion_command__convert_to_py(item);
      if (!pyitem) {
        Py_DECREF(field);
        return NULL;
      }
      int rc = PyList_SetItem(field, i, pyitem);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "pivot_command", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // settle_command
    PyObject * field = NULL;
    size_t size = ros_message->settle_command.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    rj_msgs__msg__SettleMotionCommand * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->settle_command.data[i]);
      PyObject * pyitem = rj_msgs__msg__settle_motion_command__convert_to_py(item);
      if (!pyitem) {
        Py_DECREF(field);
        return NULL;
      }
      int rc = PyList_SetItem(field, i, pyitem);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "settle_command", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // collect_command
    PyObject * field = NULL;
    size_t size = ros_message->collect_command.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    rj_msgs__msg__CollectMotionCommand * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->collect_command.data[i]);
      PyObject * pyitem = rj_msgs__msg__collect_motion_command__convert_to_py(item);
      if (!pyitem) {
        Py_DECREF(field);
        return NULL;
      }
      int rc = PyList_SetItem(field, i, pyitem);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "collect_command", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // line_kick_command
    PyObject * field = NULL;
    size_t size = ros_message->line_kick_command.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    rj_msgs__msg__LineKickMotionCommand * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->line_kick_command.data[i]);
      PyObject * pyitem = rj_msgs__msg__line_kick_motion_command__convert_to_py(item);
      if (!pyitem) {
        Py_DECREF(field);
        return NULL;
      }
      int rc = PyList_SetItem(field, i, pyitem);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "line_kick_command", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // intercept_command
    PyObject * field = NULL;
    size_t size = ros_message->intercept_command.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    rj_msgs__msg__InterceptMotionCommand * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->intercept_command.data[i]);
      PyObject * pyitem = rj_msgs__msg__intercept_motion_command__convert_to_py(item);
      if (!pyitem) {
        Py_DECREF(field);
        return NULL;
      }
      int rc = PyList_SetItem(field, i, pyitem);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "intercept_command", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
