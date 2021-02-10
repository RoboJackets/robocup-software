// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from rj_msgs:msg/WorldState.idl
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
#include "rj_msgs/msg/detail/world_state__struct.h"
#include "rj_msgs/msg/detail/world_state__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

// Nested array functions includes
#include "rj_msgs/msg/detail/robot_state__functions.h"
// end nested array functions include
ROSIDL_GENERATOR_C_IMPORT
bool builtin_interfaces__msg__time__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * builtin_interfaces__msg__time__convert_to_py(void * raw_ros_message);
bool rj_msgs__msg__robot_state__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * rj_msgs__msg__robot_state__convert_to_py(void * raw_ros_message);
bool rj_msgs__msg__robot_state__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * rj_msgs__msg__robot_state__convert_to_py(void * raw_ros_message);
bool rj_msgs__msg__ball_state__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * rj_msgs__msg__ball_state__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool rj_msgs__msg__world_state__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[36];
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
    assert(strncmp("rj_msgs.msg._world_state.WorldState", full_classname_dest, 35) == 0);
  }
  rj_msgs__msg__WorldState * ros_message = _ros_message;
  {  // last_update_time
    PyObject * field = PyObject_GetAttrString(_pymsg, "last_update_time");
    if (!field) {
      return false;
    }
    if (!builtin_interfaces__msg__time__convert_from_py(field, &ros_message->last_update_time)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // their_robots
    PyObject * field = PyObject_GetAttrString(_pymsg, "their_robots");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'their_robots'");
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
    if (!rj_msgs__msg__RobotState__Sequence__init(&(ros_message->their_robots), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create rj_msgs__msg__RobotState__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    rj_msgs__msg__RobotState * dest = ros_message->their_robots.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!rj_msgs__msg__robot_state__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }
  {  // our_robots
    PyObject * field = PyObject_GetAttrString(_pymsg, "our_robots");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'our_robots'");
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
    if (!rj_msgs__msg__RobotState__Sequence__init(&(ros_message->our_robots), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create rj_msgs__msg__RobotState__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    rj_msgs__msg__RobotState * dest = ros_message->our_robots.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!rj_msgs__msg__robot_state__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }
  {  // ball
    PyObject * field = PyObject_GetAttrString(_pymsg, "ball");
    if (!field) {
      return false;
    }
    if (!rj_msgs__msg__ball_state__convert_from_py(field, &ros_message->ball)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * rj_msgs__msg__world_state__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of WorldState */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("rj_msgs.msg._world_state");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "WorldState");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  rj_msgs__msg__WorldState * ros_message = (rj_msgs__msg__WorldState *)raw_ros_message;
  {  // last_update_time
    PyObject * field = NULL;
    field = builtin_interfaces__msg__time__convert_to_py(&ros_message->last_update_time);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "last_update_time", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // their_robots
    PyObject * field = NULL;
    size_t size = ros_message->their_robots.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    rj_msgs__msg__RobotState * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->their_robots.data[i]);
      PyObject * pyitem = rj_msgs__msg__robot_state__convert_to_py(item);
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
      int rc = PyObject_SetAttrString(_pymessage, "their_robots", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // our_robots
    PyObject * field = NULL;
    size_t size = ros_message->our_robots.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    rj_msgs__msg__RobotState * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->our_robots.data[i]);
      PyObject * pyitem = rj_msgs__msg__robot_state__convert_to_py(item);
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
      int rc = PyObject_SetAttrString(_pymessage, "our_robots", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ball
    PyObject * field = NULL;
    field = rj_msgs__msg__ball_state__convert_to_py(&ros_message->ball);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "ball", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
