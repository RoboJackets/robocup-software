// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from rj_msgs:msg/PivotMotionCommand.idl
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
#include "rj_msgs/msg/detail/pivot_motion_command__struct.h"
#include "rj_msgs/msg/detail/pivot_motion_command__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool rj_geometry_msgs__msg__point__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * rj_geometry_msgs__msg__point__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool rj_geometry_msgs__msg__point__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * rj_geometry_msgs__msg__point__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool rj_msgs__msg__pivot_motion_command__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[53];
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
    assert(strncmp("rj_msgs.msg._pivot_motion_command.PivotMotionCommand", full_classname_dest, 52) == 0);
  }
  rj_msgs__msg__PivotMotionCommand * ros_message = _ros_message;
  {  // pivot_point
    PyObject * field = PyObject_GetAttrString(_pymsg, "pivot_point");
    if (!field) {
      return false;
    }
    if (!rj_geometry_msgs__msg__point__convert_from_py(field, &ros_message->pivot_point)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // pivot_target
    PyObject * field = PyObject_GetAttrString(_pymsg, "pivot_target");
    if (!field) {
      return false;
    }
    if (!rj_geometry_msgs__msg__point__convert_from_py(field, &ros_message->pivot_target)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * rj_msgs__msg__pivot_motion_command__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of PivotMotionCommand */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("rj_msgs.msg._pivot_motion_command");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "PivotMotionCommand");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  rj_msgs__msg__PivotMotionCommand * ros_message = (rj_msgs__msg__PivotMotionCommand *)raw_ros_message;
  {  // pivot_point
    PyObject * field = NULL;
    field = rj_geometry_msgs__msg__point__convert_to_py(&ros_message->pivot_point);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "pivot_point", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pivot_target
    PyObject * field = NULL;
    field = rj_geometry_msgs__msg__point__convert_to_py(&ros_message->pivot_target);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "pivot_target", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
