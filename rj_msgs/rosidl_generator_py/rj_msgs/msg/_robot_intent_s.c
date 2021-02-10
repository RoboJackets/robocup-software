// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from rj_msgs:msg/RobotIntent.idl
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
#include "rj_msgs/msg/detail/robot_intent__struct.h"
#include "rj_msgs/msg/detail/robot_intent__functions.h"

bool rj_msgs__msg__motion_command__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * rj_msgs__msg__motion_command__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool rj_geometry_msgs__msg__shape_set__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * rj_geometry_msgs__msg__shape_set__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool rj_msgs__msg__robot_intent__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[38];
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
    assert(strncmp("rj_msgs.msg._robot_intent.RobotIntent", full_classname_dest, 37) == 0);
  }
  rj_msgs__msg__RobotIntent * ros_message = _ros_message;
  {  // motion_command
    PyObject * field = PyObject_GetAttrString(_pymsg, "motion_command");
    if (!field) {
      return false;
    }
    if (!rj_msgs__msg__motion_command__convert_from_py(field, &ros_message->motion_command)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // local_obstacles
    PyObject * field = PyObject_GetAttrString(_pymsg, "local_obstacles");
    if (!field) {
      return false;
    }
    if (!rj_geometry_msgs__msg__shape_set__convert_from_py(field, &ros_message->local_obstacles)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // shoot_mode
    PyObject * field = PyObject_GetAttrString(_pymsg, "shoot_mode");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->shoot_mode = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // trigger_mode
    PyObject * field = PyObject_GetAttrString(_pymsg, "trigger_mode");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->trigger_mode = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // kick_speed
    PyObject * field = PyObject_GetAttrString(_pymsg, "kick_speed");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->kick_speed = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // dribbler_speed
    PyObject * field = PyObject_GetAttrString(_pymsg, "dribbler_speed");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->dribbler_speed = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // is_active
    PyObject * field = PyObject_GetAttrString(_pymsg, "is_active");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->is_active = (Py_True == field);
    Py_DECREF(field);
  }
  {  // priority
    PyObject * field = PyObject_GetAttrString(_pymsg, "priority");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->priority = (int8_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * rj_msgs__msg__robot_intent__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of RobotIntent */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("rj_msgs.msg._robot_intent");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "RobotIntent");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  rj_msgs__msg__RobotIntent * ros_message = (rj_msgs__msg__RobotIntent *)raw_ros_message;
  {  // motion_command
    PyObject * field = NULL;
    field = rj_msgs__msg__motion_command__convert_to_py(&ros_message->motion_command);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "motion_command", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // local_obstacles
    PyObject * field = NULL;
    field = rj_geometry_msgs__msg__shape_set__convert_to_py(&ros_message->local_obstacles);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "local_obstacles", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // shoot_mode
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->shoot_mode);
    {
      int rc = PyObject_SetAttrString(_pymessage, "shoot_mode", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // trigger_mode
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->trigger_mode);
    {
      int rc = PyObject_SetAttrString(_pymessage, "trigger_mode", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kick_speed
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->kick_speed);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kick_speed", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // dribbler_speed
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->dribbler_speed);
    {
      int rc = PyObject_SetAttrString(_pymessage, "dribbler_speed", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // is_active
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->is_active ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "is_active", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // priority
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->priority);
    {
      int rc = PyObject_SetAttrString(_pymessage, "priority", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
