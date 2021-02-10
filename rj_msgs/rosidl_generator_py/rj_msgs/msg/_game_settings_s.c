// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from rj_msgs:msg/GameSettings.idl
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
#include "rj_msgs/msg/detail/game_settings__struct.h"
#include "rj_msgs/msg/detail/game_settings__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool rj_msgs__msg__game_settings__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[40];
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
    assert(strncmp("rj_msgs.msg._game_settings.GameSettings", full_classname_dest, 39) == 0);
  }
  rj_msgs__msg__GameSettings * ros_message = _ros_message;
  {  // simulation
    PyObject * field = PyObject_GetAttrString(_pymsg, "simulation");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->simulation = (Py_True == field);
    Py_DECREF(field);
  }
  {  // request_blue_team
    PyObject * field = PyObject_GetAttrString(_pymsg, "request_blue_team");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->request_blue_team = (Py_True == field);
    Py_DECREF(field);
  }
  {  // request_goalie_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "request_goalie_id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->request_goalie_id = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // defend_plus_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "defend_plus_x");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->defend_plus_x = (Py_True == field);
    Py_DECREF(field);
  }
  {  // use_our_half
    PyObject * field = PyObject_GetAttrString(_pymsg, "use_our_half");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->use_our_half = (Py_True == field);
    Py_DECREF(field);
  }
  {  // use_their_half
    PyObject * field = PyObject_GetAttrString(_pymsg, "use_their_half");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->use_their_half = (Py_True == field);
    Py_DECREF(field);
  }
  {  // paused
    PyObject * field = PyObject_GetAttrString(_pymsg, "paused");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->paused = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * rj_msgs__msg__game_settings__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of GameSettings */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("rj_msgs.msg._game_settings");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "GameSettings");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  rj_msgs__msg__GameSettings * ros_message = (rj_msgs__msg__GameSettings *)raw_ros_message;
  {  // simulation
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->simulation ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "simulation", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // request_blue_team
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->request_blue_team ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "request_blue_team", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // request_goalie_id
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->request_goalie_id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "request_goalie_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // defend_plus_x
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->defend_plus_x ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "defend_plus_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // use_our_half
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->use_our_half ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "use_our_half", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // use_their_half
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->use_their_half ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "use_their_half", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // paused
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->paused ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "paused", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
