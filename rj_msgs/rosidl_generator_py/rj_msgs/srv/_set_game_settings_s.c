// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from rj_msgs:srv/SetGameSettings.idl
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
#include "rj_msgs/srv/detail/set_game_settings__struct.h"
#include "rj_msgs/srv/detail/set_game_settings__functions.h"

bool rj_msgs__msg__game_settings__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * rj_msgs__msg__game_settings__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool rj_msgs__srv__set_game_settings__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[55];
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
    assert(strncmp("rj_msgs.srv._set_game_settings.SetGameSettings_Request", full_classname_dest, 54) == 0);
  }
  rj_msgs__srv__SetGameSettings_Request * ros_message = _ros_message;
  {  // game_settings
    PyObject * field = PyObject_GetAttrString(_pymsg, "game_settings");
    if (!field) {
      return false;
    }
    if (!rj_msgs__msg__game_settings__convert_from_py(field, &ros_message->game_settings)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * rj_msgs__srv__set_game_settings__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of SetGameSettings_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("rj_msgs.srv._set_game_settings");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "SetGameSettings_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  rj_msgs__srv__SetGameSettings_Request * ros_message = (rj_msgs__srv__SetGameSettings_Request *)raw_ros_message;
  {  // game_settings
    PyObject * field = NULL;
    field = rj_msgs__msg__game_settings__convert_to_py(&ros_message->game_settings);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "game_settings", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
// already included above
// #include <Python.h>
// already included above
// #include <stdbool.h>
// already included above
// #include "numpy/ndarrayobject.h"
// already included above
// #include "rosidl_runtime_c/visibility_control.h"
// already included above
// #include "rj_msgs/srv/detail/set_game_settings__struct.h"
// already included above
// #include "rj_msgs/srv/detail/set_game_settings__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool rj_msgs__srv__set_game_settings__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[56];
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
    assert(strncmp("rj_msgs.srv._set_game_settings.SetGameSettings_Response", full_classname_dest, 55) == 0);
  }
  rj_msgs__srv__SetGameSettings_Response * ros_message = _ros_message;
  ros_message->structure_needs_at_least_one_member = 0;

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * rj_msgs__srv__set_game_settings__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of SetGameSettings_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("rj_msgs.srv._set_game_settings");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "SetGameSettings_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  (void)raw_ros_message;

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
