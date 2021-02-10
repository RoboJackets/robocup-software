// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from rj_msgs:msg/FieldOrientation.idl
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
#include "rj_msgs/msg/detail/field_orientation__struct.h"
#include "rj_msgs/msg/detail/field_orientation__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool rj_msgs__msg__field_orientation__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[48];
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
    assert(strncmp("rj_msgs.msg._field_orientation.FieldOrientation", full_classname_dest, 47) == 0);
  }
  rj_msgs__msg__FieldOrientation * ros_message = _ros_message;
  {  // defend_plus_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "defend_plus_x");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->defend_plus_x = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * rj_msgs__msg__field_orientation__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of FieldOrientation */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("rj_msgs.msg._field_orientation");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "FieldOrientation");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  rj_msgs__msg__FieldOrientation * ros_message = (rj_msgs__msg__FieldOrientation *)raw_ros_message;
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

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
