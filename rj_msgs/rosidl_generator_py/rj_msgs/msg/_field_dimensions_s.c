// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from rj_msgs:msg/FieldDimensions.idl
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
#include "rj_msgs/msg/detail/field_dimensions__struct.h"
#include "rj_msgs/msg/detail/field_dimensions__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool rj_msgs__msg__field_dimensions__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[46];
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
    assert(strncmp("rj_msgs.msg._field_dimensions.FieldDimensions", full_classname_dest, 45) == 0);
  }
  rj_msgs__msg__FieldDimensions * ros_message = _ros_message;
  {  // length
    PyObject * field = PyObject_GetAttrString(_pymsg, "length");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->length = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // width
    PyObject * field = PyObject_GetAttrString(_pymsg, "width");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->width = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // border
    PyObject * field = PyObject_GetAttrString(_pymsg, "border");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->border = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // line_width
    PyObject * field = PyObject_GetAttrString(_pymsg, "line_width");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->line_width = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // goal_width
    PyObject * field = PyObject_GetAttrString(_pymsg, "goal_width");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->goal_width = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // goal_depth
    PyObject * field = PyObject_GetAttrString(_pymsg, "goal_depth");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->goal_depth = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // goal_height
    PyObject * field = PyObject_GetAttrString(_pymsg, "goal_height");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->goal_height = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // penalty_short_dist
    PyObject * field = PyObject_GetAttrString(_pymsg, "penalty_short_dist");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->penalty_short_dist = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // penalty_long_dist
    PyObject * field = PyObject_GetAttrString(_pymsg, "penalty_long_dist");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->penalty_long_dist = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // center_radius
    PyObject * field = PyObject_GetAttrString(_pymsg, "center_radius");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->center_radius = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // center_diameter
    PyObject * field = PyObject_GetAttrString(_pymsg, "center_diameter");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->center_diameter = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // goal_flat
    PyObject * field = PyObject_GetAttrString(_pymsg, "goal_flat");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->goal_flat = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // floor_length
    PyObject * field = PyObject_GetAttrString(_pymsg, "floor_length");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->floor_length = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // floor_width
    PyObject * field = PyObject_GetAttrString(_pymsg, "floor_width");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->floor_width = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * rj_msgs__msg__field_dimensions__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of FieldDimensions */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("rj_msgs.msg._field_dimensions");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "FieldDimensions");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  rj_msgs__msg__FieldDimensions * ros_message = (rj_msgs__msg__FieldDimensions *)raw_ros_message;
  {  // length
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->length);
    {
      int rc = PyObject_SetAttrString(_pymessage, "length", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // width
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->width);
    {
      int rc = PyObject_SetAttrString(_pymessage, "width", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // border
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->border);
    {
      int rc = PyObject_SetAttrString(_pymessage, "border", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // line_width
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->line_width);
    {
      int rc = PyObject_SetAttrString(_pymessage, "line_width", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // goal_width
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->goal_width);
    {
      int rc = PyObject_SetAttrString(_pymessage, "goal_width", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // goal_depth
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->goal_depth);
    {
      int rc = PyObject_SetAttrString(_pymessage, "goal_depth", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // goal_height
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->goal_height);
    {
      int rc = PyObject_SetAttrString(_pymessage, "goal_height", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // penalty_short_dist
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->penalty_short_dist);
    {
      int rc = PyObject_SetAttrString(_pymessage, "penalty_short_dist", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // penalty_long_dist
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->penalty_long_dist);
    {
      int rc = PyObject_SetAttrString(_pymessage, "penalty_long_dist", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // center_radius
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->center_radius);
    {
      int rc = PyObject_SetAttrString(_pymessage, "center_radius", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // center_diameter
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->center_diameter);
    {
      int rc = PyObject_SetAttrString(_pymessage, "center_diameter", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // goal_flat
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->goal_flat);
    {
      int rc = PyObject_SetAttrString(_pymessage, "goal_flat", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // floor_length
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->floor_length);
    {
      int rc = PyObject_SetAttrString(_pymessage, "floor_length", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // floor_width
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->floor_width);
    {
      int rc = PyObject_SetAttrString(_pymessage, "floor_width", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
