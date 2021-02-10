// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from rj_msgs:msg/TeamInfo.idl
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
#include "rj_msgs/msg/detail/team_info__struct.h"
#include "rj_msgs/msg/detail/team_info__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

// Nested array functions includes
#include "builtin_interfaces/msg/detail/duration__functions.h"
// end nested array functions include
ROSIDL_GENERATOR_C_IMPORT
bool builtin_interfaces__msg__duration__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * builtin_interfaces__msg__duration__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool builtin_interfaces__msg__duration__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * builtin_interfaces__msg__duration__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool rj_msgs__msg__team_info__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[32];
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
    assert(strncmp("rj_msgs.msg._team_info.TeamInfo", full_classname_dest, 31) == 0);
  }
  rj_msgs__msg__TeamInfo * ros_message = _ros_message;
  {  // name
    PyObject * field = PyObject_GetAttrString(_pymsg, "name");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->name, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // score
    PyObject * field = PyObject_GetAttrString(_pymsg, "score");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->score = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // num_red_cards
    PyObject * field = PyObject_GetAttrString(_pymsg, "num_red_cards");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->num_red_cards = PyLong_AsUnsignedLongLong(field);
    Py_DECREF(field);
  }
  {  // num_yellow_cards
    PyObject * field = PyObject_GetAttrString(_pymsg, "num_yellow_cards");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->num_yellow_cards = PyLong_AsUnsignedLongLong(field);
    Py_DECREF(field);
  }
  {  // yellow_card_remaining_times
    PyObject * field = PyObject_GetAttrString(_pymsg, "yellow_card_remaining_times");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'yellow_card_remaining_times'");
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
    if (!builtin_interfaces__msg__Duration__Sequence__init(&(ros_message->yellow_card_remaining_times), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create builtin_interfaces__msg__Duration__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    builtin_interfaces__msg__Duration * dest = ros_message->yellow_card_remaining_times.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!builtin_interfaces__msg__duration__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }
  {  // timeouts_left
    PyObject * field = PyObject_GetAttrString(_pymsg, "timeouts_left");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->timeouts_left = PyLong_AsUnsignedLongLong(field);
    Py_DECREF(field);
  }
  {  // remaining_timeout_time
    PyObject * field = PyObject_GetAttrString(_pymsg, "remaining_timeout_time");
    if (!field) {
      return false;
    }
    if (!builtin_interfaces__msg__duration__convert_from_py(field, &ros_message->remaining_timeout_time)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // goalie_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "goalie_id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->goalie_id = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * rj_msgs__msg__team_info__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of TeamInfo */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("rj_msgs.msg._team_info");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "TeamInfo");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  rj_msgs__msg__TeamInfo * ros_message = (rj_msgs__msg__TeamInfo *)raw_ros_message;
  {  // name
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->name.data,
      strlen(ros_message->name.data),
      "strict");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "name", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // score
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->score);
    {
      int rc = PyObject_SetAttrString(_pymessage, "score", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // num_red_cards
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLongLong(ros_message->num_red_cards);
    {
      int rc = PyObject_SetAttrString(_pymessage, "num_red_cards", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // num_yellow_cards
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLongLong(ros_message->num_yellow_cards);
    {
      int rc = PyObject_SetAttrString(_pymessage, "num_yellow_cards", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // yellow_card_remaining_times
    PyObject * field = NULL;
    size_t size = ros_message->yellow_card_remaining_times.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    builtin_interfaces__msg__Duration * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->yellow_card_remaining_times.data[i]);
      PyObject * pyitem = builtin_interfaces__msg__duration__convert_to_py(item);
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
      int rc = PyObject_SetAttrString(_pymessage, "yellow_card_remaining_times", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // timeouts_left
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLongLong(ros_message->timeouts_left);
    {
      int rc = PyObject_SetAttrString(_pymessage, "timeouts_left", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // remaining_timeout_time
    PyObject * field = NULL;
    field = builtin_interfaces__msg__duration__convert_to_py(&ros_message->remaining_timeout_time);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "remaining_timeout_time", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // goalie_id
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->goalie_id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "goalie_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
