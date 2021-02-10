// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from rj_msgs:msg/DetectionFrame.idl
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
#include "rj_msgs/msg/detail/detection_frame__struct.h"
#include "rj_msgs/msg/detail/detection_frame__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

// Nested array functions includes
#include "rj_msgs/msg/detail/detection_ball__functions.h"
#include "rj_msgs/msg/detail/detection_robot__functions.h"
// end nested array functions include
ROSIDL_GENERATOR_C_IMPORT
bool builtin_interfaces__msg__time__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * builtin_interfaces__msg__time__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool builtin_interfaces__msg__time__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * builtin_interfaces__msg__time__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool builtin_interfaces__msg__time__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * builtin_interfaces__msg__time__convert_to_py(void * raw_ros_message);
bool rj_msgs__msg__detection_ball__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * rj_msgs__msg__detection_ball__convert_to_py(void * raw_ros_message);
bool rj_msgs__msg__detection_robot__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * rj_msgs__msg__detection_robot__convert_to_py(void * raw_ros_message);
bool rj_msgs__msg__detection_robot__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * rj_msgs__msg__detection_robot__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool rj_msgs__msg__detection_frame__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[44];
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
    assert(strncmp("rj_msgs.msg._detection_frame.DetectionFrame", full_classname_dest, 43) == 0);
  }
  rj_msgs__msg__DetectionFrame * ros_message = _ros_message;
  {  // frame_number
    PyObject * field = PyObject_GetAttrString(_pymsg, "frame_number");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->frame_number = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // t_capture
    PyObject * field = PyObject_GetAttrString(_pymsg, "t_capture");
    if (!field) {
      return false;
    }
    if (!builtin_interfaces__msg__time__convert_from_py(field, &ros_message->t_capture)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // t_sent
    PyObject * field = PyObject_GetAttrString(_pymsg, "t_sent");
    if (!field) {
      return false;
    }
    if (!builtin_interfaces__msg__time__convert_from_py(field, &ros_message->t_sent)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // t_received
    PyObject * field = PyObject_GetAttrString(_pymsg, "t_received");
    if (!field) {
      return false;
    }
    if (!builtin_interfaces__msg__time__convert_from_py(field, &ros_message->t_received)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // camera_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "camera_id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->camera_id = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // balls
    PyObject * field = PyObject_GetAttrString(_pymsg, "balls");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'balls'");
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
    if (!rj_msgs__msg__DetectionBall__Sequence__init(&(ros_message->balls), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create rj_msgs__msg__DetectionBall__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    rj_msgs__msg__DetectionBall * dest = ros_message->balls.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!rj_msgs__msg__detection_ball__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }
  {  // robots_yellow
    PyObject * field = PyObject_GetAttrString(_pymsg, "robots_yellow");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'robots_yellow'");
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
    if (!rj_msgs__msg__DetectionRobot__Sequence__init(&(ros_message->robots_yellow), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create rj_msgs__msg__DetectionRobot__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    rj_msgs__msg__DetectionRobot * dest = ros_message->robots_yellow.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!rj_msgs__msg__detection_robot__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }
  {  // robots_blue
    PyObject * field = PyObject_GetAttrString(_pymsg, "robots_blue");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'robots_blue'");
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
    if (!rj_msgs__msg__DetectionRobot__Sequence__init(&(ros_message->robots_blue), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create rj_msgs__msg__DetectionRobot__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    rj_msgs__msg__DetectionRobot * dest = ros_message->robots_blue.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!rj_msgs__msg__detection_robot__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
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
PyObject * rj_msgs__msg__detection_frame__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of DetectionFrame */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("rj_msgs.msg._detection_frame");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "DetectionFrame");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  rj_msgs__msg__DetectionFrame * ros_message = (rj_msgs__msg__DetectionFrame *)raw_ros_message;
  {  // frame_number
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->frame_number);
    {
      int rc = PyObject_SetAttrString(_pymessage, "frame_number", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // t_capture
    PyObject * field = NULL;
    field = builtin_interfaces__msg__time__convert_to_py(&ros_message->t_capture);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "t_capture", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // t_sent
    PyObject * field = NULL;
    field = builtin_interfaces__msg__time__convert_to_py(&ros_message->t_sent);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "t_sent", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // t_received
    PyObject * field = NULL;
    field = builtin_interfaces__msg__time__convert_to_py(&ros_message->t_received);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "t_received", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // camera_id
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->camera_id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "camera_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // balls
    PyObject * field = NULL;
    size_t size = ros_message->balls.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    rj_msgs__msg__DetectionBall * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->balls.data[i]);
      PyObject * pyitem = rj_msgs__msg__detection_ball__convert_to_py(item);
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
      int rc = PyObject_SetAttrString(_pymessage, "balls", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // robots_yellow
    PyObject * field = NULL;
    size_t size = ros_message->robots_yellow.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    rj_msgs__msg__DetectionRobot * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->robots_yellow.data[i]);
      PyObject * pyitem = rj_msgs__msg__detection_robot__convert_to_py(item);
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
      int rc = PyObject_SetAttrString(_pymessage, "robots_yellow", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // robots_blue
    PyObject * field = NULL;
    size_t size = ros_message->robots_blue.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    rj_msgs__msg__DetectionRobot * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->robots_blue.data[i]);
      PyObject * pyitem = rj_msgs__msg__detection_robot__convert_to_py(item);
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
      int rc = PyObject_SetAttrString(_pymessage, "robots_blue", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
