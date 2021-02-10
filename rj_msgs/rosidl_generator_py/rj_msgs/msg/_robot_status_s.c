// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from rj_msgs:msg/RobotStatus.idl
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
#include "rj_msgs/msg/detail/robot_status__struct.h"
#include "rj_msgs/msg/detail/robot_status__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool rj_msgs__msg__robot_status__convert_from_py(PyObject * _pymsg, void * _ros_message)
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
    assert(strncmp("rj_msgs.msg._robot_status.RobotStatus", full_classname_dest, 37) == 0);
  }
  rj_msgs__msg__RobotStatus * ros_message = _ros_message;
  {  // robot_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "robot_id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->robot_id = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // battery_voltage
    PyObject * field = PyObject_GetAttrString(_pymsg, "battery_voltage");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->battery_voltage = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // motor_errors
    PyObject * field = PyObject_GetAttrString(_pymsg, "motor_errors");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'motor_errors'");
    if (!seq_field) {
      Py_DECREF(field);
      return false;
    }
    Py_ssize_t size = 5;
    bool * dest = ros_message->motor_errors;
    for (Py_ssize_t i = 0; i < size; ++i) {
      PyObject * item = PySequence_Fast_GET_ITEM(seq_field, i);
      if (!item) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
      assert(PyBool_Check(item));
      bool tmp = (item == Py_True);
      memcpy(&dest[i], &tmp, sizeof(bool));
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }
  {  // has_ball_sense
    PyObject * field = PyObject_GetAttrString(_pymsg, "has_ball_sense");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->has_ball_sense = (Py_True == field);
    Py_DECREF(field);
  }
  {  // kicker_charged
    PyObject * field = PyObject_GetAttrString(_pymsg, "kicker_charged");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->kicker_charged = (Py_True == field);
    Py_DECREF(field);
  }
  {  // kicker_healthy
    PyObject * field = PyObject_GetAttrString(_pymsg, "kicker_healthy");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->kicker_healthy = (Py_True == field);
    Py_DECREF(field);
  }
  {  // fpga_error
    PyObject * field = PyObject_GetAttrString(_pymsg, "fpga_error");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->fpga_error = (Py_True == field);
    Py_DECREF(field);
  }
  {  // encoder_deltas
    PyObject * field = PyObject_GetAttrString(_pymsg, "encoder_deltas");
    if (!field) {
      return false;
    }
    // TODO(dirk-thomas) use a better way to check the type before casting
    assert(field->ob_type != NULL);
    assert(field->ob_type->tp_name != NULL);
    assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
    PyArrayObject * seq_field = (PyArrayObject *)field;
    Py_INCREF(seq_field);
    assert(PyArray_NDIM(seq_field) == 1);
    assert(PyArray_TYPE(seq_field) == NPY_INT16);
    Py_ssize_t size = 4;
    int16_t * dest = ros_message->encoder_deltas;
    for (Py_ssize_t i = 0; i < size; ++i) {
      int16_t tmp = *(npy_int16 *)PyArray_GETPTR1(seq_field, i);
      memcpy(&dest[i], &tmp, sizeof(int16_t));
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * rj_msgs__msg__robot_status__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of RobotStatus */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("rj_msgs.msg._robot_status");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "RobotStatus");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  rj_msgs__msg__RobotStatus * ros_message = (rj_msgs__msg__RobotStatus *)raw_ros_message;
  {  // robot_id
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->robot_id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "robot_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // battery_voltage
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->battery_voltage);
    {
      int rc = PyObject_SetAttrString(_pymessage, "battery_voltage", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // motor_errors
    PyObject * field = NULL;
    size_t size = 5;
    bool * src = ros_message->motor_errors;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    for (size_t i = 0; i < size; ++i) {
      int rc = PyList_SetItem(field, i, PyBool_FromLong(src[i] ? 1 : 0));
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "motor_errors", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // has_ball_sense
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->has_ball_sense ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "has_ball_sense", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kicker_charged
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->kicker_charged ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kicker_charged", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kicker_healthy
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->kicker_healthy ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kicker_healthy", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fpga_error
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->fpga_error ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fpga_error", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // encoder_deltas
    PyObject * field = NULL;
    field = PyObject_GetAttrString(_pymessage, "encoder_deltas");
    if (!field) {
      return NULL;
    }
    assert(field->ob_type != NULL);
    assert(field->ob_type->tp_name != NULL);
    assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
    PyArrayObject * seq_field = (PyArrayObject *)field;
    assert(PyArray_NDIM(seq_field) == 1);
    assert(PyArray_TYPE(seq_field) == NPY_INT16);
    assert(sizeof(npy_int16) == sizeof(int16_t));
    npy_int16 * dst = (npy_int16 *)PyArray_GETPTR1(seq_field, 0);
    int16_t * src = &(ros_message->encoder_deltas[0]);
    memcpy(dst, src, 4 * sizeof(int16_t));
    Py_DECREF(field);
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
