// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from feedback_msg:msg/YoloResults.idl
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
#include "feedback_msg/msg/detail/yolo_results__struct.h"
#include "feedback_msg/msg/detail/yolo_results__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool feedback_msg__msg__yolo_results__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[43];
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
    assert(strncmp("feedback_msg.msg._yolo_results.YoloResults", full_classname_dest, 42) == 0);
  }
  feedback_msg__msg__YoloResults * ros_message = _ros_message;
  {  // dotted_line
    PyObject * field = PyObject_GetAttrString(_pymsg, "dotted_line");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->dotted_line = (Py_True == field);
    Py_DECREF(field);
  }
  {  // signal
    PyObject * field = PyObject_GetAttrString(_pymsg, "signal");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->signal = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // traffic_light
    PyObject * field = PyObject_GetAttrString(_pymsg, "traffic_light");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->traffic_light = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * feedback_msg__msg__yolo_results__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of YoloResults */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("feedback_msg.msg._yolo_results");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "YoloResults");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  feedback_msg__msg__YoloResults * ros_message = (feedback_msg__msg__YoloResults *)raw_ros_message;
  {  // dotted_line
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->dotted_line ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "dotted_line", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // signal
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->signal);
    {
      int rc = PyObject_SetAttrString(_pymessage, "signal", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // traffic_light
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->traffic_light);
    {
      int rc = PyObject_SetAttrString(_pymessage, "traffic_light", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
