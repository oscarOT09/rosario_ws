// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from action_msg:msg/YoloAction.idl
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
#include "action_msg/msg/detail/yolo_action__struct.h"
#include "action_msg/msg/detail/yolo_action__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool action_msg__msg__yolo_action__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[39];
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
    assert(strncmp("action_msg.msg._yolo_action.YoloAction", full_classname_dest, 38) == 0);
  }
  action_msg__msg__YoloAction * ros_message = _ros_message;
  {  // rojo
    PyObject * field = PyObject_GetAttrString(_pymsg, "rojo");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->rojo = (Py_True == field);
    Py_DECREF(field);
  }
  {  // amarillo
    PyObject * field = PyObject_GetAttrString(_pymsg, "amarillo");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->amarillo = (Py_True == field);
    Py_DECREF(field);
  }
  {  // verde
    PyObject * field = PyObject_GetAttrString(_pymsg, "verde");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->verde = (Py_True == field);
    Py_DECREF(field);
  }
  {  // adelante
    PyObject * field = PyObject_GetAttrString(_pymsg, "adelante");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->adelante = (Py_True == field);
    Py_DECREF(field);
  }
  {  // girar_l
    PyObject * field = PyObject_GetAttrString(_pymsg, "girar_l");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->girar_l = (Py_True == field);
    Py_DECREF(field);
  }
  {  // girar_r
    PyObject * field = PyObject_GetAttrString(_pymsg, "girar_r");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->girar_r = (Py_True == field);
    Py_DECREF(field);
  }
  {  // trabajo
    PyObject * field = PyObject_GetAttrString(_pymsg, "trabajo");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->trabajo = (Py_True == field);
    Py_DECREF(field);
  }
  {  // ceder
    PyObject * field = PyObject_GetAttrString(_pymsg, "ceder");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->ceder = (Py_True == field);
    Py_DECREF(field);
  }
  {  // alto
    PyObject * field = PyObject_GetAttrString(_pymsg, "alto");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->alto = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * action_msg__msg__yolo_action__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of YoloAction */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("action_msg.msg._yolo_action");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "YoloAction");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  action_msg__msg__YoloAction * ros_message = (action_msg__msg__YoloAction *)raw_ros_message;
  {  // rojo
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->rojo ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rojo", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // amarillo
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->amarillo ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "amarillo", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // verde
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->verde ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "verde", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // adelante
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->adelante ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "adelante", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // girar_l
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->girar_l ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "girar_l", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // girar_r
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->girar_r ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "girar_r", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // trabajo
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->trabajo ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "trabajo", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ceder
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->ceder ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ceder", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // alto
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->alto ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "alto", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
