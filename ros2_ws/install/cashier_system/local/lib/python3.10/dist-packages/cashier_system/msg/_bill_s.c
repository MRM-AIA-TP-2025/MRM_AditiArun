// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from cashier_system:msg/Bill.idl
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
#include "cashier_system/msg/detail/bill__struct.h"
#include "cashier_system/msg/detail/bill__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool builtin_interfaces__msg__time__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * builtin_interfaces__msg__time__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool cashier_system__msg__bill__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[30];
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
    assert(strncmp("cashier_system.msg._bill.Bill", full_classname_dest, 29) == 0);
  }
  cashier_system__msg__Bill * ros_message = _ros_message;
  {  // bill_number
    PyObject * field = PyObject_GetAttrString(_pymsg, "bill_number");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->bill_number = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // timestamp
    PyObject * field = PyObject_GetAttrString(_pymsg, "timestamp");
    if (!field) {
      return false;
    }
    if (!builtin_interfaces__msg__time__convert_from_py(field, &ros_message->timestamp)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // quantity
    PyObject * field = PyObject_GetAttrString(_pymsg, "quantity");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->quantity = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // price
    PyObject * field = PyObject_GetAttrString(_pymsg, "price");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->price = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // total
    PyObject * field = PyObject_GetAttrString(_pymsg, "total");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->total = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * cashier_system__msg__bill__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Bill */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("cashier_system.msg._bill");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Bill");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  cashier_system__msg__Bill * ros_message = (cashier_system__msg__Bill *)raw_ros_message;
  {  // bill_number
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->bill_number);
    {
      int rc = PyObject_SetAttrString(_pymessage, "bill_number", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // timestamp
    PyObject * field = NULL;
    field = builtin_interfaces__msg__time__convert_to_py(&ros_message->timestamp);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "timestamp", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // quantity
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->quantity);
    {
      int rc = PyObject_SetAttrString(_pymessage, "quantity", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // price
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->price);
    {
      int rc = PyObject_SetAttrString(_pymessage, "price", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // total
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->total);
    {
      int rc = PyObject_SetAttrString(_pymessage, "total", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
