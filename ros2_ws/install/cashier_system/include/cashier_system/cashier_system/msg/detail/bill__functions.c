// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from cashier_system:msg/Bill.idl
// generated code does not contain a copyright notice
#include "cashier_system/msg/detail/bill__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `timestamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
cashier_system__msg__Bill__init(cashier_system__msg__Bill * msg)
{
  if (!msg) {
    return false;
  }
  // bill_number
  // timestamp
  if (!builtin_interfaces__msg__Time__init(&msg->timestamp)) {
    cashier_system__msg__Bill__fini(msg);
    return false;
  }
  // quantity
  // price
  // total
  return true;
}

void
cashier_system__msg__Bill__fini(cashier_system__msg__Bill * msg)
{
  if (!msg) {
    return;
  }
  // bill_number
  // timestamp
  builtin_interfaces__msg__Time__fini(&msg->timestamp);
  // quantity
  // price
  // total
}

bool
cashier_system__msg__Bill__are_equal(const cashier_system__msg__Bill * lhs, const cashier_system__msg__Bill * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // bill_number
  if (lhs->bill_number != rhs->bill_number) {
    return false;
  }
  // timestamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->timestamp), &(rhs->timestamp)))
  {
    return false;
  }
  // quantity
  if (lhs->quantity != rhs->quantity) {
    return false;
  }
  // price
  if (lhs->price != rhs->price) {
    return false;
  }
  // total
  if (lhs->total != rhs->total) {
    return false;
  }
  return true;
}

bool
cashier_system__msg__Bill__copy(
  const cashier_system__msg__Bill * input,
  cashier_system__msg__Bill * output)
{
  if (!input || !output) {
    return false;
  }
  // bill_number
  output->bill_number = input->bill_number;
  // timestamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->timestamp), &(output->timestamp)))
  {
    return false;
  }
  // quantity
  output->quantity = input->quantity;
  // price
  output->price = input->price;
  // total
  output->total = input->total;
  return true;
}

cashier_system__msg__Bill *
cashier_system__msg__Bill__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cashier_system__msg__Bill * msg = (cashier_system__msg__Bill *)allocator.allocate(sizeof(cashier_system__msg__Bill), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(cashier_system__msg__Bill));
  bool success = cashier_system__msg__Bill__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
cashier_system__msg__Bill__destroy(cashier_system__msg__Bill * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    cashier_system__msg__Bill__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
cashier_system__msg__Bill__Sequence__init(cashier_system__msg__Bill__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cashier_system__msg__Bill * data = NULL;

  if (size) {
    data = (cashier_system__msg__Bill *)allocator.zero_allocate(size, sizeof(cashier_system__msg__Bill), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = cashier_system__msg__Bill__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        cashier_system__msg__Bill__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
cashier_system__msg__Bill__Sequence__fini(cashier_system__msg__Bill__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      cashier_system__msg__Bill__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

cashier_system__msg__Bill__Sequence *
cashier_system__msg__Bill__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cashier_system__msg__Bill__Sequence * array = (cashier_system__msg__Bill__Sequence *)allocator.allocate(sizeof(cashier_system__msg__Bill__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = cashier_system__msg__Bill__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
cashier_system__msg__Bill__Sequence__destroy(cashier_system__msg__Bill__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    cashier_system__msg__Bill__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
cashier_system__msg__Bill__Sequence__are_equal(const cashier_system__msg__Bill__Sequence * lhs, const cashier_system__msg__Bill__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!cashier_system__msg__Bill__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
cashier_system__msg__Bill__Sequence__copy(
  const cashier_system__msg__Bill__Sequence * input,
  cashier_system__msg__Bill__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(cashier_system__msg__Bill);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    cashier_system__msg__Bill * data =
      (cashier_system__msg__Bill *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!cashier_system__msg__Bill__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          cashier_system__msg__Bill__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!cashier_system__msg__Bill__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
