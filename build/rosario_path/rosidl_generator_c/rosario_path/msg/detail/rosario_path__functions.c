// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rosario_path:msg/RosarioPath.idl
// generated code does not contain a copyright notice
#include "rosario_path/msg/detail/rosario_path__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
rosario_path__msg__RosarioPath__init(rosario_path__msg__RosarioPath * msg)
{
  if (!msg) {
    return false;
  }
  // error
  // curva
  return true;
}

void
rosario_path__msg__RosarioPath__fini(rosario_path__msg__RosarioPath * msg)
{
  if (!msg) {
    return;
  }
  // error
  // curva
}

bool
rosario_path__msg__RosarioPath__are_equal(const rosario_path__msg__RosarioPath * lhs, const rosario_path__msg__RosarioPath * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // error
  if (lhs->error != rhs->error) {
    return false;
  }
  // curva
  if (lhs->curva != rhs->curva) {
    return false;
  }
  return true;
}

bool
rosario_path__msg__RosarioPath__copy(
  const rosario_path__msg__RosarioPath * input,
  rosario_path__msg__RosarioPath * output)
{
  if (!input || !output) {
    return false;
  }
  // error
  output->error = input->error;
  // curva
  output->curva = input->curva;
  return true;
}

rosario_path__msg__RosarioPath *
rosario_path__msg__RosarioPath__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosario_path__msg__RosarioPath * msg = (rosario_path__msg__RosarioPath *)allocator.allocate(sizeof(rosario_path__msg__RosarioPath), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rosario_path__msg__RosarioPath));
  bool success = rosario_path__msg__RosarioPath__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rosario_path__msg__RosarioPath__destroy(rosario_path__msg__RosarioPath * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rosario_path__msg__RosarioPath__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rosario_path__msg__RosarioPath__Sequence__init(rosario_path__msg__RosarioPath__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosario_path__msg__RosarioPath * data = NULL;

  if (size) {
    data = (rosario_path__msg__RosarioPath *)allocator.zero_allocate(size, sizeof(rosario_path__msg__RosarioPath), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rosario_path__msg__RosarioPath__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rosario_path__msg__RosarioPath__fini(&data[i - 1]);
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
rosario_path__msg__RosarioPath__Sequence__fini(rosario_path__msg__RosarioPath__Sequence * array)
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
      rosario_path__msg__RosarioPath__fini(&array->data[i]);
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

rosario_path__msg__RosarioPath__Sequence *
rosario_path__msg__RosarioPath__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosario_path__msg__RosarioPath__Sequence * array = (rosario_path__msg__RosarioPath__Sequence *)allocator.allocate(sizeof(rosario_path__msg__RosarioPath__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rosario_path__msg__RosarioPath__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rosario_path__msg__RosarioPath__Sequence__destroy(rosario_path__msg__RosarioPath__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rosario_path__msg__RosarioPath__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rosario_path__msg__RosarioPath__Sequence__are_equal(const rosario_path__msg__RosarioPath__Sequence * lhs, const rosario_path__msg__RosarioPath__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rosario_path__msg__RosarioPath__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rosario_path__msg__RosarioPath__Sequence__copy(
  const rosario_path__msg__RosarioPath__Sequence * input,
  rosario_path__msg__RosarioPath__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rosario_path__msg__RosarioPath);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rosario_path__msg__RosarioPath * data =
      (rosario_path__msg__RosarioPath *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rosario_path__msg__RosarioPath__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rosario_path__msg__RosarioPath__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rosario_path__msg__RosarioPath__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
