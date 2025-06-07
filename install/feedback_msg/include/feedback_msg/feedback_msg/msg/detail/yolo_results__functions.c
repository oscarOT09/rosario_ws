// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from feedback_msg:msg/YoloResults.idl
// generated code does not contain a copyright notice
#include "feedback_msg/msg/detail/yolo_results__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
feedback_msg__msg__YoloResults__init(feedback_msg__msg__YoloResults * msg)
{
  if (!msg) {
    return false;
  }
  // dotted_line
  // signal
  // traffic_light
  return true;
}

void
feedback_msg__msg__YoloResults__fini(feedback_msg__msg__YoloResults * msg)
{
  if (!msg) {
    return;
  }
  // dotted_line
  // signal
  // traffic_light
}

bool
feedback_msg__msg__YoloResults__are_equal(const feedback_msg__msg__YoloResults * lhs, const feedback_msg__msg__YoloResults * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // dotted_line
  if (lhs->dotted_line != rhs->dotted_line) {
    return false;
  }
  // signal
  if (lhs->signal != rhs->signal) {
    return false;
  }
  // traffic_light
  if (lhs->traffic_light != rhs->traffic_light) {
    return false;
  }
  return true;
}

bool
feedback_msg__msg__YoloResults__copy(
  const feedback_msg__msg__YoloResults * input,
  feedback_msg__msg__YoloResults * output)
{
  if (!input || !output) {
    return false;
  }
  // dotted_line
  output->dotted_line = input->dotted_line;
  // signal
  output->signal = input->signal;
  // traffic_light
  output->traffic_light = input->traffic_light;
  return true;
}

feedback_msg__msg__YoloResults *
feedback_msg__msg__YoloResults__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  feedback_msg__msg__YoloResults * msg = (feedback_msg__msg__YoloResults *)allocator.allocate(sizeof(feedback_msg__msg__YoloResults), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(feedback_msg__msg__YoloResults));
  bool success = feedback_msg__msg__YoloResults__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
feedback_msg__msg__YoloResults__destroy(feedback_msg__msg__YoloResults * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    feedback_msg__msg__YoloResults__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
feedback_msg__msg__YoloResults__Sequence__init(feedback_msg__msg__YoloResults__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  feedback_msg__msg__YoloResults * data = NULL;

  if (size) {
    data = (feedback_msg__msg__YoloResults *)allocator.zero_allocate(size, sizeof(feedback_msg__msg__YoloResults), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = feedback_msg__msg__YoloResults__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        feedback_msg__msg__YoloResults__fini(&data[i - 1]);
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
feedback_msg__msg__YoloResults__Sequence__fini(feedback_msg__msg__YoloResults__Sequence * array)
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
      feedback_msg__msg__YoloResults__fini(&array->data[i]);
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

feedback_msg__msg__YoloResults__Sequence *
feedback_msg__msg__YoloResults__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  feedback_msg__msg__YoloResults__Sequence * array = (feedback_msg__msg__YoloResults__Sequence *)allocator.allocate(sizeof(feedback_msg__msg__YoloResults__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = feedback_msg__msg__YoloResults__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
feedback_msg__msg__YoloResults__Sequence__destroy(feedback_msg__msg__YoloResults__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    feedback_msg__msg__YoloResults__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
feedback_msg__msg__YoloResults__Sequence__are_equal(const feedback_msg__msg__YoloResults__Sequence * lhs, const feedback_msg__msg__YoloResults__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!feedback_msg__msg__YoloResults__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
feedback_msg__msg__YoloResults__Sequence__copy(
  const feedback_msg__msg__YoloResults__Sequence * input,
  feedback_msg__msg__YoloResults__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(feedback_msg__msg__YoloResults);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    feedback_msg__msg__YoloResults * data =
      (feedback_msg__msg__YoloResults *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!feedback_msg__msg__YoloResults__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          feedback_msg__msg__YoloResults__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!feedback_msg__msg__YoloResults__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
