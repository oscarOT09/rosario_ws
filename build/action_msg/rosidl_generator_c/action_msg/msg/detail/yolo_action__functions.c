// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from action_msg:msg/YoloAction.idl
// generated code does not contain a copyright notice
#include "action_msg/msg/detail/yolo_action__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
action_msg__msg__YoloAction__init(action_msg__msg__YoloAction * msg)
{
  if (!msg) {
    return false;
  }
  // rojo
  // amarillo
  // verde
  // adelante
  // girar_l
  // girar_r
  // trabajo
  // ceder
  // alto
  return true;
}

void
action_msg__msg__YoloAction__fini(action_msg__msg__YoloAction * msg)
{
  if (!msg) {
    return;
  }
  // rojo
  // amarillo
  // verde
  // adelante
  // girar_l
  // girar_r
  // trabajo
  // ceder
  // alto
}

bool
action_msg__msg__YoloAction__are_equal(const action_msg__msg__YoloAction * lhs, const action_msg__msg__YoloAction * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // rojo
  if (lhs->rojo != rhs->rojo) {
    return false;
  }
  // amarillo
  if (lhs->amarillo != rhs->amarillo) {
    return false;
  }
  // verde
  if (lhs->verde != rhs->verde) {
    return false;
  }
  // adelante
  if (lhs->adelante != rhs->adelante) {
    return false;
  }
  // girar_l
  if (lhs->girar_l != rhs->girar_l) {
    return false;
  }
  // girar_r
  if (lhs->girar_r != rhs->girar_r) {
    return false;
  }
  // trabajo
  if (lhs->trabajo != rhs->trabajo) {
    return false;
  }
  // ceder
  if (lhs->ceder != rhs->ceder) {
    return false;
  }
  // alto
  if (lhs->alto != rhs->alto) {
    return false;
  }
  return true;
}

bool
action_msg__msg__YoloAction__copy(
  const action_msg__msg__YoloAction * input,
  action_msg__msg__YoloAction * output)
{
  if (!input || !output) {
    return false;
  }
  // rojo
  output->rojo = input->rojo;
  // amarillo
  output->amarillo = input->amarillo;
  // verde
  output->verde = input->verde;
  // adelante
  output->adelante = input->adelante;
  // girar_l
  output->girar_l = input->girar_l;
  // girar_r
  output->girar_r = input->girar_r;
  // trabajo
  output->trabajo = input->trabajo;
  // ceder
  output->ceder = input->ceder;
  // alto
  output->alto = input->alto;
  return true;
}

action_msg__msg__YoloAction *
action_msg__msg__YoloAction__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  action_msg__msg__YoloAction * msg = (action_msg__msg__YoloAction *)allocator.allocate(sizeof(action_msg__msg__YoloAction), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(action_msg__msg__YoloAction));
  bool success = action_msg__msg__YoloAction__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
action_msg__msg__YoloAction__destroy(action_msg__msg__YoloAction * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    action_msg__msg__YoloAction__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
action_msg__msg__YoloAction__Sequence__init(action_msg__msg__YoloAction__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  action_msg__msg__YoloAction * data = NULL;

  if (size) {
    data = (action_msg__msg__YoloAction *)allocator.zero_allocate(size, sizeof(action_msg__msg__YoloAction), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = action_msg__msg__YoloAction__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        action_msg__msg__YoloAction__fini(&data[i - 1]);
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
action_msg__msg__YoloAction__Sequence__fini(action_msg__msg__YoloAction__Sequence * array)
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
      action_msg__msg__YoloAction__fini(&array->data[i]);
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

action_msg__msg__YoloAction__Sequence *
action_msg__msg__YoloAction__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  action_msg__msg__YoloAction__Sequence * array = (action_msg__msg__YoloAction__Sequence *)allocator.allocate(sizeof(action_msg__msg__YoloAction__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = action_msg__msg__YoloAction__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
action_msg__msg__YoloAction__Sequence__destroy(action_msg__msg__YoloAction__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    action_msg__msg__YoloAction__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
action_msg__msg__YoloAction__Sequence__are_equal(const action_msg__msg__YoloAction__Sequence * lhs, const action_msg__msg__YoloAction__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!action_msg__msg__YoloAction__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
action_msg__msg__YoloAction__Sequence__copy(
  const action_msg__msg__YoloAction__Sequence * input,
  action_msg__msg__YoloAction__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(action_msg__msg__YoloAction);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    action_msg__msg__YoloAction * data =
      (action_msg__msg__YoloAction *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!action_msg__msg__YoloAction__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          action_msg__msg__YoloAction__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!action_msg__msg__YoloAction__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
