// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from click_to_motion_msgs:msg/WorkspacePosition.idl
// generated code does not contain a copyright notice
#include "click_to_motion_msgs/msg/detail/workspace_position__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
click_to_motion_msgs__msg__WorkspacePosition__init(click_to_motion_msgs__msg__WorkspacePosition * msg)
{
  if (!msg) {
    return false;
  }
  // robot_id
  // x
  // y
  // z
  // roll
  // pitch
  // yaw
  return true;
}

void
click_to_motion_msgs__msg__WorkspacePosition__fini(click_to_motion_msgs__msg__WorkspacePosition * msg)
{
  if (!msg) {
    return;
  }
  // robot_id
  // x
  // y
  // z
  // roll
  // pitch
  // yaw
}

bool
click_to_motion_msgs__msg__WorkspacePosition__are_equal(const click_to_motion_msgs__msg__WorkspacePosition * lhs, const click_to_motion_msgs__msg__WorkspacePosition * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // robot_id
  if (lhs->robot_id != rhs->robot_id) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  // z
  if (lhs->z != rhs->z) {
    return false;
  }
  // roll
  if (lhs->roll != rhs->roll) {
    return false;
  }
  // pitch
  if (lhs->pitch != rhs->pitch) {
    return false;
  }
  // yaw
  if (lhs->yaw != rhs->yaw) {
    return false;
  }
  return true;
}

bool
click_to_motion_msgs__msg__WorkspacePosition__copy(
  const click_to_motion_msgs__msg__WorkspacePosition * input,
  click_to_motion_msgs__msg__WorkspacePosition * output)
{
  if (!input || !output) {
    return false;
  }
  // robot_id
  output->robot_id = input->robot_id;
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // z
  output->z = input->z;
  // roll
  output->roll = input->roll;
  // pitch
  output->pitch = input->pitch;
  // yaw
  output->yaw = input->yaw;
  return true;
}

click_to_motion_msgs__msg__WorkspacePosition *
click_to_motion_msgs__msg__WorkspacePosition__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  click_to_motion_msgs__msg__WorkspacePosition * msg = (click_to_motion_msgs__msg__WorkspacePosition *)allocator.allocate(sizeof(click_to_motion_msgs__msg__WorkspacePosition), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(click_to_motion_msgs__msg__WorkspacePosition));
  bool success = click_to_motion_msgs__msg__WorkspacePosition__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
click_to_motion_msgs__msg__WorkspacePosition__destroy(click_to_motion_msgs__msg__WorkspacePosition * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    click_to_motion_msgs__msg__WorkspacePosition__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
click_to_motion_msgs__msg__WorkspacePosition__Sequence__init(click_to_motion_msgs__msg__WorkspacePosition__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  click_to_motion_msgs__msg__WorkspacePosition * data = NULL;

  if (size) {
    data = (click_to_motion_msgs__msg__WorkspacePosition *)allocator.zero_allocate(size, sizeof(click_to_motion_msgs__msg__WorkspacePosition), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = click_to_motion_msgs__msg__WorkspacePosition__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        click_to_motion_msgs__msg__WorkspacePosition__fini(&data[i - 1]);
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
click_to_motion_msgs__msg__WorkspacePosition__Sequence__fini(click_to_motion_msgs__msg__WorkspacePosition__Sequence * array)
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
      click_to_motion_msgs__msg__WorkspacePosition__fini(&array->data[i]);
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

click_to_motion_msgs__msg__WorkspacePosition__Sequence *
click_to_motion_msgs__msg__WorkspacePosition__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  click_to_motion_msgs__msg__WorkspacePosition__Sequence * array = (click_to_motion_msgs__msg__WorkspacePosition__Sequence *)allocator.allocate(sizeof(click_to_motion_msgs__msg__WorkspacePosition__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = click_to_motion_msgs__msg__WorkspacePosition__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
click_to_motion_msgs__msg__WorkspacePosition__Sequence__destroy(click_to_motion_msgs__msg__WorkspacePosition__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    click_to_motion_msgs__msg__WorkspacePosition__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
click_to_motion_msgs__msg__WorkspacePosition__Sequence__are_equal(const click_to_motion_msgs__msg__WorkspacePosition__Sequence * lhs, const click_to_motion_msgs__msg__WorkspacePosition__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!click_to_motion_msgs__msg__WorkspacePosition__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
click_to_motion_msgs__msg__WorkspacePosition__Sequence__copy(
  const click_to_motion_msgs__msg__WorkspacePosition__Sequence * input,
  click_to_motion_msgs__msg__WorkspacePosition__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(click_to_motion_msgs__msg__WorkspacePosition);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    click_to_motion_msgs__msg__WorkspacePosition * data =
      (click_to_motion_msgs__msg__WorkspacePosition *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!click_to_motion_msgs__msg__WorkspacePosition__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          click_to_motion_msgs__msg__WorkspacePosition__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!click_to_motion_msgs__msg__WorkspacePosition__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
