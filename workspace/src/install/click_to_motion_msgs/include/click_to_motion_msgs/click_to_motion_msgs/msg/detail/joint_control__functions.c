// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from click_to_motion_msgs:msg/JointControl.idl
// generated code does not contain a copyright notice
#include "click_to_motion_msgs/msg/detail/joint_control__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `joint_positions`
// Member `joint_velocities`
// Member `joint_accelerations`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
click_to_motion_msgs__msg__JointControl__init(click_to_motion_msgs__msg__JointControl * msg)
{
  if (!msg) {
    return false;
  }
  // robot_id
  // joint_positions
  if (!rosidl_runtime_c__double__Sequence__init(&msg->joint_positions, 0)) {
    click_to_motion_msgs__msg__JointControl__fini(msg);
    return false;
  }
  // joint_velocities
  if (!rosidl_runtime_c__double__Sequence__init(&msg->joint_velocities, 0)) {
    click_to_motion_msgs__msg__JointControl__fini(msg);
    return false;
  }
  // joint_accelerations
  if (!rosidl_runtime_c__double__Sequence__init(&msg->joint_accelerations, 0)) {
    click_to_motion_msgs__msg__JointControl__fini(msg);
    return false;
  }
  return true;
}

void
click_to_motion_msgs__msg__JointControl__fini(click_to_motion_msgs__msg__JointControl * msg)
{
  if (!msg) {
    return;
  }
  // robot_id
  // joint_positions
  rosidl_runtime_c__double__Sequence__fini(&msg->joint_positions);
  // joint_velocities
  rosidl_runtime_c__double__Sequence__fini(&msg->joint_velocities);
  // joint_accelerations
  rosidl_runtime_c__double__Sequence__fini(&msg->joint_accelerations);
}

bool
click_to_motion_msgs__msg__JointControl__are_equal(const click_to_motion_msgs__msg__JointControl * lhs, const click_to_motion_msgs__msg__JointControl * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // robot_id
  if (lhs->robot_id != rhs->robot_id) {
    return false;
  }
  // joint_positions
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->joint_positions), &(rhs->joint_positions)))
  {
    return false;
  }
  // joint_velocities
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->joint_velocities), &(rhs->joint_velocities)))
  {
    return false;
  }
  // joint_accelerations
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->joint_accelerations), &(rhs->joint_accelerations)))
  {
    return false;
  }
  return true;
}

bool
click_to_motion_msgs__msg__JointControl__copy(
  const click_to_motion_msgs__msg__JointControl * input,
  click_to_motion_msgs__msg__JointControl * output)
{
  if (!input || !output) {
    return false;
  }
  // robot_id
  output->robot_id = input->robot_id;
  // joint_positions
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->joint_positions), &(output->joint_positions)))
  {
    return false;
  }
  // joint_velocities
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->joint_velocities), &(output->joint_velocities)))
  {
    return false;
  }
  // joint_accelerations
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->joint_accelerations), &(output->joint_accelerations)))
  {
    return false;
  }
  return true;
}

click_to_motion_msgs__msg__JointControl *
click_to_motion_msgs__msg__JointControl__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  click_to_motion_msgs__msg__JointControl * msg = (click_to_motion_msgs__msg__JointControl *)allocator.allocate(sizeof(click_to_motion_msgs__msg__JointControl), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(click_to_motion_msgs__msg__JointControl));
  bool success = click_to_motion_msgs__msg__JointControl__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
click_to_motion_msgs__msg__JointControl__destroy(click_to_motion_msgs__msg__JointControl * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    click_to_motion_msgs__msg__JointControl__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
click_to_motion_msgs__msg__JointControl__Sequence__init(click_to_motion_msgs__msg__JointControl__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  click_to_motion_msgs__msg__JointControl * data = NULL;

  if (size) {
    data = (click_to_motion_msgs__msg__JointControl *)allocator.zero_allocate(size, sizeof(click_to_motion_msgs__msg__JointControl), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = click_to_motion_msgs__msg__JointControl__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        click_to_motion_msgs__msg__JointControl__fini(&data[i - 1]);
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
click_to_motion_msgs__msg__JointControl__Sequence__fini(click_to_motion_msgs__msg__JointControl__Sequence * array)
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
      click_to_motion_msgs__msg__JointControl__fini(&array->data[i]);
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

click_to_motion_msgs__msg__JointControl__Sequence *
click_to_motion_msgs__msg__JointControl__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  click_to_motion_msgs__msg__JointControl__Sequence * array = (click_to_motion_msgs__msg__JointControl__Sequence *)allocator.allocate(sizeof(click_to_motion_msgs__msg__JointControl__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = click_to_motion_msgs__msg__JointControl__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
click_to_motion_msgs__msg__JointControl__Sequence__destroy(click_to_motion_msgs__msg__JointControl__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    click_to_motion_msgs__msg__JointControl__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
click_to_motion_msgs__msg__JointControl__Sequence__are_equal(const click_to_motion_msgs__msg__JointControl__Sequence * lhs, const click_to_motion_msgs__msg__JointControl__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!click_to_motion_msgs__msg__JointControl__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
click_to_motion_msgs__msg__JointControl__Sequence__copy(
  const click_to_motion_msgs__msg__JointControl__Sequence * input,
  click_to_motion_msgs__msg__JointControl__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(click_to_motion_msgs__msg__JointControl);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    click_to_motion_msgs__msg__JointControl * data =
      (click_to_motion_msgs__msg__JointControl *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!click_to_motion_msgs__msg__JointControl__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          click_to_motion_msgs__msg__JointControl__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!click_to_motion_msgs__msg__JointControl__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
