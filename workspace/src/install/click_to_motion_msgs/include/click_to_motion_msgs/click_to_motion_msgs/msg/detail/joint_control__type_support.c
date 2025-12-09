// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from click_to_motion_msgs:msg/JointControl.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "click_to_motion_msgs/msg/detail/joint_control__rosidl_typesupport_introspection_c.h"
#include "click_to_motion_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "click_to_motion_msgs/msg/detail/joint_control__functions.h"
#include "click_to_motion_msgs/msg/detail/joint_control__struct.h"


// Include directives for member types
// Member `joint_positions`
// Member `joint_velocities`
// Member `joint_accelerations`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__JointControl_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  click_to_motion_msgs__msg__JointControl__init(message_memory);
}

void click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__JointControl_fini_function(void * message_memory)
{
  click_to_motion_msgs__msg__JointControl__fini(message_memory);
}

size_t click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__size_function__JointControl__joint_positions(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__get_const_function__JointControl__joint_positions(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__get_function__JointControl__joint_positions(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__fetch_function__JointControl__joint_positions(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__get_const_function__JointControl__joint_positions(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__assign_function__JointControl__joint_positions(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__get_function__JointControl__joint_positions(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__resize_function__JointControl__joint_positions(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__size_function__JointControl__joint_velocities(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__get_const_function__JointControl__joint_velocities(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__get_function__JointControl__joint_velocities(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__fetch_function__JointControl__joint_velocities(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__get_const_function__JointControl__joint_velocities(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__assign_function__JointControl__joint_velocities(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__get_function__JointControl__joint_velocities(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__resize_function__JointControl__joint_velocities(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__size_function__JointControl__joint_accelerations(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__get_const_function__JointControl__joint_accelerations(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__get_function__JointControl__joint_accelerations(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__fetch_function__JointControl__joint_accelerations(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__get_const_function__JointControl__joint_accelerations(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__assign_function__JointControl__joint_accelerations(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__get_function__JointControl__joint_accelerations(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__resize_function__JointControl__joint_accelerations(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__JointControl_message_member_array[4] = {
  {
    "robot_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(click_to_motion_msgs__msg__JointControl, robot_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "joint_positions",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(click_to_motion_msgs__msg__JointControl, joint_positions),  // bytes offset in struct
    NULL,  // default value
    click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__size_function__JointControl__joint_positions,  // size() function pointer
    click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__get_const_function__JointControl__joint_positions,  // get_const(index) function pointer
    click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__get_function__JointControl__joint_positions,  // get(index) function pointer
    click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__fetch_function__JointControl__joint_positions,  // fetch(index, &value) function pointer
    click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__assign_function__JointControl__joint_positions,  // assign(index, value) function pointer
    click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__resize_function__JointControl__joint_positions  // resize(index) function pointer
  },
  {
    "joint_velocities",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(click_to_motion_msgs__msg__JointControl, joint_velocities),  // bytes offset in struct
    NULL,  // default value
    click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__size_function__JointControl__joint_velocities,  // size() function pointer
    click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__get_const_function__JointControl__joint_velocities,  // get_const(index) function pointer
    click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__get_function__JointControl__joint_velocities,  // get(index) function pointer
    click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__fetch_function__JointControl__joint_velocities,  // fetch(index, &value) function pointer
    click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__assign_function__JointControl__joint_velocities,  // assign(index, value) function pointer
    click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__resize_function__JointControl__joint_velocities  // resize(index) function pointer
  },
  {
    "joint_accelerations",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(click_to_motion_msgs__msg__JointControl, joint_accelerations),  // bytes offset in struct
    NULL,  // default value
    click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__size_function__JointControl__joint_accelerations,  // size() function pointer
    click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__get_const_function__JointControl__joint_accelerations,  // get_const(index) function pointer
    click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__get_function__JointControl__joint_accelerations,  // get(index) function pointer
    click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__fetch_function__JointControl__joint_accelerations,  // fetch(index, &value) function pointer
    click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__assign_function__JointControl__joint_accelerations,  // assign(index, value) function pointer
    click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__resize_function__JointControl__joint_accelerations  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__JointControl_message_members = {
  "click_to_motion_msgs__msg",  // message namespace
  "JointControl",  // message name
  4,  // number of fields
  sizeof(click_to_motion_msgs__msg__JointControl),
  click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__JointControl_message_member_array,  // message members
  click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__JointControl_init_function,  // function to initialize message memory (memory has to be allocated)
  click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__JointControl_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__JointControl_message_type_support_handle = {
  0,
  &click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__JointControl_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_click_to_motion_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, click_to_motion_msgs, msg, JointControl)() {
  if (!click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__JointControl_message_type_support_handle.typesupport_identifier) {
    click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__JointControl_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &click_to_motion_msgs__msg__JointControl__rosidl_typesupport_introspection_c__JointControl_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
