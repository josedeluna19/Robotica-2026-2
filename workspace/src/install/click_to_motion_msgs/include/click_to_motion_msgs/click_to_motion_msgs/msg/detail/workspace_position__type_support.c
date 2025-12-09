// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from click_to_motion_msgs:msg/WorkspacePosition.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "click_to_motion_msgs/msg/detail/workspace_position__rosidl_typesupport_introspection_c.h"
#include "click_to_motion_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "click_to_motion_msgs/msg/detail/workspace_position__functions.h"
#include "click_to_motion_msgs/msg/detail/workspace_position__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void click_to_motion_msgs__msg__WorkspacePosition__rosidl_typesupport_introspection_c__WorkspacePosition_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  click_to_motion_msgs__msg__WorkspacePosition__init(message_memory);
}

void click_to_motion_msgs__msg__WorkspacePosition__rosidl_typesupport_introspection_c__WorkspacePosition_fini_function(void * message_memory)
{
  click_to_motion_msgs__msg__WorkspacePosition__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember click_to_motion_msgs__msg__WorkspacePosition__rosidl_typesupport_introspection_c__WorkspacePosition_message_member_array[7] = {
  {
    "robot_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(click_to_motion_msgs__msg__WorkspacePosition, robot_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(click_to_motion_msgs__msg__WorkspacePosition, x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(click_to_motion_msgs__msg__WorkspacePosition, y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "z",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(click_to_motion_msgs__msg__WorkspacePosition, z),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "roll",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(click_to_motion_msgs__msg__WorkspacePosition, roll),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pitch",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(click_to_motion_msgs__msg__WorkspacePosition, pitch),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "yaw",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(click_to_motion_msgs__msg__WorkspacePosition, yaw),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers click_to_motion_msgs__msg__WorkspacePosition__rosidl_typesupport_introspection_c__WorkspacePosition_message_members = {
  "click_to_motion_msgs__msg",  // message namespace
  "WorkspacePosition",  // message name
  7,  // number of fields
  sizeof(click_to_motion_msgs__msg__WorkspacePosition),
  click_to_motion_msgs__msg__WorkspacePosition__rosidl_typesupport_introspection_c__WorkspacePosition_message_member_array,  // message members
  click_to_motion_msgs__msg__WorkspacePosition__rosidl_typesupport_introspection_c__WorkspacePosition_init_function,  // function to initialize message memory (memory has to be allocated)
  click_to_motion_msgs__msg__WorkspacePosition__rosidl_typesupport_introspection_c__WorkspacePosition_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t click_to_motion_msgs__msg__WorkspacePosition__rosidl_typesupport_introspection_c__WorkspacePosition_message_type_support_handle = {
  0,
  &click_to_motion_msgs__msg__WorkspacePosition__rosidl_typesupport_introspection_c__WorkspacePosition_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_click_to_motion_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, click_to_motion_msgs, msg, WorkspacePosition)() {
  if (!click_to_motion_msgs__msg__WorkspacePosition__rosidl_typesupport_introspection_c__WorkspacePosition_message_type_support_handle.typesupport_identifier) {
    click_to_motion_msgs__msg__WorkspacePosition__rosidl_typesupport_introspection_c__WorkspacePosition_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &click_to_motion_msgs__msg__WorkspacePosition__rosidl_typesupport_introspection_c__WorkspacePosition_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
