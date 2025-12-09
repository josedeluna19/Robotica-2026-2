// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from click_to_motion_msgs:msg/WorkspacePosition.idl
// generated code does not contain a copyright notice

#ifndef CLICK_TO_MOTION_MSGS__MSG__DETAIL__WORKSPACE_POSITION__STRUCT_H_
#define CLICK_TO_MOTION_MSGS__MSG__DETAIL__WORKSPACE_POSITION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/WorkspacePosition in the package click_to_motion_msgs.
/**
  * Robot ID (1 o 2)
 */
typedef struct click_to_motion_msgs__msg__WorkspacePosition
{
  int32_t robot_id;
  /// Posición cartesiana deseada del efector
  double x;
  double y;
  double z;
  /// Rotación deseada (tres ejes)
  double roll;
  double pitch;
  double yaw;
} click_to_motion_msgs__msg__WorkspacePosition;

// Struct for a sequence of click_to_motion_msgs__msg__WorkspacePosition.
typedef struct click_to_motion_msgs__msg__WorkspacePosition__Sequence
{
  click_to_motion_msgs__msg__WorkspacePosition * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} click_to_motion_msgs__msg__WorkspacePosition__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CLICK_TO_MOTION_MSGS__MSG__DETAIL__WORKSPACE_POSITION__STRUCT_H_
