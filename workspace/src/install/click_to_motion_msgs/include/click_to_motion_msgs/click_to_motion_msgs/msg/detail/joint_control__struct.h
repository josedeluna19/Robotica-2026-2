// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from click_to_motion_msgs:msg/JointControl.idl
// generated code does not contain a copyright notice

#ifndef CLICK_TO_MOTION_MSGS__MSG__DETAIL__JOINT_CONTROL__STRUCT_H_
#define CLICK_TO_MOTION_MSGS__MSG__DETAIL__JOINT_CONTROL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'joint_positions'
// Member 'joint_velocities'
// Member 'joint_accelerations'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/JointControl in the package click_to_motion_msgs.
/**
  * Robot ID (1 o 2)
 */
typedef struct click_to_motion_msgs__msg__JointControl
{
  int32_t robot_id;
  /// Posiciones articulares deseadas (rads)
  rosidl_runtime_c__double__Sequence joint_positions;
  /// Velocidades articulares deseadas (rads/s)
  rosidl_runtime_c__double__Sequence joint_velocities;
  /// Aceleraciones deseadas opcionales
  rosidl_runtime_c__double__Sequence joint_accelerations;
} click_to_motion_msgs__msg__JointControl;

// Struct for a sequence of click_to_motion_msgs__msg__JointControl.
typedef struct click_to_motion_msgs__msg__JointControl__Sequence
{
  click_to_motion_msgs__msg__JointControl * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} click_to_motion_msgs__msg__JointControl__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CLICK_TO_MOTION_MSGS__MSG__DETAIL__JOINT_CONTROL__STRUCT_H_
