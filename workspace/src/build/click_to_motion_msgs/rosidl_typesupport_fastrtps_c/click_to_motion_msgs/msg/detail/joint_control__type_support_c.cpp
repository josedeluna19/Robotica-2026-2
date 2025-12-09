// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from click_to_motion_msgs:msg/JointControl.idl
// generated code does not contain a copyright notice
#include "click_to_motion_msgs/msg/detail/joint_control__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "click_to_motion_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "click_to_motion_msgs/msg/detail/joint_control__struct.h"
#include "click_to_motion_msgs/msg/detail/joint_control__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/primitives_sequence.h"  // joint_accelerations, joint_positions, joint_velocities
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // joint_accelerations, joint_positions, joint_velocities

// forward declare type support functions


using _JointControl__ros_msg_type = click_to_motion_msgs__msg__JointControl;

static bool _JointControl__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _JointControl__ros_msg_type * ros_message = static_cast<const _JointControl__ros_msg_type *>(untyped_ros_message);
  // Field name: robot_id
  {
    cdr << ros_message->robot_id;
  }

  // Field name: joint_positions
  {
    size_t size = ros_message->joint_positions.size;
    auto array_ptr = ros_message->joint_positions.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: joint_velocities
  {
    size_t size = ros_message->joint_velocities.size;
    auto array_ptr = ros_message->joint_velocities.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: joint_accelerations
  {
    size_t size = ros_message->joint_accelerations.size;
    auto array_ptr = ros_message->joint_accelerations.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  return true;
}

static bool _JointControl__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _JointControl__ros_msg_type * ros_message = static_cast<_JointControl__ros_msg_type *>(untyped_ros_message);
  // Field name: robot_id
  {
    cdr >> ros_message->robot_id;
  }

  // Field name: joint_positions
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->joint_positions.data) {
      rosidl_runtime_c__double__Sequence__fini(&ros_message->joint_positions);
    }
    if (!rosidl_runtime_c__double__Sequence__init(&ros_message->joint_positions, size)) {
      fprintf(stderr, "failed to create array for field 'joint_positions'");
      return false;
    }
    auto array_ptr = ros_message->joint_positions.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: joint_velocities
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->joint_velocities.data) {
      rosidl_runtime_c__double__Sequence__fini(&ros_message->joint_velocities);
    }
    if (!rosidl_runtime_c__double__Sequence__init(&ros_message->joint_velocities, size)) {
      fprintf(stderr, "failed to create array for field 'joint_velocities'");
      return false;
    }
    auto array_ptr = ros_message->joint_velocities.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: joint_accelerations
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->joint_accelerations.data) {
      rosidl_runtime_c__double__Sequence__fini(&ros_message->joint_accelerations);
    }
    if (!rosidl_runtime_c__double__Sequence__init(&ros_message->joint_accelerations, size)) {
      fprintf(stderr, "failed to create array for field 'joint_accelerations'");
      return false;
    }
    auto array_ptr = ros_message->joint_accelerations.data;
    cdr.deserializeArray(array_ptr, size);
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_click_to_motion_msgs
size_t get_serialized_size_click_to_motion_msgs__msg__JointControl(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _JointControl__ros_msg_type * ros_message = static_cast<const _JointControl__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name robot_id
  {
    size_t item_size = sizeof(ros_message->robot_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name joint_positions
  {
    size_t array_size = ros_message->joint_positions.size;
    auto array_ptr = ros_message->joint_positions.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name joint_velocities
  {
    size_t array_size = ros_message->joint_velocities.size;
    auto array_ptr = ros_message->joint_velocities.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name joint_accelerations
  {
    size_t array_size = ros_message->joint_accelerations.size;
    auto array_ptr = ros_message->joint_accelerations.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _JointControl__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_click_to_motion_msgs__msg__JointControl(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_click_to_motion_msgs
size_t max_serialized_size_click_to_motion_msgs__msg__JointControl(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: robot_id
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: joint_positions
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: joint_velocities
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: joint_accelerations
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = click_to_motion_msgs__msg__JointControl;
    is_plain =
      (
      offsetof(DataType, joint_accelerations) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _JointControl__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_click_to_motion_msgs__msg__JointControl(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_JointControl = {
  "click_to_motion_msgs::msg",
  "JointControl",
  _JointControl__cdr_serialize,
  _JointControl__cdr_deserialize,
  _JointControl__get_serialized_size,
  _JointControl__max_serialized_size
};

static rosidl_message_type_support_t _JointControl__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_JointControl,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, click_to_motion_msgs, msg, JointControl)() {
  return &_JointControl__type_support;
}

#if defined(__cplusplus)
}
#endif
