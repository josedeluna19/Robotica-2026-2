// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from click_to_motion_msgs:msg/JointControl.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "click_to_motion_msgs/msg/detail/joint_control__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace click_to_motion_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void JointControl_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) click_to_motion_msgs::msg::JointControl(_init);
}

void JointControl_fini_function(void * message_memory)
{
  auto typed_message = static_cast<click_to_motion_msgs::msg::JointControl *>(message_memory);
  typed_message->~JointControl();
}

size_t size_function__JointControl__joint_positions(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__JointControl__joint_positions(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__JointControl__joint_positions(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__JointControl__joint_positions(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__JointControl__joint_positions(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__JointControl__joint_positions(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__JointControl__joint_positions(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__JointControl__joint_positions(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__JointControl__joint_velocities(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__JointControl__joint_velocities(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__JointControl__joint_velocities(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__JointControl__joint_velocities(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__JointControl__joint_velocities(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__JointControl__joint_velocities(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__JointControl__joint_velocities(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__JointControl__joint_velocities(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__JointControl__joint_accelerations(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__JointControl__joint_accelerations(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__JointControl__joint_accelerations(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__JointControl__joint_accelerations(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__JointControl__joint_accelerations(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__JointControl__joint_accelerations(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__JointControl__joint_accelerations(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__JointControl__joint_accelerations(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember JointControl_message_member_array[4] = {
  {
    "robot_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(click_to_motion_msgs::msg::JointControl, robot_id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "joint_positions",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(click_to_motion_msgs::msg::JointControl, joint_positions),  // bytes offset in struct
    nullptr,  // default value
    size_function__JointControl__joint_positions,  // size() function pointer
    get_const_function__JointControl__joint_positions,  // get_const(index) function pointer
    get_function__JointControl__joint_positions,  // get(index) function pointer
    fetch_function__JointControl__joint_positions,  // fetch(index, &value) function pointer
    assign_function__JointControl__joint_positions,  // assign(index, value) function pointer
    resize_function__JointControl__joint_positions  // resize(index) function pointer
  },
  {
    "joint_velocities",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(click_to_motion_msgs::msg::JointControl, joint_velocities),  // bytes offset in struct
    nullptr,  // default value
    size_function__JointControl__joint_velocities,  // size() function pointer
    get_const_function__JointControl__joint_velocities,  // get_const(index) function pointer
    get_function__JointControl__joint_velocities,  // get(index) function pointer
    fetch_function__JointControl__joint_velocities,  // fetch(index, &value) function pointer
    assign_function__JointControl__joint_velocities,  // assign(index, value) function pointer
    resize_function__JointControl__joint_velocities  // resize(index) function pointer
  },
  {
    "joint_accelerations",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(click_to_motion_msgs::msg::JointControl, joint_accelerations),  // bytes offset in struct
    nullptr,  // default value
    size_function__JointControl__joint_accelerations,  // size() function pointer
    get_const_function__JointControl__joint_accelerations,  // get_const(index) function pointer
    get_function__JointControl__joint_accelerations,  // get(index) function pointer
    fetch_function__JointControl__joint_accelerations,  // fetch(index, &value) function pointer
    assign_function__JointControl__joint_accelerations,  // assign(index, value) function pointer
    resize_function__JointControl__joint_accelerations  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers JointControl_message_members = {
  "click_to_motion_msgs::msg",  // message namespace
  "JointControl",  // message name
  4,  // number of fields
  sizeof(click_to_motion_msgs::msg::JointControl),
  JointControl_message_member_array,  // message members
  JointControl_init_function,  // function to initialize message memory (memory has to be allocated)
  JointControl_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t JointControl_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &JointControl_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace click_to_motion_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<click_to_motion_msgs::msg::JointControl>()
{
  return &::click_to_motion_msgs::msg::rosidl_typesupport_introspection_cpp::JointControl_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, click_to_motion_msgs, msg, JointControl)() {
  return &::click_to_motion_msgs::msg::rosidl_typesupport_introspection_cpp::JointControl_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
