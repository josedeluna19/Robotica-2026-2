// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from click_to_motion_msgs:msg/WorkspacePosition.idl
// generated code does not contain a copyright notice

#ifndef CLICK_TO_MOTION_MSGS__MSG__DETAIL__WORKSPACE_POSITION__TRAITS_HPP_
#define CLICK_TO_MOTION_MSGS__MSG__DETAIL__WORKSPACE_POSITION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "click_to_motion_msgs/msg/detail/workspace_position__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace click_to_motion_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const WorkspacePosition & msg,
  std::ostream & out)
{
  out << "{";
  // member: robot_id
  {
    out << "robot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_id, out);
    out << ", ";
  }

  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: z
  {
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << ", ";
  }

  // member: roll
  {
    out << "roll: ";
    rosidl_generator_traits::value_to_yaml(msg.roll, out);
    out << ", ";
  }

  // member: pitch
  {
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << ", ";
  }

  // member: yaw
  {
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const WorkspacePosition & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: robot_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_id, out);
    out << "\n";
  }

  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << "\n";
  }

  // member: roll
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "roll: ";
    rosidl_generator_traits::value_to_yaml(msg.roll, out);
    out << "\n";
  }

  // member: pitch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << "\n";
  }

  // member: yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const WorkspacePosition & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace click_to_motion_msgs

namespace rosidl_generator_traits
{

[[deprecated("use click_to_motion_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const click_to_motion_msgs::msg::WorkspacePosition & msg,
  std::ostream & out, size_t indentation = 0)
{
  click_to_motion_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use click_to_motion_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const click_to_motion_msgs::msg::WorkspacePosition & msg)
{
  return click_to_motion_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<click_to_motion_msgs::msg::WorkspacePosition>()
{
  return "click_to_motion_msgs::msg::WorkspacePosition";
}

template<>
inline const char * name<click_to_motion_msgs::msg::WorkspacePosition>()
{
  return "click_to_motion_msgs/msg/WorkspacePosition";
}

template<>
struct has_fixed_size<click_to_motion_msgs::msg::WorkspacePosition>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<click_to_motion_msgs::msg::WorkspacePosition>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<click_to_motion_msgs::msg::WorkspacePosition>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CLICK_TO_MOTION_MSGS__MSG__DETAIL__WORKSPACE_POSITION__TRAITS_HPP_
