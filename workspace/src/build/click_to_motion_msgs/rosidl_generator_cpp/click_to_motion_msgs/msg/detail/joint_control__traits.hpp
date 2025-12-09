// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from click_to_motion_msgs:msg/JointControl.idl
// generated code does not contain a copyright notice

#ifndef CLICK_TO_MOTION_MSGS__MSG__DETAIL__JOINT_CONTROL__TRAITS_HPP_
#define CLICK_TO_MOTION_MSGS__MSG__DETAIL__JOINT_CONTROL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "click_to_motion_msgs/msg/detail/joint_control__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace click_to_motion_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const JointControl & msg,
  std::ostream & out)
{
  out << "{";
  // member: robot_id
  {
    out << "robot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_id, out);
    out << ", ";
  }

  // member: joint_positions
  {
    if (msg.joint_positions.size() == 0) {
      out << "joint_positions: []";
    } else {
      out << "joint_positions: [";
      size_t pending_items = msg.joint_positions.size();
      for (auto item : msg.joint_positions) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: joint_velocities
  {
    if (msg.joint_velocities.size() == 0) {
      out << "joint_velocities: []";
    } else {
      out << "joint_velocities: [";
      size_t pending_items = msg.joint_velocities.size();
      for (auto item : msg.joint_velocities) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: joint_accelerations
  {
    if (msg.joint_accelerations.size() == 0) {
      out << "joint_accelerations: []";
    } else {
      out << "joint_accelerations: [";
      size_t pending_items = msg.joint_accelerations.size();
      for (auto item : msg.joint_accelerations) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const JointControl & msg,
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

  // member: joint_positions
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.joint_positions.size() == 0) {
      out << "joint_positions: []\n";
    } else {
      out << "joint_positions:\n";
      for (auto item : msg.joint_positions) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: joint_velocities
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.joint_velocities.size() == 0) {
      out << "joint_velocities: []\n";
    } else {
      out << "joint_velocities:\n";
      for (auto item : msg.joint_velocities) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: joint_accelerations
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.joint_accelerations.size() == 0) {
      out << "joint_accelerations: []\n";
    } else {
      out << "joint_accelerations:\n";
      for (auto item : msg.joint_accelerations) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const JointControl & msg, bool use_flow_style = false)
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
  const click_to_motion_msgs::msg::JointControl & msg,
  std::ostream & out, size_t indentation = 0)
{
  click_to_motion_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use click_to_motion_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const click_to_motion_msgs::msg::JointControl & msg)
{
  return click_to_motion_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<click_to_motion_msgs::msg::JointControl>()
{
  return "click_to_motion_msgs::msg::JointControl";
}

template<>
inline const char * name<click_to_motion_msgs::msg::JointControl>()
{
  return "click_to_motion_msgs/msg/JointControl";
}

template<>
struct has_fixed_size<click_to_motion_msgs::msg::JointControl>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<click_to_motion_msgs::msg::JointControl>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<click_to_motion_msgs::msg::JointControl>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CLICK_TO_MOTION_MSGS__MSG__DETAIL__JOINT_CONTROL__TRAITS_HPP_
