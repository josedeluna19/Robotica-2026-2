// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from click_to_motion_msgs:msg/JointControl.idl
// generated code does not contain a copyright notice

#ifndef CLICK_TO_MOTION_MSGS__MSG__DETAIL__JOINT_CONTROL__BUILDER_HPP_
#define CLICK_TO_MOTION_MSGS__MSG__DETAIL__JOINT_CONTROL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "click_to_motion_msgs/msg/detail/joint_control__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace click_to_motion_msgs
{

namespace msg
{

namespace builder
{

class Init_JointControl_joint_accelerations
{
public:
  explicit Init_JointControl_joint_accelerations(::click_to_motion_msgs::msg::JointControl & msg)
  : msg_(msg)
  {}
  ::click_to_motion_msgs::msg::JointControl joint_accelerations(::click_to_motion_msgs::msg::JointControl::_joint_accelerations_type arg)
  {
    msg_.joint_accelerations = std::move(arg);
    return std::move(msg_);
  }

private:
  ::click_to_motion_msgs::msg::JointControl msg_;
};

class Init_JointControl_joint_velocities
{
public:
  explicit Init_JointControl_joint_velocities(::click_to_motion_msgs::msg::JointControl & msg)
  : msg_(msg)
  {}
  Init_JointControl_joint_accelerations joint_velocities(::click_to_motion_msgs::msg::JointControl::_joint_velocities_type arg)
  {
    msg_.joint_velocities = std::move(arg);
    return Init_JointControl_joint_accelerations(msg_);
  }

private:
  ::click_to_motion_msgs::msg::JointControl msg_;
};

class Init_JointControl_joint_positions
{
public:
  explicit Init_JointControl_joint_positions(::click_to_motion_msgs::msg::JointControl & msg)
  : msg_(msg)
  {}
  Init_JointControl_joint_velocities joint_positions(::click_to_motion_msgs::msg::JointControl::_joint_positions_type arg)
  {
    msg_.joint_positions = std::move(arg);
    return Init_JointControl_joint_velocities(msg_);
  }

private:
  ::click_to_motion_msgs::msg::JointControl msg_;
};

class Init_JointControl_robot_id
{
public:
  Init_JointControl_robot_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_JointControl_joint_positions robot_id(::click_to_motion_msgs::msg::JointControl::_robot_id_type arg)
  {
    msg_.robot_id = std::move(arg);
    return Init_JointControl_joint_positions(msg_);
  }

private:
  ::click_to_motion_msgs::msg::JointControl msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::click_to_motion_msgs::msg::JointControl>()
{
  return click_to_motion_msgs::msg::builder::Init_JointControl_robot_id();
}

}  // namespace click_to_motion_msgs

#endif  // CLICK_TO_MOTION_MSGS__MSG__DETAIL__JOINT_CONTROL__BUILDER_HPP_
