// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from click_to_motion_msgs:msg/WorkspacePosition.idl
// generated code does not contain a copyright notice

#ifndef CLICK_TO_MOTION_MSGS__MSG__DETAIL__WORKSPACE_POSITION__BUILDER_HPP_
#define CLICK_TO_MOTION_MSGS__MSG__DETAIL__WORKSPACE_POSITION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "click_to_motion_msgs/msg/detail/workspace_position__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace click_to_motion_msgs
{

namespace msg
{

namespace builder
{

class Init_WorkspacePosition_yaw
{
public:
  explicit Init_WorkspacePosition_yaw(::click_to_motion_msgs::msg::WorkspacePosition & msg)
  : msg_(msg)
  {}
  ::click_to_motion_msgs::msg::WorkspacePosition yaw(::click_to_motion_msgs::msg::WorkspacePosition::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::click_to_motion_msgs::msg::WorkspacePosition msg_;
};

class Init_WorkspacePosition_pitch
{
public:
  explicit Init_WorkspacePosition_pitch(::click_to_motion_msgs::msg::WorkspacePosition & msg)
  : msg_(msg)
  {}
  Init_WorkspacePosition_yaw pitch(::click_to_motion_msgs::msg::WorkspacePosition::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_WorkspacePosition_yaw(msg_);
  }

private:
  ::click_to_motion_msgs::msg::WorkspacePosition msg_;
};

class Init_WorkspacePosition_roll
{
public:
  explicit Init_WorkspacePosition_roll(::click_to_motion_msgs::msg::WorkspacePosition & msg)
  : msg_(msg)
  {}
  Init_WorkspacePosition_pitch roll(::click_to_motion_msgs::msg::WorkspacePosition::_roll_type arg)
  {
    msg_.roll = std::move(arg);
    return Init_WorkspacePosition_pitch(msg_);
  }

private:
  ::click_to_motion_msgs::msg::WorkspacePosition msg_;
};

class Init_WorkspacePosition_z
{
public:
  explicit Init_WorkspacePosition_z(::click_to_motion_msgs::msg::WorkspacePosition & msg)
  : msg_(msg)
  {}
  Init_WorkspacePosition_roll z(::click_to_motion_msgs::msg::WorkspacePosition::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_WorkspacePosition_roll(msg_);
  }

private:
  ::click_to_motion_msgs::msg::WorkspacePosition msg_;
};

class Init_WorkspacePosition_y
{
public:
  explicit Init_WorkspacePosition_y(::click_to_motion_msgs::msg::WorkspacePosition & msg)
  : msg_(msg)
  {}
  Init_WorkspacePosition_z y(::click_to_motion_msgs::msg::WorkspacePosition::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_WorkspacePosition_z(msg_);
  }

private:
  ::click_to_motion_msgs::msg::WorkspacePosition msg_;
};

class Init_WorkspacePosition_x
{
public:
  explicit Init_WorkspacePosition_x(::click_to_motion_msgs::msg::WorkspacePosition & msg)
  : msg_(msg)
  {}
  Init_WorkspacePosition_y x(::click_to_motion_msgs::msg::WorkspacePosition::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_WorkspacePosition_y(msg_);
  }

private:
  ::click_to_motion_msgs::msg::WorkspacePosition msg_;
};

class Init_WorkspacePosition_robot_id
{
public:
  Init_WorkspacePosition_robot_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WorkspacePosition_x robot_id(::click_to_motion_msgs::msg::WorkspacePosition::_robot_id_type arg)
  {
    msg_.robot_id = std::move(arg);
    return Init_WorkspacePosition_x(msg_);
  }

private:
  ::click_to_motion_msgs::msg::WorkspacePosition msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::click_to_motion_msgs::msg::WorkspacePosition>()
{
  return click_to_motion_msgs::msg::builder::Init_WorkspacePosition_robot_id();
}

}  // namespace click_to_motion_msgs

#endif  // CLICK_TO_MOTION_MSGS__MSG__DETAIL__WORKSPACE_POSITION__BUILDER_HPP_
