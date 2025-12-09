// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from click_to_motion_msgs:msg/WorkspacePosition.idl
// generated code does not contain a copyright notice

#ifndef CLICK_TO_MOTION_MSGS__MSG__DETAIL__WORKSPACE_POSITION__STRUCT_HPP_
#define CLICK_TO_MOTION_MSGS__MSG__DETAIL__WORKSPACE_POSITION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__click_to_motion_msgs__msg__WorkspacePosition __attribute__((deprecated))
#else
# define DEPRECATED__click_to_motion_msgs__msg__WorkspacePosition __declspec(deprecated)
#endif

namespace click_to_motion_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct WorkspacePosition_
{
  using Type = WorkspacePosition_<ContainerAllocator>;

  explicit WorkspacePosition_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = 0l;
      this->x = 0.0;
      this->y = 0.0;
      this->z = 0.0;
      this->roll = 0.0;
      this->pitch = 0.0;
      this->yaw = 0.0;
    }
  }

  explicit WorkspacePosition_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = 0l;
      this->x = 0.0;
      this->y = 0.0;
      this->z = 0.0;
      this->roll = 0.0;
      this->pitch = 0.0;
      this->yaw = 0.0;
    }
  }

  // field types and members
  using _robot_id_type =
    int32_t;
  _robot_id_type robot_id;
  using _x_type =
    double;
  _x_type x;
  using _y_type =
    double;
  _y_type y;
  using _z_type =
    double;
  _z_type z;
  using _roll_type =
    double;
  _roll_type roll;
  using _pitch_type =
    double;
  _pitch_type pitch;
  using _yaw_type =
    double;
  _yaw_type yaw;

  // setters for named parameter idiom
  Type & set__robot_id(
    const int32_t & _arg)
  {
    this->robot_id = _arg;
    return *this;
  }
  Type & set__x(
    const double & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const double & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__z(
    const double & _arg)
  {
    this->z = _arg;
    return *this;
  }
  Type & set__roll(
    const double & _arg)
  {
    this->roll = _arg;
    return *this;
  }
  Type & set__pitch(
    const double & _arg)
  {
    this->pitch = _arg;
    return *this;
  }
  Type & set__yaw(
    const double & _arg)
  {
    this->yaw = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    click_to_motion_msgs::msg::WorkspacePosition_<ContainerAllocator> *;
  using ConstRawPtr =
    const click_to_motion_msgs::msg::WorkspacePosition_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<click_to_motion_msgs::msg::WorkspacePosition_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<click_to_motion_msgs::msg::WorkspacePosition_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      click_to_motion_msgs::msg::WorkspacePosition_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<click_to_motion_msgs::msg::WorkspacePosition_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      click_to_motion_msgs::msg::WorkspacePosition_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<click_to_motion_msgs::msg::WorkspacePosition_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<click_to_motion_msgs::msg::WorkspacePosition_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<click_to_motion_msgs::msg::WorkspacePosition_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__click_to_motion_msgs__msg__WorkspacePosition
    std::shared_ptr<click_to_motion_msgs::msg::WorkspacePosition_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__click_to_motion_msgs__msg__WorkspacePosition
    std::shared_ptr<click_to_motion_msgs::msg::WorkspacePosition_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const WorkspacePosition_ & other) const
  {
    if (this->robot_id != other.robot_id) {
      return false;
    }
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->z != other.z) {
      return false;
    }
    if (this->roll != other.roll) {
      return false;
    }
    if (this->pitch != other.pitch) {
      return false;
    }
    if (this->yaw != other.yaw) {
      return false;
    }
    return true;
  }
  bool operator!=(const WorkspacePosition_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct WorkspacePosition_

// alias to use template instance with default allocator
using WorkspacePosition =
  click_to_motion_msgs::msg::WorkspacePosition_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace click_to_motion_msgs

#endif  // CLICK_TO_MOTION_MSGS__MSG__DETAIL__WORKSPACE_POSITION__STRUCT_HPP_
