// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from click_to_motion_msgs:msg/JointControl.idl
// generated code does not contain a copyright notice

#ifndef CLICK_TO_MOTION_MSGS__MSG__DETAIL__JOINT_CONTROL__STRUCT_HPP_
#define CLICK_TO_MOTION_MSGS__MSG__DETAIL__JOINT_CONTROL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__click_to_motion_msgs__msg__JointControl __attribute__((deprecated))
#else
# define DEPRECATED__click_to_motion_msgs__msg__JointControl __declspec(deprecated)
#endif

namespace click_to_motion_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct JointControl_
{
  using Type = JointControl_<ContainerAllocator>;

  explicit JointControl_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = 0l;
    }
  }

  explicit JointControl_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = 0l;
    }
  }

  // field types and members
  using _robot_id_type =
    int32_t;
  _robot_id_type robot_id;
  using _joint_positions_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _joint_positions_type joint_positions;
  using _joint_velocities_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _joint_velocities_type joint_velocities;
  using _joint_accelerations_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _joint_accelerations_type joint_accelerations;

  // setters for named parameter idiom
  Type & set__robot_id(
    const int32_t & _arg)
  {
    this->robot_id = _arg;
    return *this;
  }
  Type & set__joint_positions(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->joint_positions = _arg;
    return *this;
  }
  Type & set__joint_velocities(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->joint_velocities = _arg;
    return *this;
  }
  Type & set__joint_accelerations(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->joint_accelerations = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    click_to_motion_msgs::msg::JointControl_<ContainerAllocator> *;
  using ConstRawPtr =
    const click_to_motion_msgs::msg::JointControl_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<click_to_motion_msgs::msg::JointControl_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<click_to_motion_msgs::msg::JointControl_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      click_to_motion_msgs::msg::JointControl_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<click_to_motion_msgs::msg::JointControl_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      click_to_motion_msgs::msg::JointControl_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<click_to_motion_msgs::msg::JointControl_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<click_to_motion_msgs::msg::JointControl_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<click_to_motion_msgs::msg::JointControl_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__click_to_motion_msgs__msg__JointControl
    std::shared_ptr<click_to_motion_msgs::msg::JointControl_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__click_to_motion_msgs__msg__JointControl
    std::shared_ptr<click_to_motion_msgs::msg::JointControl_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const JointControl_ & other) const
  {
    if (this->robot_id != other.robot_id) {
      return false;
    }
    if (this->joint_positions != other.joint_positions) {
      return false;
    }
    if (this->joint_velocities != other.joint_velocities) {
      return false;
    }
    if (this->joint_accelerations != other.joint_accelerations) {
      return false;
    }
    return true;
  }
  bool operator!=(const JointControl_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct JointControl_

// alias to use template instance with default allocator
using JointControl =
  click_to_motion_msgs::msg::JointControl_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace click_to_motion_msgs

#endif  // CLICK_TO_MOTION_MSGS__MSG__DETAIL__JOINT_CONTROL__STRUCT_HPP_
