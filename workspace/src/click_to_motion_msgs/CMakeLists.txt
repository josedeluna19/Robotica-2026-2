cmake_minimum_required(VERSION 3.8)
project(click_to_motion_msgs)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointControl.msg"
  "msg/WorkspacePosition.msg"
)

ament_package()
