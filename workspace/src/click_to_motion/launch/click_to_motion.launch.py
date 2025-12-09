from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg = get_package_share_directory('click_to_motion')

    urdf1 = os.path.join(pkg, 'urdf', 'robot1_planar.urdf')
    urdf2 = os.path.join(pkg, 'urdf', 'robot2_spatial.urdf')

    # --------------------------
    # ROBOT 1
    # --------------------------
    robot1_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='robot1',
        parameters=[
            {"robot_description": open(urdf1).read()},
            {"frame_prefix": "robot1/"}
        ],
        remappings=[("joint_states", "joint_states")]
    )

    tf_robot1 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "robot1/base_link"]
    )

    click_to_motion_1 = Node(
        package="click_to_motion",
        executable="click_to_motion_node",
        namespace="robot1",
        parameters=[{"robot_type": "planar"}],
        remappings=[
            ("/robot1/clicked_point", "clicked_point"),
            ("joint_states", "joint_states")
        ]
    )

    # --------------------------
    # ROBOT 2
    # --------------------------
    robot2_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='robot2',
        parameters=[
            {"robot_description": open(urdf2).read()},
            {"frame_prefix": "robot2/"}
        ],
        remappings=[("joint_states", "joint_states")]
    )

    tf_robot2 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["1", "0", "0", "0", "0", "0", "world", "robot2/base_link"]
    )

    click_to_motion_2 = Node(
        package="click_to_motion",
        executable="click_to_motion_node",
        namespace="robot2",
        parameters=[{"robot_type": "spatial"}],
        remappings=[
            ("/robot2/clicked_point", "clicked_point"),
            ("joint_states", "joint_states")
        ]
    )

    # --------------------------
    # RVIZ
    # --------------------------
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(pkg, "rviz", "click_to_motion.rviz")]
    )

    return LaunchDescription([
        robot1_state_pub,
        robot2_state_pub,
        tf_robot1,
        tf_robot2,
        click_to_motion_1,
        click_to_motion_2,
        rviz
    ])
