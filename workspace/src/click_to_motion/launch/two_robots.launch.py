from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_path = get_package_share_directory('click_to_motion')

    urdf_robot1 = os.path.join(pkg_path, 'urdf', 'robot1_planar.urdf')
    urdf_robot2 = os.path.join(pkg_path, 'urdf', 'robot2_spatial.urdf')

    # Robot 1: robot_state_publisher with frame_prefix -> robot1/
    robot1_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='robot1',
        name='state_publisher',
        parameters=[
            {'robot_description': open(urdf_robot1).read()},
            {'frame_prefix': 'robot1/'}
        ],
        remappings=[('joint_states', 'joint_states')]
    )

    # keep robot connected initially (publishes zeros until click node executes)
    robot1_fake_js = Node(
        package='click_to_motion',
        executable='fake_joint_state_pub',
        namespace='robot1',
        name='fake_js_pub'
    )

    # World -> robot1/base_link
    tf_world_robot1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_world_robot1',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'robot1/base_link']
    )

    # Robot 2:
    robot2_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='robot2',
        name='state_publisher',
        parameters=[
            {'robot_description': open(urdf_robot2).read()},
            {'frame_prefix': 'robot2/'}
        ],
        remappings=[('joint_states', 'joint_states')]
    )

    robot2_fake_js = Node(
        package='click_to_motion',
        executable='fake_joint_state_pub',
        namespace='robot2',
        name='fake_js_pub'
    )

    tf_world_robot2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_world_robot2',
        arguments=['1.0', '0', '0', '0', '0', '0', 'world', 'robot2/base_link']
    )

    # Click nodes (one per robot) - ensure entry_points exist in setup.py
    click_robot1 = Node(
        package='click_to_motion',
        executable='click_to_motion_node',
        namespace='robot1',
        name='click_to_motion'
    )

    click_robot2 = Node(
        package='click_to_motion',
        executable='click_to_motion_node',
        namespace='robot2',
        name='click_to_motion'
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_path, 'rviz', 'click_to_motion.rviz')],
        output='screen'
    )

    return LaunchDescription([
        robot1_state_pub,
        robot1_fake_js,
        tf_world_robot1,

        robot2_state_pub,
        robot2_fake_js,
        tf_world_robot2,

        click_robot1,
        click_robot2,

        rviz,
    ])
