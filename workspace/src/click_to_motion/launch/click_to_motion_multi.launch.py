# src/click_to_motion/launch/click_to_motion_multi.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():

    pkg = FindPackageShare('click_to_motion').find('click_to_motion')

    # Archivos
    rviz_config = os.path.join(pkg, 'rviz', 'two_robots_fixed.rviz')
    robot1_urdf = os.path.join(pkg, 'urdf', 'robot1_planar.urdf')
    robot2_urdf = os.path.join(pkg, 'urdf', 'robot2_spatial.urdf')

    # ---------- robot1 ----------
    robot1_urdf_pub = Node(
        package='click_to_motion',
        executable='urdf_publisher',
        name='robot1_urdf_publisher',
        namespace='',
        parameters=[{'urdf_file': robot1_urdf, 'robot_ns': 'robot1'}],
        output='screen'
    )

    robot1_description = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='robot1',
        # IMPORTANT: add frame_prefix so frames become robot1/base_link, robot1/link1, ...
        parameters=[
            {'robot_description': open(robot1_urdf).read()},
            {'frame_prefix': 'robot1/'}   # <<-- aquí
        ],
        output='screen'
    )

    robot1_fake_js = Node(
        package='click_to_motion',
        executable='fake_joint_state_pub',
        name='robot1_fake_js',
        namespace='robot1',
        output='screen'
    )

    robot1_click = Node(
        package='click_to_motion',
        executable='click_to_motion',
        namespace='robot1',
        output='screen'
    )

    # ---------- robot2 ----------
    robot2_urdf_pub = Node(
        package='click_to_motion',
        executable='urdf_publisher',
        name='robot2_urdf_publisher',
        namespace='',
        parameters=[{'urdf_file': robot2_urdf, 'robot_ns': 'robot2'}],
        output='screen'
    )

    robot2_description = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='robot2',
        parameters=[
            {'robot_description': open(robot2_urdf).read()},
            {'frame_prefix': 'robot2/'}   # <<-- aquí
        ],
        output='screen'
    )

    robot2_fake_js = Node(
        package='click_to_motion',
        executable='fake_joint_state_pub',
        name='robot2_fake_js',
        namespace='robot2',
        output='screen'
    )

    robot2_click = Node(
        package='click_to_motion',
        executable='click_to_motion',
        namespace='robot2',
        output='screen'
    )

    # ---------- RVIZ ----------
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        robot1_urdf_pub,
        robot1_description,
        robot1_fake_js,
        robot1_click,

        robot2_urdf_pub,
        robot2_description,
        robot2_fake_js,
        robot2_click,

        rviz
    ])
