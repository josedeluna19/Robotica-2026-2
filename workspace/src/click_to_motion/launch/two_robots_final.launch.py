from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('click_to_motion')
    urdf1 = os.path.join(pkg, 'urdf', 'robot1_planar.urdf')
    urdf2 = os.path.join(pkg, 'urdf', 'robot2_spatial.urdf')

    # Read URDF strings (we will pass them as parameters to urdf_publisher)
    with open(urdf1, 'r') as f:
        urdf1_txt = f.read()
    with open(urdf2, 'r') as f:
        urdf2_txt = f.read()

    # robot_state_publisher nodes (provide URDF as parameter too so they can parse the model)
    robot1_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='robot1',
        name='robot1_state_pub',
        parameters=[{'robot_description': urdf1_txt}, {'frame_prefix': 'robot1/'}]
    )

    robot2_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='robot2',
        name='robot2_state_pub',
        parameters=[{'robot_description': urdf2_txt}, {'frame_prefix': 'robot2/'}]
    )

    # URDF publishers (publish on /robot1/robot_description and /robot2/robot_description transient_local)
    urdf_pub_1 = Node(
        package='click_to_motion',
        executable='urdf_publisher',
        namespace='robot1',
        name='robot1_urdf_pub',
        parameters=[{'robot_description': urdf1_txt}]
    )

    urdf_pub_2 = Node(
        package='click_to_motion',
        executable='urdf_publisher',
        namespace='robot2',
        name='robot2_urdf_pub',
        parameters=[{'robot_description': urdf2_txt}]
    )

    # static transforms: world -> robot1/base_link and world -> robot2/base_link
    tf1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_world_robot1',
        arguments=['0','0','0','0','0','0','world','robot1/base_link']
    )
    tf2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_world_robot2',
        arguments=['1.0','0','0','0','0','0','world','robot2/base_link']
    )

    # Click-to-motion nodes (these publish joint_states directly)
    click1 = Node(
        package='click_to_motion',
        executable='click_to_motion_planar',
        namespace='robot1',
        name='click_to_motion_planar',
        parameters=[{'l1': 0.4, 'l2': 0.35, 'l3': 0.35}]
    )

    click2 = Node(
        package='click_to_motion',
        executable='click_to_motion_spatial',
        namespace='robot2',
        name='click_to_motion_spatial',
        parameters=[{'l1': 0.05, 'l2': 0.4, 'l3': 0.3}]
    )

    # RViz (use your rviz config that has Fixed Frame world)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg, 'rviz', 'click_to_motion.rviz')],
        output='screen'
    )

    return LaunchDescription([
        robot1_state_pub,
        urdf_pub_1,
        robot2_state_pub,
        urdf_pub_2,
        tf1,
        tf2,
        click1,
        click2,
        rviz
    ])
