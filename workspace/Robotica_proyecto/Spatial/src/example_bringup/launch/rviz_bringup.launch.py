from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    description_path = get_package_share_directory('example_description')
    model_path = os.path.join(description_path, 'urdf', 'robot2_spatial.urdf')
    rviz_conf_path = os.path.join(description_path, 'rviz', 'rviz_config.rviz')

    # Leer URDF directamente
    with open(model_path, 'r') as f:
        robot_description_content = f.read()

    robot_description = {
        'robot_description': ParameterValue(
            robot_description_content,
            value_type=str
        )
    }

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_conf_path],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node,
    ])