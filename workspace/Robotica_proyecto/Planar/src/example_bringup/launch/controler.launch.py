from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Ruta al paquete y al URDF
    description_path = get_package_share_directory('example_description')
    model_path = os.path.join(description_path, 'urdf', 'robot1_planar.urdf')
    rviz_conf_path = os.path.join(description_path, 'rviz', 'rviz_config.rviz')

    # Leer el contenido del URDF directamente (SIN XACRO)
    with open(model_path, 'r') as f:
        robot_description_content = f.read()

    robot_description = {
        'robot_description': ParameterValue(
            robot_description_content,
            value_type=str
        )
    }

    controller_manager_node = Node(
        package='example_control',
        executable='controller_manager',
        output='screen'
    )

    manipulator_controller_node = Node(
        package='example_control',
        executable='manipulator_controller',
        output='screen'
    )

    hardware_interface_node = Node(
        package='example_control',
        executable='hardware_interface',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_conf_path],
        output='screen'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

    return LaunchDescription([
        controller_manager_node,
        manipulator_controller_node,
        hardware_interface_node,
        rviz_node,
        robot_state_publisher_node,
    ])