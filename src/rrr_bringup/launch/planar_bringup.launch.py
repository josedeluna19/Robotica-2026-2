from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    urdf_path = os.path.expanduser(
        "~/Repo_Robotica_2026_2/Robotica-2026-2/src/rrr_description/urdf/rrr_planar.urdf"
    )

    return LaunchDescription([
        # Publicar URDF
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": open(urdf_path).read()}]
        ),

        # GUI para mover juntas
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen"
        ),

        # RViz
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen"
        )
    ])

