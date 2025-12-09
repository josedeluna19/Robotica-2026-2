#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def load_urdf(path):
    with open(path, 'r') as file:
        return file.read()


def generate_launch_description():

    pkg = get_package_share_directory("click_to_motion")

    # ------------------------
    #   Paths URDF
    # ------------------------
    robot1_urdf = os.path.join(pkg, "urdf", "robot1.urdf")
    robot2_urdf = os.path.join(pkg, "urdf", "robot2.urdf")

    urdf1 = load_urdf(robot1_urdf)
    urdf2 = load_urdf(robot2_urdf)

    # ------------------------
    #   RViz config
    # ------------------------
    rviz_config = os.path.join(pkg, "rviz", "multi_robot.rviz")

    # ============================================================
    #                   NODOS DE ROBOT 1
    # ============================================================

    robot1_rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="/robot1",
        parameters=[{"robot_description": urdf1}],
        output="screen"
    )

    robot1_click = Node(
        package="click_to_motion",
        executable="click_listener",
        namespace="/robot1",
        output="screen"
    )

    robot1_ik = Node(
        package="click_to_motion",
        executable="ik_solver",
        namespace="/robot1",
        output="screen"
    )

    robot1_traj = Node(
        package="click_to_motion",
        executable="trajectory_generator",
        namespace="/robot1",
        output="screen"
    )

    robot1_torque = Node(
        package="click_to_motion",
        executable="torque_computer",
        namespace="/robot1",
        output="screen"
    )

    robot1_plotter = Node(
        package="click_to_motion",
        executable="plotter",
        namespace="/robot1",
        output="screen"
    )

    robot1_motion = Node(
        package="click_to_motion",
        executable="motion_executor",
        namespace="/robot1",
        output="screen"
    )

    # ============================================================
    #                   NODOS DE ROBOT 2
    # ============================================================

    robot2_rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="/robot2",
        parameters=[{"robot_description": urdf2}],
        output="screen"
    )

    robot2_click = Node(
        package="click_to_motion",
        executable="click_listener",
        namespace="/robot2",
        output="screen"
    )

    robot2_ik = Node(
        package="click_to_motion",
        executable="ik_solver",
        namespace="/robot2",
        output="screen"
    )

    robot2_traj = Node(
        package="click_to_motion",
        executable="trajectory_generator",
        namespace="/robot2",
        output="screen"
    )

    robot2_torque = Node(
        package="click_to_motion",
        executable="torque_computer",
        namespace="/robot2",
        output="screen"
    )

    robot2_plotter = Node(
        package="click_to_motion",
        executable="plotter",
        namespace="/robot2",
        output="screen"
    )

    robot2_motion = Node(
        package="click_to_motion",
        executable="motion_executor",
        namespace="/robot2",
        output="screen"
    )

    # ============================================================
    #                   RVIZ (ambos robots)
    # ============================================================

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        output="screen"
    )

    # ============================================================
    #                   LAUNCH DESCRIPTION
    # ============================================================

    return LaunchDescription([
        # Robot 1
        robot1_rsp,
        robot1_click,
        robot1_ik,
        robot1_traj,
        robot1_torque,
        robot1_plotter,
        robot1_motion,

        # Robot 2
        robot2_rsp,
        robot2_click,
        robot2_ik,
        robot2_traj,
        robot2_torque,
        robot2_plotter,
        robot2_motion,

        # RViz
        rviz
    ])
