"""RViz: RH-P12 gripper only + real pulse feedback.

Requires dsr bringup (real mode) in another terminal for /dsr01/drl/drl_start (TCP).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("rviz_gripper_e0509")
    config = os.path.join(pkg, "config", "gripper_rviz_real.yaml")
    urdf_path = os.path.join(pkg, "urdf", "rh_p12_gripper_only.urdf")
    with open(urdf_path, encoding="utf-8") as f:
        robot_description = f.read()

    robot_ip = LaunchConfiguration("robot_ip")

    return LaunchDescription([
        DeclareLaunchArgument("robot_ip", default_value="110.120.1.40"),
        Node(
            package="rh_p12_rna_controller",
            executable="gripper_service_node",
            name="gripper_service_node",
            output="screen",
            parameters=[config, {"robot_ip": robot_ip}],
        ),
        Node(
            package="rviz_gripper_e0509",
            executable="gripper_joint_state",
            name="gripper_joint_state",
            output="screen",
            parameters=[config],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-f", "world"],
        ),
    ])
