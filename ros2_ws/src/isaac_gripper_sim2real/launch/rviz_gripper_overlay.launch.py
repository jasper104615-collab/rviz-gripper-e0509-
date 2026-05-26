"""Launch gripper feedback + joint_states merger for existing dsr_bringup2 RViz session.

Use when dsr_bringup2_rviz is already running with arm-only URDF.
Replace robot_state_publisher robot_description with e0509_with_gripper.urdf
and remap joint_states -> joint_states_rviz (see README).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("isaac_gripper_sim2real")
    merger_config = os.path.join(pkg_share, "config", "rviz_gripper.yaml")
    gripper_config = os.path.join(pkg_share, "config", "rviz_gripper_service.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("robot_ns", default_value="dsr01"),
        DeclareLaunchArgument("robot_ip", default_value="110.120.1.40"),
        Node(
            package="rh_p12_rna_controller",
            executable="gripper_service_node",
            name="gripper_service_node",
            output="screen",
            parameters=[gripper_config, {"robot_ip": LaunchConfiguration("robot_ip")}],
        ),
        Node(
            package="isaac_gripper_sim2real",
            executable="rviz_joint_state_merger",
            name="rviz_joint_state_merger",
            output="screen",
            parameters=[merger_config, {"robot_ns": LaunchConfiguration("robot_ns")}],
        ),
    ])
