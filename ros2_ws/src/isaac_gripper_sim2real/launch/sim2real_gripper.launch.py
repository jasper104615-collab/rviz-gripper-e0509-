"""Launch gripper_service_node (TCP) + isaac_gripper_bridge for sim2real.

DRL conflict avoidance:
  - Arm: sub_joint_state -> move_joint/servoj_rt (DRFL, not drl_start)
  - Gripper: command_transport:=tcp -> Modbus via persistent TCP socket
             (one drl_start at init for TCP server only; no drl_start per move)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('isaac_gripper_sim2real')
    default_config = os.path.join(pkg_share, 'config', 'sim2real_gripper.yaml')

    robot_ip = LaunchConfiguration('robot_ip')
    enable_command = LaunchConfiguration('enable_command')
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_ip',
            default_value='127.0.0.1',
            description='Doosan controller IP for gripper TCP server',
        ),
        DeclareLaunchArgument(
            'enable_command',
            default_value='true',
            description='Send gripper commands to real hardware',
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Parameter YAML path',
        ),
        Node(
            package='rh_p12_rna_controller',
            executable='gripper_service_node',
            name='gripper_service_node',
            output='screen',
            parameters=[
                config_file,
                {'robot_ip': robot_ip},
            ],
        ),
        Node(
            package='isaac_gripper_sim2real',
            executable='gripper_bridge',
            name='isaac_gripper_bridge',
            output='screen',
            parameters=[
                config_file,
                {'enable_command': enable_command},
            ],
        ),
    ])
