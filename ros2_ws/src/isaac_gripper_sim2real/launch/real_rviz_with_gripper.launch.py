# RViz + real robot: e0509 arm (dsr_bringup2) + RH-P12 gripper feedback in URDF
#
# Usage:
#   ros2 launch isaac_gripper_sim2real real_rviz_with_gripper.launch.py \
#     mode:=real host:=110.120.1.40 port:=12345 model:=e0509 robot_ip:=110.120.1.40

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare

from dsr_bringup2.utils import read_update_rate, show_git_info


def generate_launch_description():
    pkg_share = get_package_share_directory("isaac_gripper_sim2real")
    rviz_config = os.path.join(pkg_share, "config", "rviz_gripper.yaml")
    gripper_config = os.path.join(pkg_share, "config", "rviz_gripper_service.yaml")

    gripper_urdf_path = os.path.join(pkg_share, "urdf", "e0509_with_gripper.urdf")
    with open(gripper_urdf_path, encoding="utf-8") as f:
        gripper_urdf = f.read()

    update_rate = str(read_update_rate())
    show_git_info()
    mode = LaunchConfiguration("mode")

    arguments = [
        DeclareLaunchArgument("name", default_value="dsr01", description="Robot namespace"),
        DeclareLaunchArgument("host", default_value="127.0.0.1", description="Doosan controller IP"),
        DeclareLaunchArgument("port", default_value="12345", description="Doosan controller port"),
        DeclareLaunchArgument("mode", default_value="real", description="virtual or real"),
        DeclareLaunchArgument("model", default_value="e0509", description="Robot model"),
        DeclareLaunchArgument("color", default_value="white", description="Robot mesh color"),
        DeclareLaunchArgument("rt_host", default_value="192.168.137.50", description="RT IP"),
        DeclareLaunchArgument("remap_tf", default_value="false", description="Remap /tf"),
        DeclareLaunchArgument("robot_ip", default_value=LaunchConfiguration("host"), description="Gripper TCP IP"),
        DeclareLaunchArgument("launch_gripper_service", default_value="true", description="Start gripper_service_node"),
        DeclareLaunchArgument("launch_merger", default_value="true", description="Merge arm+gripper joint_states"),
    ]

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("dsr_description2"), "xacro", LaunchConfiguration("model")]
            ),
            ".urdf.xacro",
            " name:=", LaunchConfiguration("name"),
            " host:=", LaunchConfiguration("host"),
            " rt_host:=", LaunchConfiguration("rt_host"),
            " port:=", LaunchConfiguration("port"),
            " mode:=", LaunchConfiguration("mode"),
            " model:=", LaunchConfiguration("model"),
            " update_rate:=", update_rate,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = [
        PathJoinSubstitution(
            [FindPackageShare("dsr_controller2"), "config", "dsr_update_rate.yaml"]
        ),
        PathJoinSubstitution(
            [FindPackageShare("dsr_controller2"), "config", "dsr_controller2.yaml"]
        ),
    ]

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("dsr_description2"), "rviz", "default.rviz"]
    )

    run_emulator_node = Node(
        package="dsr_bringup2",
        executable="run_emulator",
        namespace=LaunchConfiguration("name"),
        parameters=[
            {"name": LaunchConfiguration("name")},
            {"rate": 100},
            {"standby": 5000},
            {"command": True},
            {"host": LaunchConfiguration("host")},
            {"port": LaunchConfiguration("port")},
            {"mode": LaunchConfiguration("mode")},
            {"model": LaunchConfiguration("model")},
            {"gripper": "none"},
            {"mobile": "none"},
            {"rt_host": LaunchConfiguration("rt_host")},
        ],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'virtual'"])),
        output="screen",
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=LaunchConfiguration("name"),
        parameters=[robot_description] + robot_controllers,
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=LaunchConfiguration("name"),
        output="both",
        remappings=[("joint_states", "joint_states_rviz")],
        parameters=[{"robot_description": gripper_urdf}],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        namespace=LaunchConfiguration("name"),
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    original_tf_nodes = GroupAction(
        actions=[robot_state_pub_node],
        condition=UnlessCondition(LaunchConfiguration("remap_tf")),
    )

    remapped_tf_nodes = GroupAction(
        actions=[
            SetRemap(src="/tf", dst="tf"),
            SetRemap(src="/tf_static", dst="tf_static"),
            robot_state_pub_node,
        ],
        condition=IfCondition(LaunchConfiguration("remap_tf")),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration("name"),
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration("name"),
        executable="spawner",
        arguments=["dsr_controller2", "-c", "controller_manager"],
    )

    gripper_service_node = Node(
        package="rh_p12_rna_controller",
        executable="gripper_service_node",
        name="gripper_service_node",
        output="screen",
        parameters=[
            gripper_config,
            {
                "robot_ip": LaunchConfiguration("robot_ip"),
                "command_transport": "tcp",
                "tcp_state_stream_enabled": True,
                "tcp_state_hz": 10.0,
                "state_hz": 10.0,
            },
        ],
        condition=IfCondition(LaunchConfiguration("launch_gripper_service")),
    )

    joint_state_merger_node = Node(
        package="isaac_gripper_sim2real",
        executable="rviz_joint_state_merger",
        name="rviz_joint_state_merger",
        output="screen",
        parameters=[rviz_config, {"robot_ns": LaunchConfiguration("name")}],
        condition=IfCondition(LaunchConfiguration("launch_merger")),
    )

    # dsr_controller2 올라간 뒤 DRL(/dsr01/drl/drl_start) 준비 → gripper TCP 주입
    delay_gripper_after_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[gripper_service_node, joint_state_merger_node],
        )
    )

    delay_rviz_after_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[rviz_node],
        )
    )

    return LaunchDescription(
        arguments
        + [
            run_emulator_node,
            original_tf_nodes,
            remapped_tf_nodes,
            robot_controller_spawner,
            joint_state_broadcaster_spawner,
            control_node,
            delay_gripper_after_controller,
            delay_rviz_after_controller,
        ]
    )
