#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0

"""Launch the UR10 MoveIt demo with its in-process controller manager."""

# Python is required because XML cannot start actions from OnProcessExit.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.logging import get_logger
from launch.substitutions import (
    AnonName,
    FileContent,
    LaunchConfiguration,
    LaunchLogDir,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

_LOGGER = get_logger(__name__)


def generate_launch_description():
    package_share = FindPackageShare("isaac_ros2_control_demo")
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    robot_description_semantic = ParameterValue(
        FileContent(PathJoinSubstitution([package_share, "srdf", "ur10.srdf"])),
        value_type=str,
    )
    robot_description_parameter_file = PathJoinSubstitution(
        [LaunchLogDir(), ["robot_description-", AnonName("ur10"), ".params.yaml"]]
    )

    create_robot_description = Node(
        package="isaac_ros2_control_demo",
        executable="create_robot_description_node",
        output="screen",
        arguments=[
            "--topic",
            LaunchConfiguration("robot_description_topic"),
            "--log-interval",
            LaunchConfiguration("robot_description_log_interval"),
            "--output-file",
            robot_description_parameter_file,
            "--input-file",
            LaunchConfiguration("robot_description_file"),
        ],
    )

    def config_file(name):
        return PathJoinSubstitution([package_share, "config", name])

    dependent_nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[{"use_sim_time": use_sim_time}, robot_description_parameter_file],
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                {
                    "robot_description_semantic": robot_description_semantic,
                    "use_sim_time": use_sim_time,
                },
                config_file("kinematics.yaml"),
                config_file("ompl_planning.yaml"),
                config_file("joint_limits.yaml"),
                config_file("moveit_controllers.yaml"),
                robot_description_parameter_file,
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="log",
            arguments=["-d", PathJoinSubstitution([package_share, "rviz", "ur10_moveit.rviz"])],
            parameters=[
                {
                    "robot_description_semantic": robot_description_semantic,
                    "use_sim_time": use_sim_time,
                },
                config_file("kinematics.yaml"),
                robot_description_parameter_file,
            ],
        ),
        Node(package="isaac_ros2_control_demo", executable="add_floor", output="log"),
        Node(
            package="controller_manager",
            executable="spawner",
            output="log",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            output="log",
            arguments=["scaled_joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        ),
    ]

    def start_dependent_nodes(event, _context):
        if event.returncode == 0:
            return dependent_nodes
        reason = f"create_robot_description_node exited with code {event.returncode}"
        _LOGGER.error(reason)
        return [Shutdown(reason=reason)]

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true", description="Use the simulation clock"),
            DeclareLaunchArgument(
                "robot_description_file",
                default_value="",
                description=(
                    "Optional URDF file. When empty, wait for robot_description_topic with transient-local reliable "
                    "QoS."
                ),
            ),
            DeclareLaunchArgument(
                "robot_description_topic",
                default_value="/robot_description",
                description="Isaac Sim std_msgs/String topic containing the URDF",
            ),
            DeclareLaunchArgument(
                "robot_description_log_interval",
                default_value="60.0",
                description="Seconds between messages while waiting for Isaac Sim's robot description",
            ),
            RegisterEventHandler(OnProcessExit(target_action=create_robot_description, on_exit=start_dependent_nodes)),
            create_robot_description,
        ]
    )
