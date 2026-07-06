# SPDX-License-Identifier: Apache-2.0
"""External ROS2 stack for topic-based ros2_control benchmark.

Launches:
  1. controller_manager with TopicBasedSystem (bridges to Isaac Sim via topics)
  2. joint_state_broadcaster spawner
  3. scaled_joint_trajectory_controller spawner (delayed 2 s to let JSB activate first)
  4. benchmark_trajectory_publisher — sinusoidal goal publisher

Run alongside:
  python benchmark_ros2_control.py --mode topic_based
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("isaac_ros2_control_demo")
    urdf_path = os.path.join(pkg, "urdf", "ur10_topic_based.urdf")
    controllers_yaml = os.path.join(pkg, "config", "ur10_topic_based_controllers.yaml")

    with open(urdf_path, "r") as f:
        robot_description = f.read()

    # robot_state_publisher publishes the URDF on /robot_description (latched)
    # so ros2_control_node can subscribe to it during initialisation.
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            controllers_yaml,
        ],
        output="screen",
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # Delay JTC spawner so joint_state_broadcaster is active first.
    jtc_spawner = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["scaled_joint_trajectory_controller"],
                output="screen",
            )
        ],
    )

    traj_publisher = Node(
        package="isaac_ros2_control_demo",
        executable="benchmark_trajectory_publisher",
        output="screen",
    )

    return LaunchDescription([
        rsp,
        controller_manager,
        jsb_spawner,
        jtc_spawner,
        traj_publisher,
    ])
