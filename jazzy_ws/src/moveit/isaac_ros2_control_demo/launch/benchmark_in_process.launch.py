# SPDX-License-Identifier: Apache-2.0
"""External ROS2 stack for in-process ros2_control benchmark.

The controller_manager is hosted in-process by Isaac Sim. This launch file
spawns controllers against it and starts the trajectory publisher.

Run alongside:
  python benchmark_ros2_control.py --mode in_process
"""
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
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

    return LaunchDescription([jsb_spawner, jtc_spawner, traj_publisher])
