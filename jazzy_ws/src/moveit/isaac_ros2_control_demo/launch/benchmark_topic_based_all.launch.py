# SPDX-License-Identifier: Apache-2.0
"""All-in-one benchmark launch for topic_based mode.

Single terminal:
  ros2 launch isaac_ros2_control_demo benchmark_topic_based_all.launch.py

Optional:
  ... install_path:=/path/to/_build/linux-x86_64/release
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

BENCHMARK_SCRIPT = (
    "/home/aayoung/Pseudos/nvidia/isaac-sim/omni_isaac_sim_ros2_control"
    "/source/standalone_examples/testing/isaacsim.ros2.control/benchmark_ros2_control.py"
)
DEFAULT_INSTALL_PATH = (
    "/home/aayoung/Pseudos/nvidia/isaac-sim/omni_isaac_sim_ros2_control"
    "/_build/linux-x86_64/release"
)

STANDALONE_ARGS = f"{BENCHMARK_SCRIPT} --mode topic_based --wait 30 --warmup 30 --steps 500"


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("isaac_ros2_control_demo")
    urdf_path = os.path.join(pkg, "urdf", "ur10_topic_based.urdf")
    controllers_yaml = os.path.join(pkg, "config", "ur10_topic_based_controllers.yaml")
    with open(urdf_path) as f:
        robot_description = f.read()

    isaac_sim = Node(
        package="isaacsim_bringup",
        executable="run_isaacsim",
        name="isaacsim_bringup",
        output="screen",
        parameters=[{
            "install_path": LaunchConfiguration("install_path"),
            "standalone": STANDALONE_ARGS,
        }],
    )

    # robot_state_publisher + ros2_control_node start immediately so the
    # external CM gets /robot_description before it times out.
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description}, controllers_yaml],
        output="screen",
    )

    # Spawn controllers once the external CM is ready (~5s).
    jsb_spawner = TimerAction(period=5.0, actions=[Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster"], output="screen",
    )])
    jtc_spawner = TimerAction(period=7.0, actions=[Node(
        package="controller_manager", executable="spawner",
        arguments=["scaled_joint_trajectory_controller"], output="screen",
    )])
    traj_publisher = TimerAction(period=7.0, actions=[Node(
        package="isaac_ros2_control_demo",
        executable="benchmark_trajectory_publisher",
        output="screen",
    )])

    shutdown_on_exit = RegisterEventHandler(
        OnProcessExit(target_action=isaac_sim, on_exit=[Shutdown()])
    )

    return LaunchDescription([
        DeclareLaunchArgument("install_path", default_value=DEFAULT_INSTALL_PATH,
                              description="Isaac Sim _build release dir"),
        isaac_sim,
        rsp,
        controller_manager,
        jsb_spawner,
        jtc_spawner,
        traj_publisher,
        shutdown_on_exit,
    ])
