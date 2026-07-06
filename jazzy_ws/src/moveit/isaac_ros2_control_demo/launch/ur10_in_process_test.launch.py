# SPDX-License-Identifier: Apache-2.0
"""Headless plan-and-execute test for the in-process ControllerManager demo."""

import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROBOT_DESCRIPTION_TIMEOUT = 60.0


def _fetch_latched_urdf(timeout: float) -> str:
    import rclpy
    from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
    from std_msgs.msg import String

    rclpy.init()
    try:
        node = rclpy.create_node("ur10_test_urdf_reader")
        qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        holder = {"urdf": None}
        node.create_subscription(String, "/robot_description", lambda m: holder.update(urdf=m.data), qos)
        deadline = time.monotonic() + timeout
        while holder["urdf"] is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)
        node.destroy_node()
    finally:
        rclpy.shutdown()
    if holder["urdf"] is None:
        raise RuntimeError("Timed out waiting for /robot_description. Is the Isaac Sim half running and at Play?")
    return holder["urdf"]


def _build_nodes(context, *args, **kwargs):
    pkg_share = get_package_share_directory("isaac_ros2_control_demo")

    def cfg(*parts):
        return os.path.join(pkg_share, *parts)

    urdf = _fetch_latched_urdf(ROBOT_DESCRIPTION_TIMEOUT)
    with open(cfg("srdf", "ur10.srdf")) as f:
        srdf = f.read()

    robot_description = {"robot_description": urdf}
    robot_description_semantic = {"robot_description_semantic": srdf}
    use_sim_time = {"use_sim_time": LaunchConfiguration("use_sim_time").perform(context) == "true"}

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description, robot_description_semantic,
            cfg("config", "kinematics.yaml"), cfg("config", "ompl_planning.yaml"),
            cfg("config", "joint_limits.yaml"), cfg("config", "moveit_controllers.yaml"),
            use_sim_time,
        ],
    )
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="log",
        parameters=[robot_description, use_sim_time],
    )
    spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            output="log",
            arguments=[name, "--controller-manager", "/controller_manager"],
        )
        for name in ("joint_state_broadcaster", "scaled_joint_trajectory_controller")
    ]
    motion_test = Node(
        package="isaac_ros2_control_demo",
        executable="ur10_motion_test",
        output="screen",
        parameters=[use_sim_time],
    )
    shutdown_on_test_exit = RegisterEventHandler(
        OnProcessExit(target_action=motion_test, on_exit=_shutdown_after_test)
    )
    return [rsp, move_group, *spawners, motion_test, shutdown_on_test_exit]


def _shutdown_after_test(event, _context):
    if event.returncode:
        raise RuntimeError(f"UR10 motion test exited with code {event.returncode}")
    return [EmitEvent(event=Shutdown())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        OpaqueFunction(function=_build_nodes),
    ])
