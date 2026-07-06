# SPDX-License-Identifier: Apache-2.0
"""MoveIt 2 and RViz against Isaac Sim's in-process ControllerManager."""

import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


ROBOT_DESCRIPTION_TIMEOUT = 60.0  # First-run asset download can take 30s+.


def _fetch_latched_urdf(timeout: float) -> str:
    import rclpy
    from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
    from std_msgs.msg import String

    rclpy.init()
    try:
        node = rclpy.create_node("isaac_ros2_control_demo_urdf_reader")
        qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        holder = {"urdf": None}
        node.create_subscription(String, "/robot_description", lambda m: holder.update(urdf=m.data), qos)
        node.get_logger().info(f"Waiting up to {timeout:.0f}s for /robot_description from Isaac Sim...")
        deadline = time.time() + timeout
        while holder["urdf"] is None and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)
        node.destroy_node()
    finally:
        rclpy.shutdown()

    if holder["urdf"] is None:
        raise RuntimeError(
            "Timed out waiting for /robot_description. Is Isaac Sim running and at Play?"
        )
    return holder["urdf"]


def _build_nodes(context, *args, **kwargs):
    pkg_share = get_package_share_directory("isaac_ros2_control_demo")
    srdf_path = os.path.join(pkg_share, "srdf", "ur10.srdf")
    kinematics_yaml = os.path.join(pkg_share, "config", "kinematics.yaml")
    ompl_yaml = os.path.join(pkg_share, "config", "ompl_planning.yaml")
    joint_limits_yaml = os.path.join(pkg_share, "config", "joint_limits.yaml")
    moveit_controllers_yaml = os.path.join(pkg_share, "config", "moveit_controllers.yaml")
    rviz_config = os.path.join(pkg_share, "rviz", "ur10_moveit.rviz")

    urdf = _fetch_latched_urdf(ROBOT_DESCRIPTION_TIMEOUT)
    with open(srdf_path) as f:
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
            kinematics_yaml, ompl_yaml, joint_limits_yaml, moveit_controllers_yaml,
            use_sim_time,
        ],
    )
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, use_sim_time],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[robot_description, robot_description_semantic, kinematics_yaml, use_sim_time],
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
    return [rsp, move_group, rviz, *spawners]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        OpaqueFunction(function=_build_nodes),
    ])
