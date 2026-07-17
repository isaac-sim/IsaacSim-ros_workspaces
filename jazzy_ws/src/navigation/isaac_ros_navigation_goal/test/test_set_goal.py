# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

from pathlib import Path

import rclpy
from isaac_ros_navigation_goal.set_goal import SetNavigationGoal
from rclpy.parameter import Parameter


def _node_args(namespace=""):
    goals = Path(__file__).parents[1] / "assets" / "goals.txt"
    args = [
        "--ros-args",
        "-p",
        "goal_generator_type:=GoalReader",
        "-p",
        f"goal_text_file_path:={goals}",
        "-p",
        "initial_pose:=[0.0,0.0,0.0,0.0,0.0,0.0,1.0]",
    ]
    if namespace:
        args.extend(["-r", f"__ns:=/{namespace}"])
    return args


def test_initial_pose_topic_follows_node_namespace():
    rclpy.init(args=_node_args("carter1"))
    node = SetNavigationGoal()
    try:
        publisher = node._SetNavigationGoal__initial_goal_publisher
        assert publisher.topic_name == "/carter1/initialpose"
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_action_server_wait_is_bounded():
    rclpy.init(args=_node_args())
    node = SetNavigationGoal()
    node.set_parameters([Parameter("action_server_timeout_sec", value=0.0)])
    try:
        assert node.send_goal() is False
        assert not rclpy.ok()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_use_sim_time_activates_ros_clock():
    args = _node_args()
    args.extend(["-p", "use_sim_time:=true"])
    rclpy.init(args=args)
    node = SetNavigationGoal()
    try:
        assert node.get_parameter("use_sim_time").value is True
        assert node.get_clock().ros_time_is_active
    finally:
        node.destroy_node()
        rclpy.shutdown()
