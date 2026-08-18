# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

from pathlib import Path

import rclpy
from isaac_ros_navigation_goal.goal_generators import RandomGoalGenerator
from isaac_ros_navigation_goal.set_goal import SetNavigationGoal, _normalize_quaternion
from lifecycle_msgs.msg import State
from rclpy.parameter import Parameter


class _AlwaysValidGridMap:
    def get_range(self):
        return [[-1.0, 1.0], [-1.0, 1.0]]

    def is_valid_pose(self, point, distance):
        return True


class _PublishedMessageRecorder:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _LifecycleState:
    id = State.PRIMARY_STATE_ACTIVE
    label = "active"


class _LifecycleResponse:
    current_state = _LifecycleState()


class _LifecycleFuture:
    def done(self):
        return True

    def result(self):
        return _LifecycleResponse()


class _ActiveLifecycleClient:
    def wait_for_service(self, timeout_sec):
        return True

    def call_async(self, request):
        return _LifecycleFuture()


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


def test_lifecycle_state_timeout_default_matches_launch_files():
    rclpy.init(args=_node_args())
    node = SetNavigationGoal()
    try:
        assert node.get_parameter("lifecycle_state_timeout_sec").value == 180.0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_normalize_quaternion_returns_unit_quaternion():
    orientation = _normalize_quaternion([0.0, 0.0, 0.99, 0.02])
    squared_norm = sum(value * value for value in orientation)
    assert abs(squared_norm - 1.0) < 1e-12


def test_initial_pose_publish_normalizes_quaternion():
    rclpy.init(args=_node_args())
    node = SetNavigationGoal()
    recorder = _PublishedMessageRecorder()
    node._SetNavigationGoal__initial_goal_publisher = recorder
    node._SetNavigationGoal__initial_pose = [-6.4, -1.04, 0.0, 0.0, 0.0, 0.99, 0.02]
    try:
        assert node._SetNavigationGoal__send_initial_pose() is True
        orientation = recorder.messages[0].pose.pose.orientation
        squared_norm = (
            orientation.x * orientation.x
            + orientation.y * orientation.y
            + orientation.z * orientation.z
            + orientation.w * orientation.w
        )
        assert abs(squared_norm - 1.0) < 1e-12
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_random_goal_generator_returns_planar_unit_quaternion():
    generator = RandomGoalGenerator(_AlwaysValidGridMap(), 0.2)
    goal = generator.generate_goal()
    orientation = goal[2:6]
    squared_norm = sum(value * value for value in orientation)
    assert orientation[0] == 0.0
    assert orientation[1] == 0.0
    assert abs(squared_norm - 1.0) < 1e-12


def test_lifecycle_wait_is_skipped_after_active_state_seen():
    rclpy.init(args=_node_args())
    node = SetNavigationGoal()
    node._SetNavigationGoal__is_navigation_lifecycle_active = True
    node.create_client = lambda *args, **kwargs: (_ for _ in ()).throw(
        AssertionError("create_client should not be called")
    )
    try:
        assert node._SetNavigationGoal__wait_for_navigation_lifecycle_active() is True
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_lifecycle_wait_destroys_state_client():
    rclpy.init(args=_node_args())
    node = SetNavigationGoal()
    state_client = _ActiveLifecycleClient()
    destroyed_clients = []
    node.create_client = lambda *args, **kwargs: state_client
    node.destroy_client = lambda client: destroyed_clients.append(client) or True
    try:
        assert node._SetNavigationGoal__wait_for_navigation_lifecycle_active() is True
        assert destroyed_clients == [state_client]
    finally:
        node.destroy_node()
        rclpy.shutdown()
