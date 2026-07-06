#!/usr/bin/python3
# SPDX-License-Identifier: Apache-2.0
"""Plan, execute, and verify one UR10 motion through MoveIt."""

import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, PlanningOptions
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

GROUP = "ur_manipulator"
TARGET = {
    "shoulder_pan_joint": 0.0,
    "shoulder_lift_joint": -1.5708,
    "elbow_joint": 0.0,
    "wrist_1_joint": -1.5708,
    "wrist_2_joint": 0.0,
    "wrist_3_joint": 0.0,
}
TOLERANCE = 0.05  # rad


class MotionTest(Node):
    def __init__(self):
        super().__init__("ur10_motion_test")
        self._client = ActionClient(self, MoveGroup, "move_action")
        self._state = {}
        self.create_subscription(JointState, "joint_states", self._on_state, 10)

    def _on_state(self, msg):
        self._state = dict(zip(msg.name, msg.position))

    def _spin(self, seconds):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def _goal(self):
        request = MotionPlanRequest()
        request.group_name = GROUP
        request.num_planning_attempts = 10
        request.allowed_planning_time = 10.0
        request.max_velocity_scaling_factor = 0.3
        request.max_acceleration_scaling_factor = 0.3
        constraints = Constraints()
        for name, value in TARGET.items():
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = value
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        request.goal_constraints.append(constraints)
        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options = PlanningOptions()
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        return goal

    def run(self):
        if not self._client.wait_for_server(timeout_sec=90.0):
            self.get_logger().error("move_group action /move_action did not appear")
            return False
        deadline = time.monotonic() + 30.0
        while rclpy.ok() and not self._state and time.monotonic() < deadline:
            self._spin(0.1)
        if not self._state:
            self.get_logger().error("joint_states did not appear")
            return False

        trajectory = ActionClient(self, FollowJointTrajectory, "scaled_joint_trajectory_controller/follow_joint_trajectory")
        if not trajectory.wait_for_server(timeout_sec=60.0):
            self.get_logger().error("scaled_joint_trajectory_controller is not active")
            return False

        self.get_logger().info("planning and executing to the home configuration...")
        send = self._client.send_goal_async(self._goal())
        rclpy.spin_until_future_complete(self, send, timeout_sec=10.0)
        if not send.done():
            self.get_logger().error("MoveGroup goal request timed out")
            return False
        handle = send.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("MoveGroup goal was rejected")
            return False
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result, timeout_sec=60.0)
        if not result.done():
            self.get_logger().error("MoveGroup execution timed out")
            return False
        wrapped = result.result()
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or wrapped.result.error_code.val != 1:
            self.get_logger().error(
                f"MoveGroup failed (status={wrapped.status}, error_code={wrapped.result.error_code.val})"
            )
            return False

        self._spin(1.0)
        errors = {n: abs(self._state.get(n, 1e9) - v) for n, v in TARGET.items()}
        worst = max(errors.values())
        if worst > TOLERANCE:
            self.get_logger().error(f"arm did not reach the goal (max joint error {worst:.4f} rad)")
            return False
        self.get_logger().info(f"arm reached the goal (max joint error {worst:.4f} rad)")
        return True


def main():
    rclpy.init()
    node = MotionTest()
    try:
        ok = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print("DEMO SELF-TEST: PASS" if ok else "DEMO SELF-TEST: FAIL", flush=True)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
