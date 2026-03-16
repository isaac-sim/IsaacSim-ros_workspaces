#!/usr/bin/env python3

# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Direct bridge: MoveIt GripperCommand <-> Isaac Sim joint commands/states
for panda_finger_joint1 and panda_finger_joint2.

Handles both command forwarding and state feedback so MoveIt sees
accurate finger positions without ros2_control managing the hand.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from control_msgs.action import GripperCommand
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from sensor_msgs.msg import JointState
import threading
import time

FINGER_JOINTS = ['panda_finger_joint1', 'panda_finger_joint2']
OPEN_POSITION = 0.04
GOAL_TOLERANCE = 0.005
GOAL_TIMEOUT = 5.0


class GripperToIsaac(Node):
    def __init__(self):
        super().__init__('gripper_to_isaac')

        self.get_logger().info('Starting Gripper to Isaac bridge')

        self.cb_group = ReentrantCallbackGroup()

        self.target_position = OPEN_POSITION
        self.current_position = OPEN_POSITION
        self.position_lock = threading.Lock()

        self.isaac_cmd_pub = self.create_publisher(
            JointState, '/isaac_joint_commands', 10
        )

        self.joint_state_pub = self.create_publisher(
            JointState, '/joint_states', 10
        )

        self.create_subscription(
            JointState, '/isaac_joint_states',
            self._isaac_state_cb, 10,
            callback_group=self.cb_group
        )

        self.create_timer(
            0.05, self._publish_command_tick,
            callback_group=self.cb_group
        )

        self._action_server = ActionServer(
            self,
            GripperCommand,
            '/panda_hand_controller/gripper_cmd',
            self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self.cb_group
        )

        self.get_logger().info('Gripper action server ready!')

    def _isaac_state_cb(self, msg: JointState):
        """Read finger positions from Isaac Sim and republish on /joint_states."""
        finger_pos = {}
        for name, pos in zip(msg.name, msg.position):
            if name in FINGER_JOINTS:
                finger_pos[name] = pos

        if not finger_pos:
            return

        with self.position_lock:
            if FINGER_JOINTS[0] in finger_pos:
                self.current_position = finger_pos[FINGER_JOINTS[0]]

        state_msg = JointState()
        state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.name = list(finger_pos.keys())
        state_msg.position = list(finger_pos.values())
        self.joint_state_pub.publish(state_msg)

    def _publish_command_tick(self):
        with self.position_lock:
            pos = self.target_position

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = FINGER_JOINTS
        msg.position = [pos, pos]
        self.isaac_cmd_pub.publish(msg)

    def _goal_cb(self, goal_request):
        self.get_logger().info(
            f'Received goal: position={goal_request.command.position:.4f}'
        )
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        self.get_logger().info('Cancel requested')
        return CancelResponse.ACCEPT

    def _execute_cb(self, goal_handle: ServerGoalHandle):
        target = goal_handle.request.command.position
        self.get_logger().info(f'Executing gripper command: {target:.4f}')

        with self.position_lock:
            self.target_position = target

        deadline = time.monotonic() + GOAL_TIMEOUT
        canceled = False
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                canceled = True
                break
            with self.position_lock:
                if abs(self.current_position - target) < GOAL_TOLERANCE:
                    break
            time.sleep(0.05)

        with self.position_lock:
            reached = abs(self.current_position - target) < GOAL_TOLERANCE
            final_pos = self.current_position

        if canceled:
            self.get_logger().info(f'Gripper goal canceled (current={final_pos:.4f})')
            goal_handle.canceled()
        elif reached:
            self.get_logger().info(f'Gripper reached target ({final_pos:.4f})')
            goal_handle.succeed()
        else:
            self.get_logger().warn(
                f'Gripper timed out (current={final_pos:.4f}, target={target:.4f})'
            )
            goal_handle.abort()

        result = GripperCommand.Result()
        result.position = final_pos
        result.reached_goal = reached
        result.stalled = not reached
        return result


def main(args=None):
    rclpy.init(args=args)
    node = GripperToIsaac()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
