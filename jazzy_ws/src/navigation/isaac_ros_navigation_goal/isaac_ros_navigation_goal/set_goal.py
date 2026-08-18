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

import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

from .goal_generators import GoalReader, RandomGoalGenerator
from .obstacle_map import GridMap


def _normalize_quaternion(quaternion):
    if len(quaternion) != 4:
        raise ValueError("Quaternion must contain exactly 4 values")

    if not all(math.isfinite(value) for value in quaternion):
        raise ValueError("Quaternion contains a non-finite value")

    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm <= 0.0:
        raise ValueError("Quaternion norm must be greater than zero")

    return [value / norm for value in quaternion]


class SetNavigationGoal(Node):
    def __init__(self):
        super().__init__("set_navigation_goal")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("iteration_count", 1),
                ("goal_generator_type", "RandomGoalGenerator"),
                ("action_server_name", "navigate_to_pose"),
                ("obstacle_search_distance_in_meters", 0.2),
                ("frame_id", "map"),
                ("map_yaml_path", rclpy.Parameter.Type.STRING),
                ("goal_text_file_path", rclpy.Parameter.Type.STRING),
                ("initial_pose", rclpy.Parameter.Type.DOUBLE_ARRAY),
                ("action_server_timeout_sec", 60.0),
                ("initial_pose_subscriber_timeout_sec", 60.0),
                ("initial_pose_settle_time_sec", 10.0),
                ("lifecycle_node_name", "bt_navigator"),
                ("lifecycle_state_timeout_sec", 180.0),
            ],
        )

        self.__goal_generator = self.__create_goal_generator()
        action_server_name = self.get_parameter("action_server_name").value
        self._action_client = ActionClient(self, NavigateToPose, action_server_name)

        self.MAX_ITERATION_COUNT = self.get_parameter("iteration_count").value
        assert self.MAX_ITERATION_COUNT > 0
        self.curr_iteration_count = 1

        # A relative topic keeps the publisher inside this node's namespace for
        # multi-robot launches (for example, /carter1/initialpose).
        self.__initial_goal_publisher = self.create_publisher(PoseWithCovarianceStamped, "initialpose", 1)

        self.__initial_pose = self.get_parameter("initial_pose").value
        self.__is_initial_pose_sent = True if self.__initial_pose is None else False
        self.__is_navigation_lifecycle_active = False

    def __send_initial_pose(self):
        """
        Publishes the initial pose.
        This function is only called once that too before sending any goal pose
        to the mission server.
        """
        goal = PoseWithCovarianceStamped()
        goal.header.frame_id = self.get_parameter("frame_id").value
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.__initial_pose[0]
        goal.pose.pose.position.y = self.__initial_pose[1]
        goal.pose.pose.position.z = self.__initial_pose[2]
        try:
            orientation = _normalize_quaternion(self.__initial_pose[3:7])
        except ValueError as exc:
            self.get_logger().error(f"Invalid initial pose orientation: {exc}")
            return False

        goal.pose.pose.orientation.x = orientation[0]
        goal.pose.pose.orientation.y = orientation[1]
        goal.pose.pose.orientation.z = orientation[2]
        goal.pose.pose.orientation.w = orientation[3]
        self.__initial_goal_publisher.publish(goal)
        return True

    def __wait_for_navigation_lifecycle_active(self):
        if self.__is_navigation_lifecycle_active:
            return True

        lifecycle_node_name = self.get_parameter("lifecycle_node_name").value
        lifecycle_state_timeout = self.get_parameter("lifecycle_state_timeout_sec").value
        if not lifecycle_node_name or lifecycle_state_timeout <= 0.0:
            self.__is_navigation_lifecycle_active = True
            return True

        service_name = lifecycle_node_name.rstrip("/") + "/get_state"
        state_client = self.create_client(GetState, service_name)
        try:
            deadline = time.monotonic() + lifecycle_state_timeout
            self.get_logger().info(
                f"Waiting up to {lifecycle_state_timeout:.1f}s for " f"{lifecycle_node_name} to become active"
            )

            last_state_label = None
            while time.monotonic() < deadline:
                if not state_client.wait_for_service(timeout_sec=0.5):
                    continue

                future = state_client.call_async(GetState.Request())
                while not future.done() and time.monotonic() < deadline:
                    rclpy.spin_once(self, timeout_sec=0.1)

                if not future.done():
                    break

                response = future.result()
                if response is None:
                    continue

                current_state = response.current_state
                if current_state.id == State.PRIMARY_STATE_ACTIVE:
                    self.get_logger().info(f"{lifecycle_node_name} is active")
                    self.__is_navigation_lifecycle_active = True
                    return True

                if current_state.label != last_state_label:
                    self.get_logger().info(f"{lifecycle_node_name} is {current_state.label}; waiting for active")
                    last_state_label = current_state.label

                time.sleep(0.5)

            self.get_logger().error(f"{lifecycle_node_name} did not become active before the timeout")
            return False
        finally:
            self.destroy_client(state_client)

    def send_goal(self):
        """
        Sends the goal to the action server.
        """

        action_server_timeout = self.get_parameter("action_server_timeout_sec").value
        self.get_logger().info(f"Waiting up to {action_server_timeout:.1f}s for the navigation action server")
        if not self._action_client.wait_for_server(timeout_sec=action_server_timeout):
            self.get_logger().error("Navigation action server did not become ready before the timeout")
            rclpy.shutdown()
            return False

        if not self.__wait_for_navigation_lifecycle_active():
            rclpy.shutdown()
            return False

        if not self.__is_initial_pose_sent:
            subscriber_timeout = self.get_parameter("initial_pose_subscriber_timeout_sec").value
            deadline = time.monotonic() + subscriber_timeout
            while self.__initial_goal_publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
            if self.__initial_goal_publisher.get_subscription_count() == 0:
                self.get_logger().error("No initial pose subscriber became ready before the timeout")
                rclpy.shutdown()
                return False

            self.get_logger().info("Sending initial pose")
            if not self.__send_initial_pose():
                rclpy.shutdown()
                return False
            self.__is_initial_pose_sent = True

            # Preserve the existing localization settling period, but make it
            # configurable now that this node owns launch readiness.
            time.sleep(self.get_parameter("initial_pose_settle_time_sec").value)
            self.get_logger().info("Sending first goal")

        goal_msg = self.__get_goal()

        if goal_msg is None:
            rclpy.shutdown()
            return False

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.__feedback_callback
        )
        self._send_goal_future.add_done_callback(self.__goal_response_callback)
        return True

    def __goal_response_callback(self, future):
        """
        Callback function to check the response(goal accpted/rejected) from the server.\n
        If the Goal is rejected it stops the execution for now.(We can change to resample the pose if rejected.)
        """

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            rclpy.shutdown()
            return

        self.get_logger().info("Goal accepted :)")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.__get_result_callback)

    def __get_goal(self):
        """
        Get the next goal from the goal generator.

        Returns
        -------
        [NavigateToPose][goal] or None if the next goal couldn't be generated.

        """

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.get_parameter("frame_id").value
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        pose = self.__goal_generator.generate_goal()

        # couldn't sample a pose which is not close to obstacles. Rare but might happen in dense maps.
        if pose is None:
            self.get_logger().error(
                "Could not generate next goal. Returning. Possible reasons for this error could be:"
            )
            self.get_logger().error(
                "1. If you are using GoalReader then please make sure iteration count <= number of goals avaiable in file."
            )
            self.get_logger().error(
                "2. If RandomGoalGenerator is being used then it was not able to sample a pose which is given distance away from the obstacles."
            )
            return

        try:
            orientation = _normalize_quaternion(pose[2:6])
        except ValueError as exc:
            self.get_logger().error(f"Generated goal has invalid orientation: {exc}")
            return

        self.get_logger().info("Generated goal pose: {0}".format(pose))
        goal_msg.pose.pose.position.x = pose[0]
        goal_msg.pose.pose.position.y = pose[1]
        goal_msg.pose.pose.orientation.x = orientation[0]
        goal_msg.pose.pose.orientation.y = orientation[1]
        goal_msg.pose.pose.orientation.z = orientation[2]
        goal_msg.pose.pose.orientation.w = orientation[3]
        return goal_msg

    def __get_result_callback(self, future):
        """
        Callback to check result.\n
        It calls the send_goal() function in case current goal sent count < required goals count.
        """
        # Nav2 is sending empty message for success as well as for failure.
        result = future.result().result
        self.get_logger().info("Result: {0}".format(result))

        if self.curr_iteration_count < self.MAX_ITERATION_COUNT:
            self.curr_iteration_count += 1
            self.send_goal()
        else:
            rclpy.shutdown()

    def __feedback_callback(self, feedback_msg):
        """
        This is feeback callback. We can compare/compute/log while the robot is on its way to goal.
        """
        # self.get_logger().info('FEEDBACK: {}\n'.format(feedback_msg))
        pass

    def __create_goal_generator(self):
        """
        Creates the GoalGenerator object based on the specified ros param value.
        """

        goal_generator_type = self.get_parameter("goal_generator_type").value
        goal_generator = None
        if goal_generator_type == "RandomGoalGenerator":
            if self.get_parameter("map_yaml_path").value is None:
                self.get_logger().info("Yaml file path is not given. Returning..")
                sys.exit(1)

            yaml_file_path = self.get_parameter("map_yaml_path").value
            grid_map = GridMap(yaml_file_path)
            obstacle_search_distance_in_meters = self.get_parameter("obstacle_search_distance_in_meters").value
            assert obstacle_search_distance_in_meters > 0
            goal_generator = RandomGoalGenerator(grid_map, obstacle_search_distance_in_meters)

        elif goal_generator_type == "GoalReader":
            if self.get_parameter("goal_text_file_path").value is None:
                self.get_logger().info("Goal text file path is not given. Returning..")
                sys.exit(1)

            file_path = self.get_parameter("goal_text_file_path").value
            goal_generator = GoalReader(file_path)
        else:
            self.get_logger().info("Invalid goal generator specified. Returning...")
            sys.exit(1)
        return goal_generator


def main():
    rclpy.init()
    set_goal = SetNavigationGoal()
    if not set_goal.send_goal():
        set_goal.destroy_node()
        return 1

    rclpy.spin(set_goal)
    set_goal.destroy_node()
    return 0


if __name__ == "__main__":
    sys.exit(main())
