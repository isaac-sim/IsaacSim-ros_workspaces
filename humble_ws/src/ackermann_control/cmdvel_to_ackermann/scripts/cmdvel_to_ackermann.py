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

import math

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdvelToAckermann(Node):
    """Subscribe to the Twist message and converts it to AckermannDrive message and publish.
    """

    def __init__(self):
        super().__init__('cmdvel_to_ackermann')

        self.declare_parameter('publish_period_ms', 20)
        self.declare_parameter('track_width', 0.24)
        self.declare_parameter('acceleration', 0.0)
        self.declare_parameter('steering_velocity', 0.0)
        
        self._cmd_vel_subscription = self.create_subscription(Twist, '/cmd_vel',
                                                              self._cmd_vel_callback, 1)
        self._ackermann_publisher = self.create_publisher(AckermannDriveStamped, '/ackermann_cmd',
                                                          1)
        publish_period_ms = self.get_parameter(
            'publish_period_ms').value / 1000
        self.create_timer(publish_period_ms, self._timer_callback)
        self.track_width = self.get_parameter("track_width").value
        self.acceleration = self.get_parameter("acceleration").value
        self.steering_velocity = self.get_parameter("steering_velocity").value

        self.get_logger().info(f"track_width: {self.track_width}")
        self.get_logger().info(f"acceleration: {self.acceleration}")
        self.get_logger().info(f"steering_velocity: {self.steering_velocity}")
        self._ackermann_msg = None
        

    def _convert_trans_rot_vel_to_steering_angle(self, v, omega) -> float:
        if omega == 0 or v == 0:
            if omega != 0:
                self.get_logger().warn(
                    f'Invalid command for ackermann drive with zero vel {v} but non zero '
                    f'omega {omega}')
            return 0.0

        turning_radius = v / omega
        return math.atan(self.track_width / turning_radius)

    def _cmd_vel_callback(self, msg):
        self._ackermann_msg = AckermannDriveStamped()
        self._ackermann_msg.header.stamp = self.get_clock().now().to_msg()
        # Conversion logic (simplified example)
        self._ackermann_msg.drive.speed = msg.linear.x
        steering_angle = self._convert_trans_rot_vel_to_steering_angle(
            self._ackermann_msg.drive.speed, msg.angular.z)
        self._ackermann_msg.drive.steering_angle = steering_angle
        self._ackermann_msg.drive.acceleration = self.acceleration
        self._ackermann_msg.drive.steering_angle_velocity = self.steering_velocity

    def _timer_callback(self):
        if self._ackermann_msg:
            self._ackermann_publisher.publish(self._ackermann_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdvelToAckermann()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
