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

import signal
import sys

import numpy as np

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('test_ackermann')
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'ackermann_cmd', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = AckermannDriveStamped()

        # Only command ackermann drive robot using speed and steering angle
        degrees1 = np.arange(0, 60)
        degrees2 = np.arange(-60, 0)
        degrees = np.concatenate((degrees1, degrees1[::-1], degrees2[::-1], degrees2))
        msg.header.frame_id = "ackermann"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.steering_angle =  0.0174533 * (degrees[self.i % degrees.size])

        msg.drive.speed = float(degrees[self.i % degrees.size])
        msg.drive.acceleration = 1.0
        self.publisher_.publish(msg)
        self.i += 1

    def stop_robot(self):
        msg = AckermannDriveStamped()

        # Publish zero-velocity message
        msg.drive.steering_angle = 0.0
        msg.drive.speed = 0.0
        self.publisher_.publish(msg)  


def main(args=None):

    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    def signal_handler(sig, frame):
        minimal_publisher.stop_robot()
        minimal_publisher.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    # Register the SIGINT handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)

    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
