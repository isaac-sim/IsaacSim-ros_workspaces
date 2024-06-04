#!/usr/bin/env python3

# Copyright (c) 2020-2024, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('test_ackermann')
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'ackermann_cmd', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = AckermannDriveStamped()
        
        # Only command forklift using acceleration and steering angle
        degrees1 = np.arange(0, 60)
        degrees2 = np.arange(-60, 0)
        degrees = np.concatenate((degrees1, degrees1[::-1], degrees2[::-1], degrees2))
        msg.header.frame_id = "forklift"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.steering_angle =  0.0174533 * (degrees[self.i % degrees.size])
        
        msg.drive.acceleration = float(degrees[self.i % degrees.size])
        
        self.publisher_.publish(msg)
        self.i += 1


def main(args=None):

    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
