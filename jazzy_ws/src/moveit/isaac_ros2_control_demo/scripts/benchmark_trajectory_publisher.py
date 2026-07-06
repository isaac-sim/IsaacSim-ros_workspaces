#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Publishes sinusoidal JointTrajectory goals at 10 Hz for benchmark purposes.

Sends all 6 UR10 joints through ±0.3 rad sinusoids with 4-second period.
The same script is used for both benchmark modes; the controller name and
topic are the same in both cases.
"""
import math
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
AMPLITUDE = 0.3   # rad
PERIOD = 4.0      # seconds
TOPIC = "/scaled_joint_trajectory_controller/joint_trajectory"
RATE_HZ = 10


class BenchmarkTrajectoryPublisher(Node):
    def __init__(self) -> None:
        super().__init__("benchmark_trajectory_publisher")
        self._pub = self.create_publisher(JointTrajectory, TOPIC, 10)
        self._t0 = time.monotonic()
        self.create_timer(1.0 / RATE_HZ, self._publish)
        self.get_logger().info(f"Publishing sinusoidal goals on {TOPIC} at {RATE_HZ} Hz")

    def _publish(self) -> None:
        t = time.monotonic() - self._t0
        pos = AMPLITUDE * math.sin(2.0 * math.pi * t / PERIOD)
        msg = JointTrajectory()
        msg.joint_names = JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [pos] * len(JOINTS)
        pt.velocities = [0.0] * len(JOINTS)
        pt.time_from_start = Duration(sec=0, nanosec=500_000_000)  # 0.5 s horizon
        msg.points = [pt]
        self._pub.publish(msg)


def main() -> int:
    rclpy.init()
    node = BenchmarkTrajectoryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
