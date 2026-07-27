#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Print one transient-local std_msgs/String robot description to stdout."""

import argparse
import sys
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/robot_description")
    parser.add_argument("--timeout", type=float, default=60.0)
    args = parser.parse_args()

    context = rclpy.Context()
    node = None
    executor = None
    description = None
    try:
        rclpy.init(args=[], context=context)
        node = rclpy.create_node(
            "isaac_ros2_control_demo_robot_description_reader",
            context=context,
            enable_rosout=False,
        )
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        def receive(message):
            nonlocal description
            description = message.data

        node.create_subscription(String, args.topic, receive, qos)
        deadline = time.monotonic() + args.timeout
        while description is None and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=min(0.5, max(0.0, deadline - time.monotonic())))
    finally:
        if executor is not None:
            if node is not None:
                executor.remove_node(node)
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        context.try_shutdown()

    if description is None:
        print(
            f"Timed out after {args.timeout:g}s waiting for {args.topic}. " "Is Isaac Sim running and at Play?",
            file=sys.stderr,
        )
        return 1
    if not description.strip():
        print(f"Received an empty robot description from {args.topic}", file=sys.stderr)
        return 1

    sys.stdout.write(description)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
