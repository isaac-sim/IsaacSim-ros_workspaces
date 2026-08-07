#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Load a URDF from a file or ROS topic and write a ROS 2 parameter file."""

import argparse
import json
import math
import os
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.utilities import remove_ros_args
from std_msgs.msg import String


def _validate_robot_description(description):
    if not description.strip():
        raise ValueError("robot description is empty")
    if "\0" in description:
        raise ValueError("robot description contains a NUL character")
    try:
        root = ET.fromstring(description)
    except ET.ParseError as error:
        raise ValueError(f"robot description is not valid XML: {error}") from error
    if root.tag.rsplit("}", 1)[-1] != "robot":
        raise ValueError("robot description XML root element is not <robot>")


def _write_parameter_file(output_file, description):
    output_path = Path(output_file)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    document = json.dumps(
        {"/**": {"ros__parameters": {"robot_description": description}}},
        ensure_ascii=True,
        indent=2,
    )

    descriptor, staging_file = tempfile.mkstemp(prefix=f".{output_path.name}.", suffix=".tmp", dir=output_path.parent)
    try:
        stream = os.fdopen(descriptor, "w", encoding="utf-8", newline="\n")
        descriptor = None
        with stream:
            stream.write(document)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(staging_file, output_path)
    finally:
        if descriptor is not None:
            os.close(descriptor)
        try:
            Path(staging_file).unlink()
        except OSError:
            pass


def _receive_description(node, topic, log_interval):
    description = None
    qos = QoSProfile(
        depth=1,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
    )

    def receive(message):
        nonlocal description
        description = message.data

    subscription = node.create_subscription(String, topic, receive, qos)
    wait_message = f"Waiting for robot description on {topic}. Press Ctrl+C to stop."
    node.get_logger().info(wait_message)
    next_log = time.monotonic() + log_interval
    while description is None:
        rclpy.spin_once(node, timeout_sec=min(0.5, max(0.0, next_log - time.monotonic())))
        if description is None and time.monotonic() >= next_log:
            node.get_logger().info(wait_message)
            next_log = time.monotonic() + log_interval
    node.destroy_subscription(subscription)
    return description


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/robot_description")
    parser.add_argument("--log-interval", type=float, default=60.0)
    parser.add_argument("--output-file", required=True)
    parser.add_argument("--input-file", default="")
    args = parser.parse_args(remove_ros_args()[1:])

    node = None
    try:
        if not math.isfinite(args.log_interval) or args.log_interval <= 0:
            raise ValueError("log interval must be a finite positive number")
        if args.input_file:
            description = Path(args.input_file).read_text(encoding="utf-8-sig")
        else:
            rclpy.init(args=[])
            node = rclpy.create_node(f"create_robot_description_node_{os.getpid()}", enable_rosout=False)
            description = _receive_description(node, args.topic, args.log_interval)

        _validate_robot_description(description)
        _write_parameter_file(args.output_file, description)
        return 0
    except Exception as error:
        message = f"Failed to create robot-description parameter file: {error}"
        if node is None:
            print(message, file=sys.stderr)
        else:
            node.get_logger().error(message)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
