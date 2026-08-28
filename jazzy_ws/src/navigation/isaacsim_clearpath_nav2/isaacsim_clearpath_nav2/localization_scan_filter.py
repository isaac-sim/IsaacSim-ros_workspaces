# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Filter incomplete laser scans before localization."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class LocalizationScanFilter(Node):
    """Laser scan relay that rejects messages with too few valid ranges."""

    def __init__(self) -> None:
        super().__init__("localization_scan_filter")
        self.declare_parameter("minimum_valid_ranges", 100)
        self._minimum_valid_ranges = self.get_parameter("minimum_valid_ranges").get_parameter_value().integer_value
        if self._minimum_valid_ranges < 1:
            raise ValueError("minimum_valid_ranges must be greater than zero")

        self._publisher = self.create_publisher(LaserScan, "scan_filtered", qos_profile_sensor_data)
        self._subscription = self.create_subscription(LaserScan, "scan", self._filter_scan, qos_profile_sensor_data)

    def _filter_scan(self, message: LaserScan) -> None:
        valid_ranges = sum(message.range_min <= value <= message.range_max for value in message.ranges)
        if valid_ranges >= self._minimum_valid_ranges:
            self._publisher.publish(message)


def main() -> None:
    """Run the localization scan filter."""
    rclpy.init()
    node = LocalizationScanFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
