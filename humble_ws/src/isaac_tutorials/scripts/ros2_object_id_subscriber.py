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

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points, dtype_from_fields
from std_msgs.msg import String

import json
import time
import numpy as np


class ROS2ObjectIDSubscriber(Node):
    def __init__(self):

        super().__init__("ros2_object_id_subscriber")

        self.point_cloud2_subscriber = self.create_subscription(PointCloud2, "point_cloud", self.point_cloud2_callback, 10)
        self.object_id_map_subscriber = self.create_subscription(String, "object_id_map", self.object_id_map_callback, 10)
        self.object_id_map = None


    def point_cloud2_callback(self, msg: PointCloud2):
        self.get_logger().info(f"Received point cloud.")
        points = read_points(msg, field_names=("x", "y", "z", "object_id_0", "object_id_1", "object_id_2", "object_id_3"), skip_nans=True)
        if self.object_id_map is None:
            return
        object_ids_as_uint32 = np.stack(
            [
                points["object_id_0"],
                points["object_id_1"],
                points["object_id_2"],
                points["object_id_3"],
            ],
            axis=1,
        ).flatten().reshape(-1, 4)
        object_ids_as_uint128 = [int.from_bytes(group.tobytes(), byteorder='little') for group in object_ids_as_uint32]
        prim_paths = [self.object_id_map[str(i)] for i in object_ids_as_uint128]
        print(prim_paths)


    def object_id_map_callback(self, msg: String):
        self.get_logger().info(f"Received object id map: {msg.data}")
        self.object_id_map = json.loads(msg.data)["id_to_labels"]


def main(args=None):
    rclpy.init(args=args)

    ros2_object_id_subscriber = ROS2ObjectIDSubscriber()

    rclpy.spin(ros2_object_id_subscriber)

    # Destroy the node explicitly
    ros2_object_id_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
