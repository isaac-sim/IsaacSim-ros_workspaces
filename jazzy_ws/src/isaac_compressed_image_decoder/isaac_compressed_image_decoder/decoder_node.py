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

"""ROS 2 node that decodes H264-compressed images and republishes them as raw images."""

import av
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image


class CompressedImageDecoder(Node):
    """Subscribe to H264 CompressedImage messages, decode with PyAV, and republish as raw Image."""

    def __init__(self):
        super().__init__('compressed_image_decoder')

        # Declare parameters for topic names
        self.declare_parameter('input_topic', 'image_raw/compressed')
        self.declare_parameter('output_topic', 'image_decoded')

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        # Create H264 codec context for decoding
        self._codec = av.CodecContext.create('h264', 'r')

        # Publisher for decoded raw images
        self._publisher = self.create_publisher(Image, output_topic, 10)

        # Subscriber for compressed images
        self._subscription = self.create_subscription(
            CompressedImage,
            input_topic,
            self._on_compressed_image,
            10,
        )

        self.get_logger().info(
            f'Compressed image decoder started: {input_topic} -> {output_topic}'
        )

    def _on_compressed_image(self, msg: CompressedImage):
        """Decode an H264 CompressedImage and publish as a raw Image."""
        h264_data = bytes(msg.data)
        if len(h264_data) == 0:
            return

        # Parse the H264 bitstream into packets and decode.
        # Each message from Isaac Sim contains a complete IDR frame,
        # so every packet is independently decodable.
        packet = av.Packet(h264_data)
        try:
            frames = self._codec.decode(packet)
        except av.error.InvalidDataError as e:
            self.get_logger().warn(f'Failed to decode H264 packet: {e}')
            return

        for frame in frames:
            # Convert the decoded frame to RGB24 numpy array
            rgb_frame = frame.to_ndarray(format='rgb24')
            height, width, channels = rgb_frame.shape

            # Build the output Image message
            out_msg = Image()
            out_msg.header = msg.header
            out_msg.height = height
            out_msg.width = width
            out_msg.encoding = 'rgb8'
            out_msg.is_bigendian = 0
            out_msg.step = width * channels
            out_msg.data = rgb_frame.tobytes()

            self._publisher.publish(out_msg)


def main(args=None):
    """Entry point for the compressed image decoder node."""
    rclpy.init(args=args)
    node = CompressedImageDecoder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
