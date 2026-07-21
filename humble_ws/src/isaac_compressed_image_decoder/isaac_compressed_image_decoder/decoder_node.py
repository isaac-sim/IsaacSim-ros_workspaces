#!/usr/bin/env python3

# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Decode Isaac Sim H.264/HEVC CompressedImage topics and republish raw images."""

from dataclasses import dataclass, field
from typing import Any

import av
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

COMPRESSED_IMAGE_TYPE = "sensor_msgs/msg/CompressedImage"
SUPPORTED_CODECS = {
    "avc": "h264",
    "avc1": "h264",
    "h.264": "h264",
    "h264": "h264",
    "hevc": "hevc",
    "h.265": "hevc",
    "h265": "hevc",
}


@dataclass
class TopicDecoder:
    """Runtime state for one compressed image topic."""

    publisher: Any
    subscription: Any
    output_topic: str
    codec_name: str | None = None
    codec: av.CodecContext | None = None
    warned_formats: set[str] = field(default_factory=set)


class CompressedImageDecoder(Node):
    """Decode H.264/HEVC CompressedImage topics and republish them as raw Image topics."""

    def __init__(self):
        super().__init__("compressed_image_decoder")

        self.declare_parameter("input_topic", "")
        self.declare_parameter("output_topic", "")
        self.declare_parameter("codec", "auto")
        self.declare_parameter("discovery_period", 1.0)

        self._input_topic = self.get_parameter("input_topic").get_parameter_value().string_value.strip()
        self._output_topic = self.get_parameter("output_topic").get_parameter_value().string_value.strip()
        codec_parameter = self.get_parameter("codec").get_parameter_value().string_value.strip().lower()
        self._codec_override = None if codec_parameter in ("", "auto") else self._normalize_codec(codec_parameter)
        discovery_period = self.get_parameter("discovery_period").get_parameter_value().double_value
        self._decoders: dict[str, TopicDecoder] = {}

        if codec_parameter not in ("", "auto") and self._codec_override is None:
            supported = ", ".join(sorted(SUPPORTED_CODECS))
            raise ValueError(f"Unsupported codec parameter '{codec_parameter}'. Supported values: auto, {supported}")

        if self._input_topic:
            output_topic = self._output_topic or self._default_output_topic(self._input_topic)
            self._add_decoder(self._input_topic, output_topic)
            self.get_logger().info(
                f"Compressed image decoder started in single-topic mode: {self._input_topic} -> {output_topic}"
            )
        else:
            if self._output_topic:
                self.get_logger().warn(
                    "Ignoring output_topic because input_topic is empty; "
                    "output_topic only applies in single-topic mode."
                )
            self._discovery_timer = self.create_timer(max(discovery_period, 0.1), self._discover_topics)
            self._discover_topics()
            self.get_logger().info("Compressed image decoder started in auto-discovery mode")

    def _discover_topics(self) -> None:
        """Subscribe to newly discovered CompressedImage topics."""
        for topic_name, topic_types in self.get_topic_names_and_types():
            if topic_name in self._decoders or COMPRESSED_IMAGE_TYPE not in topic_types:
                continue
            self._add_decoder(topic_name, self._default_output_topic(topic_name))

    def _add_decoder(self, input_topic: str, output_topic: str) -> None:
        """Create publisher/subscriber pair for a compressed image topic."""
        publisher = self.create_publisher(Image, output_topic, 10)
        subscription = self.create_subscription(
            CompressedImage,
            input_topic,
            self._make_callback(input_topic),
            qos_profile_sensor_data,
        )
        self._decoders[input_topic] = TopicDecoder(
            publisher=publisher,
            subscription=subscription,
            output_topic=output_topic,
        )
        self.get_logger().info(f"Decoding compressed image topic: {input_topic} -> {output_topic}")

    def _make_callback(self, input_topic: str):
        """Create a callback bound to an input topic."""

        def callback(msg: CompressedImage) -> None:
            self._on_compressed_image(input_topic, msg)

        return callback

    def _on_compressed_image(self, input_topic: str, msg: CompressedImage) -> None:
        """Decode one CompressedImage message and publish any decoded raw frames."""
        compressed_data = bytes(msg.data)
        if not compressed_data:
            return

        decoder = self._decoders[input_topic]
        codec_name = self._select_codec(input_topic, msg, decoder)
        if codec_name is None:
            return

        if decoder.codec_name != codec_name or decoder.codec is None:
            try:
                new_codec = av.CodecContext.create(codec_name, "r")
            except Exception as exc:  # noqa: BLE001 - PyAV exposes codec/backend-specific exception classes.
                self.get_logger().warn(f"Failed to create {codec_name} decoder for {input_topic}: {exc}")
                return
            if decoder.codec is not None:
                decoder.codec.close()
            decoder.codec_name = codec_name
            decoder.codec = new_codec
            self.get_logger().info(f"Using {codec_name} decoder for {input_topic}")

        packet = av.Packet(compressed_data)
        try:
            frames = decoder.codec.decode(packet)
        except Exception as exc:  # noqa: BLE001 - PyAV exposes codec/backend-specific exception classes.
            self.get_logger().warn(f"Failed to decode {codec_name} packet from {input_topic}: {exc}")
            return

        for frame in frames:
            rgb_frame = frame.to_ndarray(format="rgb24")
            height, width, channels = rgb_frame.shape

            out_msg = Image()
            out_msg.header = msg.header
            out_msg.height = height
            out_msg.width = width
            out_msg.encoding = "rgb8"
            out_msg.is_bigendian = 0
            out_msg.step = width * channels
            out_msg.data = rgb_frame.tobytes()

            decoder.publisher.publish(out_msg)

    def _select_codec(self, input_topic: str, msg: CompressedImage, decoder: TopicDecoder) -> str | None:
        """Select the decoder codec from an override or the message format field."""
        if self._codec_override is not None:
            return self._codec_override

        codec_name = self._normalize_codec(msg.format)
        if codec_name is not None:
            return codec_name

        format_label = msg.format.strip() or "<empty>"
        if format_label not in decoder.warned_formats:
            decoder.warned_formats.add(format_label)
            self.get_logger().warn(
                f"Skipping {input_topic}: unsupported CompressedImage format '{format_label}'. "
                "Expected h264, h265, or hevc. Set codec:=h264 or codec:=hevc to override."
            )
        return None

    @staticmethod
    def _normalize_codec(format_value: str) -> str | None:
        """Map a ROS CompressedImage format string or codec alias to a PyAV codec name."""
        normalized = format_value.strip().lower().replace("_", "-")
        for separator in (";", ",", " ", ":"):
            normalized = normalized.replace(separator, " ")
        for token in normalized.split():
            codec_name = SUPPORTED_CODECS.get(token)
            if codec_name is not None:
                return codec_name
        return None

    @staticmethod
    def _default_output_topic(input_topic: str) -> str:
        """Derive the raw image topic from a compressed image topic."""
        suffix = "/compressed"
        if input_topic.endswith(suffix):
            output_topic = input_topic[: -len(suffix)]
            return output_topic or "/image"
        return input_topic.rstrip("/") + "/raw"


def main(args=None):
    """Run the compressed image decoder node."""
    rclpy.init(args=args)
    node = CompressedImageDecoder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
