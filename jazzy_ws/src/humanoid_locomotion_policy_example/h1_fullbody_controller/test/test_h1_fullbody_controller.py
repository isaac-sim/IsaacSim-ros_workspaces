# SPDX-FileCopyrightText: 2026 NVIDIA CORPORATION & AFFILIATES
# SPDX-License-Identifier: Apache-2.0

"""Tests for the H1 full-body controller."""

import unittest

import numpy as np
from geometry_msgs.msg import Twist
from h1_fullbody_controller.h1_fullbody_controller import H1FullbodyController
from sensor_msgs.msg import Imu, JointState


class TestH1FullbodyController(unittest.TestCase):
    """Test policy observation construction."""

    def test_compute_observation_accounts_for_rotating_body_frame(self):
        """Account for coordinate changes caused by body rotation."""
        controller = object.__new__(H1FullbodyController)
        controller._cmd_vel = Twist()
        controller._dt = 0.1
        controller._lin_vel_b = np.array([1.0, 0.0, 0.0])
        controller._previous_action = np.zeros(19)
        controller.default_pos = np.zeros(19)
        controller.joint_names = []

        imu = Imu()
        imu.orientation.w = 1.0
        imu.angular_velocity.z = 1.0

        observation = controller._compute_observation(JointState(), imu)

        np.testing.assert_allclose(observation[:3], [1.0, -0.1, 0.0])


if __name__ == "__main__":
    unittest.main()
