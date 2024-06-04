#!/usr/bin/env python

# Copyright (c) 2021-2024, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np

rospy.init_node("test_rosbridge", anonymous=True)

pub = rospy.Publisher("/ackermann_cmd", AckermannDriveStamped, queue_size=10)

msg = AckermannDriveStamped()

i = 0

rate = rospy.Rate(20)
while not rospy.is_shutdown():
    
    # Only command forklift using acceleration and steering angle
    degrees1 = np.arange(0, 60)
    degrees2 = np.arange(-60, 0)
    degrees = np.concatenate((degrees1, degrees1[::-1], degrees2[::-1], degrees2))
    msg.header.frame_id = "forklift"
    msg.header.stamp = rospy.Time.now()
    msg.drive.steering_angle =  0.0174533 * (degrees[i % degrees.size])    
    msg.drive.acceleration = float(degrees[i % degrees.size])

    pub.publish(msg)
    
    i += 1
    rate.sleep()
