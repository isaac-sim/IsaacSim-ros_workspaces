import threading

import rclpy
from sensor_msgs.msg import JointState

rclpy.init()
node = rclpy.create_node('position_velocity_publisher')
pub = node.create_publisher(JointState, 'joint_command', 10)

# Spin in a separate thread
thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
thread.start()

joint_state_position = JointState()
joint_state_velocity = JointState()

joint_state_position.name = ["joint1", "joint2","joint3"]
joint_state_velocity.name = ["wheel_left_joint", "wheel_right_joint"]
joint_state_position.position = [0.2,0.2,0.2]
joint_state_velocity.velocity = [20.0, -20.0]

rate = node.create_rate(10)
try:
    while rclpy.ok():
        pub.publish(joint_state_position)
        pub.publish(joint_state_velocity)
        rate.sleep()
except KeyboardInterrupt:
    pass
rclpy.shutdown()
thread.join()