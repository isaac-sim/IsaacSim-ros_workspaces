#!/usr/bin/env python3
"""
Direct bridge: Gripper commands -> Isaac Sim joint commands
"""

import rclpy
from rclpy.node import Node
from control_msgs.action import GripperCommand
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from sensor_msgs.msg import JointState
import threading


class GripperToIsaac(Node):
    def __init__(self):
        super().__init__('gripper_to_isaac')
        
        self.get_logger().info('Starting Gripper to Isaac bridge')
        
        # Current target position
        self.target_position = 0.04  # Start open
        self.position_lock = threading.Lock()
        
        # Publisher to Isaac Sim
        self.isaac_pub = self.create_publisher(
            JointState,
            '/isaac_joint_commands',
            10
        )
        
        # Timer to continuously republish commands to Isaac Sim (20Hz)
        self.timer = self.create_timer(0.05, self.publish_gripper_command)
        
        # Action server to intercept MoveIt's gripper commands
        self._action_server = ActionServer(
            self,
            GripperCommand,
            '/panda_hand_controller/gripper_cmd',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        
        self.get_logger().info('Gripper action server ready!')
    
    def publish_gripper_command(self):
        """Continuously publish the current gripper target to Isaac Sim"""
        with self.position_lock:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ['panda_finger_joint1', 'panda_finger_joint2']
            msg.position = [self.target_position, self.target_position]
            self.isaac_pub.publish(msg)
    
    def goal_callback(self, goal_request):
        self.get_logger().info(f'Received goal: position={goal_request.command.position}')
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        self.get_logger().info('Cancel requested')
        return CancelResponse.ACCEPT
    
    async def execute_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info(f'Executing gripper command: {goal_handle.request.command.position}')
        
        # Update target position (timer will continuously publish it)
        with self.position_lock:
            self.target_position = goal_handle.request.command.position
        
        # Wait for gripper to move (simulate execution time)
        import time
        time.sleep(1.0)
        
        self.get_logger().info('Gripper command completed!')
        goal_handle.succeed()
        
        result = GripperCommand.Result()
        result.position = goal_handle.request.command.position
        result.reached_goal = True
        result.stalled = False
        
        return result


def main(args=None):
    rclpy.init(args=args)
    node = GripperToIsaac()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

