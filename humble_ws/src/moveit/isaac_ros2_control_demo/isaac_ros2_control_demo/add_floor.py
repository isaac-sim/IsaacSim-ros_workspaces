#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0

"""Add a floor collision object below the UR10 base."""

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


def main():
    rclpy.init()
    node = Node("isaac_ros2_control_demo_floor")

    floor = CollisionObject()
    floor.header.frame_id = "base_link"
    floor.id = "floor"
    floor.operation = CollisionObject.ADD

    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = [4.0, 4.0, 0.1]
    floor.primitives.append(box)

    pose = Pose()
    pose.orientation.w = 1.0
    pose.position.z = -0.10
    floor.primitive_poses.append(pose)

    scene = PlanningScene()
    scene.is_diff = True
    scene.world.collision_objects.append(floor)

    client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    if not client.wait_for_service(timeout_sec=30.0):
        raise RuntimeError("Timed out waiting for the /apply_planning_scene service")
    request = ApplyPlanningScene.Request()
    request.scene = scene
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is None or not future.result().success:
        raise RuntimeError("MoveIt rejected the floor collision object")
    node.get_logger().info("Added floor collision object at base_link z=-0.10")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
