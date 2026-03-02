#!/usr/bin/env python3
"""Spawns scene objects from objects.yaml into the MoveIt planning scene."""

import time

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, ObjectColor, PlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA

from robotic_arm_bringup.arm_controller import load_objects_config


class SceneManager(Node):
    def __init__(self):
        super().__init__("scene_manager")
        self.pub = self.create_publisher(PlanningScene, "/planning_scene", 10)
        self.get_logger().info("Waiting for MoveIt to initialize...")
        time.sleep(5.0)

    def spawn_object(self, name: str, cfg: dict):
        obj = CollisionObject()
        obj.header.frame_id = "base_link"
        obj.id = name

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = list(cfg["dimensions"])

        pos = cfg["position"]
        pose = Pose()
        pose.position.x = pos[0]
        pose.position.y = pos[1]
        pose.position.z = pos[2]
        pose.orientation.w = 1.0

        obj.primitives.append(primitive)
        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD

        rgba = cfg["color"]
        color = ObjectColor()
        color.id = name
        color.color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])

        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        scene_msg.world.collision_objects.append(obj)
        scene_msg.object_colors.append(color)

        self.pub.publish(scene_msg)
        self.get_logger().info(f"Published {name} to planning scene")


def main():
    rclpy.init()
    node = SceneManager()

    objects = load_objects_config()

    node.get_logger().info("Publishing objects to planning scene...")
    for i in range(3):
        for name, cfg in objects.items():
            node.spawn_object(name, cfg)
        time.sleep(0.5)

    node.get_logger().info("Scene setup complete!")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
