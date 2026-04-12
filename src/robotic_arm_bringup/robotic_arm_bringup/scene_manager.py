#!/usr/bin/env python3
"""Spawns scene objects from objects.yaml into the MoveIt planning scene."""

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, ObjectColor, PlanningScene
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA

from robotic_arm_bringup.arm_controller import load_arm_config, load_objects_config


class SceneManager(Node):
    def __init__(self):
        super().__init__("scene_manager")

        self._base_frame = load_arm_config()["arm"]["base_frame"]

        self._apply_client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self._get_client = self.create_client(GetPlanningScene, "/get_planning_scene")

        self.get_logger().info("Waiting for MoveIt services...")
        self._apply_client.wait_for_service(timeout_sec=30.0)
        self._get_client.wait_for_service(timeout_sec=30.0)
        self.get_logger().info("MoveIt services ready")

    def _apply_object(self, name: str, cfg: dict) -> bool:
        obj = CollisionObject()
        obj.header.frame_id = self._base_frame
        obj.id = name

        primitive = SolidPrimitive()
        shape = cfg.get("shape", "box").lower()
        if shape == "cylinder":
            primitive.type = SolidPrimitive.CYLINDER
        else:
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

        request = ApplyPlanningScene.Request()
        request.scene = scene_msg
        future = self._apply_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.done() and future.result() is not None and future.result().success:
            self.get_logger().info(f"Applied {name} to planning scene")
            return True
        else:
            self.get_logger().error(f"Failed to apply {name}")
            return False

    def _verify_objects(self, expected_names: list) -> list:
        """Return list of names that are missing from MoveIt scene."""
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        future = self._get_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if not future.done() or future.result() is None:
            return expected_names  # assume all missing if we can't check

        present = {obj.id for obj in future.result().scene.world.collision_objects}
        return [name for name in expected_names if name not in present]


def main():
    rclpy.init()
    node = SceneManager()
    objects = load_objects_config()

    node.get_logger().info("Applying objects to planning scene...")
    for name, cfg in objects.items():
        node.get_logger().info(f"Adding {name}...")
        node._apply_object(name, cfg)

    # Verify all objects landed
    missing = node._verify_objects(list(objects.keys()))
    if missing:
        node.get_logger().warn(f"Objects not confirmed in scene: {missing}. Retrying...")
        for name in missing:
            node._apply_object(name, objects[name])

        missing = node._verify_objects(list(objects.keys()))
        if missing:
            node.get_logger().error(f"Failed to add objects after retry: {missing}")
        else:
            node.get_logger().info("All objects confirmed after retry")
    else:
        node.get_logger().info(f"All {len(objects)} objects confirmed in scene")

    node.get_logger().info("Scene setup complete!")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
