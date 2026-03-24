#!/usr/bin/env python3
"""Synthetic top-down orthographic camera that renders the MoveIt planning scene.

Queries /get_planning_scene for object positions, draws them as colored shapes
using OpenCV, and publishes sensor_msgs/Image on /camera/image_raw.

This node is the ONLY synthetic component — it gets replaced when migrating to
Gazebo or real hardware. Everything downstream (vision_node, motion_server)
works unchanged.
"""

import threading

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from shape_msgs.msg import SolidPrimitive

from robotic_arm_bringup.arm_controller import load_objects_config


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")

        # Camera parameters (orthographic top-down)
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        self.declare_parameter("scale", 500.0)  # pixels per meter
        self.declare_parameter("center_x", 320.0)  # pixel center = world x=0
        self.declare_parameter("center_y", 240.0)  # pixel center = world y=0
        self.declare_parameter("publish_rate", 10.0)  # Hz
        self.declare_parameter("scene_query_rate", 5.0)  # Hz

        self._width = self.get_parameter("image_width").value
        self._height = self.get_parameter("image_height").value
        self._scale = self.get_parameter("scale").value
        self._cx = self.get_parameter("center_x").value
        self._cy = self.get_parameter("center_y").value
        rate = self.get_parameter("publish_rate").value
        scene_rate = self.get_parameter("scene_query_rate").value

        # Load color config from objects.yaml (MoveIt doesn't reliably return colors)
        self._object_colors = {}
        try:
            objects_cfg = load_objects_config()
            for name, cfg in objects_cfg.items():
                rgba = cfg.get("color", [0.5, 0.5, 0.5, 1.0])
                # Store as BGR for OpenCV (0-255)
                self._object_colors[name] = (
                    int(rgba[2] * 255),
                    int(rgba[1] * 255),
                    int(rgba[0] * 255),
                )
        except Exception as e:
            self.get_logger().warn(f"Could not load objects.yaml colors: {e}")

        # Cached scene objects — updated asynchronously by scene query callback
        self._cached_objects = []  # [(name, shape_type, dims, x, y)]
        self._cache_lock = threading.Lock()
        self._query_in_progress = False

        # All callbacks in ReentrantCallbackGroup + MultiThreadedExecutor
        # so service response can be processed while other callbacks run
        cb_group = ReentrantCallbackGroup()

        # ROS2 setup
        self._bridge = CvBridge()
        self._image_pub = self.create_publisher(Image, "/camera/image_raw", 10)
        self._info_pub = self.create_publisher(CameraInfo, "/camera/camera_info", 10)
        self._scene_client = self.create_client(
            GetPlanningScene, "/get_planning_scene",
            callback_group=cb_group,
        )

        # Render timer — publishes images at publish_rate
        self._render_timer = self.create_timer(
            1.0 / rate, self._render_callback, callback_group=cb_group
        )
        # Scene query timer — sends async service requests at scene_query_rate
        self._scene_timer = self.create_timer(
            1.0 / scene_rate, self._scene_query_callback, callback_group=cb_group
        )

        self.get_logger().info(
            f"Synthetic camera started: {self._width}x{self._height} @ {rate}Hz, "
            f"scale={self._scale} px/m, scene query @ {scene_rate}Hz"
        )

    def _world_to_pixel(self, x: float, y: float) -> tuple:
        """Convert world XY (meters) to pixel UV."""
        u = int(self._cx + x * self._scale)
        v = int(self._cy - y * self._scale)  # Y flipped (image Y grows down)
        return u, v

    def _render_callback(self):
        """Render cached scene objects and publish image."""
        img = np.full((self._height, self._width, 3), 60, dtype=np.uint8)
        self._draw_grid(img)

        with self._cache_lock:
            objects = list(self._cached_objects)

        for name, shape_type, dims, x, y in objects:
            color = self._object_colors.get(name, (128, 128, 128))
            u, v = self._world_to_pixel(x, y)

            if shape_type == SolidPrimitive.CYLINDER:
                radius_px = max(int(dims[1] * self._scale), 3)
                cv2.circle(img, (u, v), radius_px, color, -1)
            else:
                half_w = max(int(dims[0] * self._scale / 2), 2)
                half_h = max(int(dims[1] * self._scale / 2), 2)
                cv2.rectangle(
                    img, (u - half_w, v - half_h), (u + half_w, v + half_h),
                    color, -1,
                )

        msg = self._bridge.cv2_to_imgmsg(img, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_frame"
        self._image_pub.publish(msg)

        info = CameraInfo()
        info.header = msg.header
        info.width = self._width
        info.height = self._height
        self._info_pub.publish(info)

    def _scene_query_callback(self):
        """Send async scene query — response handled by _scene_response_callback."""
        if self._query_in_progress:
            return
        if not self._scene_client.service_is_ready():
            self.get_logger().debug("Scene service not ready")
            return

        self._query_in_progress = True
        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        )

        future = self._scene_client.call_async(request)
        future.add_done_callback(self._scene_response_callback)

    def _scene_response_callback(self, future):
        """Process scene query response (called when future completes)."""
        self._query_in_progress = False

        try:
            result = future.result()
        except Exception as e:
            self.get_logger().warn(f"Scene query failed: {e}")
            return
        if result is None:
            return

        objects = []

        # World objects — position is in co.pose, NOT co.primitive_poses
        # (primitive_poses are relative to co.pose, typically identity)
        for co in result.scene.world.collision_objects:
            if co.primitives:
                prim = co.primitives[0]
                self.get_logger().info(
                    f"  {co.id}: co.pose=({co.pose.position.x:.3f},{co.pose.position.y:.3f})"
                )
                objects.append((
                    co.id,
                    prim.type,
                    list(prim.dimensions),
                    co.pose.position.x,
                    co.pose.position.y,
                ))

        # Attached objects (held by gripper)
        for aco in result.scene.robot_state.attached_collision_objects:
            co = aco.object
            if co.primitives:
                prim = co.primitives[0]
                objects.append((
                    co.id,
                    prim.type,
                    list(prim.dimensions),
                    co.pose.position.x,
                    co.pose.position.y,
                ))

        with self._cache_lock:
            if len(objects) != len(self._cached_objects):
                self.get_logger().info(
                    f"Scene updated: {len(objects)} objects"
                )
            self._cached_objects = objects

    def _draw_grid(self, img):
        """Draw faint grid lines at 0.1m intervals for spatial reference."""
        grid_color = (80, 80, 80)
        for meters in np.arange(-1.0, 1.0, 0.1):
            u, _ = self._world_to_pixel(meters, 0)
            if 0 <= u < self._width:
                cv2.line(img, (u, 0), (u, self._height), grid_color, 1)
            _, v = self._world_to_pixel(0, meters)
            if 0 <= v < self._height:
                cv2.line(img, (0, v), (self._width, v), grid_color, 1)

        # Draw origin cross (brighter)
        u0, v0 = self._world_to_pixel(0, 0)
        cv2.line(img, (u0 - 10, v0), (u0 + 10, v0), (120, 120, 120), 1)
        cv2.line(img, (u0, v0 - 10), (u0, v0 + 10), (120, 120, 120), 1)


def main():
    rclpy.init()
    node = CameraNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
