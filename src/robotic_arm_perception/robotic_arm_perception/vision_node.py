#!/usr/bin/env python3
"""Real HSV color detection node — processes pixels only, never queries MoveIt.

Subscribes to /camera/image_raw, detects colored objects via HSV thresholding
and contour analysis, back-projects pixel centroids to world XY, and publishes
DetectedObjects on /detected_objects.

This node is REAL pipeline code — works unchanged with synthetic camera, Gazebo,
or real USB camera. Only reads objects.yaml for color→name mapping and Z defaults.
"""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from robotic_arm_bringup.arm_controller import load_objects_config
from robotic_arm_interfaces.msg import DetectedObject, DetectedObjects


class VisionNode(Node):
    def __init__(self):
        super().__init__("vision_node")

        # Camera parameters (must match camera_node)
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        self.declare_parameter("scale", 500.0)
        self.declare_parameter("center_x", 320.0)
        self.declare_parameter("center_y", 240.0)
        self.declare_parameter("min_contour_area", 50.0)  # pixels^2, filter noise
        self.declare_parameter("hue_tolerance", 15)  # HSV hue ± tolerance

        self._width = self.get_parameter("image_width").value
        self._height = self.get_parameter("image_height").value
        self._scale = self.get_parameter("scale").value
        self._cx = self.get_parameter("center_x").value
        self._cy = self.get_parameter("center_y").value
        self._min_area = self.get_parameter("min_contour_area").value
        self._hue_tol = self.get_parameter("hue_tolerance").value

        # Load object color definitions from objects.yaml
        self._object_defs = []  # [(name, hsv_lower, hsv_upper, default_z)]
        try:
            objects_cfg = load_objects_config()
            for name, cfg in objects_cfg.items():
                rgba = cfg.get("color", [0.5, 0.5, 0.5, 1.0])
                default_z = cfg.get("position", [0, 0, 0.3])[2]

                # Convert RGB [0-1] to BGR [0-255] for OpenCV, then to HSV
                bgr = np.array([[[
                    int(rgba[2] * 255),
                    int(rgba[1] * 255),
                    int(rgba[0] * 255),
                ]]], dtype=np.uint8)
                hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)[0][0]

                h, s, v = int(hsv[0]), int(hsv[1]), int(hsv[2])

                # Define HSV range for detection
                h_low = max(0, h - self._hue_tol)
                h_high = min(179, h + self._hue_tol)
                # Be generous with saturation/value to handle rendering variations
                s_low = max(30, s - 80)
                s_high = min(255, s + 80)
                v_low = max(30, v - 80)
                v_high = min(255, v + 80)

                lower = np.array([h_low, s_low, v_low])
                upper = np.array([h_high, s_high, v_high])

                self._object_defs.append((name, lower, upper, default_z))
                self.get_logger().info(
                    f"Color target: {name} HSV=[{h_low}-{h_high}, {s_low}-{s_high}, {v_low}-{v_high}]"
                )
        except Exception as e:
            self.get_logger().error(f"Could not load objects.yaml: {e}")

        # ROS2 setup
        self._bridge = CvBridge()
        self._sub = self.create_subscription(
            Image, "/camera/image_raw", self._image_callback, 10
        )
        self._pub = self.create_publisher(DetectedObjects, "/detected_objects", 10)

        self.get_logger().info(
            f"Vision node started: detecting {len(self._object_defs)} object colors"
        )

    def _pixel_to_world(self, u: float, v: float) -> tuple:
        """Convert pixel UV to world XY (meters)."""
        x = (u - self._cx) / self._scale
        y = -(v - self._cy) / self._scale  # Y flipped
        return x, y

    def _image_callback(self, msg: Image):
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        detected = DetectedObjects()
        detected.header = msg.header

        for name, lower, upper, default_z in self._object_defs:
            # Threshold in HSV space
            mask = cv2.inRange(hsv, lower, upper)

            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue

            # Filter by area, take largest
            valid = [c for c in contours if cv2.contourArea(c) >= self._min_area]
            if not valid:
                continue

            largest = max(valid, key=cv2.contourArea)
            area = cv2.contourArea(largest)

            # Compute centroid
            moments = cv2.moments(largest)
            if moments["m00"] == 0:
                continue

            cu = moments["m10"] / moments["m00"]
            cv_val = moments["m01"] / moments["m00"]

            # Back-project to world coordinates
            wx, wy = self._pixel_to_world(cu, cv_val)

            # Confidence based on contour area (larger = more confident)
            # Normalize: 50px² → 0.5, 500px² → 1.0
            confidence = min(1.0, 0.5 + (area - self._min_area) / 900.0)

            obj = DetectedObject()
            obj.name = name
            obj.x = wx
            obj.y = wy
            obj.z = default_z  # Fixed Z from YAML (top-down can't see height)
            obj.confidence = confidence

            detected.objects.append(obj)

        self._pub.publish(detected)


def main():
    rclpy.init()
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
