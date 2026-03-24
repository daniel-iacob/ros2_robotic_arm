#!/usr/bin/env python3
"""Persistent ROS2 action server for robotic arm control.

Holds an ArmController instance in memory and exposes arm operations
as ROS2 actions. Eliminates per-command startup cost and the need for
file-based state persistence (/tmp/ position cache).
"""

import threading

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from robotic_arm_bringup.arm_controller import ArmController
from robotic_arm_interfaces.action import (
    CloseGripper,
    Home,
    MoveTo,
    OpenGripper,
    Pick,
    Place,
    Reset,
)
from robotic_arm_interfaces.msg import DetectedObjects, ObjectInfo
from robotic_arm_interfaces.srv import ListObjects


class MotionServer(Node):
    """Persistent node that serves arm control actions."""

    def __init__(self):
        super().__init__("motion_server")
        self.get_logger().info("Starting motion server...")

        # Serialize all arm operations — ArmController is not thread-safe
        self._lock = threading.Lock()

        # Use ReentrantCallbackGroup so action callbacks can call
        # spin_until_future_complete inside ArmController
        cb_group = ReentrantCallbackGroup()

        # Action servers
        self._pick_server = ActionServer(
            self, Pick, "/pick",
            self._pick_callback,
            callback_group=cb_group,
        )
        self._place_server = ActionServer(
            self, Place, "/place",
            self._place_callback,
            callback_group=cb_group,
        )
        self._move_to_server = ActionServer(
            self, MoveTo, "/move_to",
            self._move_to_callback,
            callback_group=cb_group,
        )
        self._home_server = ActionServer(
            self, Home, "/home",
            self._home_callback,
            callback_group=cb_group,
        )
        self._reset_server = ActionServer(
            self, Reset, "/reset",
            self._reset_callback,
            callback_group=cb_group,
        )
        self._open_gripper_server = ActionServer(
            self, OpenGripper, "/open_gripper",
            self._open_gripper_callback,
            callback_group=cb_group,
        )
        self._close_gripper_server = ActionServer(
            self, CloseGripper, "/close_gripper",
            self._close_gripper_callback,
            callback_group=cb_group,
        )

        # List objects service
        self._list_objects_srv = self.create_service(
            ListObjects, "/list_objects",
            self._list_objects_callback,
            callback_group=cb_group,
        )

        # Subscribe to vision detections
        self._detection_sub = self.create_subscription(
            DetectedObjects, "/detected_objects",
            self._detected_objects_callback,
            10,
            callback_group=cb_group,
        )

        # Initialize ArmController (connects to MoveIt)
        self._controller = ArmController(self)
        self.get_logger().info("Motion server ready")

    # ── Action callbacks ──────────────────────────────────────────────

    def _make_feedback_callback(self, goal_handle, feedback_msg):
        """Create a progress callback that publishes action feedback."""
        def _on_progress(step, progress):
            feedback_msg.step = step
            feedback_msg.progress = progress
            goal_handle.publish_feedback(feedback_msg)
        return _on_progress

    def _finish(self, goal_handle, result_type, success, message):
        """Set result and mark goal as succeeded or aborted."""
        result = result_type()
        result.success = success
        result.message = message
        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _pick_callback(self, goal_handle):
        name = goal_handle.request.object_name
        self.get_logger().info(f"Pick request: {name}")
        on_progress = self._make_feedback_callback(goal_handle, Pick.Feedback())

        with self._lock:
            success = self._controller.pick(name, on_progress=on_progress)

        return self._finish(
            goal_handle, Pick.Result, success,
            f"Picked {name}" if success else f"Failed to pick {name}",
        )

    def _place_callback(self, goal_handle):
        req = goal_handle.request
        name = req.object_name
        # Use provided coordinates, or None to use current position
        x = req.x if req.x != 0.0 or req.y != 0.0 or req.z != 0.0 else None
        y = req.y if x is not None else None
        z = req.z if x is not None else None
        self.get_logger().info(f"Place request: {name} at ({x}, {y}, {z})")
        on_progress = self._make_feedback_callback(goal_handle, Place.Feedback())

        with self._lock:
            success = self._controller.place(name, x, y, z, on_progress=on_progress)

        return self._finish(
            goal_handle, Place.Result, success,
            f"Placed {name}" if success else f"Failed to place {name}",
        )

    def _move_to_callback(self, goal_handle):
        req = goal_handle.request
        self.get_logger().info(f"MoveTo request: ({req.x}, {req.y}, {req.z})")
        on_progress = self._make_feedback_callback(goal_handle, MoveTo.Feedback())
        on_progress("moving to position", 0.0)

        with self._lock:
            success = self._controller.move_to(req.x, req.y, req.z)

        return self._finish(
            goal_handle, MoveTo.Result, success,
            "Moved to position" if success else "Failed to move to position",
        )

    def _home_callback(self, goal_handle):
        self.get_logger().info("Home request")
        on_progress = self._make_feedback_callback(goal_handle, Home.Feedback())
        on_progress("moving to home", 0.0)

        with self._lock:
            success = self._controller.home()

        return self._finish(
            goal_handle, Home.Result, success,
            "Home position reached" if success else "Failed to reach home",
        )

    def _reset_callback(self, goal_handle):
        self.get_logger().info("Reset request")
        on_progress = self._make_feedback_callback(goal_handle, Reset.Feedback())

        with self._lock:
            success = self._controller.reset(on_progress=on_progress)

        return self._finish(
            goal_handle, Reset.Result, success,
            "Reset complete" if success else "Reset failed",
        )

    def _open_gripper_callback(self, goal_handle):
        self.get_logger().info("OpenGripper request")
        on_progress = self._make_feedback_callback(goal_handle, OpenGripper.Feedback())
        on_progress("opening gripper", 0.0)

        with self._lock:
            success = self._controller.open_gripper()

        return self._finish(
            goal_handle, OpenGripper.Result, success,
            "Gripper opened" if success else "Failed to open gripper",
        )

    def _close_gripper_callback(self, goal_handle):
        self.get_logger().info("CloseGripper request")
        on_progress = self._make_feedback_callback(goal_handle, CloseGripper.Feedback())
        on_progress("closing gripper", 0.0)

        with self._lock:
            success = self._controller.close_gripper()

        return self._finish(
            goal_handle, CloseGripper.Result, success,
            "Gripper closed" if success else "Failed to close gripper",
        )

    # ── Detection callback ─────────────────────────────────────────────

    def _detected_objects_callback(self, msg: DetectedObjects):
        """Log vision detections (position update disabled until vision verified)."""
        # TODO: Enable position updates once camera+vision pipeline is verified
        # For now, just log what vision sees — don't overwrite MoveIt positions
        pass

    # ── Service callback ──────────────────────────────────────────────

    def _list_objects_callback(self, request, response):
        self.get_logger().info("ListObjects request")

        with self._lock:
            for name, pos in self._controller.objects.items():
                obj = ObjectInfo()
                obj.name = name
                obj.x = pos[0]
                obj.y = pos[1]
                obj.z = pos[2]
                obj.held = self._controller._is_object_attached(name)
                response.objects.append(obj)

        return response


def main(args=None):
    rclpy.init(args=args)
    node = MotionServer()

    # MultiThreadedExecutor allows action callbacks to run concurrently
    # with spin_until_future_complete calls inside ArmController
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down motion server")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
