#!/usr/bin/env python3
"""Programmatic interface for the robotic arm.

Provides a clean API for pick-and-place operations, motion control,
and scene management. All object definitions come from objects.yaml.
"""

import json
import math
import os
import threading
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    AllowedCollisionMatrix,
    AttachedCollisionObject,
    BoundingVolume,
    CollisionObject,
    Constraints,
    JointConstraint,
    ObjectColor,
    PlanningScene,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA


_POSITION_CACHE = "/tmp/arm_object_positions.json"


def _wait_for_future(node, future, timeout_sec):
    """Wait for a future to complete. Works whether or not an executor is spinning.

    When called from within an executor callback (e.g., motion_server), the executor
    is already spinning so we just poll the future. When called standalone (e.g., old
    CLI mode), we need to spin the node ourselves.
    """
    # Check if an executor is already spinning this node
    if node.executor is not None:
        # Executor is spinning — just wait without trying to spin again
        import time as _time
        start = _time.monotonic()
        while not future.done():
            if _time.monotonic() - start > timeout_sec:
                return
            _time.sleep(0.01)
    else:
        # No executor — spin the node ourselves (standalone mode)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)


def load_objects_config():
    """Load object definitions from objects.yaml in the package share directory."""
    share_dir = get_package_share_directory("robotic_arm_bringup")
    config_path = f"{share_dir}/config/objects.yaml"
    with open(config_path) as f:
        return yaml.safe_load(f)["objects"]


class ArmController:
    """Programmatic interface for the robotic arm."""

    # Human-readable MoveIt error code descriptions
    _ERROR_CODES = {
        1: "SUCCESS",
        99999: "FAILURE (general)",
        -1: "PLANNING_FAILED",
        -2: "INVALID_MOTION_PLAN",
        -3: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
        -4: "CONTROL_FAILED (controller rejected trajectory)",
        -5: "UNABLE_TO_AQUIRE_SENSOR_DATA",
        -6: "TIMED_OUT",
        -7: "PREEMPTED",
        -10: "START_STATE_IN_COLLISION",
        -11: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
        -12: "GOAL_IN_COLLISION",
        -13: "GOAL_VIOLATES_PATH_CONSTRAINTS",
        -14: "GOAL_CONSTRAINTS_VIOLATED",
        -15: "INVALID_GROUP_NAME",
        -16: "INVALID_GOAL_CONSTRAINTS",
        -17: "INVALID_ROBOT_STATE",
        -18: "INVALID_LINK_NAME",
        -19: "INVALID_OBJECT_NAME",
        -31: "NO_IK_SOLUTION",
    }

    def __init__(self, node: Node):
        self.node = node
        self.logger = node.get_logger()

        # Load object definitions from YAML
        self._object_config = load_objects_config()

        # Current positions — initialized from YAML, updated by place/pick
        self.objects = {}
        for name, cfg in self._object_config.items():
            pos = cfg["position"]
            self.objects[name] = (pos[0], pos[1], pos[2])

        # Action client for MoveGroup
        self.logger.info("Connecting to move_group action server...")
        self._move_group_client = ActionClient(self.node, MoveGroup, "/move_action")
        if not self._move_group_client.wait_for_server(timeout_sec=10.0):
            self.logger.error("move_group action server not available!")
            raise RuntimeError("move_group action server not available")
        self.logger.info("Connected to move_group action server")

        # Planning scene publisher (fallback)
        self._scene_publisher = node.create_publisher(PlanningScene, "/planning_scene", 10)

        # Synchronous planning scene services
        self._apply_scene_client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
        if not self._apply_scene_client.wait_for_service(timeout_sec=5.0):
            self.logger.warn("apply_planning_scene service not available — will fall back to topic")

        self._get_scene_client = node.create_client(GetPlanningScene, "/get_planning_scene")
        if not self._get_scene_client.wait_for_service(timeout_sec=5.0):
            self.logger.warn("get_planning_scene service not available — will use YAML defaults")

        # Joint state tracking
        self._joint_state_lock = threading.Lock()
        self._current_joint_state = None
        self._joint_state_sub = node.create_subscription(
            JointState, "/joint_states", self._joint_state_callback, 10
        )
        # Wait for initial joint state — if executor is spinning, just sleep;
        # otherwise spin_once to process the callback
        if node.executor is not None:
            time.sleep(0.5)
        else:
            rclpy.spin_once(node, timeout_sec=0.5)

        self._log_diagnostics()

        # Sync object positions from MoveIt (overrides YAML defaults if objects exist)
        self._sync_object_positions()

        # Planning parameters
        self.planning_time = 5.0
        self.max_velocity_scaling = 0.4
        self.max_acceleration_scaling = 0.4

    # ── Public API ────────────────────────────────────────────────────

    def pick(self, object_name: str, on_progress=None) -> bool:
        """Open gripper, approach object, descend, attach, close gripper, lift.

        Args:
            on_progress: Optional callback(step: str, progress: float) for reporting progress.
        """
        if object_name not in self.objects:
            self.logger.error(f"Unknown object: {object_name}. Available: {list(self.objects.keys())}")
            return False

        def _report(step, progress):
            if on_progress:
                on_progress(step, progress)

        pos = self.objects[object_name]
        object_id = object_name
        self.logger.info(f"Picking {object_name} at ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")

        # Open gripper
        _report("opening gripper", 0.0)
        self.open_gripper()
        time.sleep(0.1)

        # Move to approach position (10cm above)
        _report("approaching object", 0.17)
        if not self._move_to_position(pos[0], pos[1], pos[2] + 0.1):
            self.logger.error("Failed to move to approach position")
            return False

        # Descend to 2cm above (ACM allows passing through object)
        _report("descending to grasp", 0.33)
        if not self._move_to_position(pos[0], pos[1], pos[2] + 0.02, allowed_object=object_id):
            self.logger.error("Failed to approach for grasp")
            return False
        time.sleep(0.1)

        # Descend to object center
        if not self._move_to_position(pos[0], pos[1], pos[2], allowed_object=object_id):
            self.logger.error("Failed to reach grasp position")
            return False
        time.sleep(0.1)

        # Attach before close — touch_links allow finger-object contact
        _report("attaching and closing gripper", 0.50)
        self._attach_object(object_name)
        self._write_position_cache(object_name, pos[0], pos[1], pos[2])
        time.sleep(0.2)

        # Close gripper
        self.close_gripper()
        time.sleep(0.1)

        # Lift
        _report("lifting", 0.83)
        self._move_to_position(pos[0], pos[1], pos[2] + 0.1)
        self.logger.info(f"Pick completed: {object_name}")
        return True

    def place(self, object_name: str, x: float = None, y: float = None, z: float = None,
              on_progress=None) -> bool:
        """Release object at specified position (or current if not specified).

        Assumes object is already being held (picked via pick()).
        If x, y, z not provided, uses the object's last known position.
        Lowers Z by 0.05m before opening gripper.

        Args:
            on_progress: Optional callback(step: str, progress: float) for reporting progress.
        """
        if object_name not in self.objects:
            self.logger.error(f"Unknown object: {object_name}")
            return False

        def _report(step, progress):
            if on_progress:
                on_progress(step, progress)

        # If position not specified, use current object position (arm should be there with it attached)
        if x is None or y is None:
            current_pos = self.objects[object_name]
            x, y = current_pos[0], current_pos[1]

        if z is None:
            z = self.objects[object_name][2]  # Use original Z of object (where it sits)

        release_z = z - 0.05  # Lower slightly before opening

        self.logger.info(f"Placing {object_name} at ({x:.3f}, {y:.3f}, {release_z:.3f})")

        # Step 1: Detach object from gripper
        _report("detaching object", 0.0)
        self._detach_object(object_name)
        self._remove_object_from_scene(object_name)
        time.sleep(0.1)

        # Step 2: Lower slightly (Z - 0.05)
        _report("lowering to release position", 0.20)
        if not self._move_to_position(x, y, release_z):
            self.logger.warning(f"Could not lower to Z={release_z:.3f}, opening at current position")

        time.sleep(0.1)

        # Step 3: Open gripper
        _report("opening gripper", 0.40)
        self.open_gripper()
        time.sleep(0.1)

        # Step 4: Move away (lift up Z + 0.15)
        _report("lifting away", 0.60)
        self._move_to_position(x, y, release_z + 0.15)

        # Step 5: Re-add object at release position with color
        _report("updating scene", 0.80)
        self.update_object_position(object_name, x, y, release_z)
        self._clear_position_cache(object_name)

        self.logger.info(f"Placed: {object_name}")
        return True

    def home(self) -> bool:
        """Return arm to home position. Handles detach and retry logic."""
        self._detach_all()

        # Try going home — usually works (arm not at object position)
        success = self._go_home()
        if not success:
            # Arm may be at an object's position → START_STATE_IN_COLLISION
            for name in list(self.objects.keys()):
                self._remove_object_from_scene(name)
            success = self._go_home()

        # Re-add all objects with correct colors
        for name, pos in self.objects.items():
            self.update_object_position(name, pos[0], pos[1], pos[2])

        return success

    def move_to(self, x: float, y: float, z: float) -> bool:
        """Move end-effector to a Cartesian position."""
        success = self._move_to_position(x, y, z)
        if success:
            # If carrying an object, persist its new world position to cache so the
            # next CLI invocation (place) knows where to release it.
            for name in self.objects:
                if self._is_object_attached(name):
                    self.objects[name] = (x, y, z)
                    self._write_position_cache(name, x, y, z)
        return success

    def open_gripper(self) -> bool:
        """Open gripper to fully open position."""
        self.logger.info("Opening gripper...")
        return self._move_gripper(0.0)

    def close_gripper(self) -> bool:
        """Close gripper to fully closed position."""
        self.logger.info("Closing gripper...")
        return self._move_gripper(-0.04)

    def reset(self, on_progress=None) -> bool:
        """Full recovery: detach all, clear scene, open gripper, go home, re-add all objects.

        Args:
            on_progress: Optional callback(step: str, progress: float) for reporting progress.
        """
        self.logger.info("Resetting arm and scene...")

        def _report(step, progress):
            if on_progress:
                on_progress(step, progress)

        # Detach everything
        _report("detaching all objects", 0.0)
        self._detach_all()

        # Remove all objects from world scene
        _report("clearing scene", 0.20)
        for name in list(self.objects.keys()):
            self._remove_object_from_scene(name)

        # Open gripper (nothing in the way now)
        _report("opening gripper", 0.40)
        self.open_gripper()

        # Go home
        _report("moving to home", 0.60)
        success = self._go_home()

        # Restore all objects at original YAML positions
        _report("restoring objects", 0.80)
        self._clear_position_cache()
        for name, cfg in self._object_config.items():
            pos = cfg["position"]
            self.objects[name] = (pos[0], pos[1], pos[2])
            self.update_object_position(name, pos[0], pos[1], pos[2])

        self.logger.info(f"Reset {'completed' if success else 'failed (arm did not reach home)'}")
        return success

    def list_objects(self) -> bool:
        """Query MoveIt planning scene and print all world and attached collision objects."""
        if not self._get_scene_client.service_is_ready():
            self.logger.error("get_planning_scene service not available — is the sim running?")
            return False

        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY |
            PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        )

        future = self._get_scene_client.call_async(request)
        _wait_for_future(self.node, future, timeout_sec=5.0)

        if not future.done() or future.result() is None:
            self.logger.error("Failed to query planning scene")
            return False

        scene = future.result().scene
        world_objects = scene.world.collision_objects
        attached_objects = scene.robot_state.attached_collision_objects

        total = len(world_objects) + len(attached_objects)
        if total == 0:
            self.logger.info("No objects in the planning scene")
            return True

        self.logger.info(f"Objects in scene ({total}):")
        for obj in world_objects:
            x = obj.pose.position.x + (obj.primitive_poses[0].position.x if obj.primitive_poses else 0)
            y = obj.pose.position.y + (obj.primitive_poses[0].position.y if obj.primitive_poses else 0)
            z = obj.pose.position.z + (obj.primitive_poses[0].position.z if obj.primitive_poses else 0)
            self.logger.info(f"  {obj.id}: ({x:.3f}, {y:.3f}, {z:.3f})")
        for att in attached_objects:
            self.logger.info(f"  {att.object.id}: (held)")
        return True

    def update_object_position(self, object_name: str, x: float, y: float, z: float) -> bool:
        """Re-add object to planning scene at new position with correct color."""
        if object_name not in self._object_config:
            self.logger.error(f"Unknown object: {object_name}")
            return False

        cfg = self._object_config[object_name]

        self.logger.info(f"Updating {object_name} position to ({x:.3f}, {y:.3f}, {z:.3f})")

        obj = CollisionObject()
        obj.header.frame_id = "base_link"
        obj.id = object_name

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = list(cfg["dimensions"])

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0

        obj.primitives.append(primitive)
        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD

        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        scene_msg.world.collision_objects.append(obj)

        # Apply color from config
        rgba = cfg["color"]
        obj_color = ObjectColor()
        obj_color.id = object_name
        obj_color.color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
        scene_msg.object_colors.append(obj_color)

        self._apply_scene_sync(scene_msg)
        self.objects[object_name] = (x, y, z)
        self.logger.info("Object position updated")
        return True

    # ── Internal methods ──────────────────────────────────────────────

    def _read_position_cache(self) -> dict:
        """Read held-object positions from cache file (persists across CLI calls)."""
        try:
            with open(_POSITION_CACHE) as f:
                return json.load(f)
        except (FileNotFoundError, json.JSONDecodeError):
            return {}

    def _write_position_cache(self, name: str, x: float, y: float, z: float):
        """Persist a held object's world position to cache."""
        cache = self._read_position_cache()
        cache[name] = [x, y, z]
        with open(_POSITION_CACHE, "w") as f:
            json.dump(cache, f)

    def _clear_position_cache(self, name: str = None):
        """Remove one entry (or clear all) from the position cache."""
        if name is None:
            try:
                os.remove(_POSITION_CACHE)
            except FileNotFoundError:
                pass
            return
        cache = self._read_position_cache()
        if name in cache:
            del cache[name]
            with open(_POSITION_CACHE, "w") as f:
                json.dump(cache, f)

    def _sync_object_positions(self):
        """Query MoveIt for actual object positions, overriding YAML defaults."""
        if not self._get_scene_client.service_is_ready():
            self.logger.info("get_planning_scene not available, using YAML positions")
            return

        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY |
            PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        )

        future = self._get_scene_client.call_async(request)
        _wait_for_future(self.node, future, timeout_sec=5.0)

        if not future.done() or future.result() is None:
            self.logger.warn("Failed to query planning scene, using YAML positions")
            return

        scene = future.result().scene
        updated = 0

        for obj in scene.world.collision_objects:
            if obj.id in self.objects and obj.primitive_poses:
                old = self.objects[obj.id]
                # MoveIt normalizes: world pos → obj.pose, primitive offset → primitive_poses[0]
                # Sum both to handle either storage convention
                x = obj.pose.position.x + obj.primitive_poses[0].position.x
                y = obj.pose.position.y + obj.primitive_poses[0].position.y
                z = obj.pose.position.z + obj.primitive_poses[0].position.z
                new = (x, y, z)
                if old != new:
                    self.objects[obj.id] = new
                    self.logger.info(
                        f"Synced {obj.id}: ({old[0]:.3f},{old[1]:.3f},{old[2]:.3f}) → "
                        f"({new[0]:.3f},{new[1]:.3f},{new[2]:.3f})"
                    )
                    updated += 1

        # For attached objects, MoveIt stores poses in link frame (not base_link),
        # so we use the position cache written by pick()/move_to() instead.
        cache = self._read_position_cache()
        for att in scene.robot_state.attached_collision_objects:
            obj = att.object
            if obj.id in self.objects and obj.id in cache:
                old = self.objects[obj.id]
                cached = cache[obj.id]
                new = (cached[0], cached[1], cached[2])
                if old != new:
                    self.objects[obj.id] = new
                    self.logger.info(
                        f"Synced {obj.id} (held, from cache): "
                        f"({old[0]:.3f},{old[1]:.3f},{old[2]:.3f}) → "
                        f"({new[0]:.3f},{new[1]:.3f},{new[2]:.3f})"
                    )
                    updated += 1

        if updated:
            self.logger.info(f"Synced {updated} object position(s) from MoveIt")

    def _cleanup_after_failed_place(self, object_name: str, original_pos: tuple):
        """Common cleanup when place fails mid-sequence."""
        self._detach_object(object_name)
        self._remove_object_from_scene(object_name)
        self.open_gripper()
        self.update_object_position(object_name, *original_pos)

    def _joint_state_callback(self, msg: JointState):
        with self._joint_state_lock:
            self._current_joint_state = msg

    def _get_current_robot_state(self) -> RobotState:
        with self._joint_state_lock:
            if self._current_joint_state is None:
                state = RobotState()
                state.joint_state.name = ["joint_1", "joint_2", "joint_3", "left_finger_joint"]
                state.joint_state.position = [0.0, 0.0, 0.0, 0.0]
                return state

            state = RobotState()
            state.joint_state = JointState(
                header=self._current_joint_state.header,
                name=self._current_joint_state.name,
                position=list(self._current_joint_state.position),
                velocity=list(self._current_joint_state.velocity) if self._current_joint_state.velocity else [],
                effort=list(self._current_joint_state.effort) if self._current_joint_state.effort else [],
            )
            return state

    def _log_diagnostics(self):
        import subprocess

        self.logger.info("=== DIAGNOSTIC INFO ===")
        try:
            result = subprocess.run(
                ["ros2", "control", "list_controllers"],
                capture_output=True, text=True, timeout=5,
            )
            if result.returncode == 0:
                self.logger.info("Active controllers:\n" + result.stdout)
            else:
                self.logger.warn(f"Could not list controllers: {result.stderr}")
        except Exception as e:
            self.logger.warn(f"Failed to check controllers: {e}")
        self.logger.info("=== END DIAGNOSTICS ===")

    def _send_move_goal(self, goal_msg: MoveGroup.Goal) -> bool:
        send_goal_future = self._move_group_client.send_goal_async(goal_msg)
        _wait_for_future(self.node, send_goal_future, timeout_sec=5.0)

        if not send_goal_future.done():
            self.logger.error("Failed to send goal (timeout)")
            return False

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.logger.error("Goal rejected by action server")
            return False

        self.logger.info("Goal accepted, waiting for result...")

        result_future = goal_handle.get_result_async()
        _wait_for_future(self.node, result_future, timeout_sec=30.0)

        if not result_future.done():
            self.logger.error("Motion execution timeout (>30s)")
            return False

        result = result_future.result().result
        code = result.error_code.val
        code_str = self._ERROR_CODES.get(code, f"UNKNOWN({code})")

        if hasattr(result, "planning_time"):
            self.logger.info(f"Planning time: {result.planning_time:.3f}s")

        if hasattr(result, "planned_trajectory"):
            traj = result.planned_trajectory.joint_trajectory
            n_points = len(traj.points)
            if n_points > 0:
                self.logger.info(
                    f"Planned trajectory: {n_points} waypoints, joints: {traj.joint_names}"
                )
                first = traj.points[0]
                last = traj.points[-1]
                self.logger.info(
                    f"  START: positions={[f'{p:.4f}' for p in first.positions]} "
                    f"time_from_start={first.time_from_start.sec}s"
                )
                self.logger.info(
                    f"  END:   positions={[f'{p:.4f}' for p in last.positions]} "
                    f"time_from_start={last.time_from_start.sec}s"
                )
                if hasattr(result, "executed_trajectory"):
                    exec_traj = result.executed_trajectory.joint_trajectory
                    exec_points = len(exec_traj.points)
                    self.logger.info(
                        f"Executed: {exec_points} waypoints "
                        f"(planned {n_points}, executed {exec_points})"
                    )

        if code == 1:
            self.logger.info("Motion completed successfully")
            return True
        else:
            self.logger.error(f"Motion failed with error: {code_str}")
            msg = getattr(result.error_code, "message", "")
            src = getattr(result.error_code, "source", "")
            if msg:
                self.logger.error(f"MoveIt reason: {msg}" + (f" (from: {src})" if src else ""))
            if not msg and code in (99999, -1, -31):
                self.logger.error(
                    "Hint: the target position may be outside the arm's reachable workspace. "
                    "Max reach is ~0.78m from the base (upper arm 0.4m + forearm 0.3m + gripper 0.08m). "
                    "Keep sqrt(x²+y²) < 0.7m and z between 0.1–0.8m."
                )
            return False

    def _move_to_joints(self, joint_positions: list) -> bool:
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "arm_group"
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = self.planning_time
        goal_msg.request.max_velocity_scaling_factor = self.max_velocity_scaling
        goal_msg.request.max_acceleration_scaling_factor = self.max_acceleration_scaling
        goal_msg.request.start_state = self._get_current_robot_state()

        goal_constraints = Constraints()
        for joint_name, position in joint_positions:
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = position
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            goal_constraints.joint_constraints.append(jc)

        goal_msg.request.goal_constraints.append(goal_constraints)
        goal_msg.planning_options.planning_scene_diff.is_diff = True
        goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = True
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.replan = True
        goal_msg.planning_options.replan_attempts = 3

        return self._send_move_goal(goal_msg)

    def _move_to_position(self, x: float, y: float, z: float, allowed_object: str = None) -> bool:
        self.logger.info(
            f"Planning motion to ({x:.3f}, {y:.3f}, {z:.3f})"
        )

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "arm_group"
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = self.planning_time
        goal_msg.request.max_velocity_scaling_factor = self.max_velocity_scaling
        goal_msg.request.max_acceleration_scaling_factor = self.max_acceleration_scaling
        goal_msg.request.start_state = self._get_current_robot_state()

        # Position constraint
        pc = PositionConstraint()
        pc.header.frame_id = "base_link"
        pc.link_name = "grasp_link"
        pc.weight = 1.0

        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        target_pose.orientation.w = 1.0

        tolerance = SolidPrimitive()
        tolerance.type = SolidPrimitive.SPHERE
        tolerance.dimensions = [0.01]

        bv = BoundingVolume()
        bv.primitives.append(tolerance)
        bv.primitive_poses.append(target_pose)
        pc.constraint_region = bv

        goal_constraints = Constraints()
        goal_constraints.position_constraints.append(pc)

        # Joint constraint on joint_1 biased toward natural angle for this XY target
        expected_j1 = math.atan2(y, x)
        jc = JointConstraint()
        jc.joint_name = "joint_1"
        jc.position = expected_j1
        jc.tolerance_above = 0.5
        jc.tolerance_below = 0.5
        jc.weight = 0.1
        goal_constraints.joint_constraints.append(jc)

        goal_msg.request.goal_constraints.append(goal_constraints)

        if allowed_object:
            goal_msg.planning_options.planning_scene_diff = self._make_allowed_collision_diff(
                allowed_object
            )
        else:
            goal_msg.planning_options.planning_scene_diff.is_diff = True
            goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = True

        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.replan = True
        goal_msg.planning_options.replan_attempts = 3

        return self._send_move_goal(goal_msg)

    def _move_gripper(self, position: float, allowed_object: str = None) -> bool:
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "gripper_group"
        goal_msg.request.num_planning_attempts = 5
        goal_msg.request.allowed_planning_time = 2.0
        goal_msg.request.max_velocity_scaling_factor = 0.4
        goal_msg.request.max_acceleration_scaling_factor = 0.4
        goal_msg.request.start_state = self._get_current_robot_state()

        jc = JointConstraint()
        jc.joint_name = "left_finger_joint"
        jc.position = position
        jc.tolerance_above = 0.001
        jc.tolerance_below = 0.001
        jc.weight = 1.0

        goal_constraints = Constraints()
        goal_constraints.joint_constraints.append(jc)
        goal_msg.request.goal_constraints.append(goal_constraints)

        if allowed_object:
            goal_msg.planning_options.planning_scene_diff = self._make_allowed_collision_diff(
                allowed_object
            )
        else:
            goal_msg.planning_options.planning_scene_diff.is_diff = True
            goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = True
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.replan = False

        send_goal_future = self._move_group_client.send_goal_async(goal_msg)
        _wait_for_future(self.node, send_goal_future, timeout_sec=5.0)

        if not send_goal_future.done():
            self.logger.error("Failed to send gripper goal")
            return False

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.logger.error("Gripper goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        _wait_for_future(self.node, result_future, timeout_sec=10.0)

        if not result_future.done():
            self.logger.error("Gripper motion timeout")
            return False

        result = result_future.result().result
        if result.error_code.val == 1:
            self.logger.info("Gripper motion completed")
            return True
        else:
            self.logger.error(f"Gripper motion failed: {result.error_code.val}")
            return False

    def _go_home(self) -> bool:
        self.logger.info("Moving to home position...")
        return self._move_to_joints([("joint_1", 0.0), ("joint_2", 0.0), ("joint_3", 0.0)])

    def _make_allowed_collision_diff(self, object_id: str) -> PlanningScene:
        acm = AllowedCollisionMatrix()
        acm.default_entry_names = [object_id]
        acm.default_entry_values = [True]

        scene_diff = PlanningScene()
        scene_diff.is_diff = True
        scene_diff.robot_state.is_diff = True
        scene_diff.allowed_collision_matrix = acm
        return scene_diff

    def _attach_object(self, object_name: str):
        pos = self.objects[object_name]
        cfg = self._object_config[object_name]

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = list(cfg["dimensions"])

        pose = Pose()
        pose.position.x = pos[0]
        pose.position.y = pos[1]
        pose.position.z = pos[2]
        pose.orientation.w = 1.0

        obj = CollisionObject()
        obj.header.frame_id = "base_link"
        obj.id = object_name
        obj.primitives.append(primitive)
        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD

        attached = AttachedCollisionObject()
        attached.link_name = "grasp_link"
        attached.object = obj
        attached.touch_links = ["left_finger", "right_finger", "grasp_link", "gripper_base"]

        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        scene_msg.robot_state.is_diff = True
        scene_msg.robot_state.attached_collision_objects.append(attached)
        self.logger.info(f"Attaching {object_name} to gripper...")
        result = self._apply_scene_sync(scene_msg)
        self.logger.info(f"Attached {object_name} (success={result})")

    def _detach_object(self, object_name: str):
        detach = AttachedCollisionObject()
        detach.object.id = object_name
        detach.object.operation = CollisionObject.REMOVE

        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        scene_msg.robot_state.is_diff = True
        scene_msg.robot_state.attached_collision_objects.append(detach)
        self.logger.info(f"Detaching {object_name}...")
        result = self._apply_scene_sync(scene_msg)
        self.logger.info(f"Detached {object_name} (success={result})")

    def _is_object_attached(self, object_name: str) -> bool:
        """Check if an object is currently attached to the robot."""
        if not self._get_scene_client.service_is_ready():
            return False
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        future = self._get_scene_client.call_async(request)
        _wait_for_future(self.node, future, timeout_sec=2.0)
        if not future.done() or future.result() is None:
            return False
        for obj in future.result().scene.robot_state.attached_collision_objects:
            if obj.object.id == object_name:
                return True
        return False

    def _detach_all(self):
        for name in self.objects:
            self._detach_object(name)

    def _remove_object_from_scene(self, object_name: str):
        obj = CollisionObject()
        obj.header.frame_id = "base_link"
        obj.id = object_name
        obj.operation = CollisionObject.REMOVE

        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        scene_msg.world.collision_objects.append(obj)

        self.logger.info(f"Removing {object_name} from scene...")
        result = self._apply_scene_sync(scene_msg)
        self.logger.info(f"Removed {object_name} (success={result})")

    def _apply_scene_sync(self, scene_msg: PlanningScene) -> bool:
        if self._apply_scene_client.service_is_ready():
            request = ApplyPlanningScene.Request()
            request.scene = scene_msg
            future = self._apply_scene_client.call_async(request)
            _wait_for_future(self.node, future, timeout_sec=5.0)
            if future.done() and future.result() is not None:
                return future.result().success
            self.logger.warn("apply_planning_scene service call timed out or failed")
        # Fallback: topic + sleep
        self._scene_publisher.publish(scene_msg)
        time.sleep(0.5)
        return True
