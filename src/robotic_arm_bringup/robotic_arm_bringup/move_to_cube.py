#!/usr/bin/env python3
"""
Simple motion control script for moving robotic arm to colored cubes.

This script provides CLI interface to move a 3-DOF robotic arm to specific
cube positions using MoveIt2 action clients.
"""

import argparse
import sys
import time

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import ApplyPlanningScene
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
from std_msgs.msg import ColorRGBA
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
import threading


class SimpleMoveArm:
    """Simple motion control for moving arm to cubes using MoveIt2 action clients."""

    def __init__(self, node: Node):
        """
        Initialize the motion controller.

        Args:
            node: ROS2 node for action clients and services
        """
        self.node = node
        self.logger = node.get_logger()

        # Create action client for move_group
        self.logger.info("Connecting to move_group action server...")
        self._move_group_client = ActionClient(self.node, MoveGroup, "/move_action")

        # Wait for action server
        if not self._move_group_client.wait_for_server(timeout_sec=10.0):
            self.logger.error("move_group action server not available!")
            self.logger.error("Make sure simulation is running (./run.sh sim)")
            raise RuntimeError("move_group action server not available")

        self.logger.info("Connected to move_group action server")

        # Publisher for planning scene updates
        self.scene_publisher = node.create_publisher(PlanningScene, "/planning_scene", 10)

        # Service client for synchronous planning scene updates (attach/detach)
        self._apply_scene_client = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
        if not self._apply_scene_client.wait_for_service(timeout_sec=5.0):
            self.logger.warn('apply_planning_scene service not available — will fall back to topic')

        # Subscribe to joint states early (before diagnostics subprocess blocks ROS callbacks)
        # so _get_current_robot_state() returns the actual arm position on the first plan.
        self.joint_state_lock = threading.Lock()
        self.current_joint_state = None
        self.joint_state_sub = node.create_subscription(
            JointState, "/joint_states", self._joint_state_callback, 10
        )
        # Spin briefly to receive at least one joint state message
        rclpy.spin_once(node, timeout_sec=0.5)

        # Log diagnostics (runs a blocking subprocess — must come after joint state setup)
        self._log_diagnostics()

        # Cube positions (tracked dynamically)
        # Format: (x, y, z) in base_link frame
        self.cubes = {"blue": (0.5, 0.0, 0.3), "red": (0.0, 0.5, 0.3)}

        # Planning parameters
        self.planning_time = 5.0  # seconds
        self.max_velocity_scaling = 0.1
        self.max_acceleration_scaling = 0.1

    def _joint_state_callback(self, msg: JointState):
        """Store the latest joint state for planning."""
        with self.joint_state_lock:
            self.current_joint_state = msg

    def _get_current_robot_state(self) -> RobotState:
        """
        Get the current robot state from joint state feedback.

        Returns:
            RobotState with current joint positions
        """
        with self.joint_state_lock:
            if self.current_joint_state is None:
                # Return zero state if no feedback yet
                state = RobotState()
                state.joint_state.name = ["joint_1", "joint_2", "joint_3", "left_finger_joint"]
                state.joint_state.position = [0.0, 0.0, 0.0, 0.0]
                return state

            # Build RobotState from current joint state
            state = RobotState()
            state.joint_state = JointState(
                header=self.current_joint_state.header,
                name=self.current_joint_state.name,
                position=list(self.current_joint_state.position),
                velocity=list(self.current_joint_state.velocity) if self.current_joint_state.velocity else [],
                effort=list(self.current_joint_state.effort) if self.current_joint_state.effort else [],
            )
            return state

    def _log_diagnostics(self):
        """Log diagnostic info about system state at startup."""
        import subprocess

        self.logger.info("=== DIAGNOSTIC INFO ===")

        # Check if controllers are loaded
        try:
            result = subprocess.run(
                ["ros2", "control", "list_controllers"],
                capture_output=True,
                text=True,
                timeout=5,
            )
            if result.returncode == 0:
                self.logger.info("Active controllers:\n" + result.stdout)
            else:
                self.logger.warn(
                    f"Could not list controllers: {result.stderr}"
                )
        except Exception as e:
            self.logger.warn(f"Failed to check controllers: {e}")

        # Check controller config
        try:
            result = subprocess.run(
                ["cat", "/proc/self/cwd/../../../src/robotic_arm_moveit_config/config/ros2_controllers.yaml"],
                capture_output=True,
                text=True,
                timeout=2,
            )
        except Exception:
            pass

        self.logger.info("=== END DIAGNOSTICS ===")

    def move_to_cube(self, cube_name: str, offset_z: float = 0.1, grasp: bool = False) -> bool:
        """
        Move end-effector above specified cube.

        Args:
            cube_name: Name of cube ("blue" or "red")
            offset_z: Vertical offset above cube in meters
            grasp: If True, perform full grasp sequence

        Returns:
            True if succeeded, False otherwise
        """
        if cube_name not in self.cubes:
            self.logger.error(f"Unknown cube: {cube_name}. Available: {list(self.cubes.keys())}")
            return False

        cube_pos = self.cubes[cube_name]
        self.logger.info(
            f"Moving to {cube_name} cube at ({cube_pos[0]:.3f}, {cube_pos[1]:.3f}, {cube_pos[2]:.3f})"
        )

        # Step 1: Open gripper
        if grasp:
            self.open_gripper()
            time.sleep(1.0)

        # Step 2: Move to approach position (offset_z above cube)
        target_x = cube_pos[0]
        target_y = cube_pos[1]
        target_z = cube_pos[2] + offset_z

        success = self._move_to_position(target_x, target_y, target_z)

        if not success:
            self.logger.error("Failed to move to cube approach position")
            return False

        # Step 3: If grasping, move closer and close gripper
        if grasp:
            # Use per-plan ACM to allow cube-gripper collision during descent and close.
            # The cube stays in the world scene (stays visible in RViz) — only these
            # specific plans ignore the cube collision object.
            cube_id = f"{cube_name}_piece"

            # Move closer (2cm above) — cube ignored for collision checking
            if self._move_to_position(target_x, target_y, cube_pos[2] + 0.02, allowed_object=cube_id):
                time.sleep(0.5)
                # Move to grasp position — cube ignored
                if self._move_to_position(target_x, target_y, cube_pos[2], allowed_object=cube_id):
                    time.sleep(0.5)
                    # Attach cube to gripper BEFORE closing — touch_links allows
                    # finger-cube contact so MoveIt won't block the gripper close
                    self._attach_cube_to_gripper(cube_name)
                    time.sleep(0.2)
                    # Close gripper — succeeds because cube is now an attached object
                    # with touch_links covering the finger links
                    self.close_gripper()
                    time.sleep(1.0)
                    # Lift
                    self._move_to_position(target_x, target_y, cube_pos[2] + offset_z)
                    self.logger.info("Grasp sequence completed")
                else:
                    self.logger.error("Failed to reach grasp position")
                    return False
            else:
                self.logger.error("Failed to approach for grasp")
                return False

        self.logger.info(f"Successfully moved to {cube_name} cube")
        return True

    # Human-readable MoveIt error code descriptions
    _ERROR_CODES = {
        1: "SUCCESS",
        99999: "FAILURE (general)",
        -1: "PLANNING_FAILED",
        -2: "INVALID_MOTION_PLAN",
        -3: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
        -4: "CONTROL_FAILED (controller rejected trajectory - check controller config)",
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

    def _send_move_goal(self, goal_msg: MoveGroup.Goal) -> bool:
        """
        Send a MoveGroup goal and wait for result.

        Args:
            goal_msg: Fully constructed MoveGroup.Goal

        Returns:
            True if succeeded, False otherwise
        """
        send_goal_future = self._move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, send_goal_future, timeout_sec=5.0)

        if not send_goal_future.done():
            self.logger.error("Failed to send goal (timeout)")
            return False

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.logger.error("Goal rejected by action server")
            return False

        self.logger.info("Goal accepted, waiting for result...")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=30.0)

        if not result_future.done():
            self.logger.error("Motion execution timeout (>30s)")
            return False

        result = result_future.result().result
        code = result.error_code.val
        code_str = self._ERROR_CODES.get(code, f"UNKNOWN({code})")

        # Log planning details
        if hasattr(result, "planning_time"):
            self.logger.info(f"Planning time: {result.planning_time:.3f}s")

        if hasattr(result, "planned_trajectory"):
            traj = result.planned_trajectory.joint_trajectory
            n_points = len(traj.points)
            if n_points > 0:
                self.logger.info(
                    f"Planned trajectory: {n_points} waypoints, "
                    f"joints: {traj.joint_names}"
                )

                # Log first and last waypoint details
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

                # Log execution status
                if hasattr(result, "executed_trajectory"):
                    exec_traj = result.executed_trajectory.joint_trajectory
                    exec_points = len(exec_traj.points)
                    self.logger.info(
                        f"Executed: {exec_points} waypoints "
                        f"(planned {n_points}, executed {exec_points})"
                    )

        if code == 1:  # SUCCESS
            self.logger.info("Motion completed successfully")
            return True
        else:
            self.logger.error(f"Motion failed with error: {code_str}")
            # Log additional error details if available
            if hasattr(result, "status") and result.status:
                self.logger.error(f"  Status: {result.status}")
            return False

    def _move_to_joints(self, joint_positions: list) -> bool:
        """
        Plan and execute motion to target joint positions.

        Args:
            joint_positions: List of (joint_name, position) tuples

        Returns:
            True if succeeded, False otherwise
        """
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "arm_group"
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = self.planning_time
        goal_msg.request.max_velocity_scaling_factor = self.max_velocity_scaling
        goal_msg.request.max_acceleration_scaling_factor = self.max_acceleration_scaling

        # Set current state as the starting point for planning
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
        """
        Move end-effector to specified Cartesian position.

        Uses MoveIt's PositionConstraint to let MoveIt handle IK and planning.
        Only constrains position (not orientation) since we have a 3-DOF arm.

        Adds joint constraint on joint_1 to bias IK solver toward current solution
        (avoids unnecessary 180° base rotations).

        Args:
            x, y, z: Target position in base_link frame
            allowed_object: Optional collision object ID to ignore during this plan.
                Used during grasp to allow the arm to move through the cube's space
                without removing it from the scene (keeps it visible in RViz).

        Returns:
            True if succeeded, False otherwise
        """
        self.logger.info(
            f"Planning Cartesian motion to ({x:.3f}, {y:.3f}, {z:.3f}) "
            f"[1cm sphere tolerance, joint_1 bias ±90°]"
        )

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "arm_group"
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = self.planning_time
        goal_msg.request.max_velocity_scaling_factor = self.max_velocity_scaling
        goal_msg.request.max_acceleration_scaling_factor = self.max_acceleration_scaling

        # Set current state as the starting point for planning
        goal_msg.request.start_state = self._get_current_robot_state()

        # Create position constraint - let MoveIt solve IK
        pc = PositionConstraint()
        pc.header.frame_id = "base_link"
        pc.link_name = "grasp_link"  # end-effector link
        pc.weight = 1.0

        # Target position
        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        target_pose.orientation.w = 1.0

        # Tolerance region: small sphere around target
        tolerance = SolidPrimitive()
        tolerance.type = SolidPrimitive.SPHERE
        tolerance.dimensions = [0.01]  # 1cm radius tolerance

        bv = BoundingVolume()
        bv.primitives.append(tolerance)
        bv.primitive_poses.append(target_pose)
        pc.constraint_region = bv

        goal_constraints = Constraints()
        goal_constraints.position_constraints.append(pc)

        # Add joint constraint on joint_1 to keep base rotation close to current value
        # This prevents 180° flips and biases IK toward sensible solutions
        jc = JointConstraint()
        jc.joint_name = "joint_1"
        jc.position = 0.0  # Prefer position near 0 (forward facing)
        jc.tolerance_above = 1.57  # ±90°, allows [-π/2, π/2] range
        jc.tolerance_below = 1.57
        jc.weight = 0.1  # Lower weight: position constraint is primary
        goal_constraints.joint_constraints.append(jc)

        goal_msg.request.goal_constraints.append(goal_constraints)

        # Apply per-plan ACM diff if an object should be ignored for this plan
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

    def open_gripper(self, allowed_object: str = None) -> bool:
        """
        Open gripper to fully open position.

        Args:
            allowed_object: Optional collision object ID to ignore during planning.
                Pass the cube ID after detaching, since MoveIt re-adds the cube to the
                world scene at the arm's current position on detach.

        Returns:
            True if succeeded, False otherwise
        """
        self.logger.info("Opening gripper...")
        return self._move_gripper(0.0, allowed_object=allowed_object)

    def close_gripper(self, allowed_object: str = None) -> bool:
        """
        Close gripper to closed position.

        Args:
            allowed_object: Optional collision object ID to ignore during planning.

        Returns:
            True if succeeded, False otherwise
        """
        self.logger.info("Closing gripper...")
        return self._move_gripper(-0.04, allowed_object=allowed_object)

    def _move_gripper(self, position: float, allowed_object: str = None) -> bool:
        """
        Move gripper to specified position.

        Args:
            position: Target position for left_finger_joint
            allowed_object: Optional collision object ID to ignore during this plan.

        Returns:
            True if succeeded, False otherwise
        """
        # Create goal message
        goal_msg = MoveGroup.Goal()

        # Set planning group
        goal_msg.request.group_name = "gripper_group"
        goal_msg.request.num_planning_attempts = 5
        goal_msg.request.allowed_planning_time = 2.0
        goal_msg.request.max_velocity_scaling_factor = 0.1
        goal_msg.request.max_acceleration_scaling_factor = 0.1

        # Set current state as the starting point for planning
        goal_msg.request.start_state = self._get_current_robot_state()

        # Create joint constraint
        joint_constraint = JointConstraint()
        joint_constraint.joint_name = "left_finger_joint"
        joint_constraint.position = position
        joint_constraint.tolerance_above = 0.001
        joint_constraint.tolerance_below = 0.001
        joint_constraint.weight = 1.0

        # Create goal constraints
        goal_constraints = Constraints()
        goal_constraints.joint_constraints.append(joint_constraint)

        goal_msg.request.goal_constraints.append(goal_constraints)

        # Apply per-plan ACM diff if an object should be ignored for this plan
        if allowed_object:
            goal_msg.planning_options.planning_scene_diff = self._make_allowed_collision_diff(
                allowed_object
            )
        else:
            goal_msg.planning_options.planning_scene_diff.is_diff = True
            goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = True
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.replan = False

        # Send goal
        send_goal_future = self._move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, send_goal_future, timeout_sec=5.0)

        if not send_goal_future.done():
            self.logger.error("Failed to send gripper goal")
            return False

        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.logger.error("Gripper goal rejected")
            return False

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=10.0)

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

    def go_home(self) -> bool:
        """
        Return arm to home position (all joints at 0).

        Returns:
            True if succeeded, False otherwise
        """
        self.logger.info("Moving to home position...")
        home_joints = [("joint_1", 0.0), ("joint_2", 0.0), ("joint_3", 0.0)]
        return self._move_to_joints(home_joints)

    def update_cube_position(self, cube_name: str, x: float, y: float, z: float) -> bool:
        """
        Update cube position in the planning scene.

        This allows the robot to track where cubes are after moving them.

        Args:
            cube_name: Name of cube ("blue" or "red")
            x, y, z: New position in base_link frame

        Returns:
            True if update succeeded, False otherwise
        """
        if cube_name not in self.cubes:
            self.logger.error(f"Unknown cube: {cube_name}")
            return False

        self.logger.info(f"Updating {cube_name} cube position to ({x:.3f}, {y:.3f}, {z:.3f})")

        # Create collision object with new position
        cube = CollisionObject()
        cube.header.frame_id = "base_link"
        cube.id = f"{cube_name}_piece"

        # Define cube shape
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.05, 0.05, 0.05]  # 5cm cube

        # Set new position
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0

        cube.primitives.append(primitive)
        cube.primitive_poses.append(pose)
        cube.operation = CollisionObject.ADD  # ADD works whether object exists or not (after removal)

        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        scene_msg.world.collision_objects.append(cube)

        # Re-apply color — MoveIt's decoupleObject() re-adds the cube geometry but loses
        # the ObjectColor set by scene_manager. Always re-publish the color here.
        _colors = {
            "blue": ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0),
            "red": ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),
        }
        if cube_name in _colors:
            obj_color = ObjectColor()
            obj_color.id = f"{cube_name}_piece"
            obj_color.color = _colors[cube_name]
            scene_msg.object_colors.append(obj_color)

        self._apply_scene_sync(scene_msg)

        # Update internal tracking
        self.cubes[cube_name] = (x, y, z)

        self.logger.info("Cube position updated successfully")
        return True

    def _remove_cube_from_scene(self, cube_name: str):
        """Remove cube world collision object from planning scene synchronously."""
        cube = CollisionObject()
        cube.header.frame_id = "base_link"
        cube.id = f"{cube_name}_piece"
        cube.operation = CollisionObject.REMOVE

        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        scene_msg.world.collision_objects.append(cube)

        self.logger.info(f"Removing {cube_name} cube from world scene...")
        result = self._apply_scene_sync(scene_msg)
        self.logger.info(f"Removed {cube_name} cube from world scene (success={result})")

    def _make_allowed_collision_diff(self, object_id: str) -> PlanningScene:
        """Return a PlanningScene diff that marks object_id as collision-passthrough.

        Included in planning_options.planning_scene_diff so only the specific plan
        ignores the object — the world scene (and RViz visual) is unaffected.
        """
        acm = AllowedCollisionMatrix()
        acm.default_entry_names = [object_id]
        acm.default_entry_values = [True]

        scene_diff = PlanningScene()
        scene_diff.is_diff = True
        scene_diff.robot_state.is_diff = True
        scene_diff.allowed_collision_matrix = acm
        return scene_diff

    def _attach_cube_to_gripper(self, cube_name: str):
        """Attach cube to grasp_link so it visually follows the arm during transport."""
        cube_pos = self.cubes[cube_name]

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.05, 0.05, 0.05]

        pose = Pose()
        pose.position.x = cube_pos[0]
        pose.position.y = cube_pos[1]
        pose.position.z = cube_pos[2]
        pose.orientation.w = 1.0

        cube = CollisionObject()
        cube.header.frame_id = "base_link"
        cube.id = f"{cube_name}_piece"
        cube.primitives.append(primitive)
        cube.primitive_poses.append(pose)
        cube.operation = CollisionObject.ADD

        attached = AttachedCollisionObject()
        attached.link_name = "grasp_link"
        attached.object = cube
        attached.touch_links = ["left_finger", "right_finger", "grasp_link", "gripper_base"]

        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        scene_msg.robot_state.is_diff = True
        scene_msg.robot_state.attached_collision_objects.append(attached)
        self.logger.info(f"Attaching {cube_name} cube to gripper...")
        result = self._apply_scene_sync(scene_msg)
        self.logger.info(f"Attached {cube_name} cube to gripper (success={result})")

    def _detach_cube_from_gripper(self, cube_name: str):
        """Detach cube from gripper (leaves it at its last world position)."""
        detach = AttachedCollisionObject()
        detach.object.id = f"{cube_name}_piece"
        detach.object.operation = CollisionObject.REMOVE

        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        scene_msg.robot_state.is_diff = True
        scene_msg.robot_state.attached_collision_objects.append(detach)
        self.logger.info(f"Detaching {cube_name} cube from gripper...")
        result = self._apply_scene_sync(scene_msg)
        self.logger.info(f"Detached {cube_name} cube from gripper (success={result})")

    def _detach_all_cubes(self):
        """Defensively detach all known cubes. Safe to call even if none are attached."""
        for cube_name in self.cubes:
            self._detach_cube_from_gripper(cube_name)

    def _apply_scene_sync(self, scene_msg: PlanningScene) -> bool:
        """Apply a planning scene change synchronously via service.

        Unlike topic publish + sleep, this returns only after MoveIt has fully processed
        the scene change — eliminating the race condition where the next motion goal is
        submitted before the scene update is processed.

        Falls back to topic publish + sleep if the service is not available.
        """
        if self._apply_scene_client.service_is_ready():
            request = ApplyPlanningScene.Request()
            request.scene = scene_msg
            future = self._apply_scene_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
            if future.done() and future.result() is not None:
                return future.result().success
            self.logger.warn('apply_planning_scene service call timed out or failed')
        # Fallback: topic publish + sleep
        self.scene_publisher.publish(scene_msg)
        time.sleep(0.5)
        return True

    def place_cube_at(
        self, cube_name: str, target_x: float, target_y: float, target_z: float = None
    ) -> bool:
        """
        Pick up a cube and place it at a new location.

        This is a complete pick-and-place sequence that:
        1. Grasps the cube at its current position
        2. Moves it to the target location
        3. Updates the planning scene with new position

        Args:
            cube_name: Name of cube to move ("blue" or "red")
            target_x, target_y: Target position in base_link frame
            target_z: Target height (default: same Z as current cube position)

        Returns:
            True if succeeded, False otherwise
        """
        # Defensive cleanup: detach this cube if a prior failed invocation left it attached
        self._detach_cube_from_gripper(cube_name)

        # Default to same height as current cube so it stays on the same surface
        if target_z is None:
            target_z = self.cubes[cube_name][2]

        self.logger.info(
            f"Pick-and-place: Moving {cube_name} cube to "
            f"({target_x:.3f}, {target_y:.3f}, {target_z:.3f})"
        )

        # Step 1: Grasp the cube at current position
        if not self.move_to_cube(cube_name, grasp=True):
            self.logger.error("Failed to grasp cube")
            return False

        # Step 2: Lift cube slightly
        current_pos = self.cubes[cube_name]
        lift_z = current_pos[2] + 0.1  # 10cm above current position
        if not self._move_to_position(current_pos[0], current_pos[1], lift_z):
            self.logger.error("Failed to lift cube")
            self._detach_cube_from_gripper(cube_name)
            # After detach, decoupleObject re-adds cube to world at arm's position.
            # Remove it so open_gripper doesn't hit START_STATE_IN_COLLISION
            # (per-plan ACM is unreliable for gripper group).
            self._remove_cube_from_scene(cube_name)
            self.open_gripper()
            self.update_cube_position(cube_name, *current_pos)
            return False

        # Step 3: Move to above target position
        target_approach_z = target_z + 0.1  # 10cm above target
        if not self._move_to_position(target_x, target_y, target_approach_z):
            self.logger.error("Failed to move to target approach")
            self._detach_cube_from_gripper(cube_name)
            self._remove_cube_from_scene(cube_name)
            self.open_gripper()
            self.update_cube_position(cube_name, *current_pos)
            return False

        # Step 4: Lower to target position
        # Retry at higher Z if target is outside workspace at this XY
        placed_z = target_z
        for attempt in range(3):
            if self._move_to_position(target_x, target_y, placed_z):
                if placed_z != target_z:
                    self.logger.warning(
                        f"Placed cube at Z={placed_z:.3f} "
                        f"(workspace limit; requested Z={target_z:.3f})"
                    )
                break
            placed_z += 0.05
            self.logger.warning(f"Target Z unreachable, retrying at Z={placed_z:.3f}")
        else:
            self.logger.error("Failed to lower to target at any reachable height")
            self._detach_cube_from_gripper(cube_name)
            self._remove_cube_from_scene(cube_name)
            self.open_gripper()
            self.update_cube_position(cube_name, *current_pos)
            return False

        # Step 5: Detach cube from gripper, then release.
        # After detach, MoveIt's decoupleObject() re-adds the cube to the world scene
        # at the arm's current position. Remove it so open_gripper and lift can proceed
        # without START_STATE_IN_COLLISION.
        self._detach_cube_from_gripper(cube_name)
        self._remove_cube_from_scene(cube_name)
        self.open_gripper()

        # Step 6: Move arm up away from the placed position.
        self._move_to_position(target_x, target_y, placed_z + 0.1)

        # Step 7: Re-add cube at target position with correct color.
        self.update_cube_position(cube_name, target_x, target_y, placed_z)

        self.logger.info("Pick-and-place completed successfully")
        return True


def main(args=None):
    """Main entry point for move_to_cube CLI."""

    # Parse command-line arguments
    parser = argparse.ArgumentParser(
        description="Move robotic arm to colored cubes",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Move to blue cube (10cm above)
  ros2 run robotic_arm_bringup move_to_cube --cube blue

  # Grasp red cube
  ros2 run robotic_arm_bringup move_to_cube --cube red --grasp

  # Return to home position
  ros2 run robotic_arm_bringup move_to_cube --home

  # Gripper controls
  ros2 run robotic_arm_bringup move_to_cube --open
  ros2 run robotic_arm_bringup move_to_cube --close

  # Pick and place
  ros2 run robotic_arm_bringup move_to_cube --place blue --target-x 0.2 --target-y 0.3
        """,
    )

    # Motion commands (mutually exclusive)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--cube", type=str, choices=["blue", "red"], help="Move to specified cube")
    group.add_argument(
        "--place",
        type=str,
        choices=["blue", "red"],
        help="Pick and place cube to target position (requires --target-x and --target-y)",
    )
    group.add_argument("--home", action="store_true", help="Return to home position")
    group.add_argument("--open", action="store_true", help="Open gripper")
    group.add_argument("--close", action="store_true", help="Close gripper")
    group.add_argument(
        "--detach",
        type=str,
        choices=["blue", "red"],
        help="Detach cube from gripper and re-add it to scene at tracked position (recovery tool)",
    )

    # Optional arguments
    parser.add_argument(
        "--grasp", action="store_true", help="Perform full grasp sequence (only with --cube)"
    )
    parser.add_argument(
        "--offset",
        type=float,
        default=0.1,
        help="Vertical offset above cube in meters (default: 0.1)",
    )
    parser.add_argument("--target-x", type=float, help="Target X position for --place command")
    parser.add_argument("--target-y", type=float, help="Target Y position for --place command")
    parser.add_argument(
        "--target-z",
        type=float,
        default=None,
        help="Target Z position for --place command (default: same Z as source cube)",
    )

    parsed_args = parser.parse_args()

    # Validate arguments
    if parsed_args.grasp and not parsed_args.cube:
        parser.error("--grasp can only be used with --cube")

    if parsed_args.place:
        if parsed_args.target_x is None or parsed_args.target_y is None:
            parser.error("--place requires --target-x and --target-y")

    # Initialize ROS2
    rclpy.init(args=args)

    # Create node
    node = Node("move_to_cube_node")
    logger = node.get_logger()

    try:
        # Initialize motion controller
        logger.info("Initializing motion controller...")
        controller = SimpleMoveArm(node)

        # Execute requested motion
        success = False

        if parsed_args.cube:
            success = controller.move_to_cube(
                parsed_args.cube, offset_z=parsed_args.offset, grasp=parsed_args.grasp
            )
        elif parsed_args.place:
            success = controller.place_cube_at(
                parsed_args.place, parsed_args.target_x, parsed_args.target_y, parsed_args.target_z
            )
        elif parsed_args.home:
            controller._detach_all_cubes()
            # Try go_home first — in the normal case (arm is NOT at a cube's position,
            # e.g. 10cm above after a successful place), this succeeds without removing
            # cubes from the world scene (no visual disruption in RViz).
            success = controller.go_home()
            if not success:
                # go_home failed — arm may be stuck at a cube's position causing
                # START_STATE_IN_COLLISION. Remove world cubes and retry.
                for cube_name in list(controller.cubes.keys()):
                    controller._remove_cube_from_scene(cube_name)
                success = controller.go_home()
            # Re-add cubes at tracked positions with correct colors.
            # decoupleObject (called during detach) re-adds the cube geometry but loses
            # its ObjectColor, causing green cubes in RViz — this restores the colors.
            for cube_name, pos in controller.cubes.items():
                controller.update_cube_position(cube_name, pos[0], pos[1], pos[2])
        elif parsed_args.open:
            success = controller.open_gripper()
        elif parsed_args.close:
            success = controller.close_gripper()
        elif parsed_args.detach:
            cube_name = parsed_args.detach
            controller._detach_cube_from_gripper(cube_name)
            controller._remove_cube_from_scene(cube_name)
            controller.open_gripper()
            pos = controller.cubes[cube_name]
            controller.update_cube_position(cube_name, pos[0], pos[1], pos[2])
            success = True

        # Report result
        if success:
            logger.info("Motion completed successfully!")
            exit_code = 0
        else:
            logger.error("Motion failed!")
            exit_code = 1

    except KeyboardInterrupt:
        logger.info("Interrupted by user")
        exit_code = 130
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
        import traceback

        traceback.print_exc()
        exit_code = 1
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
