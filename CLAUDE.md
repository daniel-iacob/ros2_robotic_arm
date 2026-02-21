# ROS2 Robotic Arm Project

## Goal
Learn ROS2 concepts with a simulated robotic arm + gripper + camera → eventually natural language control ("take the blue cube, put it in the basket").

**Timeline**: Simulation-only for next 6+ months

## Instructions for Claude
**Always update this file** when you:
- Make significant changes to the codebase (update Key Files, Current State, architecture sections)
- Make an architectural or design decision — document the chosen approach, the alternatives considered, and why this one was selected
- Fix a non-obvious bug — document the root cause and the lesson learned
- Discover a constraint or ROS2/MoveIt quirk that affects how things must be done

Keep the **Latest Session Changes** section current: add a new dated entry at the top for each session with meaningful changes, summarizing what was done and why.

## Quick Start
```bash
./run.sh sim    # Launch RViz + MoveIt
```

## Architecture
- **ROS2 Jazzy** (Ubuntu 24.04)
- **MoveIt2** for motion planning
- **Mock hardware** (no Gazebo yet)
- **3 DOF arm** + parallel gripper

## Scene Objects & Coordinate System
- **Blue cube** ("blue_piece"): (0.5, 0, 0.3) - 5cm cube, right of base at table height
- **Red cube** ("red_piece"): (0, 0.5, 0.3) - 5cm cube, forward of base at table height
- **Coordinate system**: base_link frame (X-forward, Y-left, Z-up)
- **Collision objects**: Cubes added to MoveIt2 planning scene for collision avoidance

## Motion Control

### Overview
- **3-DOF Arm**: Joint 1 (continuous base rotation), Joint 2 & 3 (revolute shoulder/elbow)
- **Parallel Gripper**: 2-DOF (left_finger_joint controls both via mimic constraint)
- **End-effector**: `grasp_link` - reference frame at gripper center, 8cm above gripper_base
- **Planning Groups**:
  - `arm_group` - base_link to grasp_link (3 joints)
  - `gripper_group` - gripper fingers (2 joints)

### MoveIt2 Integration
- **IK Solver**: KDL (Kinematics & Dynamics Library)
- **Controllers**:
  - `arm_controller` - FollowJointTrajectory for joints 1-3
  - `gripper_controller` - FollowJointTrajectory for finger joints
- **Named States** (from SRDF):
  - `home` - All arm joints at 0°
  - `open` - Gripper fingers open (0.0m)
  - `closed` - Gripper fingers closed (-0.04m)

### Programmatic Control
```bash
# Move to blue cube (10cm above)
ros2 run robotic_arm_bringup move_to_cube --cube blue

# Move to red cube
ros2 run robotic_arm_bringup move_to_cube --cube red

# Move and grasp
ros2 run robotic_arm_bringup move_to_cube --cube blue --grasp

# Pick and place - move blue cube to new position
ros2 run robotic_arm_bringup move_to_cube --place blue --target-x 0.2 --target-y 0.3

# Gripper control
ros2 run robotic_arm_bringup move_to_cube --open   # Open gripper
ros2 run robotic_arm_bringup move_to_cube --close  # Close gripper

# Return home
ros2 run robotic_arm_bringup move_to_cube --home
```

**Implementation**: CLI script using MoveGroup action client (`/move_action`) — sends `PositionConstraint` goals, MoveIt handles IK via KDL
**Scene Management**: Cube positions are tracked and updated in planning scene after moves
**Future**: Can expand to action server for async control with feedback

## Key Files
- `src/robotic_arm_description/urdf/robotic_arm.urdf.xacro` - Robot model
- `src/robotic_arm_bringup/robotic_arm_bringup/scene_manager.py` - Scene setup
- `src/robotic_arm_bringup/robotic_arm_bringup/move_to_cube.py` - Motion control CLI
- `src/robotic_arm_moveit_config/` - MoveIt configuration
- `run.sh` - Build/launch wrapper

## Current State
✓ Arm + gripper URDF
✓ MoveIt2 integration with KDL IK solver
✓ Scene with colored cubes (collision objects)
✓ RViz visualization
✓ Programmatic motion control (CLI script)
✓ Gripper control (open/close)
✓ Pick-and-place with planning scene updates
✓ Dynamic cube position tracking
✗ Camera (planned)
✗ Vision-based cube detection (planned)
✗ LLM integration (planned)

## Technical Details

### MoveIt2 Configuration Files
- **SRDF**: `src/robotic_arm_moveit_config/config/robotic_arm.srdf` - Planning groups, named states, collision filtering
- **Kinematics**: `config/kinematics.yaml` - KDL solver with 0.005s timeout
- **Controllers**: `config/moveit_controllers.yaml` - arm_controller, gripper_controller
- **Joint Limits**: `config/joint_limits.yaml` - Velocity/acceleration limits (scaled to 10%)
- **ROS2 Controllers**: `config/ros2_controllers.yaml` - Low-level controller configuration

### Motion Planning
- **Python API**: MoveGroup action client (`/move_action`) — avoids MoveItPy config complexity
- **Goal type**: `PositionConstraint` (1cm sphere) + `JointConstraint` on joint_1 (±90° bias to prevent 180° flips)
- **IK**: KDL solver with `position_only_ik: true`, 5s timeout — lets MoveIt handle IK from Cartesian goals
- **Planning pipeline**: RRTConnect algorithm (default)
- **Collision checking**: Enabled for non-adjacent links, cubes treated as obstacles

### Pick-and-Place Architecture
- **Approach**: Direct planning scene updates (Approach 3)
- **Cube tracking**: Internal dictionary tracks current positions
- **Scene updates**: Publishes `MOVE` operations to `/planning_scene` topic
- **Sequence**: Grasp → Lift → Move → Lower → Release → Update scene
- **Advantages**: Simple, works with mock hardware, no separate scene manager needed
- **Future**: Can upgrade to attached collision objects when adding Gazebo physics

### Controller Configuration
- **arm_controller**: `open_loop_control: true` — required for mock hardware (bypasses state feedback checks)
- **state_interfaces**: `position` only (URDF exports only position, not velocity — must match controller config)
- **gripper_controller**: standard joint_trajectory_controller for left_finger_joint

### Known Limitations
- **3-DOF workspace**: Cannot achieve arbitrary orientations (missing wrist roll/pitch/yaw)
- **Mock hardware**: No physics simulation (Gazebo integration planned)
- **Static cubes**: Positions hardcoded, no real-time tracking yet

### Future Architecture
- **Phase 1** (Current): Simple CLI script with MoveGroup action client
- **Phase 2** (Planned): Refactor into reusable motion library
- **Phase 3** (Planned): ROS2 action server for async control
- **Phase 4** (Planned): Camera integration for dynamic object detection
- **Phase 5** (Planned): LLM interface for natural language control

---

## Latest Session Changes (2026-02-18)

### Summary of Fixes
- **Fixed CONTROL_FAILED error**: Replaced analytical IK with MoveIt's PositionConstraint approach (MoveIt handles IK internally via KDL `position_only_ik`)
- **Fixed controller initialization**: Explicitly added ros2_control controller_manager node to launch file with proper config path
- **Fixed YAML constraints syntax**: Changed per-joint constraint format to flat structure (`goal: position: 0.05` instead of nested `joint_1: goal: 0.05`)
- **Fixed mimic joint error**: Removed `right_finger_joint` from `moveit_controllers.yaml` (mimic joints cannot have command interfaces)
- **Added comprehensive debug logging**: Trajectory waypoints, execution status, and MoveIt error codes with human-readable descriptions for easier troubleshooting

### Key Files Modified
- `src/robotic_arm_bringup/launch/arm_system.launch.py` - Added explicit controller_manager node
- `src/robotic_arm_moveit_config/config/ros2_controllers.yaml` - Fixed constraint syntax, added tolerances
- `src/robotic_arm_moveit_config/config/moveit_controllers.yaml` - Removed right_finger_joint (mimic)
- `src/robotic_arm_bringup/robotic_arm_bringup/move_to_cube.py` - Replaced analytical IK with PositionConstraint, added detailed logging

---

## Session Changes (2026-02-18, Continuation)

### Summary of Fixes
- **Fixed ros2_control crash**: The real culprit was `robotic_arm.ros2_control.xacro` in MoveIt config — it gave `right_finger_joint` a `command_interface`, conflicting with the URDF `<mimic>` tag. Removed command_interface, kept only state_interface.
- **Removed duplicate controller_manager**: `arm_system.launch.py` was starting an explicit `ros2_control_node` AND `demo.launch.py` started another. Removed the duplicate — demo.launch.py handles it.
- **Fixed moveit_controllers.yaml indentation**: `gripper_controller` was not indented under `moveit_simple_controller_manager`, causing "No action namespace" error.
- **Motion now works**: `ros2 run robotic_arm_bringup move_to_cube --cube blue` successfully plans and executes (163 waypoints, ~16s trajectory).

### Key Constraint Learned
- In ROS2 Jazzy, `<param name="mimic">` inside `<ros2_control>` tags is **deprecated** — mimic info is read from the standard URDF `<mimic>` tag only. Mimic joints cannot have command interfaces.

### Key Files Modified
- `src/robotic_arm_moveit_config/config/robotic_arm.ros2_control.xacro` - Removed command_interface from right_finger_joint (the actual root cause)
- `src/robotic_arm_moveit_config/config/moveit_controllers.yaml` - Fixed gripper_controller indentation
- `src/robotic_arm_bringup/launch/arm_system.launch.py` - Removed duplicate controller_manager node
- `src/robotic_arm_description/urdf/robotic_arm.urdf.xacro` - right_finger_joint: state_interface only in ros2_control section

---

## Session Changes (2026-02-21)

### Root Cause of Motion Timeout Bug
First move (home → blue) worked, but subsequent moves (blue → red) timed out. **Root cause**: MoveIt planning was using stale start state (always assuming home position) instead of the current arm position.

**Why it failed**: When `is_diff = True` in planning options without setting an explicit `start_state`, MoveIt defaults to assuming the robot is at home (0,0,0). Works fine for the first move, but after moving to blue cube, the next plan thinks the arm is still at home — planning invalid trajectories that violate constraints during execution.

### Solution
Added joint state subscription to track the arm's actual current position, then pass it as `start_state` in every MoveGroup goal:
1. Subscribe to `/joint_states` topic with callback to capture current state
2. Implement `_get_current_robot_state()` method that builds a RobotState from latest feedback
3. Set `goal_msg.request.start_state = self._get_current_robot_state()` in all three motion methods: `_move_to_position()`, `_move_to_joints()`, `_move_gripper()`

**Why this works**: Now MoveIt always knows the true starting position before planning, generating valid trajectories from wherever the arm actually is.

### Key Files Modified
- `src/robotic_arm_bringup/robotic_arm_bringup/move_to_cube.py` - Added joint state tracking, `_get_current_robot_state()` method, set current state in all motion planning calls
- `CLAUDE.md` - Added "Instructions for Claude" section to maintain this file as a living document

### Key Constraint Learned
- **MoveIt2 planning state assumption**: When using `is_diff = True` without explicit `start_state`, MoveIt assumes home position. Always set `start_state` for every plan to ensure planning reflects the robot's actual current pose.
