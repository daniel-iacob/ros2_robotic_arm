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

**Also read and update [`doc/architecture.md`](doc/architecture.md)** when you:
- Add or remove ROS2 nodes, topics, or actions
- Make a significant architectural or design decision
- Change the phase roadmap
- Discover a new MoveIt/ROS2 constraint

---

## Quick Reference for Claude

> Read this first. It gives you enough context to start working without parsing the whole file.
> Also read **[`MEMORY.md`](MEMORY.md)** — compact accumulated patterns and constraints.

**What this is**: ROS2 Jazzy simulation of a 3-DOF arm + gripper. Goal: eventually LLM-controlled manipulation. Currently Phase 1 — CLI-driven pick-and-place with MoveIt2 mock hardware. No Gazebo, no camera, no LLM yet.

**Running nodes** (after `./run.sh sim`):
- `scene_manager` — one-shot startup node, adds blue/red cube collision objects to MoveIt
- `move_group` — MoveIt2 planning server, action server at `/move_action`
- `arm_controller` / `gripper_controller` / `joint_state_broadcaster` — ros2_control layer
- `robot_state_publisher` — TF broadcaster
- `rviz2` — visualization

**Entry point for motion**: `ros2 run robotic_arm_bringup move_to_cube [--cube/--place/--home/--open/--close/--detach]`
**Primary file**: `src/robotic_arm_bringup/robotic_arm_bringup/move_to_cube.py`

**Critical constraints — always remember these**:

| Constraint | What breaks if ignored |
|------------|------------------------|
| Always set `start_state` on every MoveGroup plan | Second+ moves plan from home (0,0,0) → wrong trajectories |
| `AttachedCollisionObject` persists across CLI calls | Cube stays glued to arm across commands; `--home` drags it home |
| Detach cube before every `return False` in `place_cube_at()` | Cube stays attached after failed place; subsequent moves carry it |
| After `_detach_cube_from_gripper`, call `_remove_cube_from_scene` before open_gripper/move | MoveIt `decoupleObject()` re-adds cube to world at arm's position → START_STATE_IN_COLLISION (per-plan ACM unreliable for gripper group) |
| `--home`: try `go_home()` first, remove cubes only on failure | Removing all cubes before go_home causes both cubes to disappear from RViz unnecessarily |
| Always re-add cubes with `ObjectColor` after any scene removal | Cubes reappear as green (MoveIt default) instead of blue/red — `update_cube_position()` now handles this |
| `_make_allowed_collision_diff` must set `robot_state.is_diff = True` | Empty robot_state with `is_diff=False` resets planning scene state → ACM ignored → START_STATE_IN_COLLISION |
| Attach cube with `touch_links` BEFORE `close_gripper()` | Gripper close returns error 99999 (per-plan ACM doesn't work for gripper group) |
| Use `/apply_planning_scene` service (not topic) for attach/detach | `time.sleep()` does NOT spin ROS2 executor; topic publishes aren't processed until next spin, creating a race with the next motion plan |
| Mimic joints: no command interface in ros2_control | ros2_control crash on startup |

**Key files by task**:
- Robot model / joints: `src/robotic_arm_description/urdf/robotic_arm.urdf.xacro`
- Motion control logic: `src/robotic_arm_bringup/robotic_arm_bringup/move_to_cube.py`
- Scene initialization: `src/robotic_arm_bringup/robotic_arm_bringup/scene_manager.py`
- MoveIt planning groups / named states: `src/robotic_arm_moveit_config/config/robotic_arm.srdf`
- Controller config: `src/robotic_arm_moveit_config/config/moveit_controllers.yaml`
- Launch entry point: `src/robotic_arm_bringup/launch/arm_system.launch.py`

**Full node diagrams and design decisions**: see [`doc/architecture.md`](doc/architecture.md)

---

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
# Valid targets: keep sqrt(x²+y²) between ~0.3-0.5m, avoid Z<0.3 at off-axis XY (auto-retries)
ros2 run robotic_arm_bringup move_to_cube --place blue --target-x 0.4 --target-y 0.1
ros2 run robotic_arm_bringup move_to_cube --place blue --target-x 0.35 --target-y 0.2
ros2 run robotic_arm_bringup move_to_cube --place blue --target-x 0.1 --target-y 0.4  # near red cube
# Note: (0.2, 0.3) is at workspace limit — arm can only reach Z≈0.4 there (warns but succeeds)

# Gripper control
ros2 run robotic_arm_bringup move_to_cube --open   # Open gripper
ros2 run robotic_arm_bringup move_to_cube --close  # Close gripper

# Return home
ros2 run robotic_arm_bringup move_to_cube --home

# Recovery: detach cube if it got stuck attached to gripper
ros2 run robotic_arm_bringup move_to_cube --detach blue
ros2 run robotic_arm_bringup move_to_cube --detach red
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
- **Approach**: Attach/detach + synchronous scene updates via `/apply_planning_scene` service
- **Cube tracking**: Internal dictionary tracks current positions
- **Scene updates**: `_apply_scene_sync()` — service call with topic fallback
- **Sequence**: Approach → Descend (ACM) → Attach → Close gripper → Lift → Move → Lower → Detach → Remove from scene → Open gripper → Lift away → Re-add cube with color
- **Post-detach pattern**: `_remove_cube_from_scene()` after every detach (decoupleObject re-adds cube at arm position)
- **Colors**: `update_cube_position()` includes `ObjectColor` (blue/red) — MoveIt loses color on detach

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

## Latest Session Changes (2026-02-25)

### Bug: per-plan ACM fails for post-detach open_gripper and move-up

**Root cause (1)**: `_make_allowed_collision_diff()` was missing `robot_state.is_diff = True`.
When MoveIt applies a planning scene diff with `robot_state.is_diff = False` and an empty
robot state, it resets the planning scene's robot state — the ACM diff is effectively ignored.

**Root cause (2)**: Per-plan ACM via `planning_scene_diff` is documented as unreliable for
gripper group (see 2026-02-21 Cont. 4). After fixing `robot_state.is_diff`, gripper group
still fails. The `allowed_object` approach is fundamentally unreliable for post-detach ops.

**New approach — remove from scene after detach**:
After every `_detach_cube_from_gripper()`, call `_remove_cube_from_scene()` to remove the
cube that `decoupleObject()` re-added to the world. Then `open_gripper()` and lift proceed
collision-free (no ACM needed). `update_cube_position()` re-adds the cube with correct color.

### Bug: cubes reappear as green after re-adding

**Root cause**: `update_cube_position()` published a `CollisionObject` without `ObjectColor`.
MoveIt/RViz defaults to green when no color is set. The original blue/red colors were set by
`scene_manager.py` at startup, but `decoupleObject()` and `CollisionObject.ADD` don't
preserve the color association.

**Fix**: `update_cube_position()` now includes `ObjectColor` (blue/red `ColorRGBA`) in the
`PlanningScene` diff. Also switched from topic publish to `_apply_scene_sync()` for
reliability.

### Bug: both cubes disappear from RViz during `--home`

**Root cause**: `--home` handler removed ALL world cubes before `go_home()`, even when the
arm wasn't near any cube (normal case after successful place, arm is 10cm above).

**Fix**: `--home` now tries `go_home()` first (no visual disruption). Only if it fails
(arm stuck at cube position), removes world cubes and retries. Always re-adds cubes with
correct colors at the end.

**Fixes applied**:
- `_make_allowed_collision_diff()`: added `robot_state.is_diff = True`
- `place_cube_at()` steps 2/3/4 failure + step 5: `_remove_cube_from_scene()` after detach
- `update_cube_position()`: includes `ObjectColor` + uses `_apply_scene_sync()`
- `--home` handler: try-first, remove-on-failure, always re-add with colors
- `--detach` handler: `_remove_cube_from_scene()` after detach

**Key files modified**: `src/robotic_arm_bringup/robotic_arm_bringup/move_to_cube.py`

**No rebuild required** — Python-only changes, symlink-install is in effect.

---

## Latest Session Changes (2026-02-24, continuation)

### Bug: START_STATE_IN_COLLISION after detach in `place_cube_at` and `go_home`

**Root cause**: MoveIt2's `decoupleObject()` behavior. When a detach (REMOVE on
AttachedCollisionObject) is processed, MoveIt not only removes the cube from the attached
objects list — it **immediately re-adds the cube as a world collision object at the arm's
current world position**. Now that detach is synchronous (via service), the cube re-appears in
the world *while the arm is still there* → arm start state is in collision with the cube →
START_STATE_IN_COLLISION for any subsequent plan.

Previously (with topic + sleep), the race meant the detach often wasn't processed until after
the arm had already moved away, so this wasn't observed. The service fix exposed it.

**Two affected scenarios**:
1. `place_cube_at()` step 5–6: after detach, `open_gripper()` and the step 6 lift-away both
   fail with START_STATE_IN_COLLISION because the cube just re-appeared at the arm's position.
2. `--home` after a failed place: arm stuck at cube position; `_detach_all_cubes()` is a no-op
   on world objects (only clears attached objects); `go_home()` fails with START_STATE_IN_COLLISION.

**Fixes**:
1. `open_gripper()` gains an `allowed_object` parameter (passed through to `_move_gripper()`).
2. All `open_gripper()` calls that follow `_detach_cube_from_gripper()` in `place_cube_at()`
   now pass `allowed_object=f"{cube_name}_piece"` (4 call sites: steps 2/3/4 fail paths + step 5).
3. `place_cube_at()` step 6 `_move_to_position()` now passes `allowed_object=cube_id`.
4. `_remove_cube_from_scene()` updated to use `_apply_scene_sync` (was topic + sleep).
5. `--home` handler: after `_detach_all_cubes()`, also calls `_remove_cube_from_scene()` for
   each cube (clears world objects), then calls `go_home()`, then re-adds cubes at tracked positions.

**Key constraint learned**:
- **MoveIt `decoupleObject()` re-adds to world**: When detaching an attached collision object,
  MoveIt removes it from `attached_collision_objects` AND re-adds it to `world.collision_objects`
  at the arm's current TF position. Any plan starting from that same position will be
  START_STATE_IN_COLLISION unless the cube is passed as `allowed_object`.

**Key files modified**:
- `src/robotic_arm_bringup/robotic_arm_bringup/move_to_cube.py`:
  - `open_gripper()`: added `allowed_object` parameter
  - `place_cube_at()` steps 2/3/4 failure paths + step 5: `open_gripper(allowed_object=cube_id)`
  - `place_cube_at()` step 6: `_move_to_position(..., allowed_object=cube_id)`
  - `_remove_cube_from_scene()`: uses `_apply_scene_sync` instead of topic + sleep
  - `--home` handler: removes world cubes before `go_home()`, re-adds after

**No rebuild required** — Python-only changes, symlink-install is in effect.

---

## Latest Session Changes (2026-02-24)

### Bug: Cube still follows arm on `--home` despite previous detach fixes

**Root cause**: Race condition between topic-based planning scene updates and MoveIt's planning.

`_detach_cube_from_gripper()` published to `/planning_scene` (a topic) and then called
`time.sleep(0.5)`. But `time.sleep()` is a plain Python sleep — the ROS2 executor is NOT
spinning during it. MoveIt2's `/planning_scene` subscriber callback only fires when an
executor spins. So by the time `go_home()` submitted its MoveGroup action goal (which calls
`rclpy.spin_until_future_complete()`), the detach message was still queued. MoveIt processed
the detach at the same time it was computing the home plan — creating a race where the plan
was often computed WITH the cube still attached.

**Fix**: Replaced `self.scene_publisher.publish(scene_msg)` + `time.sleep()` with a
synchronous `/apply_planning_scene` service call in both `_attach_cube_to_gripper` and
`_detach_cube_from_gripper`. The service returns only after MoveIt has fully processed the
scene change, eliminating the race.

Added `_apply_scene_sync(scene_msg)` helper that uses the service with a topic-publish
fallback if the service isn't available.

Also added `scene_msg.robot_state.is_diff = True` to both methods for explicitness.

**Key files modified**:
- `src/robotic_arm_bringup/robotic_arm_bringup/move_to_cube.py`:
  - Import: added `from moveit_msgs.srv import ApplyPlanningScene`
  - `__init__`: added `_apply_scene_client` (service client for `/apply_planning_scene`)
  - Added `_apply_scene_sync()` helper method
  - `_attach_cube_to_gripper()`: now uses `_apply_scene_sync()` + `robot_state.is_diff = True`
  - `_detach_cube_from_gripper()`: now uses `_apply_scene_sync()` + `robot_state.is_diff = True`

**No rebuild required** — Python-only changes, symlink-install is in effect.

**Key constraint learned**:
- `time.sleep()` does NOT spin the ROS2 executor. Planning scene topic publishes are queued
  but not delivered to MoveIt until an executor spin occurs. For any planning scene update
  that must be processed BEFORE the next motion goal, use `/apply_planning_scene` service
  (synchronous) instead of the `/planning_scene` topic.

---

## Latest Session Changes (2026-02-22)

### Bug: Cube follows arm on `--home` even after successful `--place`

**Root cause (primary)**: `--home` handler had no cleanup — it called `go_home()` directly without checking for or detaching any attached cube. If any prior command left a cube attached (successful place with slow MoveIt processing, failed place, unexpected exception), `--home` dragged the cube along.

**Root cause (secondary)**: `_detach_cube_from_gripper()` sleep was only 0.2s. In failure-path cleanups where `open_gripper()` completes quickly (gripper already open), the process could exit within ~0.5s of the detach publish — possibly before MoveIt's planning scene monitor processed the message.

**Root cause (tertiary)**: `place_cube_at()` had no defensive cleanup at entry. A stale attachment from a prior invocation would cause inconsistent state when the new invocation re-attached the same cube.

**Fixes**:
1. Added `_detach_all_cubes()` method — iterates `self.cubes` and calls `_detach_cube_from_gripper()` on each. REMOVE on a non-attached object is a no-op in MoveIt (safe to call always).
2. `--home` handler now calls `controller._detach_all_cubes()` before `go_home()`.
3. `place_cube_at()` now calls `_detach_cube_from_gripper(cube_name)` at the very start (before any motion) as defensive cleanup.
4. `_detach_cube_from_gripper()` sleep increased from 0.2s → 0.5s.

**Key files modified**:
- `src/robotic_arm_bringup/robotic_arm_bringup/move_to_cube.py`:
  - `_detach_cube_from_gripper()`: sleep 0.2s → 0.5s
  - Added `_detach_all_cubes()` method after `_detach_cube_from_gripper()`
  - `place_cube_at()`: added `_detach_cube_from_gripper(cube_name)` at entry
  - `main()` `--home` handler: added `controller._detach_all_cubes()` before `go_home()`

**No rebuild required** — Python-only changes, symlink-install is in effect.

---

## Latest Session Changes (2026-02-21, Continuation 5)

### Bug: Cube stays attached to gripper when --place fails mid-sequence

**Root cause**: `place_cube_at()` attaches the cube at step 1 (inside `move_to_cube(grasp=True)`). If steps 2, 3, or 4 fail, the function returns `False` without calling `_detach_cube_from_gripper()`. The `AttachedCollisionObject` persists in the MoveIt server across command invocations. Any subsequent command (including `--home`) moves the arm and the cube follows it — because it's still attached.

**Fix**: Added `_detach_cube_from_gripper(cube_name)` + `open_gripper()` cleanup before every early `return False` in `place_cube_at()` after step 1 (steps 2, 3, and the step 4 all-retries-exhausted path).

**Recovery tool**: Added `--detach <cube>` CLI command. Detaches the named cube from the gripper, opens the gripper, and re-adds the cube to the world scene at its last tracked position. Use this when the cube gets stuck attached without restarting the sim:
```bash
ros2 run robotic_arm_bringup move_to_cube --detach blue
```

**Rebuild required**: `colcon build --packages-select robotic_arm_bringup --symlink-install`

### Key Constraint Learned
- **`AttachedCollisionObject` persists across CLI invocations** — it lives in the MoveIt server, not in Python memory. Any early exit that doesn't call `_detach_cube_from_gripper()` leaves the cube attached for all future commands in the same MoveIt session.

### Key Files Modified
- `src/robotic_arm_bringup/robotic_arm_bringup/move_to_cube.py`:
  - `place_cube_at()` steps 2, 3, 4: added detach+open_gripper cleanup before each `return False`
  - Added `--detach <cube>` CLI option for manual recovery

---

## Latest Session Changes (2026-02-21, Continuation 4)

### Gripper close and pick-and-place fixes

**Bug: Gripper close error 99999 even with per-plan ACM**
The `planning_options.planning_scene_diff` ACM approach worked for arm motion (cube stayed visible) but did NOT work for the gripper group — MoveIt still detected finger-cube collision and returned 99999.

**Root cause**: `planning_options.planning_scene_diff` ACM isn't applied consistently across all planning groups; gripper group planning ignored the ACM diff.

**Fix**: Attach cube to gripper (`AttachedCollisionObject`) BEFORE calling `close_gripper()`, not after. When the cube is attached with `touch_links = ["left_finger", "right_finger", ...]`, MoveIt allows those links to contact the attached object — so the gripper close succeeds without any ACM needed.

**Bug: START_STATE_IN_COLLISION on step 7 lift-away**
After placing the cube at its target Z and re-adding it to the planning scene, the arm's start position is at the same Z as the cube center. MoveIt rejected the lift-away plan due to start state collision.

**Fix**: Pass `allowed_object=f"{cube_name}_piece"` to the step 7 `_move_to_position()` call so the arm can plan away from the cube position.

**Feature: Z fallback for workspace limits**
`place_cube_at()` now retries at Z+0.05 increments (up to 3 attempts) if the requested Z is outside the arm's workspace at the target XY. The actual placed Z is used for `update_cube_position()`, with a warning logged.

### Key Files Modified
- `src/robotic_arm_bringup/robotic_arm_bringup/move_to_cube.py`:
  - `move_to_cube()` grasp branch: `_attach_cube_to_gripper()` now called BEFORE `close_gripper()`
  - `place_cube_at()` Step 4: retry loop with Z+0.05 increments on FAILURE
  - `place_cube_at()` Step 6: `update_cube_position()` uses `placed_z` (actual) not `target_z`
  - `place_cube_at()` Step 7: `allowed_object` passed to lift-away motion

### Key Constraint Learned
- **per-plan ACM via `planning_options.planning_scene_diff`**: Works for arm group but NOT reliably for gripper group. For gripper-cube collision suppression, use `AttachedCollisionObject` with `touch_links` instead.

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

---

## Session Changes (2026-02-21, Continuation)

### Root Cause of Gripper Close Failure (error 99999) During Pick-and-Place
`--place` and `--cube --grasp` commands failed with gripper error 99999 (MoveIt general FAILURE) when trying to close the gripper at the grasp position.

**Two-part root cause**:
1. **Arm descent blocked**: The grasp step targets Z=0.3 (cube center). The cube collision object occupies Z=0.275→0.325. MoveIt may reject the arm path because the endpoint is inside the collision object.
2. **Gripper close blocked**: Even if the arm reaches the position, `close_gripper()` plans finger motion while the cube is still a collision obstacle — fingers would intersect the cube mesh, so MoveIt returns 99999.

### Solution
Remove the cube from the planning scene before the arm descends into the cube's collision zone, so both the arm motion and gripper close can execute without interference. The cube is re-added at its new position after placing via `update_cube_position()`.

Three approaches were considered:
- **Remove cube before grasp (chosen)** — minimal code, clean semantics: cube "disappears" when grasped, re-appears at target when placed
- **Allowed Collision Matrix** — more complex, harder to reset, affects all subsequent planning
- **Attach cube to gripper** — most realistic physically, but requires full attach/detach lifecycle (future improvement)

### Key Files Modified
- `src/robotic_arm_bringup/robotic_arm_bringup/move_to_cube.py`:
  - Added `_remove_cube_from_scene()` helper method
  - Call it in `move_to_cube()` before the 2cm descent step (not just before gripper close)
  - Changed `update_cube_position()` from `CollisionObject.MOVE` to `CollisionObject.ADD` so it re-adds the cube correctly after it was removed

### Key Constraint Learned
- **MoveIt2 collision objects block planning endpoints**: A collision object in the scene will block both arm motion that ends inside it AND gripper motion that would intersect it. Must remove collision objects from the scene before grasping them.

---

## Session Changes (2026-02-21, Continuation 2)

### Bugs Fixed in pick-and-place flow

**Bug: place_cube_at() default target_z=0.025 caused unreachable target**
The `place_cube_at()` default was Z=0.025 (floor level), making the approach height 0.125m — well below the arm's workspace (cubes at Z=0.3). MoveIt returned FAILURE(general) for the target motion.
- Fixed: default is now `None` → derived from the cube's current Z position at call time, keeping the cube on the same surface level
- Also fixed: CLI `--target-z` default updated to `None` (was hardcoded 0.025)

**Bug: First motion always planned from home (0,0,0)**
The joint state subscriber was created after `_log_diagnostics()`, which runs a blocking subprocess (~3 seconds). During that blocking call, ROS callbacks can't fire, so `current_joint_state` is None when the first motion is planned. Result: if the arm was at a non-home position, the first plan was wrong.
- Fixed: moved subscriber creation to before `_log_diagnostics()`, added `rclpy.spin_once(node, timeout_sec=0.5)` immediately after subscriber creation to ensure the initial joint state is captured

**Visual artifact: cube disappears while arm animation still playing**
Not a code bug — correct behavior. The cube is removed after the 10cm approach motion succeeds (before descending into collision zone). Mock hardware completes trajectory execution in real wall-clock time (~16s) matching the animation, so timing is correct. The user perception of "too early" is due to the arm already being above the cube when it disappears.

### Key Files Modified
- `src/robotic_arm_bringup/robotic_arm_bringup/move_to_cube.py`:
  - `place_cube_at()`: default target_z changed from 0.025 to None (derives from cube's current Z)
  - `__init__`: joint state subscriber moved before diagnostics, added `rclpy.spin_once()` for initial state capture
  - CLI `--target-z`: default changed from 0.025 to None

---

## Session Changes (2026-02-21, Continuation 3)

### Cube visibility during pick-and-place
User wanted the cube to NEVER disappear from RViz. Previous approach removed the cube from the planning scene before grasping (making it invisible). New approach keeps the cube in the scene always.

**Solution: per-plan ACM + AttachedCollisionObject for transport**
- `_make_allowed_collision_diff(object_id)` — returns a `PlanningScene` diff with the cube marked as collision-passthrough via `AllowedCollisionMatrix.default_entry_names/values`. This diff is passed as `planning_options.planning_scene_diff` in specific plans that need to move through the cube's space. The world scene (and RViz) is unaffected.
- `_move_to_position(allowed_object=...)` and `_move_gripper(allowed_object=...)` / `close_gripper(allowed_object=...)` — accept an optional object ID to ignore during that specific plan
- `_attach_cube_to_gripper(cube_name)` — after gripper closes, attaches the cube to `grasp_link` via `AttachedCollisionObject` so it moves with the arm in RViz during transport. `touch_links = ["left_finger", "right_finger", "grasp_link", "gripper_base"]`
- `_detach_cube_from_gripper(cube_name)` — before opening gripper, detaches the cube (leaves it at its world position)
- `update_cube_position()` — re-adds the cube at the target position after detach+release

**Workflow**:
1. Approach (10cm above): normal collision checking — cube blocks unrelated paths
2. Pre-grasp descent + grasp + gripper close: per-plan ACM ignores cube — cube stays visible
3. After close: cube attached to gripper → moves with arm in RViz
4. Lower to target: cube moves with arm
5. Detach + open gripper + update scene: cube reappears at target position

**Key constraint**: MoveIt's `planning_scene_diff.allowed_collision_matrix.default_entry_names/values` allows specific objects to be treated as collision-passthrough for a single planning request only, without modifying the global planning scene.

### Key Files Modified
- `src/robotic_arm_bringup/robotic_arm_bringup/move_to_cube.py`:
  - Imports: added `AllowedCollisionMatrix`, `AttachedCollisionObject`
  - Added `_make_allowed_collision_diff()`, `_attach_cube_to_gripper()`, `_detach_cube_from_gripper()` methods
  - `_move_to_position()`, `_move_gripper()`, `close_gripper()`: added `allowed_object` parameter
  - `move_to_cube()` grasp branch: replaced `_remove_cube_from_scene()` with per-plan ACM + attach after close
  - `place_cube_at()`: added `_detach_cube_from_gripper()` before `open_gripper()`
