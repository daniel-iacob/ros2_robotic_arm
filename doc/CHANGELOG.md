# Changelog — ROS2 Robotic Arm

Historical session notes. For current project status, see [CLAUDE.md](../CLAUDE.md).

---

## Session 2026-02-25 — Phase 2 Refactor

### Phase 2: Motion Library Refactor

Extracted all motion logic from the monolithic `move_to_cube.py` (~1000 lines) into a clean,
importable library + thin CLI wrapper + YAML config.

**What changed**:
- **New**: `arm_controller.py` — `ArmController` class with public API: `pick()`, `place()`, `home()`, `move_to()`, `open_gripper()`, `close_gripper()`, `reset()`
- **New**: `arm_cli.py` — thin CLI wrapper with subcommands (`arm pick blue_cube`, `arm place blue_cube 0.4 0.1`, etc.)
- **New**: `config/objects.yaml` — single source of truth for object names, positions, dimensions, colors
- **Refactored**: `scene_manager.py` — reads from `objects.yaml` instead of hardcoded methods
- **Deleted**: `move_to_cube.py` — replaced by `arm_controller.py` + `arm_cli.py`
- **Updated**: `setup.py` — new entry point `arm`, installs `config/*.yaml` to share dir

**Key design decisions**:
- **Generic naming**: No "cube" in API — `pick(object_name)` not `move_to_cube(cube_name)`. Object IDs match YAML keys (`blue_cube`, `red_cube`).
- **YAML config**: Both `scene_manager` and `ArmController` read the same `objects.yaml`. Adding a new object = add YAML entry.
- **`reset()` command**: Full nuclear recovery — detach all → remove all → open gripper → go home → re-add all at original YAML positions with colors.
- **`home()` absorbs retry logic**: No longer in `main()` — `home()` handles detach + retry + re-add internally.
- **`_cleanup_after_failed_place()`**: Extracted common cleanup pattern (detach → remove → open → re-add) into helper.

**CLI syntax change**:
```
# Old: ros2 run robotic_arm_bringup move_to_cube --place blue --target-x 0.4 --target-y 0.1
# New: ros2 run robotic_arm_bringup arm place blue_cube 0.4 0.1
```

---

## Session 2026-02-25 — Bug Fixes

### Bug: per-plan ACM fails for post-detach open_gripper and move-up

**Root cause (1)**: `_make_allowed_collision_diff()` was missing `robot_state.is_diff = True`.
When MoveIt applies a planning scene diff with `robot_state.is_diff = False` and an empty
robot state, it resets the planning scene's robot state — the ACM diff is effectively ignored.

**Root cause (2)**: Per-plan ACM via `planning_scene_diff` is documented as unreliable for
gripper group. After fixing `robot_state.is_diff`, gripper group still fails. The `allowed_object` approach is fundamentally unreliable for post-detach ops.

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

---

## Session 2026-02-24 — Continuation

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

---

## Session 2026-02-24 — Race Condition Fix

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

**Key constraint learned**:
- `time.sleep()` does NOT spin the ROS2 executor. Planning scene topic publishes are queued
  but not delivered to MoveIt until an executor spin occurs. For any planning scene update
  that must be processed BEFORE the next motion goal, use `/apply_planning_scene` service
  (synchronous) instead of the `/planning_scene` topic.

---

## Session 2026-02-22

### Bug: Cube follows arm on `--home` even after successful `--place`

**Root cause (primary)**: `--home` handler had no cleanup — it called `go_home()` directly without checking for or detaching any attached cube. If any prior command left a cube attached (successful place with slow MoveIt processing, failed place, unexpected exception), `--home` dragged the cube along.

**Root cause (secondary)**: `_detach_cube_from_gripper()` sleep was only 0.2s. In failure-path cleanups where `open_gripper()` completes quickly (gripper already open), the process could exit within ~0.5s of the detach publish — possibly before MoveIt's planning scene monitor processed the message.

**Root cause (tertiary)**: `place_cube_at()` had no defensive cleanup at entry. A stale attachment from a prior invocation would cause inconsistent state when the new invocation re-attached the same cube.

**Fixes**:
1. Added `_detach_all_cubes()` method — iterates `self.cubes` and calls `_detach_cube_from_gripper()` on each. REMOVE on a non-attached object is a no-op in MoveIt (safe to call always).
2. `--home` handler now calls `controller._detach_all_cubes()` before `go_home()`.
3. `place_cube_at()` now calls `_detach_cube_from_gripper(cube_name)` at the very start (before any motion) as defensive cleanup.
4. `_detach_cube_from_gripper()` sleep increased from 0.2s → 0.5s.

---

## Session 2026-02-21 — Continuation 5

### Bug: Cube stays attached to gripper when --place fails mid-sequence

**Root cause**: `place_cube_at()` attaches the cube at step 1. If steps 2, 3, or 4 fail, the function returns `False` without calling `_detach_cube_from_gripper()`. The `AttachedCollisionObject` persists in the MoveIt server across command invocations. Any subsequent command (including `--home`) moves the arm and the cube follows it — because it's still attached.

**Fix**: Added `_detach_cube_from_gripper(cube_name)` + `open_gripper()` cleanup before every early `return False` in `place_cube_at()` after step 1 (steps 2, 3, and the step 4 all-retries-exhausted path).

**Recovery tool**: Added `--detach <cube>` CLI command. Detaches the named cube from the gripper, opens the gripper, and re-adds the cube to the world scene at its last tracked position.

**Key Constraint Learned**
- **`AttachedCollisionObject` persists across CLI invocations** — it lives in the MoveIt server, not in Python memory. Any early exit that doesn't call `_detach_cube_from_gripper()` leaves the cube attached for all future commands in the same MoveIt session.

---

## Session 2026-02-21 — Continuation 4

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

### Key Constraint Learned
- **per-plan ACM via `planning_options.planning_scene_diff`**: Works for arm group but NOT reliably for gripper group. For gripper-cube collision suppression, use `AttachedCollisionObject` with `touch_links` instead.

---

## Session 2026-02-21 — Continuation 3

### Cube visibility during pick-and-place

User wanted the cube to NEVER disappear from RViz. Previous approach removed the cube from the planning scene before grasping (making it invisible). New approach keeps the cube in the scene always.

**Solution: per-plan ACM + AttachedCollisionObject for transport**
- `_make_allowed_collision_diff(object_id)` — returns a `PlanningScene` diff with the cube marked as collision-passthrough via `AllowedCollisionMatrix.default_entry_names/values`. This diff is passed as `planning_options.planning_scene_diff` in specific plans that need to move through the cube's space. The world scene (and RViz) is unaffected.
- `_move_to_position(allowed_object=...)` and `_move_gripper(allowed_object=...)` / `close_gripper(allowed_object=...)` — accept an optional object ID to ignore during that specific plan
- `_attach_cube_to_gripper(cube_name)` — after gripper closes, attaches the cube to `grasp_link` via `AttachedCollisionObject` so it moves with the arm in RViz during transport. `touch_links = ["left_finger", "right_finger", "grasp_link", "gripper_base"]`
- `_detach_cube_from_gripper(cube_name)` — before opening gripper, detaches the cube (leaves it at its world position)
- `update_cube_position()` — re-adds the cube at the target position after detach+release

**Key constraint**: MoveIt's `planning_scene_diff.allowed_collision_matrix.default_entry_names/values` allows specific objects to be treated as collision-passthrough for a single planning request only, without modifying the global planning scene.

---

## Session 2026-02-21 — Continuation 2

### Bugs Fixed in pick-and-place flow

**Bug: place_cube_at() default target_z=0.025 caused unreachable target**
The `place_cube_at()` default was Z=0.025 (floor level), making the approach height 0.125m — well below the arm's workspace (cubes at Z=0.3). MoveIt returned FAILURE(general) for the target motion.
- Fixed: default is now `None` → derived from the cube's current Z position at call time, keeping the cube on the same surface level
- Also fixed: CLI `--target-z` default updated to `None` (was hardcoded 0.025)

**Bug: First motion always planned from home (0,0,0)**
The joint state subscriber was created after `_log_diagnostics()`, which runs a blocking subprocess (~3 seconds). During that blocking call, ROS callbacks can't fire, so `current_joint_state` is None when the first motion is planned. Result: if the arm was at a non-home position, the first plan was wrong.
- Fixed: moved subscriber creation to before `_log_diagnostics()`, added `rclpy.spin_once(node, timeout_sec=0.5)` immediately after subscriber creation to ensure the initial joint state is captured

---

## Session 2026-02-21

### Root Cause of Motion Timeout Bug

First move (home → blue) worked, but subsequent moves (blue → red) timed out. **Root cause**: MoveIt planning was using stale start state (always assuming home position) instead of the current arm position.

**Why it failed**: When `is_diff = True` in planning options without setting an explicit `start_state`, MoveIt defaults to assuming the robot is at home (0,0,0). Works fine for the first move, but after moving to blue cube, the next plan thinks the arm is still at home — planning invalid trajectories that violate constraints during execution.

### Solution
Added joint state subscription to track the arm's actual current position, then pass it as `start_state` in every MoveGroup goal:
1. Subscribe to `/joint_states` topic with callback to capture current state
2. Implement `_get_current_robot_state()` method that builds a RobotState from latest feedback
3. Set `goal_msg.request.start_state = self._get_current_robot_state()` in all three motion methods

**Why this works**: Now MoveIt always knows the true starting position before planning, generating valid trajectories from wherever the arm actually is.

### Key Constraint Learned
- **MoveIt2 planning state assumption**: When using `is_diff = True` without explicit `start_state`, MoveIt assumes home position. Always set `start_state` for every plan to ensure planning reflects the robot's actual current pose.

---

## Session 2026-02-18

### Summary of Fixes
- **Fixed CONTROL_FAILED error**: Replaced analytical IK with MoveIt's PositionConstraint approach (MoveIt handles IK internally via KDL `position_only_ik`)
- **Fixed controller initialization**: Explicitly added ros2_control controller_manager node to launch file with proper config path
- **Fixed YAML constraints syntax**: Changed per-joint constraint format to flat structure (`goal: position: 0.05` instead of nested `joint_1: goal: 0.05`)
- **Fixed mimic joint error**: Removed `right_finger_joint` from `moveit_controllers.yaml` (mimic joints cannot have command interfaces)
- **Added comprehensive debug logging**: Trajectory waypoints, execution status, and MoveIt error codes with human-readable descriptions for easier troubleshooting

---

## Session 2026-02-18 — Continuation

### Summary of Fixes
- **Fixed ros2_control crash**: The real culprit was `robotic_arm.ros2_control.xacro` in MoveIt config — it gave `right_finger_joint` a `command_interface`, conflicting with the URDF `<mimic>` tag. Removed command_interface, kept only state_interface.
- **Removed duplicate controller_manager**: `arm_system.launch.py` was starting an explicit `ros2_control_node` AND `demo.launch.py` started another. Removed the duplicate — demo.launch.py handles it.
- **Fixed moveit_controllers.yaml indentation**: `gripper_controller` was not indented under `moveit_simple_controller_manager`, causing "No action namespace" error.
- **Motion now works**: `ros2 run robotic_arm_bringup move_to_cube --cube blue` successfully plans and executes (163 waypoints, ~16s trajectory).

### Key Constraint Learned
- In ROS2 Jazzy, `<param name="mimic">` inside `<ros2_control>` tags is **deprecated** — mimic info is read from the standard URDF `<mimic>` tag only. Mimic joints cannot have command interfaces.
