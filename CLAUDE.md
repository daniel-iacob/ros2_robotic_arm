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

**Also read and update these `doc/` companion files**:
- **[`doc/MEMORY.md`](doc/MEMORY.md)** — compact patterns and constraints (update when you learn something new about MoveIt/ROS2 behavior, or when a previously documented pattern is corrected)
- **[`doc/architecture.md`](doc/architecture.md)** — update when adding/removing nodes, topics, actions, or making architectural decisions
- **[`doc/CHANGELOG.md`](doc/CHANGELOG.md)** — move session notes here at the end of each session (keep Latest Session Changes in CLAUDE.md, archive older ones to CHANGELOG.md)

---

## Quick Reference for Claude

> Read this first. It gives you enough context to start working without parsing the whole file.
> Also read **[`doc/MEMORY.md`](doc/MEMORY.md)** — compact accumulated patterns and constraints.
> Historical session notes live in **[`doc/CHANGELOG.md`](doc/CHANGELOG.md)**.
> Architecture diagrams and design decisions live in **[`doc/architecture.md`](doc/architecture.md)**.

**What this is**: ROS2 Jazzy simulation of a 3-DOF arm + gripper. Goal: eventually LLM-controlled manipulation. Currently Phase 2 — importable motion library + CLI with MoveIt2 mock hardware. No Gazebo, no camera, no LLM yet.

**Running nodes** (after `./run.sh sim`):
- `scene_manager` — one-shot startup node, spawns objects from `objects.yaml` into MoveIt
- `move_group` — MoveIt2 planning server, action server at `/move_action`
- `arm_controller` / `gripper_controller` / `joint_state_broadcaster` — ros2_control layer
- `robot_state_publisher` — TF broadcaster
- `rviz2` — visualization

**Entry point for motion**: `ros2 run robotic_arm_bringup arm <command> [args]`
**Motion library**: `src/robotic_arm_bringup/robotic_arm_bringup/arm_controller.py` (`ArmController` class)
**CLI wrapper**: `src/robotic_arm_bringup/robotic_arm_bringup/arm_cli.py`

**Critical constraints — always remember these**:

| Constraint | What breaks if ignored |
|------------|------------------------|
| Always set `start_state` on every MoveGroup plan | Second+ moves plan from home (0,0,0) → wrong trajectories |
| `AttachedCollisionObject` persists across CLI calls | Cube stays glued to arm across commands; `--home` drags it home |
| Detach object before every `return False` in `place()` | Object stays attached after failed place; subsequent moves carry it |
| After `_detach_object`, call `_remove_object_from_scene` before open_gripper/move | MoveIt `decoupleObject()` re-adds object to world at arm's position → START_STATE_IN_COLLISION (per-plan ACM unreliable for gripper group) |
| `home()`: try `_go_home()` first, remove objects only on failure | Removing all objects before go_home causes them to disappear from RViz unnecessarily |
| Always re-add objects with `ObjectColor` after any scene removal | Objects reappear as green (MoveIt default) — `update_object_position()` reads color from YAML |
| `_make_allowed_collision_diff` must set `robot_state.is_diff = True` | Empty robot_state with `is_diff=False` resets planning scene state → ACM ignored → START_STATE_IN_COLLISION |
| Attach cube with `touch_links` BEFORE `close_gripper()` | Gripper close returns error 99999 (per-plan ACM doesn't work for gripper group) |
| Use `/apply_planning_scene` service (not topic) for attach/detach | `time.sleep()` does NOT spin ROS2 executor; topic publishes aren't processed until next spin, creating a race with the next motion plan |
| Mimic joints: no command interface in ros2_control | ros2_control crash on startup |

**Key files by task**:
- Robot model / joints: `src/robotic_arm_description/urdf/robotic_arm.urdf.xacro`
- Motion library: `src/robotic_arm_bringup/robotic_arm_bringup/arm_controller.py`
- CLI wrapper: `src/robotic_arm_bringup/robotic_arm_bringup/arm_cli.py`
- Object definitions: `src/robotic_arm_bringup/config/objects.yaml`
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
- **Object definitions**: `src/robotic_arm_bringup/config/objects.yaml` (single source of truth)
- **blue_cube**: (0.5, 0, 0.3) - 5cm box, right of base at table height
- **red_cube**: (0, 0.5, 0.3) - 5cm box, forward of base at table height
- **Coordinate system**: base_link frame (X-forward, Y-left, Z-up)
- **Collision objects**: Objects added to MoveIt2 planning scene via `scene_manager`

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

**CLI** (`ros2 run robotic_arm_bringup arm <command>`):
```bash
# Pick up an object
ros2 run robotic_arm_bringup arm pick blue_cube

# Pick and place to new position (x y [z])
ros2 run robotic_arm_bringup arm place blue_cube 0.4 0.1
ros2 run robotic_arm_bringup arm place blue_cube 0.4 0.1 0.35

# Move end-effector to arbitrary position
ros2 run robotic_arm_bringup arm move-to 0.5 0.0 0.4

# Gripper control
ros2 run robotic_arm_bringup arm open-gripper
ros2 run robotic_arm_bringup arm close-gripper

# Return home
ros2 run robotic_arm_bringup arm home

# Full recovery (detach all, home, reset scene to YAML defaults)
ros2 run robotic_arm_bringup arm reset
```

**Python API** (importable library):
```python
from robotic_arm_bringup.arm_controller import ArmController
controller = ArmController(node)
controller.pick("blue_cube")
controller.place("blue_cube", 0.4, 0.1)
controller.home()
controller.reset()
```

**Implementation**: `ArmController` class using MoveGroup action client (`/move_action`).
**Object config**: `objects.yaml` — single source of truth for names, positions, colors.
**Scene Management**: Object positions tracked and updated in planning scene after moves.

## Key Files
- `src/robotic_arm_description/urdf/robotic_arm.urdf.xacro` - Robot model
- `src/robotic_arm_bringup/config/objects.yaml` - Object definitions (names, positions, colors)
- `src/robotic_arm_bringup/robotic_arm_bringup/arm_controller.py` - Motion library (`ArmController`)
- `src/robotic_arm_bringup/robotic_arm_bringup/arm_cli.py` - CLI wrapper
- `src/robotic_arm_bringup/robotic_arm_bringup/scene_manager.py` - Scene setup (reads objects.yaml)
- `src/robotic_arm_moveit_config/` - MoveIt configuration
- `run.sh` - Build/launch wrapper

## Current State
✓ Arm + gripper URDF
✓ MoveIt2 integration with KDL IK solver
✓ Scene with colored cubes (collision objects)
✓ RViz visualization
✓ Programmatic motion control (importable `ArmController` library + CLI)
✓ Gripper control (open/close)
✓ Pick-and-place with planning scene updates
✓ Dynamic object position tracking
✓ YAML-based object configuration (`objects.yaml`)
✗ Camera (planned)
✗ Vision-based object detection (planned)
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
- **Object tracking**: `self.objects` dict tracks current positions; initialized from `objects.yaml`, then overridden by querying MoveIt via `/get_planning_scene` at startup (`_sync_object_positions()`). This ensures positions persist across CLI invocations — MoveIt is the authority.
- **Scene updates**: `_apply_scene_sync()` — service call with topic fallback
- **Sequence**: Approach → Descend (ACM) → Attach → Close gripper → Lift → Move → Lower → Detach → Remove from scene → Open gripper → Lift away → Re-add object with color
- **Post-detach pattern**: `_remove_object_from_scene()` after every detach (decoupleObject re-adds at arm position)
- **Colors**: `update_object_position()` reads color from `objects.yaml` — MoveIt loses color on detach

### Controller Configuration
- **arm_controller**: `open_loop_control: true` — required for mock hardware (bypasses state feedback checks)
- **state_interfaces**: `position` only (URDF exports only position, not velocity — must match controller config)
- **gripper_controller**: standard joint_trajectory_controller for left_finger_joint

### Known Limitations
- **3-DOF workspace**: Cannot achieve arbitrary orientations (missing wrist roll/pitch/yaw)
- **Mock hardware**: No physics simulation (Gazebo integration planned)
- **Object positions**: Persist via MoveIt queries between CLI calls, but reset to YAML defaults if MoveIt restarts

### Future Architecture
- **Phase 1** (Complete): Simple CLI script with MoveGroup action client
- **Phase 2** (Current): Importable motion library (`ArmController`) + YAML config + thin CLI
- **Phase 3** (Planned): ROS2 action server for async control
- **Phase 4** (Planned): Camera integration for dynamic object detection
- **Phase 5** (Planned): LLM interface for natural language control

---

## Latest Session Changes (2026-03-03)

### Bug fixes, new commands, and documentation reorganisation

**New**: `robotic_arm.sh` — simplified shell entry point; shows help with no args, forwards all commands to `ros2 run robotic_arm_bringup arm ...`.

**New**: `list-objects` subcommand — queries live MoveIt planning scene for both world and attached objects. World objects print with position; held objects print as `(held)`.

**New**: `_is_object_attached()` helper — queries `ROBOT_STATE_ATTACHED_OBJECTS` from MoveIt to check if an object is currently held.

**Fix: `place` releasing at original pick position instead of `move-to` destination**
- Root cause: `AttachedCollisionObject` pose is stored in `grasp_link` frame, not `base_link`. Reading it back gave near-zero values. Re-attaching with the Cartesian target also wrote link-relative data.
- Fix: `/tmp/arm_object_positions.json` cache persists held object world positions across CLI invocations. `pick()` writes it; `move_to()` updates it; `place()` clears the entry; `reset()` clears all. `_sync_object_positions()` reads the cache for attached objects at startup.

**Fix: KDL IK symmetry for red_cube at `(0, 0.5, 0.3)`**
- Root cause: KDL found `joint_1 = -π/2` (mirrored) → 39s trajectory → INVALID_MOTION_PLAN.
- Fix: `JointConstraint` on `joint_1` biased to `math.atan2(y, x)` with `±0.5` tolerance, weight `0.1`.

**Fix: `scene_manager` dropping objects on startup**
- Root cause: topic + `time.sleep()` — `time.sleep()` doesn't spin executor, messages dropped.
- Fix: Rewrote to use `/apply_planning_scene` service with verification + one retry.

**Fix: `list-objects` not showing held objects**
- Query both `WORLD_OBJECT_GEOMETRY | ROBOT_STATE_ATTACHED_OBJECTS` in one request.

**Speed**: `max_velocity_scaling` and `max_acceleration_scaling` raised from `0.1` → `0.4`.

**Docs**: Moved `MEMORY.md` from repo root to `doc/MEMORY.md`. Updated CLAUDE.md to reference all three `doc/` companion files.

**No rebuild required** — Python-only changes.
