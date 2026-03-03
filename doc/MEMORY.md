# Claude Memory — ROS2 Robotic Arm

Compact patterns and constraints accumulated across sessions.
Detailed context and history live in `CLAUDE.md`.
Historical session notes live in `doc/CHANGELOG.md`.
Architecture diagrams and design decisions live in `doc/architecture.md`.

---

## Project Structure (Phase 2)

- **Motion library**: `arm_controller.py` → `ArmController` class (importable)
- **CLI**: `arm_cli.py` → subcommands: `pick`, `place`, `home`, `move-to`, `open-gripper`, `close-gripper`, `reset`, `list-objects`
- **Shell wrapper**: `robotic_arm.sh` — forwards to `ros2 run robotic_arm_bringup arm ...`, shows help with no args
- **Object config**: `config/objects.yaml` → single source of truth (both `scene_manager` and `ArmController` read it)
- **Object IDs**: match YAML keys (`blue_cube`, `red_cube`)
- **Position persistence**: MoveIt is the authority for world objects. `/tmp/arm_object_positions.json` is the authority for held (attached) objects — written by `pick()`/`move_to()`, read by `_sync_object_positions()` at startup, cleared by `place()`/`reset()`.

---

## Cross-Process State (Critical)

Each `ros2 run` call is a **separate Python process**. `self.objects` in-memory state is lost between calls. Two persistence mechanisms:

1. **MoveIt world collision objects** — persist in `move_group` server. `_sync_object_positions()` reads them at startup for world objects.
2. **`/tmp/arm_object_positions.json` cache** — used for attached (held) objects only. MoveIt attached object poses are in **link frame** (`grasp_link`), not `base_link` — so they cannot be used as world positions.

**Why not use MoveIt for attached object positions**: `AttachedCollisionObject` pose is stored in the attach link's frame. Reading `obj.pose.position` gives near-zero values (offset relative to `grasp_link`), not the world position.

---

## ROS2 Executor / Message Delivery

**`time.sleep()` does NOT spin the ROS2 executor.**

For any planning scene change that must be processed BEFORE the next motion,
use the `/apply_planning_scene` **service** (synchronous) instead of the topic.
See `_apply_scene_sync()` in `arm_controller.py`.

---

## MoveIt2 Planning Scene — Attach/Detach

- Set both `scene_msg.is_diff = True` AND `scene_msg.robot_state.is_diff = True`
- REMOVE on a non-attached object is a **no-op** — does NOT remove from world scene

**`decoupleObject()` side effect — critical:**
When MoveIt processes a detach, it re-adds the object to the world at the arm's current
position → START_STATE_IN_COLLISION.

**Fix pattern after every `_detach_object()`:**
```python
self._detach_object(name)
self._remove_object_from_scene(name)
self.open_gripper()
self._move_to_position(x, y, z)
self.update_object_position(name, x, y, z)  # re-add with color from YAML
```

---

## ObjectColor

Always include `ObjectColor` when re-adding objects. MoveIt loses color on detach/ADD.
`update_object_position()` reads color from `objects.yaml` automatically.

---

## Per-plan ACM (`_make_allowed_collision_diff`)

**Must set `robot_state.is_diff = True`**. Per-plan ACM works for **arm group** descent.
Does NOT work for **gripper group**. Use `AttachedCollisionObject` with `touch_links` instead.

---

## MoveIt2 Planning — Start State

Always set `start_state = self._get_current_robot_state()` on every MoveGroup plan.

---

## Gripper Close — Error 99999

Attach object with `touch_links` BEFORE `close_gripper()`.

---

## AttachedCollisionObject Persistence

Persists in MoveIt server across CLI calls. Always detach before early returns.

---

## KDL IK Symmetry (joint_1)

For targets where `x=0` (e.g., red_cube at `(0, 0.5, 0.3)`), KDL finds two valid solutions:
`joint_1 = +π/2` and `joint_1 = -π/2`. The mirrored solution produces a ~39-second trajectory
that the controller rejects (INVALID_MOTION_PLAN).

**Fix**: Add a `JointConstraint` biasing `joint_1` to `math.atan2(y, x)` with `±0.5` tolerance
and `weight=0.1`. This guides KDL toward the natural shoulder angle without overconstraining IK.

---

## scene_manager — Service vs Topic

`scene_manager.py` uses `/apply_planning_scene` service (not topic) with verification:
applies each object, then queries `/get_planning_scene` to confirm it landed, retries once if missing.

---

## list-objects — World + Attached

`list_objects()` queries both `WORLD_OBJECT_GEOMETRY | ROBOT_STATE_ATTACHED_OBJECTS`.
World objects print with position. Attached objects print as `(held)`.

---

## Mimic Joints (ros2_control)

No `command_interface` — only `state_interface`. Mimic info from URDF `<mimic>` tag.

---

## open_loop_control

`arm_controller` requires `open_loop_control: true` in ros2_controllers.yaml for mock hardware.
