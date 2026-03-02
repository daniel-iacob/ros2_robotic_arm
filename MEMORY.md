# Claude Memory — ROS2 Robotic Arm

Compact patterns and constraints accumulated across sessions.
Detailed context and history live in `CLAUDE.md`.

---

## Project Structure (Phase 2)

- **Motion library**: `arm_controller.py` → `ArmController` class (importable)
- **CLI**: `arm_cli.py` → subcommands: `pick`, `place`, `home`, `move-to`, `open-gripper`, `close-gripper`, `reset`
- **Object config**: `config/objects.yaml` → single source of truth (both scene_manager and ArmController read it)
- **Object IDs**: match YAML keys (`blue_cube`, `red_cube` — not `blue_piece`)
- **Position persistence**: MoveIt is the authority. `_sync_object_positions()` queries `/get_planning_scene` at startup → overrides YAML defaults. After moving, `update_object_position()` writes back to MoveIt via `/apply_planning_scene`.

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
`place()` calls `_detach_object()` at entry as defensive cleanup.

---

## Mimic Joints (ros2_control)

No `command_interface` — only `state_interface`. Mimic info from URDF `<mimic>` tag.

---

## open_loop_control

`arm_controller` requires `open_loop_control: true` in ros2_controllers.yaml for mock hardware.
