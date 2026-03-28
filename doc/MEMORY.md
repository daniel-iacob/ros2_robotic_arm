# Claude Memory — ROS2 Robotic Arm

Compact patterns and constraints accumulated across sessions.
Detailed context and history live in `CLAUDE.md`.
Historical session notes live in `doc/CHANGELOG.md`.
Architecture diagrams and design decisions live in `doc/architecture.md`.

---

## Project Structure (Phase 4)

- **Motion server**: `motion_server.py` — persistent node, holds `ArmController` in memory, 7 actions + 2 services
- **Motion library**: `arm_controller.py` → `ArmController` class (importable)
- **CLI**: `arm_cli.py` → thin action client, sends goals to `motion_server`
- **Shell wrapper**: `robotic_arm.sh` — forwards to `ros2 run robotic_arm_bringup arm ...`
- **Interfaces**: `robotic_arm_interfaces` package — `.action`/`.srv`/`.msg` definitions
- **Object config**: `config/objects.yaml` → single source of truth
- **Object IDs**: match YAML keys (`blue_cube`, `red_cube`)
- **State**: lives in `motion_server` memory — no more `/tmp/` cache needed (legacy cache still in code)

---

## Executor Deadlock (Critical)

**`rclpy.spin_until_future_complete()` deadlocks if an executor is already spinning the node.**

When `ArmController` runs inside `motion_server` (which uses `MultiThreadedExecutor`), calling `spin_until_future_complete` tries to spin a node that's already being spun → deadlock.

**Fix**: `_wait_for_future()` in `arm_controller.py` checks `node.executor is not None`. If executor exists, polls with `time.sleep(0.01)` loop. If no executor (standalone mode), uses `spin_until_future_complete`.

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

---

## Phase 4 — Camera + Vision Design Decisions

- **No Gazebo** — synthetic `camera_node` renders MoveIt scene as top-down image
- **New package**: `robotic_arm_perception` (swappable for Gazebo later)
- **Swap boundary**: `/camera/image_raw` — `vision_node` and everything downstream is permanent code
- **`vision_node` must NOT access MoveIt** — only processes pixels from `/camera/image_raw`
- **Color config from `objects.yaml`** — single source of truth for both `camera_node` and `vision_node`
- **Z = fixed from YAML** — top-down orthographic camera can't determine height
- **Design doc**: `doc/camera_vision.md`
- **Status (2026-03-28)**: Phase 4 complete. Vision callback enabled with trylock + held-object filter + 2s cooldown + 2cm dead-zone. `move-object` CLI added. 42 tests, 42/42 passing.

## place() — Held-State Validation

`place()` must check `_is_object_attached(object_name)` before proceeding. Without this, placing
an unheld object succeeds silently — the detach/re-add sequence corrupts the planning scene.

---

## place() — Lift Before Re-add (Critical)

In `place()`, lift the arm to full clearance BEFORE re-adding the object to the scene.
Object not in scene = no collision possible during lift.
Old approach (lift 4cm → re-add → lift more) failed intermittently because MoveIt's position
tolerance meant 4cm wasn't always enough clearance.

---

## place() — Z-Drift Prevention

`update_object_position()` must use the intended `z`, NOT `release_z` (z - 0.05).
Using `release_z` causes 5cm drift per placement — objects sink below reachable workspace.

---

## Fresh Joint State After Motion

`_wait_for_fresh_joint_state()` in `arm_controller.py` polls `/joint_states` after each
successful motion until a newer reading arrives. Without this, the next plan may start from
a stale joint state → `START_STATE_IN_COLLISION`.

---

## Vision Callback Safety (Critical)

`_detected_objects_callback` in `motion_server.py` must follow these rules:

1. **Trylock (non-blocking)**: Use `self._lock.acquire(blocking=False)`. Blocking acquire at 10Hz
   consumes all executor threads while an action holds the lock → MoveGroup result callbacks can't
   fire → every motion times out at 30s.

2. **Skip held objects**: Track attached objects in `self._held_objects` set. Camera sees attached
   objects at the arm's position, not their actual position → vision would move them to ~(0,0).

3. **Per-object cooldown (2s)**: After pick/place/reset, set `self._vision_cooldown[name]`.
   Camera lags by ~100ms — stale frames show pre-action positions. Without cooldown, vision
   overwrites the correct post-action position with the stale pre-action one.

4. **Dead-zone (2cm)**: Ignore detections within 2cm of known position. Pixel quantization
   introduces ~1-3cm noise. Without dead-zone, positions oscillate continuously.
