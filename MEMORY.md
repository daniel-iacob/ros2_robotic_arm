# Claude Memory — ROS2 Robotic Arm

Compact patterns and constraints accumulated across sessions.
Detailed context and history live in `CLAUDE.md`.

---

## ROS2 Executor / Message Delivery

**`time.sleep()` does NOT spin the ROS2 executor.**

Topic publishes (`publisher.publish()`) enqueue the message at the DDS layer, but the
subscriber's callback only fires when an executor spins (`spin_once`,
`spin_until_future_complete`, etc.). A plain Python sleep does not spin anything.

**Consequence**: If you publish to `/planning_scene` and then `time.sleep(N)`, MoveIt has NOT
necessarily processed the update. The message will be delivered when the next
`rclpy.spin_*` call occurs — which may be inside the next motion goal submission, causing a
race condition.

**Fix**: For any planning scene change that must be processed BEFORE the next motion,
use the `/apply_planning_scene` **service** (synchronous) instead of the topic.
See `_apply_scene_sync()` in `move_to_cube.py`.

---

## MoveIt2 Planning Scene — Attach/Detach

- Attach: publish `AttachedCollisionObject` with `operation = ADD` via service ✓
- Detach: publish `AttachedCollisionObject` with `operation = REMOVE` via service ✓
- Set both `scene_msg.is_diff = True` AND `scene_msg.robot_state.is_diff = True`
- `link_name` is required on attach; not required on detach (MoveIt searches all links by ID)
- REMOVE on a non-attached object is a **no-op on world objects** — it does NOT remove the
  cube from the world scene, only from attached objects

**`decoupleObject()` side effect — critical:**
When MoveIt processes a detach (REMOVE on AttachedCollisionObject), it ALSO re-adds the cube
as a world collision object at the arm's current TF position. If the arm is still at that
position, any subsequent plan will fail with START_STATE_IN_COLLISION.

**Fix pattern after every `_detach_cube_from_gripper()`:**
```python
self._detach_cube_from_gripper(cube_name)
self._remove_cube_from_scene(cube_name)  # remove what decoupleObject re-added
self.open_gripper()                       # no collision, no ACM needed
self._move_to_position(x, y, z)           # plan freely
self.update_cube_position(cube_name, x, y, z)  # re-add with color
```
Do NOT use `allowed_object` (per-plan ACM) for post-detach ops — unreliable for gripper
group AND arm group (unless `_make_allowed_collision_diff` has `robot_state.is_diff = True`).

**For `--home`:**
Try `go_home()` first (no visual disruption). If it fails (arm stuck at cube position),
remove all world cubes, retry, then re-add all with `update_cube_position()`.

---

## ObjectColor — Cube Colors in RViz

When re-adding cubes to the planning scene (via `CollisionObject.ADD`), always include
`ObjectColor` in the `PlanningScene` message. MoveIt's `decoupleObject()` and plain ADD
operations lose the color set by `scene_manager.py` at startup → cubes appear green.

`update_cube_position()` handles this automatically (includes blue/red `ColorRGBA`).

---

## Per-plan ACM (`_make_allowed_collision_diff`)

**Must set `scene_diff.robot_state.is_diff = True`**. Without it, MoveIt resets the planning
scene's robot state (empty state with `is_diff=False`), effectively ignoring the ACM.

Per-plan ACM works for **arm group** grasp approach (descent into cube). Does NOT work
reliably for **gripper group** (error 99999 or -10). For gripper-cube interaction, use
`AttachedCollisionObject` with `touch_links` instead.

---

## MoveIt2 Planning — Start State

Always set `goal_msg.request.start_state = self._get_current_robot_state()` on every
MoveGroup plan. Without it MoveIt assumes the robot is at home (0,0,0), which causes wrong
trajectories for any move after the first.

---

## Gripper Close — Error 99999

Attach the cube to the gripper (`AttachedCollisionObject` with `touch_links`) BEFORE calling
`close_gripper()`. Per-plan ACM (`planning_scene_diff`) works for arm group but NOT reliably
for gripper group. `touch_links` is the only reliable way to allow finger-cube contact.

---

## AttachedCollisionObject Persistence

`AttachedCollisionObject` lives in the MoveIt server, not in the Python process.
It persists across CLI invocations. Any early exit that doesn't detach leaves the cube
attached for ALL future commands in the same MoveIt session.

**Pattern**: call `_detach_cube_from_gripper(cube_name)` at the start of any function
that will re-attach a cube (defensive cleanup), and before every early `return False`
after an attachment has been made.

---

## Mimic Joints (ros2_control)

Mimic joints cannot have `command_interface` in `ros2_control` config — only
`state_interface`. Mimic info is read from the standard URDF `<mimic>` tag.

---

## open_loop_control

`arm_controller` requires `open_loop_control: true` in ros2_controllers.yaml for mock
hardware (bypasses state feedback checks that don't apply to mock).
