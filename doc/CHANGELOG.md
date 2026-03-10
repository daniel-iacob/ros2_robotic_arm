# Changelog — ROS2 Robotic Arm

Historical session notes. For current status see [CLAUDE.md](../CLAUDE.md).
Patterns and constraints: [MEMORY.md](MEMORY.md). Architecture: [architecture.md](architecture.md).

---

## 2026-03-10 — Integration test suite
- Added `./run.sh tests` — pytest-based system tests that run CLI commands against a live sim

---

## 2026-03-09 — Phase 3: Motion Action Server
- **Phase 3 complete**: persistent `motion_server` node with 7 actions + 1 service
- New `robotic_arm_interfaces` package for `.action`/`.srv`/`.msg` definitions
- CLI rewritten as thin action client — no longer creates `ArmController` directly
- Added `on_progress` callback to `pick()`, `place()`, `reset()` for step-by-step feedback
- `place` CLI now accepts optional `x y [z]` position args (was missing before)
- **Fix: executor deadlock** — `ArmController` called `rclpy.spin_until_future_complete()` on a node already being spun by `MultiThreadedExecutor` → deadlock. Solution: `_wait_for_future()` polls when executor is active, spins when standalone
- Condensed CLAUDE.md and CHANGELOG.md — focus on why, not how

## 2026-03-03
- Added `list-objects` command (queries MoveIt for world + attached objects)
- Added `robotic_arm.sh` shell wrapper
- **Fix: `place` releasing at original pick position** — `AttachedCollisionObject` pose is in link frame, not world frame. Solution: `/tmp/arm_object_positions.json` cache persists world positions across CLI calls
- **Fix: KDL IK symmetry** — targets near x=0 get mirrored joint_1 solutions → 39s trajectory → rejected. Solution: `JointConstraint` biasing joint_1 toward `atan2(y,x)`
- **Fix: `scene_manager` dropping objects** — `time.sleep()` doesn't spin executor, so topic messages never delivered. Solution: use `/apply_planning_scene` service
- Speed: velocity/acceleration scaling 0.1 → 0.4

## 2026-02-25 — Phase 2 Refactor
- Extracted monolithic `move_to_cube.py` into `ArmController` class + `arm_cli.py` + `objects.yaml`
- Generic API: `pick(object_name)` not `move_to_cube(cube_name)`
- Added `reset()` for full recovery

## 2026-02-25 — Bug Fixes
- **Fix: per-plan ACM missing `robot_state.is_diff = True`** — empty robot_state resets planning scene state, ACM ignored
- **Fix: per-plan ACM unreliable for gripper group** — use `AttachedCollisionObject` with `touch_links` instead
- **Fix: cubes reappear green after detach** — MoveIt loses color; must include `ObjectColor` when re-adding
- **Fix: cubes disappear on `--home`** — was removing all objects before trying home. Now tries home first, only removes on failure

## 2026-02-24 — Detach Race Condition
- **Fix: `decoupleObject()` re-adds to world** — detaching an object puts it back as a world collision object at arm position → START_STATE_IN_COLLISION. Solution: `_remove_object_from_scene()` after every detach
- **Fix: topic-based scene updates race with planning** — `time.sleep()` doesn't spin executor. Solution: `/apply_planning_scene` service (synchronous)

## 2026-02-22
- **Fix: cube follows arm on `--home`** — no detach cleanup before going home. Solution: `_detach_all()` before `go_home()`
- **Fix: cube stays attached on failed `place`** — early returns without detach. Solution: detach before every `return False`
- Learned: `AttachedCollisionObject` persists in MoveIt server across CLI invocations

## 2026-02-21
- **Fix: gripper close error 99999** — per-plan ACM doesn't work for gripper group. Solution: attach with `touch_links` BEFORE close
- **Fix: subsequent moves plan from home** — MoveIt defaults to (0,0,0) without explicit `start_state`. Solution: subscribe to `/joint_states`, pass current state in every plan
- Cube visibility: keep cube in scene always (per-plan ACM for descent, `AttachedCollisionObject` for transport)

## 2026-02-18
- Initial MoveIt2 integration working
- Fixed analytical IK → switched to MoveIt's `PositionConstraint` approach
- Fixed mimic joint crash — removed command interface from `right_finger_joint`
- Fixed controller config — `open_loop_control: true` for mock hardware, position-only state interfaces
