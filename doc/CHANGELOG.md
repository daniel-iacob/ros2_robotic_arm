# Changelog — ROS2 Robotic Arm

Historical session notes. For current status see [CLAUDE.md](../CLAUDE.md).
Patterns and constraints: [MEMORY.md](MEMORY.md). Architecture: [architecture.md](architecture.md).

---

## 2026-03-23 — Phase 4 design session

### Phase 4 — Camera + Vision pipeline designed
- **Approach**: Synthetic camera (no Gazebo) + real HSV color detection. `camera_node` renders MoveIt scene as top-down orthographic image; `vision_node` detects objects from pixels only (no ground truth access).
- **Package**: New `robotic_arm_perception` — clean separation, entire package swappable when migrating to Gazebo.
- **New messages**: `DetectedObject.msg` / `DetectedObjects.msg` in `robotic_arm_interfaces`.
- **Color config**: `vision_node` reads `objects.yaml` for color→name mapping (single source of truth).
- **Z handling**: Fixed Z from `objects.yaml` — top-down camera can't see height.
- **Existing changes**: `motion_server` gets `/detected_objects` subscriber; launch file adds new nodes.
- **Swap boundary**: `/camera/image_raw` topic. Everything downstream (vision_node, motion_server) works unchanged with Gazebo or real HW.
- **Design doc**: [`doc/camera_vision.md`](camera_vision.md) — includes mermaid diagrams.

---

## 2026-03-17 — URDF visual overhaul + place() START_STATE_IN_COLLISION fix

### place() lift collision — root cause and fix
- **Bug**: After placing red_cylinder, the subsequent green_cylinder pick failed. Root cause: `place()` re-added the object to the planning scene at `release_z` while the arm fingers were still at that height → `left_finger - red_cylinder` START_STATE_IN_COLLISION → lift plan rejected → green's approach was never reached.
- **Why ACM didn't help**: MoveIt's `CheckStartStateCollision` adapter fires before `planning_scene_diff` is applied. The `allowed_object` ACM entry has no effect on start state validation — only on path collision.
- **Fix**: Move arm 4cm upward BEFORE re-adding the object to the world scene. At that moment the object is not in the scene, so no collision is possible. Re-add object, then lift the remaining distance. Sequence: detach → remove → open gripper → move +4cm → re-add object → lift to +15cm.
- **All 24 tests passing** post-fix.

### URDF visual overhaul — Kuka orange gradient
- Replaced plain-color boxes with Kuka-style cylinder geometry. Orange gradient (c1–c4) from base to forearm; dark gripper (c5–c6).
- Added horizontal cylindrical knuckles at revolute joints (joint_2, joint_3) to mimic Kuka elbow appearance.
- **Critical rule followed**: only `<visual>` blocks modified. All `<joint>` origins, `<collision>` geometry/origins, and `<ros2_control>` section preserved byte-for-byte from commit c70dcf4.
- **Regression risk**: During visual work, a collision geometry change (box → cylinder on upper_arm) was accidentally introduced and caused 21/24 test failures. Reset to c70dcf4 and redid visuals with collision untouched.

### RViz — show URDF colors
- `MotionPlanning > Scene Robot`: `Show Robot Visual: false → true`, `Robot Alpha: 0.5 → 1`. Required to display URDF material colors on the current robot pose (not just the planned path ghost).

---

## 2026-03-16 — Fix pick failures across workspace swings

### joint_1 constraint — root cause and fix
- **Bug**: `pick` failed at "approaching object" after any `place` that left the arm in the +X/+Y quadrant (e.g. basket). Next pick targeting -Y hemisphere (red/green cylinders) would fail planning.
- **Root cause**: `_move_to_position()` adds a `JointConstraint` on `joint_1` biased toward `atan2(y, x)` with ±0.5 rad tolerance. After placing near the basket, joint_1 is ~+0.5 rad; target for red is -1.57 rad — outside the constraint window. MoveIt couldn't plan across the ~2 rad swing.
- **Fix**: widened tolerance to ±1.5 rad. The bias still guides toward natural configurations but no longer blocks cross-workspace moves.
- **All 24 tests passing** (2:27 run time).

---

## 2026-03-15 — Cylinders, basket, expanded tests, place() reorder

### Cylinder support
- **Switched cubes → cylinders** — cubes at diagonal positions couldn't be grasped because the gripper fingers hit cube corners at 45° angles. Cylinders are rotationally symmetric → graspable from any approach angle
- Added `shape` field support in `scene_manager.py` and `arm_controller.py` (`SolidPrimitive.CYLINDER` when `shape: cylinder`)
- Cylinder dimensions are `[height, radius]` (MoveIt convention), not `[x, y, z]`

### Scene changes
- **Basket (tray)** added — flat box (0.20×0.20×0.02m) at (0.4, 0.25, 0.26) for place targets
- 3 cylinders: blue (0.45, 0.0), red (0.0, -0.45), green (0.30, -0.30)
- Green at diagonal — validates that cylinders fix the orientation issue

### Test expansion (15 → 24 tests)
- Error handling: unknown object pick, unreachable move-to
- Movement range: negative Y, high Z
- State verification: check positions after place, after reset
- Round-trip: repick green cylinder, place at new position

### place() step reorder — confirmed working
- Object stays attached during lower step, detach happens after lowering → no more brief disappearance
- `allowed_object` ACM on lift step handles `START_STATE_IN_COLLISION` after re-add

### Lessons learned
- 3-DOF arm can't reliably reach diagonal positions (r > 0.45m) at low Z — keep objects on-axis or closer
- `place()` return value doesn't check lift success → silent failures cascade to next command
- MoveIt position tolerance (1cm sphere) means commanded ≠ actual gripper position — matters when re-adding objects near gripper

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
