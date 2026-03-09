# ROS2 Robotic Arm Project

## Goal
Learn ROS2 concepts with a simulated robotic arm + gripper + camera → eventually natural language control ("take the blue cube, put it in the basket"). Simulation-only for next 6+ months.

## Instructions for Claude

**Documentation style — focus on WHY, not HOW:**
- The code is the source of truth for implementation details. Don't describe how things are implemented — document **decisions, constraints, and lessons learned**.
- No code examples in CLAUDE.md. Use `--help` or read the source.
- Keep CLAUDE.md **under 150 lines**. Move session notes to `doc/CHANGELOG.md` after each session.

**Always update this file** when you:
- Make an architectural or design decision
- Fix a non-obvious bug — document root cause and lesson
- Discover a constraint or ROS2/MoveIt quirk

**Also read and update these `doc/` companion files**:
- [`doc/MEMORY.md`](doc/MEMORY.md) — compact patterns and constraints
- [`doc/architecture.md`](doc/architecture.md) — node diagrams, topics, design decisions
- [`doc/CHANGELOG.md`](doc/CHANGELOG.md) — historical session notes

---

## Quick Reference

**What this is**: ROS2 Jazzy simulation of a 3-DOF arm + gripper. Phase 2 — importable motion library + CLI with MoveIt2 mock hardware. No Gazebo, no camera, no LLM yet.

**Quick start**: `./run.sh sim` → launches RViz + MoveIt

**Running nodes** (after `./run.sh sim`):
- `scene_manager` — one-shot, spawns objects from `objects.yaml` into MoveIt
- `move_group` — MoveIt2 planning server
- `arm_controller` / `gripper_controller` / `joint_state_broadcaster` — ros2_control
- `robot_state_publisher` + `rviz2`

**Entry points**:
- CLI: `ros2 run robotic_arm_bringup arm <command> [args]`
- Python: `from robotic_arm_bringup.arm_controller import ArmController`

**Key files**:
- Robot model: `src/robotic_arm_description/urdf/robotic_arm.urdf.xacro`
- Motion library: `src/robotic_arm_bringup/robotic_arm_bringup/arm_controller.py`
- CLI wrapper: `src/robotic_arm_bringup/robotic_arm_bringup/arm_cli.py`
- Object definitions: `src/robotic_arm_bringup/config/objects.yaml`
- Scene setup: `src/robotic_arm_bringup/robotic_arm_bringup/scene_manager.py`
- MoveIt config: `src/robotic_arm_moveit_config/`
- Launch: `src/robotic_arm_bringup/launch/arm_system.launch.py`

---

## Critical Constraints

These have caused bugs. Always remember them.

| Constraint | What breaks if ignored |
|------------|------------------------|
| Always set `start_state` on every MoveGroup plan | Second+ moves plan from home (0,0,0) → wrong trajectories |
| `AttachedCollisionObject` persists across CLI calls | Cube stays glued to arm across commands |
| Detach object before every `return False` in `place()` | Object stays attached after failed place |
| After `_detach_object`, call `_remove_object_from_scene` | MoveIt re-adds object at arm position → START_STATE_IN_COLLISION |
| Always re-add objects with `ObjectColor` after scene removal | Objects reappear green (MoveIt default) |
| `_make_allowed_collision_diff` must set `robot_state.is_diff = True` | ACM ignored → START_STATE_IN_COLLISION |
| Attach cube with `touch_links` BEFORE `close_gripper()` | Error 99999 (per-plan ACM doesn't work for gripper group) |
| Use `/apply_planning_scene` service (not topic) for attach/detach | `time.sleep()` doesn't spin executor → race condition |
| Mimic joints: no command interface in ros2_control | ros2_control crash on startup |

---

## Current State

- ✓ Arm + gripper URDF with MoveIt2 + KDL IK
- ✓ RViz visualization with colored collision objects
- ✓ Importable `ArmController` library + thin CLI
- ✓ Pick-and-place with planning scene updates
- ✓ YAML-based object config (`objects.yaml`)
- ✗ CLI `place` doesn't accept target position args (use `move-to` then `place`)
- ✗ Camera / vision (Phase 4)
- ✗ LLM integration (Phase 5)

## Roadmap

- [x] **Phase 1** — CLI motion control, MoveIt2 integration, pick-and-place
- [x] **Phase 2** — Importable `ArmController` library + YAML config + thin CLI
- [ ] **Phase 3** — Persistent ROS2 action server (replaces one-shot CLI)
- [ ] **Phase 4** — Camera + vision-based object detection
- [ ] **Phase 5** — LLM interface for natural language control

## Known Limitations
- 3-DOF: cannot achieve arbitrary orientations
- Mock hardware: no physics simulation
- Object positions reset to YAML defaults if MoveIt restarts

---

## Latest Session Changes (2026-03-09)

- Fixed CLAUDE.md: `place` CLI docs incorrectly showed position args (not wired up)
- Condensed CLAUDE.md and CHANGELOG.md — removed implementation details, focused on why

## Previous Session (2026-03-03)

- Added `list-objects` command and `robotic_arm.sh` wrapper
- Fixed `place` releasing at pick position (not move-to destination) — position cache
- Fixed KDL IK symmetry for targets near x=0 — joint_1 bias constraint
- Fixed `scene_manager` dropping objects — service instead of topic
- Speed: velocity/acceleration scaling 0.1 → 0.4
