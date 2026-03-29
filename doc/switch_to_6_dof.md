# Plan: Switch to AR3 6-DOF Arm

## Context
The codebase is fully decoupled from arm-specific values via `arm_config.yaml`. The next phase
is replacing the custom 3-DOF URDF + MoveIt config with the AR3 6-DOF arm. Goal: pick-and-place
working again on the new model, all 44 tests passing, `./run.sh sim` launching AR3 in RViz.

The AR3 is a 6-DOF 3D-printable hobby arm by Chris Annin. Community ROS2 URDF + MoveIt config
repos exist. The switch avoids designing arm geometry from scratch while getting real 6-DOF
kinematics that are representative of real hardware.

**Why AR3 before Gazebo:** Gazebo requires adding inertia/physics tags to the URDF. Doing that
work on the 3-DOF arm and then switching would mean doing it twice. Switching first means Gazebo
work happens once, on the right model.

---

## Pre-migration prep (already done — 2026-03-29)

- `arm_config.yaml` created at `src/robotic_arm_bringup/config/arm_config.yaml`
- `arm_controller.py` reads all joint names, group names, home position, gripper values, motion offsets from config
- `scene_manager.py` reads `base_frame` from config
- No hardcoded arm-specific values remain in Python source
- `move_to_cube.py` deleted (legacy, was never called)

---

## Step 1 — Find and pull AR3 URDF + MoveIt config

Search for a maintained AR3 ROS2 (Jazzy/Humble) URDF + MoveIt config repo.

Options to check:
- https://github.com/Annin-Robotics/ar4_ros_driver — official Annin Robotics driver (AR4, same kinematics as AR3)
- https://github.com/RIF-Robotics/ar3_ros — community AR3 ROS2 package
- https://github.com/ongdexter/ar3_core/tree/master — community AR3 core
- `github.com/WPI-ME-CS/AR3`
- `github.com/DARoSLab/AR3_ROS2`
- GitHub search: "AR3 ROS2 Jazzy" or "AR3 MoveIt2"
- ar2ar3.com community forum

**Validation checklist before proceeding:**
- URDF has `base_link`, exactly 6 revolute/continuous arm joints, gripper joint(s)
- SRDF exists with named planning groups for arm and gripper
- End-effector link is defined (e.g. `tool0`, `grasp_link`, or similar)
- ros2_control section exists OR plan to write it
- Check for `<mimic>` tags on gripper joints — note which joints are mimic (no command interface)

**Document before continuing:** exact joint names (in order), group names, EEF link name, base link name, mimic joints.

---

## Step 2 — Replace URDF

**File:** `src/robotic_arm_description/urdf/robotic_arm.urdf.xacro`

- Copy AR3 URDF/xacro into the package
- **CRITICAL:** Do NOT modify joint origins, collision geometries, or kinematic chain

---

## Step 3 — Replace MoveIt config + ros2_control xacro

**Directory:** `src/robotic_arm_moveit_config/config/`

Replace with AR3 equivalents:
- `robotic_arm.srdf` — planning groups, home state, end-effector chain
- `kinematics.yaml` — IK solver (KDL, search_resolution: 0.005)
- `joint_limits.yaml` — AR3 joint velocity/acceleration limits (use 0.5 rad/s if unknown)
- `ros2_controllers.yaml` — controller joint lists
- `moveit_controllers.yaml` — MoveIt controller joint lists
- `initial_positions.yaml` — home position (check AR3 docs for safe home, don't assume all zeros)
- `robotic_arm.ros2_control.xacro` — **must list all 6 arm joints + gripper joint(s)**
  - Mimic joints: `state_interface` only, no `command_interface` (same as current right_finger_joint)
  - This file is often absent in community repos — may need to write from scratch using the current file as template

Keep unchanged (not arm-specific):
- `pilz_cartesian_limits.yaml`
- `ompl_planning.yaml`

**After replacing, verify 4-file joint consistency:**
```bash
grep -E "joint_[0-9]|joint name" robotic_arm.urdf.xacro
grep "joints:" ros2_controllers.yaml
grep "joints:" moveit_controllers.yaml
grep "joint name" robotic_arm.ros2_control.xacro
```
All four must list the same joint names.

---

## Step 4 — Update arm_config.yaml

**File:** `src/robotic_arm_bringup/config/arm_config.yaml`

Update with AR3's actual values from Step 1 doc:
- `arm.joints` — list of 6 joint names **in the same order as /joint_states publishes them**
  - Wrong order = start state sent to MoveIt with joints mapped to wrong positions → wrong trajectories
  - Validate after launch: `ros2 topic echo /joint_states --once` and confirm order matches
- `arm.home` — safe home angles (from AR3 docs or reference builds)
- `arm.base_rotation_joint` — first joint name (for IK biasing in `_move_to_position`)
- `arm.end_effector_link` — from Step 1 doc
- `arm.base_frame` — from Step 1 doc (likely `base_link`)
- `arm.planning_group` — exact SRDF group name
- `gripper.planning_group` — exact SRDF gripper group name
- `gripper.joint_name` — actuated gripper joint (not mimic)
- `gripper.open_position` / `close_position` — from URDF joint limits
- `gripper.touch_links` — AR3 finger link names

**Motion offsets** (`approach_z`, `lift_z`, etc.) are a starting point only — validate experimentally with AR3's actual gripper width. The current values (10cm approach, 2cm descend) were tuned for the 3-DOF gripper.

---

## Step 5 — Update objects.yaml positions

**File:** `src/robotic_arm_bringup/config/objects.yaml`

AR3 reach: ~0.6–0.8m from base. Keep same relative layout (objects in front, basket to side).

**Validate each position before testing pick-and-place:**
```bash
ros2 run robotic_arm_bringup arm move-to <x> <y> <z>   # must succeed
```
Do this for each object position. Only proceed to Step 7 after all positions are confirmed reachable.

Also determine workspace limits for test updates:
```bash
ros2 run robotic_arm_bringup arm move-to 5.0 0.0 0.4   # expect failure (unreachable)
```

---

## Step 6 — Verify launch + basic motion

```bash
./run.sh sim
```
- AR3 appears in RViz (correct geometry, no explosions, collision geometries align with visual)
- `ros2 topic echo /joint_states --once` — confirm joint order matches arm_config.yaml
- `arm home` → arm moves to home; check RViz shows correct home pose
- `arm move-to 0.5 0.0 0.4` → arm reaches a reachable point

If `arm home` fails with `PLANNING_FAILED` or `NO_IK_SOLUTION` → kinematics.yaml or SRDF has wrong group/chain. Debug before proceeding.

---

## Step 7 — Validate pick-and-place

- `arm pick blue_cylinder` → arm grasps, object stays attached during lift
- `arm place 0.4 0.25 0.28` → arm places in basket, object released
- Validate motion offsets: if arm collides with table on descend → increase `descend_z_offset`; if object drops before resting on surface → increase `place_lower_z_offset`
- Vision pipeline: `camera_node` + `vision_node` unchanged

---

## Step 8 — Fix tests

**File:** `tests/test_arm.py`

- `test_move_to_*`: use coordinates validated reachable in Step 5
- `test_unreachable_*`: use `5.0 0.0 0.4` (well beyond AR3 reach)
- Pick/place test coordinates: match updated objects.yaml positions

Target: `./run.sh tests` → 44/44 passing.

Common failures and root causes:
- `PLANNING_FAILED` in move-to tests → coordinate outside workspace, adjust
- `CONTROL_FAILED` in gripper tests → gripper joint name wrong in arm_config.yaml
- `START_STATE_IN_COLLISION` → joint state sync issue or object position overlaps robot

---

## Key Risks

| Risk | Mitigation |
|------|-----------|
| No maintained AR3 ROS2 Jazzy URDF exists | Adapt Humble config (usually just cmake/package.xml changes) or use AR2 (same geometry) |
| robotic_arm.ros2_control.xacro missing from AR3 repo | Write from scratch using current file as template; only joint names change |
| Mimic gripper joints get a command_interface | ros2_control crashes — check `<mimic>` tags and ensure no command_interface |
| Joint order in arm_config.yaml wrong | All trajectories send wrong positions — validate against `/joint_states` order immediately after launch |
| AR3 IK fails for object positions | Adjust objects.yaml; if KDL fails consistently, try TRAC-IK plugin |
| Motion offsets need retuning for AR3 gripper | Validate pick descend/approach heights empirically before running tests |
| AR3 home position is not all-zeros | Check docs/reference builds; wrong home causes collision on startup |

---

## What stays unchanged

- `arm_controller.py` — reads from config, no arm-specific code
- `motion_server.py` — fully arm-agnostic
- `arm_cli.py` — fully arm-agnostic
- `scene_manager.py` — reads base_frame from config
- `camera_node.py` / `vision_node.py` — perception pipeline unchanged
- Action/service interfaces
- `run.sh` sim entry point
- `pilz_cartesian_limits.yaml`, `ompl_planning.yaml`

---

## Verification

1. `./run.sh sim` → AR3 in RViz, no errors
2. `ros2 topic echo /joint_states --once` → joint order matches arm_config.yaml
3. `arm home` → moves to correct home pose
4. `arm pick blue_cylinder` → picks object
5. `arm place 0.4 0.25 0.28` → places in basket
6. `./run.sh tests` → 44/44 passing
