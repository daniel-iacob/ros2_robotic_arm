# ROS2 Robotic Arm Project

## Goal
Learn ROS2 concepts with a simulated robotic arm + gripper + camera → eventually natural language control ("take the blue cube, put it in the basket").

**Timeline**: Simulation-only for next 6+ months

## Quick Start
```bash
./run.sh sim    # Launch RViz + MoveIt
```

## Architecture
- **ROS2 Jazzy** (Ubuntu 24.04)
- **MoveIt2** for motion planning
- **Mock hardware** (no Gazebo yet)
- **3 DOF arm** + parallel gripper
- **Scene objects**: Blue cube (0.4, 0, 0.025), Red cube (0, 0.4, 0.025)

## Key Files
- `src/robotic_arm_description/urdf/robotic_arm.urdf.xacro` - Robot model
- `src/robotic_arm_bringup/robotic_arm_bringup/scene_manager.py` - Scene setup
- `src/robotic_arm_moveit_config/` - MoveIt configuration
- `run.sh` - Build/launch wrapper

## Current State
✓ Arm + gripper URDF
✓ MoveIt2 integration
✓ Scene with colored cubes
✓ RViz visualization
✗ Camera (planned)
✗ LLM integration (planned)
