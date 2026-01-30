# ROS 2 Development Instructions (Jazzy)

## Project Context
- This is a ROS 2 Jazzy project using Python (`ament_python`).
- The workspace follows the standard Overlay/Underlay pattern.
- We use three primary packages: `robotic_arm_description` (URDF/meshes), `robotic_arm_controller` (Logic/Nodes), and `robotic_arm_bringup` (Launch files).

## Code Style & Standards
- **Naming:** Always use `snake_case` for packages, nodes, and topics.
- **Node Structure:** Use the Object-Oriented approach (class inheriting from `rclpy.node.Node`).
- **Imports:** Use explicit imports (e.g., `import glob` instead of `from glob import glob`) to avoid namespace collisions in `setup.py`.

## Build Instructions
- Always suggest `colcon build --symlink-install` for Python packages.
- When adding new scripts, remind the user to update the `entry_points` in `setup.py`.
- When adding launch files, remind the user to update `data_files` in `setup.py` using `glob.glob`.

## ROS 2 Specifics
- Joint Trajectories: When publishing to `/arm_controller/joint_trajectory`, always include a `time_from_start` to avoid "Goal in the past" errors.
- Packages: Differentiate between the "Outer" package directory and the "Inner" Python module directory.
