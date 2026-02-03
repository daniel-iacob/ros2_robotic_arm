import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # 1. Capture the package paths
    pkg_moveit_config = get_package_share_directory("robotic_arm_moveit_config")

    # 2. Include the MoveIt "demo" launch
    # This automatically handles:
    # - robot_state_publisher (The skeleton)
    # - move_group (The brain)
    # - rviz (The eyes)
    # - ros2_control/fake_hardware (The muscles)
    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_moveit_config, "launch", "demo.launch.py"))
    )

    # 3. (Optional) Add your custom script launch here later
    # For now, we just launch the system infrastructure.

    return LaunchDescription([moveit_demo])
