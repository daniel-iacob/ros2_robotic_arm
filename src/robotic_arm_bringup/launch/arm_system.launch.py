import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Paths to other packages
    pkg_description = get_package_share_directory("robotic_arm_description")
    pkg_controller = get_package_share_directory("robotic_arm_controller")

    # 1. The "Hardware/Simulation" Layer
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_description, "launch", "view_arm.launch.py"))
    )

    # 2. The "Intelligence" Layer
    moving_logic = Node(package="robotic_arm_controller", executable="arm_mover", output="screen")

    return LaunchDescription([simulation, moving_logic])
