import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # 1. Path to your existing simulation launch file
    pkg_description = get_package_share_directory("robotic_arm_description")

    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_description, "launch", "view_arm.launch.py"
            )  # Ensure this filename matches yours
        )
    )

    # 2. Your new Python controller node
    arm_mover_node = Node(package="robotic_arm_controller", executable="arm_mover", output="screen")

    return LaunchDescription([simulation_launch, arm_mover_node])
