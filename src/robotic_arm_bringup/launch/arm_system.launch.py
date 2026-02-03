import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # 1. Capture the package paths
    pkg_moveit_config = get_package_share_directory("robotic_arm_moveit_config")

    # 2. Include the MoveIt "demo" launch
    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_moveit_config, "launch", "demo.launch.py"))
    )

    # 3. Define the Scene Manager Node
    scene_manager_node = Node(
        package="robotic_arm_bringup",
        executable="scene_manager",
        name="scene_manager",
        output="screen",
    )

    return LaunchDescription([moveit_demo, scene_manager_node])
