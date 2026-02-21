import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_moveit_config = get_package_share_directory("robotic_arm_moveit_config")
    pkg_bringup = get_package_share_directory("robotic_arm_bringup")

    # MoveIt "demo" launch (includes ros2_control_node, controller spawners, move_group, rviz)
    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_moveit_config, "launch", "demo.launch.py"))
    )

    # Scene Manager Node (adds cubes to planning scene)
    scene_manager_node = Node(
        package="robotic_arm_bringup",
        executable="scene_manager",
        name="scene_manager",
        output="screen",
    )

    return LaunchDescription([moveit_demo, scene_manager_node])
