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

    # Motion Server Node (persistent action server for arm control)
    motion_server_node = Node(
        package="robotic_arm_bringup",
        executable="motion_server",
        name="motion_server",
        output="screen",
    )

    # Synthetic camera (renders MoveIt scene as top-down image)
    camera_node = Node(
        package="robotic_arm_perception",
        executable="camera_node",
        name="camera_node",
        output="screen",
    )

    # Vision detection (HSV color detection from camera image)
    vision_node = Node(
        package="robotic_arm_perception",
        executable="vision_node",
        name="vision_node",
        output="screen",
    )

    return LaunchDescription([
        moveit_demo,
        scene_manager_node,
        motion_server_node,
        camera_node,
        vision_node,
    ])
