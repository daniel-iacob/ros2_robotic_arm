import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
)


def load_yaml(package_name, file_path):
    package_share = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_share, file_path)
    with open(absolute_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    tf_prefix = LaunchConfiguration("tf_prefix", default="")

    pkg_description = get_package_share_directory("robotic_arm_description")
    pkg_moveit_config = get_package_share_directory("robotic_arm_moveit_config")
    pkg_bringup = get_package_share_directory("robotic_arm_bringup")

    # ── Robot description (URDF via xacro) ─────────────────────────────────
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("robotic_arm_description"),
            "urdf",
            "fake_ar.urdf.xacro",
        ]),
    ])
    robot_description = {"robot_description": robot_description_content}

    # ── SRDF (also an xacro file) ───────────────────────────────────────────
    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("robotic_arm_moveit_config"),
            "srdf",
            "ar.srdf.xacro",
        ]),
    ])
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    # ── Kinematics ─────────────────────────────────────────────────────────
    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml(
            "robotic_arm_moveit_config", "config/kinematics.yaml"
        )
    }

    # ── Joint limits ───────────────────────────────────────────────────────
    joint_limits = ParameterFile(
        PathJoinSubstitution([
            FindPackageShare("robotic_arm_moveit_config"),
            "config/joint_limits.yaml",
        ]),
        allow_substs=True,
    )

    # ── Planning pipeline (OMPL) ────────────────────────────────────────────
    ompl_planning_yaml = load_yaml("robotic_arm_moveit_config", "config/ompl_planning.yaml")
    planning_pipeline_config = {
        "default_planning_pipeline": "ompl",
        "planning_pipelines": ["ompl"],
        "ompl": ompl_planning_yaml,
    }

    # ── MoveIt controller manager ───────────────────────────────────────────
    moveit_controller_manager = {
        "moveit_controller_manager":
            "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    moveit_controllers = ParameterFile(
        PathJoinSubstitution([
            FindPackageShare("robotic_arm_moveit_config"),
            "config/moveit_controllers.yaml",
        ]),
        allow_substs=True,
    )

    # ── Trajectory execution settings ──────────────────────────────────────
    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    # ── Planning scene monitor ──────────────────────────────────────────────
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # ── move_group node ─────────────────────────────────────────────────────
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            joint_limits,
            planning_pipeline_config,
            trajectory_execution,
            moveit_controller_manager,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    # ── RViz ────────────────────────────────────────────────────────────────
    rviz_config = os.path.join(pkg_moveit_config, "config", "moveit.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            planning_pipeline_config,
        ],
    )

    # ── robot_state_publisher ───────────────────────────────────────────────
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # ── ros2_control node ───────────────────────────────────────────────────
    ros2_controllers = ParameterFile(
        PathJoinSubstitution([
            FindPackageShare("robotic_arm_moveit_config"),
            "config/ros2_controllers.yaml",
        ]),
        allow_substs=True,
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            ros2_controllers,
        ],
        output="both",
    )

    # ── Controller spawners ─────────────────────────────────────────────────
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "-c", "/controller_manager"],
    )

    # ── Scene Manager Node ──────────────────────────────────────────────────
    scene_manager_node = Node(
        package="robotic_arm_bringup",
        executable="scene_manager",
        name="scene_manager",
        output="screen",
    )

    # ── Motion Server Node ──────────────────────────────────────────────────
    motion_server_node = Node(
        package="robotic_arm_bringup",
        executable="motion_server",
        name="motion_server",
        output="screen",
    )

    # ── Synthetic camera ────────────────────────────────────────────────────
    camera_node = Node(
        package="robotic_arm_perception",
        executable="camera_node",
        name="camera_node",
        output="screen",
    )

    # ── Vision detection ────────────────────────────────────────────────────
    vision_node = Node(
        package="robotic_arm_perception",
        executable="vision_node",
        name="vision_node",
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("tf_prefix", default_value="", description="TF prefix for all joints"),
        move_group_node,
        rviz_node,
        robot_state_publisher_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        gripper_controller_spawner,
        scene_manager_node,
        motion_server_node,
        camera_node,
        vision_node,
    ])
