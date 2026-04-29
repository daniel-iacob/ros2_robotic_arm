import os
import subprocess
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue


def load_yaml(package_name, file_path):
    package_share = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_share, file_path)
    with open(absolute_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    rviz_enabled = LaunchConfiguration("rviz", default="true")

    pkg_description = get_package_share_directory("robotic_arm_description")
    pkg_moveit_config = get_package_share_directory("robotic_arm_moveit_config")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    # ── Gazebo resource path ────────────────────────────────────────────────
    # Gazebo's URDF→SDF translator rewrites `package://robotic_arm_description/...`
    # to `model://robotic_arm_description/...`. To resolve those, Gazebo searches
    # GZ_SIM_RESOURCE_PATH for a directory whose name matches the model authority.
    # We point it at the parent of the package's share dir so
    # `model://robotic_arm_description/meshes/...` maps to
    # `<install>/robotic_arm_description/share/robotic_arm_description/meshes/...`.
    existing_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    gz_resource_path = os.path.dirname(pkg_description)
    if existing_resource_path:
        gz_resource_path = f"{gz_resource_path}:{existing_resource_path}"
    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH", value=gz_resource_path
    )

    use_sim_time = {"use_sim_time": True}

    # ── Robot description (URDF via xacro, computed eagerly at launch time) ──
    # Eager computation is required: gz_ros2_control is a world-level system plugin
    # that processes the robot_description topic immediately when received. If RSP
    # publishes before the robot is spawned, initSim finds no joints in the ECM
    # and crashes with a null-pointer segfault.
    # Fix: spawn from -string (no topic), then start RSP after spawn exits so the
    # ECM is fully populated when the plugin fires.
    xacro_file = os.path.join(pkg_description, "urdf", "ar_gazebo.urdf.xacro")
    robot_urdf_string = subprocess.run(
        ["xacro", xacro_file], capture_output=True, text=True, check=True
    ).stdout

    robot_description = {"robot_description": robot_urdf_string}

    # ── SRDF ────────────────────────────────────────────────────────────────
    robot_description_semantic_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("robotic_arm_moveit_config"),
                "srdf",
                "ar.srdf.xacro",
            ]),
        ]),
        value_type=str,
    )
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

    # ── Planning pipeline (OMPL) ───────────────────────────────────────────
    ompl_planning_yaml = load_yaml("robotic_arm_moveit_config", "config/ompl_planning.yaml")
    planning_pipeline_config = {
        "default_planning_pipeline": "ompl",
        "planning_pipelines": ["ompl"],
        "ompl": ompl_planning_yaml,
    }

    # ── MoveIt controller manager ──────────────────────────────────────────
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

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # ── move_group ─────────────────────────────────────────────────────────
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
            use_sim_time,
        ],
    )

    # ── RViz ───────────────────────────────────────────────────────────────
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
            use_sim_time,
        ],
        condition=IfCondition(rviz_enabled),
    )

    # ── robot_state_publisher ──────────────────────────────────────────────
    # Started AFTER spawn_robot exits — see spawn_then_rsp below.
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description, use_sim_time],
    )

    # ── Gazebo sim ─────────────────────────────────────────────────────────
    world_file = os.path.join(pkg_description, "worlds", "arm_world.sdf")
    gz_args = f"-r {world_file}"
    if os.environ.get("HEADLESS") == "1":
        gz_args = f"-r -s {world_file}"

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": gz_args}.items(),
    )

    # ── Robot spawner — uses -string to avoid needing the topic early ───────
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-string", robot_urdf_string,
            "-name", "ar4",
            "-x", "0.0", "-y", "0.0", "-z", "0.0",
        ],
        output="screen",
    )

    # ── Clock bridge ───────────────────────────────────────────────────────
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    # ── Controller spawners (wait for controller manager from Gazebo) ──────
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "-c", "/controller_manager",
            "--controller-manager-timeout", "60",
        ],
        parameters=[use_sim_time],
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "-c", "/controller_manager",
            "--controller-manager-timeout", "60",
        ],
        parameters=[use_sim_time],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "-c", "/controller_manager",
            "--controller-manager-timeout", "60",
        ],
        parameters=[use_sim_time],
    )

    # ── Application nodes ──────────────────────────────────────────────────
    scene_manager_node = Node(
        package="robotic_arm_bringup",
        executable="scene_manager",
        name="scene_manager",
        output="screen",
        parameters=[use_sim_time],
    )

    motion_server_node = Node(
        package="robotic_arm_bringup",
        executable="motion_server",
        name="motion_server",
        output="screen",
        parameters=[use_sim_time],
    )

    camera_node = Node(
        package="robotic_arm_perception",
        executable="camera_node",
        name="camera_node",
        output="screen",
        parameters=[use_sim_time],
    )

    vision_node = Node(
        package="robotic_arm_perception",
        executable="vision_node",
        name="vision_node",
        output="screen",
        parameters=[use_sim_time],
    )

    # ── Launch sequencing ───────────────────────────────────────────────────
    # RSP must be running BEFORE spawn_robot so that gz_ros2_control's
    # Configure() callback (which fires during spawn) can call the
    # robot_state_publisher service to fetch the robot description.
    # Controllers start after spawn exits (controller_manager is ready by then).
    spawn_then_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                joint_state_broadcaster_spawner,
                joint_trajectory_controller_spawner,
                gripper_controller_spawner,
            ],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument("tf_prefix", default_value="", description="TF prefix"),
        DeclareLaunchArgument("rviz", default_value="true", description="Launch RViz"),
        set_gz_resource_path,
        gz_sim,
        clock_bridge,
        robot_state_publisher_node,
        spawn_robot,
        spawn_then_controllers,
        move_group_node,
        rviz_node,
        scene_manager_node,
        motion_server_node,
        camera_node,
        vision_node,
    ])
