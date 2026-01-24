import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # 1. Paths and Variables
    pkg_description = get_package_share_directory("robotic_arm_description")
    xacro_file = os.path.join(pkg_description, "urdf", "robotic_arm.urdf.xacro")
    controller_config = os.path.join(pkg_description, "config", "arm_controllers.yaml")
    rviz_config_path = os.path.join(pkg_description, "rviz", "view_arm.rviz")

    # 2. Process Xacro to URDF
    robot_description_content = ParameterValue(Command(["xacro ", xacro_file]), value_type=str)
    params = {"robot_description": robot_description_content}

    # 3. Define Nodes
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params],
    )

    node_controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[params, controller_config],
        output="screen",
    )

    # 4. Spawners (Triggered after Controller Manager starts)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
    )

    # 5. RViz2
    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
    )

    # 6. Launch Description
    return LaunchDescription(
        [
            node_robot_state_publisher,
            node_controller_manager,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            node_rviz,
        ]
    )
