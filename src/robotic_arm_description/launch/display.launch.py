import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Paths and Variables
    pkg_description = get_package_share_directory('robotic_arm_description')
    
    # Path to your .xacro file
    xacro_file = os.path.join(pkg_description, 'urdf', 'robotic_arm.urdf.xacro')

    # 2. Process Xacro to URDF
    robot_description_config = Command(['xacro ', xacro_file])
    params = {'robot_description': robot_description_config}

    # 3. Define Nodes
    
    # Robot State Publisher (REQUIRED)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Joint State Publisher GUI (The Sliders)
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    # RViz2
    rviz_config_path = os.path.join(pkg_description, 'rviz', 'view_arm.rviz')

    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path] # This loads your saved settings automatically
    )

    # 4. Launch Description
    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher_gui,
        node_rviz
    ])