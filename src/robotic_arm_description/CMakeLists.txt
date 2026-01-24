cmake_minimum_required(VERSION 3.8)
project(robotic_arm_description)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install the directories so ROS can find them after building
install(
  DIRECTORY
    urdf
    launch
    meshes
    rviz
    config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()