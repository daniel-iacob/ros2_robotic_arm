#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select robotic_arm_description
source install/setup.bash
