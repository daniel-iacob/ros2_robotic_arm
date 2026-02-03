#!/bin/bash
set -e

source /ros2_robotic_arm/install/setup.bash
ros2 launch robotic_arm_bringup arm_system.launch.py
