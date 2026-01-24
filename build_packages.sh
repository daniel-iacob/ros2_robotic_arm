#!/bin/bash

# Build custom defined packages
# It should be run inside conainer

set -e
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
