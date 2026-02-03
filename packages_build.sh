#!/bin/bash

# Build custom defined packages
# It should be run inside conainer

set -e
source /opt/ros/jazzy/setup.bash

echo "Checking for missing dependencies..."
sudo apt-get update
rosdep install --from-paths src --ignore-src -y -r

colcon build --symlink-install

source install/setup.bash
echo "Build complete"
