#!/bin/bash
set -e

# Used by docker to run commands after container is started
# In this way, it will be a simpler image/ubuntu resulter from Dockerfile
# And all the other install will be like on any other Linux

# Source ROS 2 setup.bash in bashrc so that every new terminal 
# has ROS 2 environment sourced
echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

echo "Entrypoint script executed" # temp

exec "$@"