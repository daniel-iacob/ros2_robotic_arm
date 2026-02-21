#!/bin/bash
set -e

# --- Colors for better readability ---
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to display help/usage
usage() {
    echo -e "Usage: ${GREEN}./run.sh [option]${NC}"
    echo ""
    echo -e "Options:"
    echo -e "  ${GREEN}build${NC}   : Build the workspace using colcon"
    echo -e "  ${GREEN}clean${NC}   : Remove build, install, and log folders"
    echo -e "  ${GREEN}sim${NC}     : Launch the entire simulation"
    echo -e "${BLUE}------------------------------------------${NC}"
}

# Check if an argument was provided
if [ -z "$1" ]; then
    usage
    exit 1
fi

# Source ROS 2 environment
source /opt/ros/$ROS_DISTRO/setup.bash

case "$1" in
    build)
        # Build custom defined packages
        # It should be run inside conainer
        echo -e "${BLUE}\nChecking for missing dependencies${NC}"
        sudo apt-get update
        rosdep install --from-paths src --ignore-src -y -r
        echo -e "${BLUE}\nBuild all packages${NC}"
        colcon build --symlink-install
        source install/setup.bash
        ;;
    clean)
        echo -e "${BLUE}Cleaning workspace...${NC}"
        rm -rf build/ install/ log/
        ;;
    sim)
        echo -e "${BLUE}Starting Simulation...${NC}"
        source install/setup.bash
        ros2 launch robotic_arm_bringup arm_system.launch.py
        ;;
    *)
        echo -e "${YELLOW}Invalid option: $1${NC}"
        usage
        exit 1
        ;;
esac