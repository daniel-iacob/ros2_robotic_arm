#!/bin/bash
set -e

GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'

usage() {
    echo -e "Usage: ${GREEN}./robotic_arm.sh <command> [args]${NC}"
    echo ""
    echo -e "Commands:"
    echo -e "  ${GREEN}pick <object>${NC}                 Pick up object at its current position"
    echo -e "  ${GREEN}move-to <x> <y> <z>${NC}          Move end-effector to position (carries object if holding)"
    echo -e "  ${GREEN}place <object>${NC}                Release object at current position"
    echo -e "  ${GREEN}home${NC}                          Return arm to home position"
    echo -e "  ${GREEN}open-gripper${NC}                  Open gripper"
    echo -e "  ${GREEN}close-gripper${NC}                 Close gripper"
    echo -e "  ${GREEN}reset${NC}                         Full recovery: home + reset scene"
    echo -e "  ${GREEN}list-objects${NC}                  List objects in the MoveIt scene"
    echo ""
    echo -e "Examples:"
    echo -e "  ./robotic_arm.sh pick blue_cube"
    echo -e "  ./robotic_arm.sh move-to 0.4 0.1 0.3"
    echo -e "  ./robotic_arm.sh place blue_cube"
    echo -e "${BLUE}------------------------------------------${NC}"
}

if [ -z "$1" ]; then
    usage
    exit 0
fi

# Source ROS2 + workspace
source /opt/ros/$ROS_DISTRO/setup.bash
source "$(dirname "$0")/install/setup.bash"

ros2 run robotic_arm_bringup arm "$@"
