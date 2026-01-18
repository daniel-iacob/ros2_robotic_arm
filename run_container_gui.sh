#!/bin/bash
set -e

# Give permission for container to open the applications with have windows
# If it's a directory (the Docker mistake), delete it. Some random error
XAUTH=/tmp/.docker.xauth
if [ -d "$XAUTH" ]; then
    echo "Fixing Docker directory mistake..."
    sudo rm -rf "$XAUTH"
fi
if [ ! -f "$XAUTH" ]; then # Create the file if it doesn't exist
    touch "$XAUTH"
fi
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge - # Fill with X11 cookie


docker compose run --remove-orphans ros2-robotic-arm
