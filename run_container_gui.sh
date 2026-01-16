#!/bin/bash

# Use this to start the container manually with GUI support
# This is the manual alternative to devcontainers
xhost +local:docker
docker compose run ros2-robotic-arm
xhost -local:docker