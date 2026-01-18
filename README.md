# ros2_robotic_arm
Use ros2 to control a robotic arm. Simulation in Gazebo

It runs inside a Docker container so that the environment can be isolated.

## Tools
- Ubuntu 24.04
- ROS2 jazzy (docker images used)
- Docker 28.1.1

## Instalation and run
The only tool that has to be manually installed is Docker.
1. Install docker: https://docs.docker.com/desktop/setup/install/linux/ubuntu/
2. Clone this repository

All the other dependencies are automatically installed

## VSCode
VSCode is prefered as it has some useful extensions for ROS and Docker

## Usage
Everything runs in a docker container (like a small virtual machine).
The official container osrf/ros:jazzy-desktop is used.
And the custom tools/packages are installed over it.

Steps:
1. build the container (official image + general dependencies)
2. run the container with gui support (so that application like rviz and rqt can start in GUI mode)
3. build robotic arm project packages

There are two ways to start:
- VSCode DevContainers: 
    - it will do automatically the steps 1 and 2 (build and run)
    - just click op "Reopen in Container button". It will appear when opening the folder in VSCode
- Manually open a terminal and run: 
    - ./build_container.sh
    - ./run_container_gui.sh

