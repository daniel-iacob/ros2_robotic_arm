# ros2_robotic_arm
Use ros2 to control a robotic arm. SImulation in Gazebo

## Tools
- Ubuntu 24.04
- ROS2 jazzy (docker images used)
- Docker 28.1.1

## Instalation and run
The only tool that has to be manually installed is Docker.
1. Install docker: https://docs.docker.com/desktop/setup/install/linux/ubuntu/
2. Clone this repository
3. Build the image: docker build -t ros2-robotic-arm .
4. Run the container: docker run -it ros2-robotic-arm

All the other dependencies are automatically installed

## VSCode
VSCode is prefered as it has some useful extensions for ROS and Docker


## GUI workaround while using devcontainers
If a GUI apllication doesn't start from dev container, run
```bash
xhost +local:docker
```
before starting devcontaoiners from VSCode