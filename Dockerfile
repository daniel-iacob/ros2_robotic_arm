FROM ros:jazzy

# Set workdir
WORKDIR /ros2_robotic_arm

# Install everything needed for GUI application to be visible because 
# they are running inside a container
RUN apt-get update && apt-get install -y \
    libx11-xcb1 \
    libxcb-dri3-0 \
    libxtst6 \
    libxrender1 \
    libxshmfence1 \
    libglu1-mesa \
    && rm -rf /var/lib/apt/lists/*

# Fix: Initialize rosdep only if not already initialized, and run update
# rosdep is used to install packages
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
        rosdep init; \
    fi && \
    rosdep update

# Fix for "System-managed environment" in Ubuntu 24.04
# Allow pip to install packages system-wide, bypassing the PEP 668 restriction
# that prevents pip from modifying Python packages managed by the system package manager.
# This is useful in containerized environments where system package conflicts are less of a concern.
# WARNING: Use with caution as it may cause conflicts between pip and system packages.
ENV PIP_BREAK_SYSTEM_PACKAGES=1

# source ros2 so that its available automatically in every opened terminal
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

# When the container is started, open a bash terminal
# This terminal gives access to the container with ROS 2 environment installed
CMD ["bash"]
