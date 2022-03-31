ARG ROS_VERSION=galactic
FROM ghcr.io/aica-technology/ros2-control-libraries:${ROS_VERSION}

# upgrade ament_cmake_python
RUN sudo apt update && sudo apt install -y ros-${ROS_DISTRO}-ament-cmake-python && sudo rm -rf /var/lib/apt/lists/*

WORKDIR /home/${USER}/ros2_ws
COPY --chown=${USER} ./ros2_examples ./src/ros2_examples
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash; colcon build --symlink-install"
