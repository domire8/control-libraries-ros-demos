ARG BASE_IMAGE_TAG=galactic
FROM ghcr.io/aica-technology/ros2-control-libraries:${BASE_IMAGE_TAG}

WORKDIR /home/${USER}/ros2_ws
COPY --chown=${USER} ./ros2_examples ./src/ros2_examples
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash; colcon build --symlink-install"
