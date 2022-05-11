ARG BASE_IMAGE_TAG=noetic
FROM ghcr.io/aica-technology/ros-control-libraries:${BASE_IMAGE_TAG}

WORKDIR /home/${USER}/ros_ws
COPY --chown=${USER} ./ros_examples ./src/ros_examples
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash; catkin_make"
