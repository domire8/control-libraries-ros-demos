ARG ROS_VERSION=noetic
FROM ghcr.io/aica-technology/ros-control-libraries:${ROS_VERSION}

WORKDIR /home/${USER}/ros_ws
COPY --chown=${USER} ./ros_examples ./src/ros_examples
RUN su ${USER} -c /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash; catkin_make"
