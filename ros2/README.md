# `ros2_examples` demonstration scripts

## Table of contents:

* [Prerequisites](#prerequisites)
* [Running a demo script](#running-demonstration-scripts)
* [Running a demo with the simulator](#running-the-simulator-simultaneously)
* [Development](#development)

## Prerequisites

This package contains control loop examples in a ROS2 environment. The folder is a fully functional ROS2 package that
can be directly copied in a ROS2 workspace.

There is also a `Dockerfile` provided that encapsulates the whole package in a containerized ROS2 environment. If you
want to run demo with the dockerized environment, you need to do the following first:

```console
cd path/to/desired/location
git clone https://github.com/aica-technology/docker-images.git
cd docker-images/scripts
./install-aica-docker.sh
```

Visit the [docker-images](https://github.com/aica-technology/docker-images) repository for more information on this.

## Running demonstration scripts

If working with Docker, build and run the image with

```console
./build-server.sh
aica-docker interactive control-libraries-ros-demos:galactic -u ros2 --net bridge
```

Running the scripts uses ROS2 commands, e.g. to run a script:

```console
ros2 launch ros2_examples demo.launch.py demo:=<demo>
```

Available demos are:

- cartesian_twist_control
- joint_space_position_control
- joint_space_velocity_control

## Running the simulator simultaneously

The scripts require a simulator (or real robot with the same interface) to be running. Start the simulator with:

```console
cd path/to/desired/location
git clone -b develop git@github.com:epfl-lasa/simulator-backend.git
cd simulator-backend/pybullet_ros2
./build-server.sh
aica-docker interactive aica-technology/ros2-simulator:galactic -u ros2 --net bridge
ros2 launch pybullet_ros2 franka.launch.py
```

Once the simulator is running (in this case the franka simulator), do

```console
ros2 launch ros2_examples demo.launch.py demo:=<demo> robot_name:=franka
```

Note that the robot name has to be the same as specified in the simulator, otherwise the topics won't be in the same
namespace and the demos don't work.

## Development

To run the Docker image as SSH server:

```console
aica-docker server control-libraries-ros-demos:galactic -u ros2 -p 2260 --net bridge
```