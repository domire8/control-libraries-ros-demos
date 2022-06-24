# `ros_examples` demonstration scripts

## Table of contents:

* [Prerequisites](#prerequisites)
* [Running a demo script](#running-demonstration-scripts)
* [Running a demo with the simulator](#running-the-simulator-simultaneously)
* [Development](#development)

## Prerequisites

This package contains control loop examples in a ROS environment. The folder is a fully functional ROS package that can
be directly copied in a ROS workspace.

There is also a `Dockerfile` provided that encapsulates the whole package in a containerized ROS environment. If you
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
aica-docker interactive control-libraries-rospy-zmq-demos:noetic -u ros --net host --no-hostname
```

Running the scripts uses ROS commands, e.g. to run a script:

```console
roslaunch rospy_zmq_examples demo.launch demo:=<demo>
```

Available demos are:

- cartesian_twist_control
- joint_space_velocity_control

## Running the simulator simultaneously

The scripts require a simulator (or real robot with the same interface) to be running. Start the simulator with:

```console
cd path/to/desired/location
git clone -b develop git@github.com:epfl-lasa/simulator-backend.git
cd simulator-backend/pybullet_zmq
./build-server.sh
aica-docker interactive aica-technology/zmq-simulator -u ros2 --net host --no-hostname
python3 pybullet_zmq/bin/zmq-simulator
```

Once the simulator is running (in this case the franka simulator), do

```console
roslaunch rospy_zmq_examples demo.launch demo:=<demo> robot_name:=franka
```

Note that the robot name has to be the same as specified in the simulator, otherwise the topics won't be in the same
namespace and the demos don't work.

## Development

To run the Docker image as SSH server:

```console
bash build-server.sh -s
```