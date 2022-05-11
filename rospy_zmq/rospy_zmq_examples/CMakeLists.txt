cmake_minimum_required(VERSION 3.9)
project(rospy_zmq_examples)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

set(DEMOS_SCRIPTS
    joint_space_velocity_control
    cartesian_twist_control
)

foreach(SCRIPT ${DEMOS_SCRIPTS})
    catkin_install_python(PROGRAMS scripts/${SCRIPT} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach()


