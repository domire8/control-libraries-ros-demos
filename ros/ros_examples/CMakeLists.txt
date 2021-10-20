cmake_minimum_required(VERSION 3.9)
project(ros_examples)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
)

list(APPEND CMAKE_PREFIX_PATH /opt/openrobots)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(osqp REQUIRED)

find_library(state_representation REQUIRED)
find_library(dynamical_systems REQUIRED)
find_library(controllers REQUIRED)
find_library(robot_model REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)
add_compile_definitions(BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS)
add_compile_definitions(BOOST_MPL_LIMIT_LIST_SIZE=30)

catkin_package(
    CATKIN_DEPENDS roscpp std_msgs sensor_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${STATE_REPRESENTATION_INCLUDE_DIR}
    ${DYNAMICAL_SYSTEMS_INCLUDE_DIR}
    ${ROBOT_MODEL_INCLUDE_DIR}
    ${CONTROLLERS_INCLUDE_DIR}
    ${PINOCCHIO_INCLUDE_DIR}
    ${OsqpEigen_INCLUDE_DIR}
    /opt/openrobots/include
)

set(DEMOS_SCRIPTS
    cartesian_twist_control
    joint_space_position_control
    joint_space_velocity_control
)

foreach(SCRIPT ${DEMOS_SCRIPTS})
    add_executable(${SCRIPT} scripts/${SCRIPT}.cpp src/RobotInterface.cpp)
    target_link_libraries(${SCRIPT}
        ${catkin_LIBRARIES}
        ${PINOCCHIO_LIBRARIES}
        OsqpEigen::OsqpEigen
        osqp::osqp
        state_representation
        dynamical_systems
        robot_model
        controllers
    )
    install(TARGETS ${SCRIPT}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
endforeach()
