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
    tf
)

list(APPEND CMAKE_PREFIX_PATH /opt/openrobots)
find_package(control_libraries 5.2 REQUIRED)

catkin_package(
    CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    tf
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    /opt/openrobots/include
)

set(DEMOS_SCRIPTS
    cartesian_twist_control
    joint_space_position_control
    joint_space_velocity_control
    task_space_control_loop
)

foreach(SCRIPT ${DEMOS_SCRIPTS})
    add_executable(${SCRIPT} scripts/${SCRIPT}.cpp src/RobotInterface.cpp)
    target_link_libraries(${SCRIPT}
        ${catkin_LIBRARIES}
        ${control_libraries_LIBRARIES}
    )
    install(TARGETS ${SCRIPT}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
endforeach()
