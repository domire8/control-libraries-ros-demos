cmake_minimum_required(VERSION 3.15)
project(ros2_examples)

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

list(APPEND CMAKE_PREFIX_PATH /opt/openrobots)
find_package(control_libraries 5.2 REQUIRED)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(
    include
    /opt/openrobots/include
)

set(DEMOS_SCRIPTS
    cartesian_twist_control
    joint_space_position_control
    joint_space_velocity_control
)

foreach(SCRIPT ${DEMOS_SCRIPTS})
    ament_auto_add_executable(${SCRIPT} scripts/${SCRIPT}.cpp src/RobotInterfaceNode.cpp)
    target_link_libraries(${SCRIPT} ${control_libraries_LIBRARIES}
    )
endforeach()

# Install Python modules
ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})

ament_auto_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
