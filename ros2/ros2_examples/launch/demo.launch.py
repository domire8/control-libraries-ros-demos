import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    demo = DeclareLaunchArgument('demo', default_value=TextSubstitution(text='joint_space_velocity_control'))
    robot_name = DeclareLaunchArgument('robot_name', default_value=TextSubstitution(text='franka'))
    start_rviz = DeclareLaunchArgument('start_rviz', default_value=TextSubstitution(text='false'))

    print(get_package_share_directory("ros2_examples"))

    return LaunchDescription([
        demo,
        robot_name,
        start_rviz,
        Node(
            package='ros2_examples',
            namespace=LaunchConfiguration('robot_name'),
            executable=LaunchConfiguration('demo'),
            name=LaunchConfiguration('demo'),
            output='screen',
            emulate_tty=True,
            parameters=[
                {'robot_name': LaunchConfiguration('robot_name')}
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(get_package_share_directory("ros2_examples"), 'rviz', 'config_file.rviz')],
            condition=IfCondition(LaunchConfiguration('start_rviz'))
        ),
    ])
