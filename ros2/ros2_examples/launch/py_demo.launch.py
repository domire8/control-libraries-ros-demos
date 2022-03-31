from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    demo = DeclareLaunchArgument('demo', default_value=TextSubstitution(text='joint_space_velocity_control'))
    robot_name = DeclareLaunchArgument('robot_name', default_value=TextSubstitution(text='franka'))

    return LaunchDescription([
        demo,
        robot_name,
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
    ])
