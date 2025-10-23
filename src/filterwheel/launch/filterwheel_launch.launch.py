from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='filterwheel',
            executable='filter_wheel_control_node',
            output='screen',
        ),

    ])
