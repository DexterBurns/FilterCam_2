from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='cam_director',
            executable='cam_director_node',
            output='screen',
        ),

    ])