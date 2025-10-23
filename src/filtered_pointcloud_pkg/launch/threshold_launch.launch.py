from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='filtered_pointcloud_pkg',
            executable='threshold_node',
            name='threshold_node',
            parameters=['config/params.yaml'],  # Optional
            output='screen'
        )
    ])
