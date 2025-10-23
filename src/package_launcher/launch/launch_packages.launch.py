from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get paths to the launch files you want to include
    velodyne_driver_launch = os.path.join(
        get_package_share_directory('velodyne_driver'),
        'launch',
        'velodyne_driver_node-HDL32E-launch.py'
    )

    velodyne_pointcloud_launch = os.path.join(
        get_package_share_directory('velodyne_pointcloud'),
        'launch',
        'velodyne_transform_node-HDL32E-launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(velodyne_driver_launch)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(velodyne_pointcloud_launch)
        ),

        # Optional: You can still launch nodes directly here too
        # Node(...),
    ])
