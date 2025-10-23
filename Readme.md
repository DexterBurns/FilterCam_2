Dont forget to download the velodyne ROS2 package for Jazzy. This wont work without it! I didnt include it because the package itself is too big and git was complaining about it.

MAKE SURE TO DOWNLOAD IT!!: https://github.com/ros-drivers/velodyne/tree/ros2

I used the default ros2 branch.

!!! Check the velodyne replacement file! Add the HDL32E params .yaml to: velodyne/velodyne_driver/config
!!! Do the same with the launch file in the velodyne_replacement driver node launch.py in that folder

For some reason the developers forgot the HDL-32 exists and so putting those two files in should be enough to get the HDL-32 working with their package.