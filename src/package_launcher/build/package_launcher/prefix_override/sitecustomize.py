import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/dexter/ROS2/CamLidarCalib/src/package_launcher/install/package_launcher'
