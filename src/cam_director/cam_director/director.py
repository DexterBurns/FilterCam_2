# Import the rclpy library for ROS2 Python nodes
import rclpy
from rclpy.node import Node
import subprocess
from std_srvs.srv import Trigger

# Import standard ROS2 message types
from std_msgs.msg import String

# Environment path for the camera sctipt. Will need to be adjusted based on the script location 
VENV_PATH = "/home/dexter/ROS2/FilterCam/src/camera_scripts/.venv/bin/python"
CAM_SCRIPT = "/home/dexter/ROS2/FilterCam/src/camera_scripts/snapshot.py"

"What do we want this camera director to do"
"""" We want this to act as the director. 
        It can start acquisition by calling the camera_script
        It can select a requested filter wheel position via the filterwheel node
        Set parameters for other nodes """

# How to call and activate the camera director via roscript, use this line in a ros terminal
# ros2 service call /capture_images std_srvs/srv/Trigger {}

# class for the node itself
class CamDirector(Node):
    def __init__(self):
        # Initialize the node with a name (used in ROS graph)
        super().__init__('camera_director')

        # Create a publisher to the topic 'chatter' that publishes String messages
        self.publisher_ = self.create_publisher(String, 'chatter', 10)

        # Optional: Log once at startup
        self.get_logger().info('Camera Director Node has been started!')
        
        # Service to capture photos: ros2 service call /capture_images std_srvs/srv/Trigger {}
        self.srv = self.create_service(Trigger, 'capture_images', self.AcquirePhotos)
        
    # Function that will set parameters for the filter wheel and will call the camera script that will take the photos.
    def AcquirePhotos(self, request, response):
        for filter_num in range(1,7):
            
            cmd_to_set_filter_pos = (f"source /opt/ros/jazzy/setup.bash && ros2 param set /filterwheel_node requested_filter {filter_num}")
            
            # First change to the filter we want
            subprocess.run(["bash", "-lc", cmd_to_set_filter_pos], check=True)
            
            # Try to call the camera script and take a photo. 
            try:
                # Camera will take a few seconds to take some photos
                subprocess.run([str(VENV_PATH), str(CAM_SCRIPT), str(filter_num)], check=True)
            
            # Break if any error/exception found
            except Exception as e:
                print("EXCEPTION: " + str(e))
                response.success = False
                return -2
            
        response.success = True
        response.message = "Request to get photos complete!"
        return response
        

def main(args=None):
    # Initialize the ROS2 Python client library
    rclpy.init(args=args)

    # Create an instance of the node
    node = CamDirector()

    # Keep the node alive and responsive to callbacks
    rclpy.spin(node)

    # Cleanup after shutdown
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()