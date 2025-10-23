#!/usr/bin/env python3

# Import the rclpy library for ROS2 Python nodes
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import numpy as np

# Import standard ROS2 message types
from std_msgs.msg import String

# Import needed files 
from .lens import Lens

# Right now we are only working within this distance for the Liquid Lens 
MAX_DISTANCE = 2
MIN_DISTANCE = 1

# DIOPTER values only within this range because 1 to 2.5 meters is all that's needed to change focus between the two distances
MIN_DIOPTER = 0.5 
MAX_DIOPTER = 1

class LensControl(Node):
    def __init__(self):
        # Initialize the node with a name (used in ROS graph)
        super().__init__('lens_controller')

        # Defining our lens object
        self.lens = Lens('/dev/ttyACM0', debug=False)  # set debug to True to see a serial communication log
        self.min_fp, self.max_fp = self.lens.to_focal_power_mode() # Setting focal power mode

        # Create a publisher to the topic 'chatter' that publishes String messages
        self.publisher_ = self.create_publisher(Float32, 'current_diopter', 10)
        self.subscriber_ = self.create_subscription(Float32, 'cluster_distance', self.adjust_diopter, 10)

        # Optional: Log once at startup
        self.get_logger().info('Liquid Lens node has been started!')

    # Function to adjust the diopter value based on distance of the cluster.
    # Plans for the future: make the reanges larger to accomodate further distances
    def adjust_diopter(self, cluster_distance_msg):

        cluster_distance = cluster_distance_msg.data

        # Get the diopter based on distance
        cluster_distance = np.clip(cluster_distance, 1, 2)
        diopter = np.interp(cluster_distance, [MIN_DISTANCE, MAX_DISTANCE], [MAX_DIOPTER, MIN_DIOPTER])

        self.lens.set_diopter(diopter)
        print("Current Diopter: {diopter}", diopter)

def main(args=None):
    # Initialize the ROS2 Python client library
    rclpy.init(args=args)

    # Create an instance of the node
    node = LensControl()

    # Keep the node alive and responsive to callbacks
    rclpy.spin(node)

    # Cleanup after shutdown
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()