#!/usr/bin/env python3

# Update python path if imports not working

# Import the rclpy library for ROS2 Python nodes
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

#To do talking to the USB filterwheel#!/usr/bin/env python3

# Import the rclpy library for ROS2 Python nodes
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

#To do talking to the USB filterwheel
import serial

# Import standard ROS2 message types
from std_msgs.msg import String

# Port of the USB filter wheel. Will need to be changed when running program on a different PC. Or if the order of connected devices have changed. 
PORT = '/dev/ttyUSB0'

# Amount of filters in the filter wheel. Will need to change when changing hardware. Either 6 or 12
WHEELNUM = 6

# Filter mapping. Wheel position <> filter in nm
filter_map = {
    1: 630,
    2: 640,
    3: 650,
    4: 660,
    5: 670,
    6: 680
}

class FilterWheelNode(Node):
    def __init__(self):
        # Initialize the node with a name (used in ROS graph)
        super().__init__('filterwheel_node')

        # Create a publisher to the topic 'chatter' that publishes the position of the filter wheel
        self.publisher_ = self.create_publisher(String, 'chatter', 10)

        # Optional: Log once at startup
        self.get_logger().info('Filter Wheel Node has been started!')

        # Initializing the serial connection via serial object
        self.filter_wheel_object = serial.Serial(
        port=PORT,
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=1  # optional: set read timeout
        )

        #Clear any existing input or command, just incase
        self.filter_wheel_object.reset_input_buffer()

        # Declaring parameters to set
        self.declare_parameter('requested_filter', 1)
        self.declare_parameter('current_filter', 1)

        # ROS2 Command to set parameters example:
        " Print nodes command = ros2 node list "
        "ros2 param set /filterwheel_node requested_filter 1"
        "ros2 param set /threshold_node high_intensity 10.0"

        # Set to first filter on startup by default
        #self.initpos = self.SetPosition(1)

        # Function that runs on paramter change
        self.add_on_set_parameters_callback(self.MoveWheelBasedonROS2Msg)

    def MoveWheelBasedonROS2Msg(self, params):
        for param in params:
            if param.name == 'requested_filter' and param.type_ == Parameter.Type.INTEGER:
                position = self.SetPosition(param.value)
                self.get_logger().info(f'Updated Filter Wheel position to {position}')

            else:
                self.get_logger().info(f'Did not change filter wheel value. Try again')

        return SetParametersResult(successful=True)

    def timer_callback(self):
        # This function is called every time the timer fires
        msg = String()
        msg.data = 'Hello from ROS2!'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

    # Function to set the position of the filter wheel 
    def SetPosition(self, requested_position):

        # Send a command. Make sure the command is not in caps
        command = f'pos={requested_position}\r'
        self.filter_wheel_object.write(command.encode())  # Send query command, must end with carriage return

        # Read the response
        response = self.filter_wheel_object.readline().decode().strip()
        print("Device response:", response)

        current_position = self.DecodeResponse_Position(response)

        return current_position

    # Function to query which position in the filter wheel we are on
    def QueryPosition(self, filter_wheel_object):

        # Send a command. Make sure the command is not in caps
        command = f'pos?\r'
        self.filter_wheel_object.write(command.encode())  # Send query command, must end with carriage return

        # Read the response
        response = filter_wheel_object.readline().decode().strip()
        print("Device response:", response)

        wheel_position = self.DecodeResponse_Position(response)

        return wheel_position

    # Function to decode position of the wheel
    def DecodeResponse_Position(self, response):

        match response:
            case 'pos=1': position = 1
            case 'pos=2': position = 2
            case 'pos=3': position = 3
            case 'pos=4': position = 4
            case 'pos=5': position = 5
            case 'pos=6': position = 6
            case 'pos=7': position = 7
            case 'pos=8': position = 8
            case 'pos=9': position = 9
            case 'pos=10': position = 10
            case 'pos=11': position = 11
            case 'pos=12': position = 12
            case _: position = 0

        return response

def main(args=None):
    # Initialize the ROS2 Python client library
    rclpy.init(args=args)

    # Create an instance of the node
    node = FilterWheelNode()

    # Keep the node alive and responsive to callbacks
    rclpy.spin(node)

    # Cleanup after shutdown
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
