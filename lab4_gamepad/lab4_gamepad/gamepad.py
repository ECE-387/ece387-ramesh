#!/usr/bin/env python3
# The above line is a shebang, which tells the system to run this script using Python 3

# Import necessary ROS 2 libraries 
import rclpy  # ROS 2 client library for Python
from rclpy.node import Node  # Base class for creating ROS 2 nodes

# Import message types
from sensor_msgs.msg import Joy # Message type for joystick (gamepad) inputs 
from geometry_msgs.msg import Twist # Message type for velocity commands
from std_msgs.msg import Bool 


class Gamepad(Node):
    """
    A ROS 2 Node that converts joystick (gamepad) inputs into velocity commands for a robot.
    """

    def __init__(self):
        """
        Constructor: Initializes the gamepad node.
        """
        # TODO: Write your code here
        super().__init__('joy_node')
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # Flag to track control status (default: True)
        self.has_control = True

        # Create a publisher for control relinquishment messages.
        # - Publishes to the 'ctrl_relinq' topic.
        # - Uses Bool messages to indicate control status.
        # - Queue size of 1 ensures only the latest control state is kept.
        self.ctrl_pub = self.create_publisher(Bool, 'ctrl_relinq', 1)
                
        # Log a message indicating that the node has started successfully
        # self.get_logger().info("Joy to cmd_vel node started!")

    def joy_callback(self, msg: Joy):
        """
        Callback function that processes incoming joystick messages.
        """
        # Create a new Bool message for "control relinquishment status"
        relinquish = Bool()

        # TODO: Check if the RC (Remote Control) has control and button A (Green) is pressed
        if self.has_control==True and msg.buttons[0]==1:
            # Set control flag to False (relinquish control)
            self.has_control=False
            # Change "control status" to relinquished.
            relinquish.data = True 
            # Publish the control status
            self.ctrl_pub.publish(relinquish)

            # Log status update
            self.get_logger().info("RC has relinquished control.")
            

        # TODO: Check if RC does not have control and button B (Red) is pressed
        if self.has_control==False and msg.buttons[1]==1:
            # Set control flag to True (regain control)
            self.has_control=True
            # Set control status to regained.
            relinquish.data = False
            # Publish the control status
            self.ctrl_pub.publish(relinquish)
            
            # Log status update
            self.get_logger().info("RC has taken over control.")  

        # If control is relinquished, stop further processing
        if not self.has_control:
            return
        
        Twist_msg = Twist()
        Twist_msg.linear.x = 0.20 * msg.axes[1]
        Twist_msg.angular.z = 2.0 * msg.axes[3]
        self.publisher_.publish(Twist_msg)

def main(args=None):
    """
    Main function to start the ROS 2 node.
    - Initializes the ROS 2 system.
    - Creates an instance of the Gamepad node.
    - Keeps the node running using `rclpy.spin()`, which listens for messages.
    - Cleans up resources when the node is shut down.
    """
    rclpy.init(args=args)  # Initialize ROS 2
    gamepad = Gamepad()  # Create an instance of the Gamepad node
    rclpy.spin(gamepad)  # Keep the node running and responsive to joystick input

        # Cleanup when the node is shutting down
    gamepad.destroy_node()  # Properly destroy the node
    rclpy.shutdown()  # Shutdown ROS 2

    # Run the script if executed directly (not imported as a module)
if __name__ == '__main__':
    main()

        

