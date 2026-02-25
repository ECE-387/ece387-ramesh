import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from tf_transformations import euler_from_quaternion


class MoveToGoal(Node):
    """
    A ROS2 node that controls a robot to move toward a specified goal position.

    This class subscribes to odometry and IMU topics to track the robot's position
    and orientation. It also listens for control relinquishment messages to determine
    whether it should take control of movement. The node follows a state machine approach
    to:

    1. Rotate towards the goal.
    2. Move straight towards the goal.
    3. Rotate to the final desired orientation.
    4. Stop once the goal is reached.

    Velocity commands are published to the '/cmd_vel' topic to control the robot's movement.
    """

    def __init__(self):
        # Initialize the ROS2 node with name 'move_to_goal'
        super().__init__("move_to_goal")  
        
        # TODO: Publisher for sending velocity commands to the robot
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # TODO: Subscribers to receive odom and imu data
        self.odom_subs = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.imu_subs = self.create_subscription(Imu, 'imu', self.imu_callback, 10)
        # TODO: Subscriber to receive control status
        self.ctrl_subs = self.create_subscription(Bool, 'ctrl_relinq', self.ctrl_relinq_callback, 10)

        # Variables to store the robot's current position and orientation
        self.x = float(0)  # Current x-coordinate
        self.y = float(0)  # Current y-coordinate
        self.yaw = float(0)  # Current orientation (yaw angle in radians)

        self.has_control = False  # Flag indicating whether this node has control

        # Goal position and final orientation
        self.goal_x = -0.61  # Target x position
        self.goal_y = 0.61  # Target y position
        self.goal_yaw = 0  # Final orientation (0 degrees in radians)

        self.state = "ROTATE_TO_GOAL"  # Initial state of the robot

        # TODO: Timer to run the control loop at a fixed rate (every 0.1 seconds)
        self.timer = self.create_timer(0.1, self.control_loop)

        print("init")

    def odom_callback(self, msg: Odometry) -> None:
        """
        Callback function for handling odometry messages.
        Updates the robot's current x and y position.
        """
        # TODO: Extract x-coordinate from odometry

        self.x  = msg.pose.pose.position.x

        # TODO: Extract y-coordinate from odometry
        self.y = msg.pose.pose.position.y

        #print("odom")

    def imu_callback(self, imu_msg: Imu) -> None:
        """
        Callback function for handling IMU messages.
        Extracts yaw (rotation around Z-axis) from the quaternion orientation.
        """
        # TODO: Extract quaternion values and convert quaternion to 
        # Euler angles - use euler_from_quaternion
        q = imu_msg.orientation
        angles = euler_from_quaternion([q.x,q.y,q.z,q.w])
        # Update yaw value
        self.yaw = angles[2]
        #print("imu callback")

    def ctrl_relinq_callback(self, relinq_msg: Bool) -> None:
        """
        Callback function for handling control relinquishment messages.
        Updates the control status based on received messages.
        """
        # TODO: Update control flag
        self.has_control=relinq_msg

        if self.has_control:
            self.get_logger().info("move2goal has taken control")
        else:
            self.get_logger().info("move2goal has lost control")
        print("ctrl relinq callback")


    def control_loop(self) -> None:
        """
        Main control loop that executes periodically.
        Controls the robot's movement towards the goal.
        """
        if not self.has_control:
            return  # Exit if this node does not have control

        cmd = Twist()  # Create a new Twist message for velocity commands
        
        # TODO: Compute the angle to the goal
        # Compute target heading angle
        goal_theta = math.atan2(self.goal_y,self.goal_x)
        
        
        # Compute difference between current and target angle
        angle_error = goal_theta-self.yaw
        # print(self.yaw)
        
        # Normalize angle error to range [-pi, pi]
        angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

        # TODO: Compute the distance to the goal
        # Difference in x direction
        error_x = self.goal_x-self.x
        # Difference in y direction
        error_y = self.goal_y-self.y
        # Euclidean distance to goal
        distance = math.sqrt((error_x*error_x) + (error_y*error_y))
        print(distance)
        if self.state == "ROTATE_TO_GOAL":
            """
            First state: Rotate the robot towards the goal.
            """
            if abs(angle_error) > 0.05:  # Allow small tolerance
                cmd.angular.z = (
                    0.4 * angle_error
                )  # Adjust rotation speed based on error
            else:
                cmd.angular.z = 0.0  # Stop rotating when aligned
                self.state = "MOVE_FORWARD"  # Transition to next state

            self.get_logger().info(
                f"yaw={math.degrees(self.yaw):.2f}, angle_error={math.degrees(angle_error):.2f}"
            )

        elif self.state == "MOVE_FORWARD":
            """
            Second state: Move towards the goal in a straight line.
            """
            self.get_logger().info(
                f"x={self.x:.3f}, y={self.y:.3f}, distance={distance:.3f}"
            )

            # TODO:
            # Continue moving if not at goal - Move forward at constant speed of 0.15
            # If the distance is less than 0.15, stop moving and transition to final rotation
            if abs(distance)>.15:
                cmd.linear.x = (
                    .15*distance
                )
            else:
                self.state="ROTATE_TO_FINAL"

        elif self.state == "ROTATE_TO_FINAL":
            """
            Third state: Rotate to match the final desired orientation.
            """
            # TODO: Compute final angle error and normalize it.
            final_angle_error = self.goal_yaw-self.yaw
            final_angle_error = (final_angle_error + math.pi) % (2 * math.pi) - math.pi

            print(final_angle_error)
            self.get_logger().info(
                f"yaw={math.degrees(self.yaw):.2f}, final_angle_error={math.degrees(final_angle_error):.2f}"
            )

            # TODO: If the error is greater than 0.05 radians, rotate the robot with a speed proportional to the error (0.5 * final_angle_error).
            if abs(final_angle_error) > .05:
                cmd.angular.z = (
                    .8*final_angle_error
                )
            else:
                cmd.angular.z = 0.0  # Stop rotating when aligned
                self.state = "GOAL_REACHED"  # Transition to next state

        elif self.state == "GOAL_REACHED":
            """
            Final state: The goal has been reached, stop movement.
            """
            self.get_logger().info("Goal reached!")
            return  # Exit the function to stop publishing commands

        # TODO: Publish velocity command
        self.publisher_.publish(cmd)


def main(args=None):
    """
    Main entry point of the node. Initializes and runs the MoveToGoal node.
    """
    rclpy.init(args=args)  # Initialize ROS2
    node = MoveToGoal()  # Create node instance
    rclpy.spin(node)  # Keep node running
    node.destroy_node()  # Cleanup before shutdown
    rclpy.shutdown()  # Shutdown ROS2


if __name__ == "__main__":
    main()  # Execute the script
