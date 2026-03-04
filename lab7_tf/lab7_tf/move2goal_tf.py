import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from tf_transformations import euler_from_quaternion
from std_srvs.srv import Empty
from nav2_msgs.srv import SetInitialPose
import math


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
        
         # TODO: Create a service server that will handle '/set_pose' service requests
        # - Service type: 'SetInitialPose'
        # - Service name: '/set_pose'
        # - Callback function: 'self.set_pose_callback' to execute when the service is called
        self.reset_service = self.create_service(SetInitialPose,'/set_pose',self.set_pose_callback) # Update this line.
        
        # TODO: Publisher for sending velocity commands to the robot
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # TODO: Subscribers to receive odom and imu data
        self.odom_subs = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.imu_subs = self.create_subscription(Imu, 'imu', self.imu_callback, 10)
        # TODO: Subscriber to receive control status
        self.ctrl_subs = self.create_subscription(Bool, 'ctrl_relinq', self.ctrl_relinq_callback, 10)

        # Local Position Variables (Start at 0.0, 0.0, 0.0)
        self.local_x = 0.0
        self.local_y = 0.0
        self.local_yaw = 0.0

        # Variables for previous position to compute displacement
        self.initial_global_x = None
        self.initial_global_y = None
        self.initial_global_yaw = None

        self.has_control = False #ndicating whether this node has control

        # Goal position and final orientation
        self.goal_x = -0.61  # Target x position
        self.goal_y = 0.61  # Target y position
        self.goal_yaw = 0  # Final orientation (0 degrees in radians)

        self.state = "ROTATE_TO_GOAL"  # Initial state of the robot

        # TODO: Timer to run the control loop at a fixed rate (every 0.1 seconds)
        self.timer = self.create_timer(0.1, self.control_loop)

        

    def set_pose_callback(self, request: SetInitialPose.Request, response: Empty.Response) -> Empty.Response:
        """
        Resets the local position coordinates to zero and clears the previous position coordinates.
        """

        # Reset the local position coordinates
        self.local_x = request.pose.pose.pose.position.x
        self.local_y = request.pose.pose.pose.position.y

        # Convert Quaternion to Euler Angles (Extract Yaw)
        q = request.pose.pose.pose.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        _, _, self.local_yaw = euler_from_quaternion(quaternion)

        # Reset the state
        self.state = "ROTATE_TO_GOAL"

        # Clear the previous position coordinates
        # This ensures that any previous position data is discarded
        self.initial_global_x = None
        self.initial_global_y = None
        self.initial_global_yaw = None

        # Log a message to indicate the local position has been reset
        self.get_logger().info(f"Local pose set to x={self.local_x}, y={self.local_y}, yaw={self.local_yaw}.")

        # Return the response to the service caller
        return response

    def odom_callback(self, msg: Odometry) -> None:
        """
        Callback function for handling odometry messages.
        Updates the local x and y coordinates relative to the starting position.
        """
        # Extract global x and y coordinates from the odometry message
        global_x = msg.pose.pose.position.x
        global_y = msg.pose.pose.position.y

        # Check if this is the first iteration (initial global position not set)
        if self.initial_global_x is None:
            # Store the initial global position as the reference point
            self.initial_global_x = global_x
            self.initial_global_y = global_y
            # Skip processing for the first iteration
            return

        # TODO: Compute the displacement of the global position from the initial global position
        dx = global_x-self.initial_global_x
        dy = global_y-self.initial_global_y

        # TODO: Rotate displacement to align with the initial local frame
        # This step is necessary to ensure the local coordinates are relative to the starting orientation
        theta=self.initial_global_yaw
        self.local_x = math.cos(theta)*dx + math.sin(theta)*dy
        self.local_y = -math.sin(theta)*dx + math.cos(theta)*dy
        


    def imu_callback(self, imu_msg: Imu) -> None:
        """
        Callback function for handling control relinquishment messages.
        Updates local yaw relative to the starting orientation.
        """
        q = imu_msg.orientation
        _, _, global_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        if self.initial_global_yaw is None:
            # Store the initial yaw as a reference
            self.initial_global_yaw = global_yaw
            return  # Skip first iteration

        # TODO: Compute local yaw relative to the initial global yaw
        self.local_yaw = global_yaw-self.initial_global_yaw

    def ctrl_relinq_callback(self, relinq_msg: Bool) -> None:
        """
        Callback function for handling control relinquishment messages.
        Updates the control status based on received messages.
        """
        # TODO: Update control flag
        if relinq_msg.data==True:
            self.has_control = True
        else:
            self.has_control = False
        
        
        if self.has_control:
            self.get_logger().info("move2goal has taken control")
        else:
            self.get_logger().info("move2goal has lost control")


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
        goal_theta = math.atan2(self.goal_y, self.goal_x)
        
        
        # Compute difference between current and target angle
        angle_error = goal_theta-self.local_yaw
        # print(self.yaw)
        
        # Normalize angle error to range [-pi, pi]
        angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

        # TODO: Compute the distance to the goal
        # Difference in x direction
        error_x = self.goal_x+self.local_x
        # Difference in y direction
        error_y = self.goal_y+self.local_y
        # Euclidean distance to goal
        distance = math.sqrt((error_x)**2 + (error_y)**2)
        if self.state == "ROTATE_TO_GOAL":
            """
            First state: Rotate the robot towards the goal.
            """
            if abs(angle_error) > 0.05:  # Allow small tolerance
                cmd.angular.z = (
                    0.3* angle_error
                )  # Adjust rotation speed based on error
            else:
                cmd.angular.z = 0.0  # Stop rotating when aligned
                self.state = "MOVE_FORWARD"  # Transition to next state

            self.get_logger().info(
                f"yaw={math.degrees(self.local_yaw):.2f}, angle_error={math.degrees(angle_error):.2f}, distance={distance:.3f}"
            )

        elif self.state == "MOVE_FORWARD":
            """
            Second state: Move towards the goal in a straight line.
            """
            self.get_logger().info(
                f"x={error_x:.3f}, y={error_y:.3f}, angle_error={math.degrees(angle_error):.2f}, distance={distance:.3f}"
            )

            # TODO:
            # Continue moving if not at goal - Move forward at constant speed of 0.15
            # If the distance is less than 0.15, stop moving and transition to final rotation
            cmd.linear.x = 0.15
            if distance < 0.15:
                cmd.linear.x=0.0
                self.state="ROTATE_TO_FINAL"

        elif self.state == "ROTATE_TO_FINAL":
            """
            Third state: Rotate to match the final desired orientation.
            """
            # TODO: Compute final angle error and normalize it.
            final_angle_error = self.goal_yaw-self.local_yaw
            final_angle_error = (final_angle_error + math.pi) % (2 * math.pi) - math.pi

            self.get_logger().info(
                f"yaw={math.degrees(self.local_yaw):.2f}, final_angle_error={math.degrees(final_angle_error):.2f}"
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
