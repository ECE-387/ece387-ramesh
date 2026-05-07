import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import numpy as np
from std_msgs.msg import Int32
import time
from rclpy.node import Node  # Base class for all ROS2 nodes
#from sensor_msgs.msg import Image  # ROS2 message type for camera images
#from cv_bridge import CvBridge  # Converts ROS2 Image messages to OpenCV images
#import os
#import cv2
#from pupil_apriltags import Detector  # AprilTag detector library
#import yaml  # Library for parsing YAML files
from scipy.spatial.transform import Rotation  # Converts rotation matrices to quaternions
from ament_index_python.packages import get_package_share_directory  # Utility to locate package directories
from visualization_msgs.msg import Marker


class Final(Node):

    def __init__(self):
        # Initialize the ROS2 node with name 'move_to_goal'
        super().__init__("final")  
        
        # TODO: Publisher for sending velocity commands to the robot
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # TODO: Subscribers to receive odom and imu data
        self.odom_subs = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.imu_subs = self.create_subscription(Imu, 'imu', self.imu_callback, 10)
        # TODO: Subscriber to receive control status
        self.ctrl_subs = self.create_subscription(Bool, 'ctrl_relinq', self.ctrl_relinq_callback, 10)
        self.center_line_sub = self.create_subscription(Marker, "/center_line",self.follow_line, 10)
        self.next_step = self.create_subscription(Bool, 'next_event', self.potato, 10)
        #self.next_step2 = self.create_subscription(Bool, 'next_event2', self.potato2, 10)
        self.ahhh = self.create_subscription(PoseStamped, '/apriltag_pose', self.aprildata, 10)
        self.ahhhh = self.create_subscription(Int32, '/april_tag_ID', self.aprildata2, 10)

        # Variables to store the robot's current position and orientation
        self.x = float(0)  # Current x-coordinate
        self.y = float(0)  # Current y-coordinate
        self.yaw = float(0)  # Current orientation (yaw angle in radians)
        self.current_orientation = None
        self.has_control = False  # Flag indicating whether this node has control


        # Goal position and final orientation
        self.goal_x = -0.61  # Target x position
        self.goal_y = 0.61  # Target y position 
        self.goal_yaw = 0  # Final orientation (0 degrees in radians)

        #self.center_line_sub = self.create_subscription(Marker, 'center_line', self.follow_line, 10) # Update this line to create the subscriber

        # Publish velocity commands to control the robot's movement
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Variables to store the robot's current orientation
        self.current_orientation = None

        # TODO: Timer to run the control loop at a fixed rate (every 0.1 seconds)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.state = "RC Controlled"
        print("Initialization Complete")

        self.timer = 0
        self.state_number = 0
        self.state_number = 0
        self.states_list = ["DRIVE_FORWARD","TURN_LEFT", "DRIVE_FORWARD", "TURN_LEFT"]
        self.APdist = 4.0
        self.APleftorright = 0.0
        self.AProtate = 0.0
        self.APid = 0
        self.yaw = 0
        self.first = True
    def aprildata(self, datapkg: PoseStamped) -> None:
        self.APdist = datapkg.pose.position.z
        self.APleftorright = datapkg.pose.position.x
        self.AProtate = 2*math.atan(datapkg.pose.orientation.z/datapkg.pose.orientation.w)
    
    def aprildata2(self, datapkg2: Int32) -> None:
        self.APid = datapkg2.data

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
        self.has_control=relinq_msg.data

        if self.has_control:
            self.get_logger().info("Final Project has taken control")
        else:
            self.get_logger().info("Final Project has lost control")
        print("ctrl relinq callback")

    def potato(self, switcher: Bool) -> None:
        self.get_logger().info(str(self.state) + str(self.states_list) + str(self.state_number))
        self.state_number = (self.state_number + 1)%len(self.states_list)
        #self.state_number += 1
        self.state = self.states_list[self.state_number]
        #self.get_logger().info("Next node")
        self.get_logger().info(str(self.state) + str(self.states_list) + str(self.state_number))
    
    #def potato2(self, switcher: Bool) -> None:
        
    #    self.state_number = (self.state_number + 1)%len(self.states_list)
    #    self.state = self.states_list[self.state_number]
    #    #self.get_logger().info("Next node")
    #    self.get_logger().info(str(self.state))

    def control_loop(self) -> None:
        """
        Main control loop that executes periodically.
        Controls the robot's movement towards the goal.
        """
        cmd = Twist()  # Create a new Twist message for velocity commands
        if not self.has_control:
            return
        self.timer += 1
        
        # TODO: Compute the angle to the goal
        # Compute target heading angle

        self.states_list = ["DRIVE_FORWARD","TURN_LEFT", "DRIVE_FORWARD", "TURN_LEFT", "DRIVE_FORWARD", "TURN_RIGHT"]

        #self.state = self.states_list[self.state_number]

       

        if (not self.has_control) or (self.state == "RC Controlled"):
            return  # Exit if this node does not have control
        
        elif self.state == "ROTATE":
            goal_theta = math.atan2(self.APleftorright,self.APdist)

            #self.get_logger().info(str(self.APid))
            
            # Compute difference between current and target angle
            angle_error = goal_theta #- self.yaw
            # print(self.yaw)
            
            # Normalize angle error to range [-pi, pi]
            angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

            # TODO: Compute the distance to the goal
            # Difference in x direction
            error_x = self.APdist
            # Difference in y direction
            error_y = self.APleftorright
            # Euclidean distance to goal
            distance = math.sqrt((error_x*error_x) + (error_y*error_y))
            #self.get_logger().info(str(angle_error))

        elif self.state == "DRIVE_FORWARD":
            """
            Second state: Move towards the goal in a straight line.
            """
            goal_theta = math.atan2(self.APleftorright,self.APdist)
            angle_error = goal_theta #- self.yaw
            angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi
            error_x = self.APdist
            error_y = self.APleftorright
            distance = math.sqrt((error_x*error_x) + (error_y*error_y))
            # TODO:
            # Continue moving if not at goal - Move forward at constant speed of 0.15
            # If the distance is less than 0.15, stop moving and transition to final rotation
            self.get_logger().info(str(distance)+ "\n" + str(angle_error))

            if abs(distance)>1.5:
                cmd.linear.x = 2.0
                self.APleftorright = 0.0
            elif abs(distance) > 0.6:
                cmd.linear.x = 2.0
                cmd.angular.z = -2*angle_error
            else:
                cmd.linear.x = 0.0
                self.state_number += 1
                self.state_number = self.state_number%len(self.states_list)
                self.state = self.states_list[self.state_number]
                distance = 4.0
                self.APdist = 4.0
                self.APleftorright = 0.0
            
        elif self.state == "TURN_LEFT":
            """
            First state: Rotate the robot towards the goal.
            """
            if self.first:
                self.turn_goal_theta = math.radians(90) + self.yaw 
                self.first = False
            else: 
                print("")
            angle_error = self.turn_goal_theta - self.yaw
            angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi
            if abs(angle_error) > 0.05:  # Allow small tolerance
                cmd.angular.z = (
                    (abs(angle_error)/angle_error) * min(abs(angle_error/2), 0.2)
                )  # Adjust rotation speed based on error
                self.get_logger().info(str(self.turn_goal_theta) + "\n" + str(angle_error))
            else:
                cmd.angular.z = 0.0  # Stop rotating when aligned
                #self.state_number = (1 + self.state_number)%len(self.states_list)
                #self.state= self.states_list[self.state_number]
                self.first = True
                self.get_logger().info("TURN COMPLETE")
                self.state_number += 1
                self.state_number = self.state_number%len(self.states_list)
                self.state = self.states_list[self.state_number]
                distance = 4.0
                self.APdist = 4.0
                self.APleftorright = 0.0

        elif self.state == "TURN_RIGHT":
            """
            First state: Rotate the robot towards the goal.
            """
            self.turn_goal_theta = 135*math.pi/180 + self.yaw
            if self.first:
                self.turn_goal_theta = self.AProtate + self.yaw
                self.first = False
            else: 
                print("")
            angle_error = self.turn_goal_theta - self.yaw
            angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi
            if abs(angle_error) > 0.05:  # Allow small tolerance
                cmd.angular.z = (
                    (abs(angle_error)/angle_error) * min(abs(angle_error), 0.2)
                )  # Adjust rotation speed based on error
                self.get_logger().info(str(self.turn_goal_theta) + "\n" + str(angle_error))
            else:
                cmd.angular.z = 0.0  # Stop rotating when aligned
                #self.state_number = (1 + self.state_number)%len(self.states_list)
                #self.state= self.states_list[self.state_number]
                self.first = True
                self.get_logger().info("TURN COMPLETE")
                self.state_number += 1
                self.state_number = self.state_number%len(self.states_list)
                self.state = self.states_list[self.state_number]


            #self.get_logger().info(
            #    f"yaw={math.degrees(self.yaw):.2f}, angle_error={math.degrees(angle_error):.2f}, goal_theta={goal_theta:.3f}"
            #)


        elif self.state == "LOOK_AROUND":
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

    def follow_line(self, center_line: Marker) -> None:
        """
        Callback function for the center line marker.
        Computes the robot's alignment with the line and adjusts its movement to follow it.
        """

        if not self.has_control:
            return

        if self.current_orientation is None:
            return  # Wait until we have IMU data

        # Extract the first and last points of the center line
        if len(center_line.points) < 2:
            return  # Not enough points to follow a line

        start_point = center_line.points[0]
        end_point = center_line.points[-1]
        # TODO: Compute the slope of the line and its angle relative to the x-axis.
        # The angle of the line relative to the x-axis is the arctangent of the slope.
        """ OG Code
        line_dx = end_point.x[0]-start_point.x[0]  # Update this line to compute the change in x (x2 - x1)
        line_dy = end_point.y[1]-start_point.y[1]  # Update this line to compute the change in y (y2 - y1)        
        """
        line_dx = end_point.x-start_point.x
        line_dy = end_point.y-start_point.y
        line_angle = np.arctan2(line_dy,line_dx)  # Update this line to compute the angle of the line
        m = line_dy/line_dx  # Update this line to compute the slope (m)
        k = start_point.y-(m*start_point.x)  # Update this line to compute the y-intercept (k)

        # TODO: Convert the line equation y = mx + k into the standard form ax + by + c = 0
        # Here, a = -m, b = 1, c = -k
        a = -m  # Update this line to compute a
        b = 1  # Update this line to compute b
        c = -k  # Update this line to compute c

        # TODO: Compute the perpendicular distance (d) from the robot to the line
        # Note that the robot is at the origin (0, 0).
        distance_error = ((b*start_point.x) + (a*start_point.y) + c) / math.sqrt(a**2 + b**2) # Update this line to compute the distance error

        # Convert quaternion to yaw angle (robot's current orientation)
        q = self.current_orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        robot_yaw = np.arctan2(siny_cosp, cosy_cosp)

        # TODO: Compute angular error to align with the center line
        angle_error = line_angle - robot_yaw  # Update this line to compute the angle error

        # Normalize angle error to range [-pi, pi] to avoid large jumps
        angle_error = (angle_error + np.pi) % (2 * np.pi) - np.pi

        # TODO: Pick your controller gains
        kh = 1  # Heading controller gain (adjusts rotation based on angle error) -- TRIAL AND ERROR
        kd = 3 #.2  # Distance controller gain (adjusts rotation based on distance error) -- TRIAL AND ERROR

        # Adjust rotation speed based on angle error and distance error
        gamma = (-kd * distance_error) + (kh * angle_error) # Update this line to compute gamma (rotation speed)

        

        # To see this message in real-time, run the node with the `--log-level DEBUG` argument:
        # ros2 run lab8_lidar wall_detector --log-level DEBUG
        self.get_logger().debug(
            f"distance error: {distance_error}, angle error: {angle_error}, gamma={gamma}"
        )

        # TODO: Publish the velocity command -- LESSON 4 GAMEPAD
        # The forward speed should be constant.
        cmd_vel_pub = Twist()
        cmd_vel_pub.linear.x = 0.2 # 0.2
        cmd_vel_pub.angular.z = gamma
        self.cmd_vel_pub.publish(cmd_vel_pub)
        #self.get_logger().info("\nGamma: " + str(gamma) + "\nAngle Error: " + str(angle_error) + "\nDistance Error: " + str(distance_error))
        self.get_logger().info(
            f"\ndistance error: {distance_error}, angle error: {angle_error}, gamma={gamma} \nline_angle: {line_angle} robot_yaw: {robot_yaw}\ncmd_vel_pub: {cmd_vel_pub.angular}"
        )

def main(args=None):
    """
    Main entry point of the node. Initializes and runs the MoveToGoal node.
    """
    rclpy.init(args=args)  # Initialize ROS2
    node = Final()  # Create node instance
    rclpy.spin(node)  # Keep node running
    node.destroy_node()  # Cleanup before shutdown
    rclpy.shutdown()  # Shutdown ROS2


if __name__ == "__main__":
    main()  # Execute the script