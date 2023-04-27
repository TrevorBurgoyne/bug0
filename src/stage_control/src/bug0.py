#!/usr/bin/env python
"""Bug0 algorithm for robot navigation."""

import rospy
import math
import numpy as np
from typing import Tuple
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion


def parse_inputs() -> Tuple[bool, float, float, float]:
    """Prompt the user for inputs."""
    left_turn = None
    x_goal = None
    y_goal = None
    theta_goal = None

    # Left-turning
    while left_turn is None:
        left_turn = input("Use a left-turning robot? (y/n): ")
        if left_turn.lower() == "y":
            left_turn = True
        elif left_turn.lower() == "n":
            left_turn = False
        else:
            print(f"Input {left_turn} not recognized. Please input 'y' or 'n'.")
            left_turn = None

    # x_goal
    while x_goal is None:
        x_goal = input("Enter x_goal location: ")
        try:
            x_goal = float(x_goal)
        except Exception:
            print(f"Input {x_goal} not recognized. Please input a number.")
            x_goal = None

    # y_goal
    while y_goal is None:
        y_goal = input("Enter y_goal location: ")
        try:
            y_goal = float(y_goal)
        except Exception:
            print(f"Input {y_goal} not recognized. Please input a number.")
            y_goal = None

    # theta_goal
    while theta_goal is None:
        theta_goal = input("Enter desired final orientation (rad, [-pi,pi]): ")
        if theta_goal.lower() == "pi":
            theta_goal = np.pi
        elif theta_goal.lower() == "-pi":
            theta_goal = -np.pi
        else:
            try:
                theta_goal = float(theta_goal)
                if abs(theta_goal) > np.pi:
                    print(f"Angle must be in radians and be between -pi and pi.")

            except Exception:
                print(f"Input {theta_goal} not recognized. Please input a number between -pi and pi.")
                theta_goal = None

    return left_turn, x_goal, y_goal, theta_goal


class Bug0():
    def __init__(
        self, 
        left_turn: bool = True,
        rate: int = 50,
        speed: float = 10,
        spin_speed: float = 2,
        min_dist: float = 3,
        avoid_area: int = 50, 
        x_goal: float = 0,
        y_goal: float = 0,
        theta_goal: float = np.pi,
        tol: float = 1e-1,
    ):
        """Initialize the Bug0 algorithm params."""
        self.left_turn = left_turn
        self.rate = rate
        self.speed = speed 
        self.spin_speed = spin_speed if self.left_turn else -spin_speed
        self.min_dist = min_dist
        self.avoid_area = avoid_area # indicies of LaserScan ranges to consider
        
        # Initial LaserScan values
        self.dist_ahead = None
        self.dist_right = None
        self.dist_theta = None
        
        # Initial Odometry values
        self.x_location = None
        self.y_location = None
        self.theta = None

        # Initial Goal
        self.x_goal = x_goal
        self.y_goal = y_goal
        self.tol = tol # Tolerance for when to be considered at the goal
        self.end_theta_goal = theta_goal # The user-defined ending orientation
        self.theta_goal = None # This is used to store the direction of the goal

        # Robot states
        self.com = Twist() # current command
        self.avoiding_obstacle = False # Whether we're currently trying to avoid an obstacle
        self.aligning_theta = False # Whether we're currently aligning with the theta_goal
        self.theta_ambiguity = False # Flag for when theta and theta_goal are within tolerance, but have a diference of 2pi (ie, -pi and pi)
    

    def at_goal(self) -> bool:
        """Determine if the robot has reached the goal."""
        if self.x_location and self.y_location and self.x_goal and self.y_goal:
            return abs(self.x_location-self.x_goal) < self.tol and abs(self.y_location-self.y_goal) < self.tol
        else:
            return False


    def theta_aligned(self) -> bool:
        """Determine if the robot is pointing towards the goal."""
        if self.theta_ambiguity:
            return True
        elif self.theta_goal and self.theta:
            return abs(self.theta_goal - self.theta) < self.tol
        else:
            return False


    def laser_callback(self, msg: LaserScan):
        """Get the distance to the closest obstacle in front of the robot, within the avoid_area."""
        n_rays = len(msg.ranges)
        ahead_idx = int(n_rays/2) # idx of the range corresponding to straight ahead
        self.dist_ahead = min(msg.ranges[ahead_idx-self.avoid_area:ahead_idx+self.avoid_area]) # distance straight ahead, in m
        # rospy.loginfo(f"Dist ahead: {self.dist_ahead}")
        
        # Find distance in the theta_goal direction
        if self.theta_goal and self.theta:
            # Catch case where theta_goal and theta are very close to pi, but one is negative and one is positive
            if self.theta_ambiguous():
                theta_idx = ahead_idx + int(abs(abs(self.theta_goal)-abs(self.theta)) / msg.angle_increment)
            else:
                theta_idx = ahead_idx + int((self.theta_goal-self.theta) / msg.angle_increment)

            # Wrap around by 2pi if possible
            if theta_idx < 0:
                # Try adding factor of 2pi
                theta_idx = ahead_idx + int((self.theta_goal-self.theta + 2*np.pi) / msg.angle_increment)
            elif theta_idx > n_rays:
                # Try subtracting factor of 2pi
                theta_idx = ahead_idx + int((self.theta_goal-self.theta - 2*np.pi) / msg.angle_increment)

            if theta_idx > 0 and theta_idx < n_rays:
                # Avoid exceeding array limits
                avoid_area_start = theta_idx-self.avoid_area if theta_idx-self.avoid_area >=0 else 0
                avoid_area_end = theta_idx+self.avoid_area if theta_idx+self.avoid_area < n_rays else n_rays - 1
                try:
                    self.dist_theta = min(msg.ranges[avoid_area_start:avoid_area_end])
                except ValueError:
                    # When we can't get a reading, assume that there's an obstacle
                    self.dist_theta = -np.inf
            else:
                # When we can't get a reading, assume that there's an obstacle
                self.dist_theta = -np.inf
            

    def odom_callback(self, msg: Odometry):
        """Get the distance to the closest obstacle in front of the robot, within the avoid_area."""
        # 2D Position
        self.x_location = msg.pose.pose.position.x
        self.y_location = msg.pose.pose.position.y
        
        # Robot Heading: in odometry this is reported as a quaternion, so convert to euler angle
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        # Since we're navigating in 2D, the only relevant angle is yaw, which we will call theta
        _, _, self.theta = euler_from_quaternion(orientation_list)

        # Compute desired heading
        self.theta_goal = math.atan2(self.y_goal-self.y_location, self.x_goal-self.x_location)


    def theta_ambiguous(self) -> bool:
        """Check if theta is ambiguous, and if so set a flag."""
        if (
            self.theta_goal is not None and self.theta is not None and 
            np.sign(self.theta_goal) != np.sign(self.theta) and 
            (
                (abs(self.theta_goal-np.pi) < self.tol and abs(self.theta-np.pi) < self.tol) or # Both close to +/- pi
                (abs(self.theta_goal) < self.tol and abs(self.theta) < self.tol) # Both close to 0
            )
        ):
            # Catch case where theta_goal and theta are very close to pi, but one is negative and one is positive
            self.theta_ambiguity = True
        else:
            self.theta_ambiguity = False
        
        return self.theta_ambiguity


    def heading_feedback_control(self, kp: float = 0.5) -> float:
        """Compute the appropriate command angular velocity using proportional control."""
        if self.theta_goal is not None and self.theta is not None:
            # Catch case where theta_goal and theta are very close to pi, but one is negative and one is positive
            if self.theta_ambiguous():
                return kp*abs(abs(self.theta_goal)-abs(self.theta))
            else:
                return kp*self.shortest_angle_distance()
        else:
            return 0


    def shortest_angle_distance(self) -> float:
        """Find the smallest magnitude angle between theta and theta_goal."""
        return math.atan2(math.sin(self.theta_goal-self.theta), math.cos(self.theta_goal-self.theta))


    def obstacle_avoidance(self):
        """Get commands to avoid collisions with objects detected in the robot's path."""
        # Skip avoidance if we're currently aligning theta
        if not self.aligning_theta:
            if self.dist_ahead < self.min_dist:
                # Turn left if there's an obstacle ahead
                self.com.linear.x = 0
                self.com.angular.z = self.spin_speed
                self.avoiding_obstacle = True
            else:
                # If there's nothing ahead, then we can proceed
                self.com.linear.x = self.speed
                self.com.angular.z = 0


    def align_theta(self):
        """Get commands to align the robot with the direction of the goal."""     
        # Once there's nothing blocking the theta_goal direction,
        # we can start turning towards it.
        if self.aligning_theta or self.dist_theta > 2*self.min_dist:
            # We're past any obstacle
            self.avoiding_obstacle = False
            self.aligning_theta = True
            self.com.angular.z = self.heading_feedback_control()
            self.com.linear.x = 0 # spin in place until we're aligned

            if self.theta_aligned():
                self.aligning_theta = False
                # Full speed ahead
                self.com.linear.x = self.speed
             

    def main_control_loop(self):
        """Run the bug0 routine."""
        print("Running bug0 algorithm...")
        rospy.init_node("bug0", anonymous=True)

        # Publisher object that decides what kind of topic to publish and how fast.
        cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        # Subscribe to the Laser Scanner to get the range to the nearest obstacle
        rospy.Subscriber("/base_scan", LaserScan, self.laser_callback)
        # Subscribe to the Odometry to get robot location
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # The main loop will run at a rate in Hz
        rate = rospy.Rate(self.rate)
        
        while not rospy.is_shutdown():
            # Exit when at the goal
            if self.at_goal():
                # Now that we've arrived, just align the theta
                self.theta_goal = self.end_theta_goal
                self.align_theta()
                if abs(self.theta - self.theta_goal) < self.tol:
                    rospy.loginfo(f"Reached the goal! Final position: x = {self.x_location}, y = {self.y_location}, theta = {self.theta}")
                    break
            
            # Run main routine once data is initialized 
            if self.dist_ahead and self.dist_theta:
                self.align_theta()          
                self.obstacle_avoidance()
            
            # if self.aligning_theta:
            #     rospy.loginfo("Aligning...")
            # if self.avoiding_obstacle:
            #     rospy.loginfo("Avoiding...")
            # if self.theta_ambiguity:
            #     rospy.loginfo(f"Theta Ambiguity")
            
            # Here's where we publish the current commands.
            cmd_vel_pub.publish(self.com)
            
            # Sleep for as long as needed to achieve the loop rate.
            rate.sleep()

if __name__ == "__main__":
    left_turn, x_goal, y_goal, theta_goal = parse_inputs()
    bug0 = Bug0(left_turn=left_turn, x_goal=x_goal, y_goal=y_goal, theta_goal=theta_goal)
    bug0.main_control_loop()