#!/usr/bin/env python
"""Bug0 algorithm for robot navigation."""

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# TOPICS
# /base_pose_ground_truth
# /base_scan sensor_msgs/LaserScan
# /clock
# /cmd_vel
# /odom
# /rosout
# /rosout_agg
# /tf


class Bug0():
    def __init__(
        self, 
        rate: int = 50,
        speed: float = 10,
        spin_speed: float = 2,
        min_dist: float = 2,
        avoid_area: int = 20, 
        left_turn: bool = True,
    ):
        """Initialize the Bug0 algorithm params."""
        self.rate = rate
        self.speed = speed
        self.spin_speed = spin_speed
        self.min_dist = min_dist
        self.avoid_area = avoid_area # indicies of LaserScan ranges to consider
        self.left_turn = left_turn
        
        # Initial LaserScan values
        self.dist_ahead = None
        self.range_max = None
        

    def laser_callback(self, msg: LaserScan):
        """Get the distance to the closest obstacle in front of the robot, within the avoid_area."""
        n_rays = len(msg.ranges)
        ahead_idx = int(n_rays/2) # idx of the range corresponding to straight ahead
        self.dist_ahead = min(msg.ranges[ahead_idx-self.avoid_area:ahead_idx+self.avoid_area]) # distance straight ahead, in m
        self.range_max = msg.range_max

    

    def main_control_loop(self):
        """Run the bug0 routine."""
        rospy.init_node("bug0", anonymous=True)

        # Publisher object that decides what kind of topic to publish and how fast.
        cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        # Subscribe to the Laser Scanner to get the range to the nearest obstacle
        rospy.Subscriber("/base_scan", LaserScan, self.laser_callback)

        # We will be sending commands of type "twist"
        com = Twist();

        # The main loop will run at a rate in Hz
        rate = rospy.Rate(self.rate)

        while not rospy.is_shutdown():
            if self.dist_ahead:
                
                
                if self.dist_ahead < self.min_dist:
                    s = "Spinning"
                    # Turn in place
                    com.linear.x = 0
                    com.angular.z = self.spin_speed
                else:
                    s = "Advancing"
                    # Full speed ahead
                    com.linear.x = self.speed
                    com.angular.z = 0
                rospy.loginfo(f"Robot is {s}, {self.dist_ahead}")

            # Here's where we publish the actual message.
            cmd_vel_pub.publish(com)
            
            # Sleep for as long as needed to achieve the loop rate.
            rate.sleep()

if __name__ == "__main__":
    bug0 = Bug0()
    bug0.main_control_loop()