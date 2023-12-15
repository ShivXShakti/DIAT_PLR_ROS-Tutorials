#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import numpy as np
class TurtleBotObstacleAvoidance:
    def __init__(self):
        rospy.init_node('obstacle_avoidance', anonymous=True)
        self.velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)

        self.min_distance = 0.8  # Minimum distance to consider an obstacle (in meters)
        self.front_angle_range = (-90, 90)  # Define the front angle range in degrees
        self.cmd_vel = Twist()

    def lidar_callback(self, msg):
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        front_range = [ranges[i] for i in range(len(ranges)) if angle_min + i * angle_increment >= math.radians(self.front_angle_range[0]) 
                                                            and angle_min + i * angle_increment <= math.radians(self.front_angle_range[1])]
        min_distance = min(front_range)
        rospy.loginfo(f'Minimum Distance to Object (Front 120 degrees): {min_distance} meters')

        if min_distance < self.min_distance:
            cmd = self.avoid_obstacle()
            self.cmd_vel.linear.x = cmd[0]
            self.cmd_vel.angular.z = cmd[1]
            self.velocity_pub.publish(self.cmd_vel)
        
        else:
            self.cmd_vel.linear.x = 0.2  # Move forward with a constant velocity
            self.velocity_pub.publish(self.cmd_vel)

    def avoid_obstacle(self):
        x = 0.0  # Stop linear motion
        z = 0.2  # Rotate to the left
        return np.array([x,z])

if __name__ == '__main__':
    try:
        avoidance_controller = TurtleBotObstacleAvoidance()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

