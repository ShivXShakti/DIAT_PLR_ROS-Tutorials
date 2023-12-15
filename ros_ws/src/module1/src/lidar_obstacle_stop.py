#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class TurtleBotController:
    def __init__(self):
        rospy.init_node('lidar_distance_controller', anonymous=True)
        self.velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.stop_cmd = Twist()
        
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)

    def lidar_callback(self, msg):
        ranges = msg.ranges
        min_distance = min(ranges)
        rospy.loginfo(f'Minimum Distance to Object: {min_distance} meters')


        if min_distance < 0.6:
            self.stop_cmd.linear.x = 0.0
            self.velocity_pub.publish(self.stop_cmd)
            rospy.loginfo("Obstacle detected. Stopping the robot.")
        else:
            self.stop_cmd.linear.x = 0.2

            self.velocity_pub.publish(self.stop_cmd)

if __name__ == '__main__':
    try:
        controller = TurtleBotController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

