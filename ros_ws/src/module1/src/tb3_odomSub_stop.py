#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class tb3_odomController:
    def __init__(self):
        rospy.init_node('odom_controller', anonymous=True)
        self.cmd_vel = Twist()
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/odom', Odometry, self.odometry_callback)

    def odometry_callback(self, msg):
        position_obj = msg.pose.pose.position
        orientation_obj = msg.pose.pose.orientation
        linear_velocity = msg.twist.twist.linear
        angular_velocity = msg.twist.twist.angular
        if position_obj.x > 2.0:
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.0
            self.pub.publish(self.cmd_vel)
        else:
            self.cmd_vel.linear.x = 0.2
            self.cmd_vel.angular.z = 0.0
            self.pub.publish(self.cmd_vel)

        rospy.loginfo("Position (x, y, z): {:.2f}, {:.2f}, {:.2f}".format(position_obj.x, position_obj.y, position_obj.z))
        rospy.loginfo(f'Orientation (x, y, z, w): {orientation_obj.x}, {orientation_obj.y}, {orientation_obj.z}, {orientation_obj.x}')
        rospy.loginfo("Linear Velocity (x): {:.2f}".format(linear_velocity.x))
        rospy.loginfo("Angular Velocity (z): {:.2f}".format(angular_velocity.z))
if __name__ == '__main__':
    try:
        tb3_odomController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

