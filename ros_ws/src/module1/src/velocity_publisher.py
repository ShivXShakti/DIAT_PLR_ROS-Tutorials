#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def velocity_publisher():
    rospy.init_node('velocity_publisher_node') #node: velocity_publisher_node
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        # Create a Twist message to represent linear and angular velocities
        velocity = Twist()
        velocity.linear.x = 0.2  # Constant linear velocity (m/s)
        velocity.angular.z = 0.1  # Constant angular velocity (rad/s)

        pub.publish(velocity)
        rate.sleep()

if __name__ == '__main__':
    try:
        velocity_publisher()
    except rospy.ROSInterruptException:
        pass

