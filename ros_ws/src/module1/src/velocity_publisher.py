#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import signal

stopped = False

# Signal handler for Ctrl+C
def signal_handler(sig, frame):
    global stopped
    stopped = True

def velocity_publisher():
    rospy.init_node('velocity_publisher_node')  # node: velocity_publisher_node
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(1)  # 1 Hz
    velocity = Twist()

    signal.signal(signal.SIGINT, signal_handler)  # Register Ctrl+C signal handler

    while not rospy.is_shutdown() and not stopped:
        velocity.linear.x = 0.2  # Constant linear velocity (m/s)
        velocity.angular.z = 0.1  # Constant angular velocity (rad/s)

        pub.publish(velocity)
        rate.sleep()

    # Set velocities to zero before exiting
    velocity.linear.x = 0.0
    velocity.angular.z = 0.0
    pub.publish(velocity)

if __name__ == '__main__':
    try:
        velocity_publisher()
    except rospy.ROSInterruptException:
        pass

