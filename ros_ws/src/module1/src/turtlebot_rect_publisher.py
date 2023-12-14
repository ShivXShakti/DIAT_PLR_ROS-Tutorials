#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import math
import signal

stopped = False

# Signal handler for Ctrl+C
def signal_handler(sig, frame):
    global stopped
    stopped = True

def turtlebot_rect_publisher():
    rospy.init_node('turtlebot_rect_publisher', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=15)

    rate = rospy.Rate(20)  # 20 Hz

    # Define the linear and angular velocities for moving in a rect
    linear_speed = 0.2  # m/s
    angular_speed = math.pi/8  # rad/s
    velocity = Twist()
    signal.signal(signal.SIGINT, signal_handler)

    while not rospy.is_shutdown() and not stopped:
    	for _ in range(4):
        	# Move forward
        	print("side:",_+1)
        	rospy.sleep(1)
        	velocity.linear.x = linear_speed
        	pub.publish(velocity)
        	rospy.sleep(4)  # Move forward for 2 seconds

        	# Stop
        	velocity.linear.x = 0.0
        	pub.publish(velocity)
        	rospy.sleep(2)  # Pause for 1 second

        	# Turn
        	velocity.angular.z = angular_speed
        	pub.publish(velocity)
        	rospy.sleep(4)  # Turn for 90 degrees (pi/2 radians)

        	# Stop turning
        	velocity.angular.z = 0.0
        	pub.publish(velocity)
        	rospy.sleep(2)  # Pause for 1 second

    # Set velocities to zero before exiting
    velocity.linear.x = 0.0
    velocity.angular.z = 0.0
    pub.publish(velocity)
    rospy.signal_shutdown("Finished moving in a rect")

if __name__ == '__main__':
    try:
        turtlebot_rect_publisher()
    except rospy.ROSInterruptException:
        pass

