#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetWorldProperties, SetModelState
from gazebo_msgs.msg import ModelState
import signal

stopped = False

# Signal handler for Ctrl+C
def signal_handler(sig, frame):
    global stopped
    stopped = True

def get_model_names():
    rospy.wait_for_service('/gazebo/get_world_properties')
    try:
        get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        response = get_world_properties()
        return response.model_names
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return []

def stop_robot(twist_obj, publisher_obj):
    model_names = get_model_names()
    if len(model_names) > 0:
        robot_model_name = model_names[1]  # Get the first model as an example
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            model_state = ModelState()
            model_state.model_name = robot_model_name
            model_state.twist.linear.x = 0.0
            model_state.twist.angular.z = 0.0
            set_model_state(model_state)
            twist_obj.linear.x = 0.0
            twist_obj.angular.z = 0.0
            publisher_obj.publish(twist_obj)
            rospy.loginfo("Robot '%s' stopped in Gazebo." % robot_model_name)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
    else:
        rospy.logwarn("No models found in Gazebo.")

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

    stop_robot(velocity, pub)

if __name__ == '__main__':
    try:
        velocity_publisher()
    except rospy.ROSInterruptException:
        pass

