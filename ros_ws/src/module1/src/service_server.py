#! /usr/bin/env python3
import rospy
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Twist
def callback(request):
    rospy.loginfo("Service has been called")
    twist_obj.linear.x = 0.2
    twist_obj.angular.z = 0.2
    pub.publish(twist_obj)
    rospy.sleep(5)
    twist_obj.linear.x = 0.2
    twist_obj.angular.z = -0.2
    pub.publish(twist_obj)
    rospy.sleep(5)
    twist_obj.linear.x = 0.0
    twist_obj.angular.z = 0.0
    pub.publish(twist_obj)
    rospy.loginfo("Service executed cleanly")
    return EmptyResponse()

rospy.init_node("service_server")
service_server = rospy.Service('/service_move_circle', Empty, callback)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
twist_obj = Twist()
rospy.spin()