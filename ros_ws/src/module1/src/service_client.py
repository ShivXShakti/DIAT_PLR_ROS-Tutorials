#! /usr/bin/env python3
import rospy
from std_srvs.srv import Empty, EmptyRequest 

rospy.init_node('service_client')
rospy.wait_for_service('/service_move_circle')
traj_name_service_client = rospy.ServiceProxy('/service_move_circle', Empty)
traj_name_request = EmptyRequest()

traj_name_service_client(traj_name_request)