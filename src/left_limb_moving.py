#!/usr/bin/env python
import rospy
import baxter_interface
from collections import deque
from geometry_msgs.msg import (
	PoseStamped,
	Pose,
	Point,
	Quaternion,
)
from std_srvs.srv import SetBool
from std_msgs.msg import Header
from sensor_msgs.msg import Range
from baxter_core_msgs.srv import (
	SolvePositionIK,
	SolvePositionIKRequest,
)

from baxter_core_msgs.msg import EndpointState
from final_project.srv import move
from final_project.msg import moveto

import numpy as np
from final_project.srv import grasp_status


def main():

	rospy.init_node("left_arm_moving")

	global start_pos
	global final_pos
	global center_pos
	global npoints
	global first_time_call

	npoints = 0
	first_time_call = True

	start_pos = Point()
	start_pos.x, start_pos.y, start_pos.z = 0.75, 0.48, -0.0548
	npoints = 0 #using default value

	final_pos = Point()
	final_pos.x, final_pos.y, final_pos.z = 0.7590, 0.1921, 0.2067

	center_pos = Point()
	center_pos.x, center_pos.y, center_pos.z = start_pos.x, start_pos.y, final_pos.z

	#Create a publisher
	command_srv = rospy.Service("/left_limb_command",SetBool,left_limb_srv_callback)
	command_srv.succuss = True
	command_srv.message = "none"
	print "Left limb is waiting for command to move..."
	rospy.spin()

def left_limb_srv_callback(req):
	# if req is true, come to middle position
	move_srv = rospy.ServiceProxy("/position_service", move)
	print req.data
	if req.data:
		global first_time_call
		if first_time_call:
			#now close the gripper to hold the bottle
			rospy.sleep(1)
			left_gripper = baxter_interface.Gripper('left')
			left_gripper.close()
			rospy.sleep(2)
			first_time_call = False

		come_to_center(move_srv)
	else:
		go_back(move_srv)

	return True, "Left_limb finished"


def come_to_center(move_srv):
	#now move to the center position
	global center_pos
	global final_pos
	global npoints
	try:
		rospy.wait_for_service("/position_service",5)
		retval = move_srv('left', center_pos, 0.5, npoints)
	except (rospy.ServiceException, rospy.ROSException), e:
		rospy.logerr("Service call failed: %s" % (e,))
		return
	rospy.sleep(0.2)
	# move to the final center position
	try:
		rospy.wait_for_service("/position_service",5)
		retval = move_srv('left', final_pos, 0.5, npoints)
	except (rospy.ServiceException, rospy.ROSException), e:
		rospy.logerr("Service call failed: %s" % (e,))
		return 




def go_back(move_srv):
	global center_pos
	global start_pos
	#now move to the center position
	try:
		rospy.wait_for_service("/position_service",5)
		retval = move_srv('left', center_pos, 0.5, npoints)
	except (rospy.ServiceException, rospy.ROSException), e:
		rospy.logerr("Service call failed: %s" % (e,))
		return 
	rospy.sleep(0.2)
	#move the start location
	try:
		rospy.wait_for_service("/position_service",5)
		retval = move_srv('left', start_pos, 0.5, npoints)
		print retval
	except (rospy.ServiceException, rospy.ROSException), e:
		rospy.logerr("Service call failed: %s" % (e,))
		return 

if __name__ == '__main__':
	main()