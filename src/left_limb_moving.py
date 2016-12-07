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


q0 = Quaternion()
q0.x, q0.y, q0.z, q0.w = 0.0, 0.71, 0.0, 0.71

q_c = Quaternion()
q_c.x = 0.29
q_c.y = 0.73
q_c.z = -0.29
q_c.w = 0.53


def main():

	global start_pos
	global final_pos
	global center_pos
	global npoints
	global first_time_call

	npoints = 0
	first_time_call = True

	start_pos = Point()
	start_pos.x, start_pos.y, start_pos.z = 0.75, 0.65, -0.0548
	npoints = 0 #using default value

	final_pos = Point()
	final_pos.x, final_pos.y, final_pos.z = 0.9, 0.25, 0.2

	center_pos = Point()
	center_pos.x = 0.78
	center_pos.y = 0.02
	center_pos.z = 0.12
	# center_pos = Point()
	# center_pos.x, center_pos.y, center_pos.z = start_pos.x, start_pos.y, final_pos.z


	#Create a publisher
	command_srv = rospy.Service("/left_limb_command", SetBool, left_limb_srv_callback)
	command_srv.succuss = True
	command_srv.message = "none"
	print "Left limb is waiting for command to move..."
	rospy.spin()


def left_limb_srv_callback(req):
	# if req is true, come to middle position
	mvsrv = rospy.ServiceProxy("/position_service", move)
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

		come_to_center(mvsrv)
	else:
		go_back(mvsrv)

	return True, "Left_limb finished"


def come_to_center(mvsrv):
	#now move to the center position
	global center_pos
	global final_pos
	global npoints

	try:
		# rospy.wait_for_service("/position_service",5)
		retval = mvsrv('left', center_pos, q_c, 0.5, npoints)
	except (rospy.ServiceException, rospy.ROSException), e:
		rospy.logerr("Service call failed: %s" % (e,))
		return
	rospy.sleep(0.2)
	# move to the final center position
	try:
		# rospy.wait_for_service("/position_service",5)
		retval = mvsrv('left', final_pos, q_c, 0.5, npoints)
	except (rospy.ServiceException, rospy.ROSException), e:
		rospy.logerr("Service call failed: %s" % (e,))
		return


def go_back(mvsrv):
	global center_pos
	global start_pos
	#now move to the center position
	try:
		# rospy.wait_for_service("/position_service",5)
		retval = mvsrv('left', center_pos, q_c, 0.5, npoints)
	except (rospy.ServiceException, rospy.ROSException), e:
		rospy.logerr("Service call failed: %s" % (e,))
		return
	rospy.sleep(0.2)
	#move the start location
	try:
		# rospy.wait_for_service("/position_service",5)
		retval = mvsrv('left', start_pos, q0, 0.5, npoints)
		print retval
	except (rospy.ServiceException, rospy.ROSException), e:
		rospy.logerr("Service call failed: %s" % (e,))
		return



if __name__ == '__main__':
	rospy.init_node("left_arm_moving")
	main()
