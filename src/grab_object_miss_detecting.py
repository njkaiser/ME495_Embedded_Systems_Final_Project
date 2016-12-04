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
from std_msgs.msg import Header
from sensor_msgs.msg import Range
from baxter_core_msgs.srv import (
	SolvePositionIK,
	SolvePositionIKRequest,
)
import numpy as np
from final_project.srv import grasp_status


def main():


	rospy.init_node("joint_move_cartesian")
	#Create a publisher
	right = baxter_interface.Limb('right')
	right_gripper = baxter_interface.Gripper('right')

	# Customize message
	hdr = Header(stamp=rospy.Time.now(), frame_id='base')
	
	# subscribe to /robot/limb/left/endpoint_state to get present position x of end-effector >>>> haven't done
	# The approximately possible bound of x and y for left arm at z = -0.7
	# ----> x : [0.77,1.03]
	# ----> y : [0.05,0.53]

	x_pos = 0.84617
	y_pos = -0.41685
	z_pos = -0.1202

	#z-axis and orientation must be kept in this form for baxter to reach outward
	#The gripper needs to be calibrated at first
	right_gripper.calibrate()

	poses = {
		'right': PoseStamped(
			header=hdr,
			pose=Pose(
				position=Point(
					x= x_pos,
					y= y_pos,
					z= z_pos,
				),
				orientation=Quaternion(
					x= -0.014,
					y= 0.70,
					z= 0.01437,
					w= 0.7137,
				),
			),
		),
	}

	limb_joints_r = cal_for_IK(poses)
	print(poses)

	right.move_to_joint_positions(limb_joints_r)

	#Now start IR sensor to sense how much further should go
	global range_list
	# length = 20
	# range_list = [None]*length
	# range_list = deque(range_list)
	IR_sub = rospy.Subscriber("/robot/range/right_hand_range/state",Range,range_callback)
	rospy.sleep(1.5)
	# while True:
	# 	try:
	# 		range_total = range_total+range_list.pop()
	# 	except IndexError:
	# 		break
	# range_mean = range_total/length
	newpose = modify_pose(poses,range_list)
	print(newpose)
	#now go to new position
	new_limb_joints_r = cal_for_IK(newpose)
	right.move_to_joint_positions(new_limb_joints_r)

	#perform the close operation, and check if it success
	#set the forces first
	right_gripper.set_moving_force(30)
	right_gripper.set_holding_force(60)
	right_gripper.set_dead_band(20)

	rospy.sleep(2.0)
	right_gripper.close()
	req = rospy.ServiceProxy("graspstatus",grasp_status)
	rep = req(1)
	success = rep.success_grasp
	print(success)
	rospy.sleep(3.0)
	right_gripper.open()


def cal_for_IK(poses):
	# Call service
	ns_r = "ExternalTools/" + 'right' + "/PositionKinematicsNode/IKService"
	iksvc_r = rospy.ServiceProxy(ns_r, SolvePositionIK)
	ikreq_r = SolvePositionIKRequest()
	ikreq_r.pose_stamp.append(poses['right'])
	try:
		rospy.wait_for_service(ns_r, 5.0)
		resp_r = iksvc_r(ikreq_r)
	except (rospy.ServiceException, rospy.ROSException), e:
		rospy.logerr("Service call failed: %s" % (e,))
		return 
	
	if (resp_r.isValid[0]):
		print("RIGHT_SUCCESS - Valid Joint Solution Found:")
		# Format solution into Limb API-compatible dictionary
		limb_joints_r = dict(zip(resp_r.joints[0].name, resp_r.joints[0].position))
		print limb_joints_r
	else:
		print("RIGHT_INVALID POSE - No Valid Joint Solution Found.")
	return limb_joints_r

def range_callback(rr):
	global range_list
	range_list = rr.range
	# if not range_list[-1]==None:
	# 	range_list = range_list.popleft()
	# 	range_list.append(rr.range)
	# else:
	# 	range_list.append(rr.range)
	#print(rr.range)

def modify_pose(old_pose,range_ave):
	#Now modify the forwarding distance
	#assuming that comes to stop when it has a 0.07 gap between object
	hdr_new = Header(stamp=rospy.Time.now(), frame_id='base')
	x_pos = old_pose["right"].pose.position.x
	y_pos = old_pose["right"].pose.position.y
	z_pos = old_pose["right"].pose.position.z
	x_ori = old_pose["right"].pose.orientation.x
	y_ori = old_pose["right"].pose.orientation.y
	z_ori = old_pose["right"].pose.orientation.z
	w_ori = old_pose["right"].pose.orientation.w
	poses_new = {
		'right': PoseStamped(
			header=hdr_new,
			pose=Pose(
				position=Point(
					x= x_pos+(range_ave-0.07),
					y= y_pos,
					z= z_pos,
				),
				orientation=Quaternion(
					x= x_ori,
					y= y_ori,
					z= z_ori,
					w= w_ori,
				),
			),
		),
	}
	return poses_new

if __name__ == '__main__':
	main()