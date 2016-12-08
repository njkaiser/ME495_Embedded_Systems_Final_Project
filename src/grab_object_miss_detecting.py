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
from final_project.msg import moveto
from final_project.srv import grasp_status, grasp, move



class range_data(object):
	def __init__(self):
		self.val = 0



def main(data):
	# extract data from message
	side = data.side
	goto = data.position
	x, y, z = goto.x, goto.y, goto.z
	command = data.command

	q0 = Quaternion()
	q0.x, q0.y, q0.z, q0.w = 0.0, 0.71, 0.0, 0.71


	# need to use this if block to either pick up or put back the item
	if command == 'get':
		pass
	elif command == 'return':
		pass
	else:
		raise(TypeError, "GRASP COMMAND HAS ONLY 2 OPTIONS: 'get' or 'return'")


	# move into initial grasping position
	retval = mvsrv(side, goto, q0, 0.5, 0) # 0 = let node determine npoints


	# set the forces first
	gripper = baxter_interface.Gripper(side)
	gripper.set_moving_force(30)
	gripper.set_holding_force(60)
	gripper.set_dead_band(20)


	# perform the close operation, and check if it success
	# gripper.close()

	# gripper.open()

	#take average for 20 times
	range_sum = 0.0
	ii = 0
	while ii<20:
		range_sum = range_sum+range_data.val
		ii = ii+1
	range_ave = range_sum/ii
	if range_ave > 0.2:
		# search_left = original_pos+0.02
		# search_right = original_pos-0.02

		# goto.y = search_right
		# retval = mvsrv(side, goto, q0, 0.2, 0)
		# if range_data.val <0.2:
		# 	#now get the object
		# 	goto.y = original_pos
		# 	retval = mvsrv(side, goto, q0, 0.2, 0)
		print "Cannot detect the object"
	else:
		print "detect object"

	if range_ave > 0.18:
		range_ave = 0.18 # clip in case IR measurement misses object

	goto.x = goto.x + range_ave - 0.06
	retval = mvsrv(side, goto, q0, 0.2, 0) # 0 = let node determine npoints
	rospy.sleep(0.3)

	gripper.close()
	rospy.sleep(0.2)
	req = rospy.ServiceProxy("graspstatus", grasp_status)
	rep = req(1)
	success = rep.success_grasp
	print(success)
	rospy.sleep(0.05)

	goto.z = goto.z + 0.3
	retval = mvsrv(side, goto, q0, 0.6, 0) # 0 = let node determine npoints

	# rospy.sleep(2.0)
	# gripper.open()
	#
	# goto.x = 0.9
	# retval = mvsrv(side, goto, 0.4, 0) # 0 = let node determine npoints

	return True



def range_callback(rr):
	range_data.val = rr.range



	# global range_list
	# range_list = rr.range
	# if not range_list[-1]==None:
	# 	range_list = range_list.popleft()
	# 	range_list.append(rr.range)
	# else:
	# 	range_list.append(rr.range)
	# print(rr.range)



if __name__ == '__main__':
	rospy.init_node("grasp")
	rospy.Service("/grasp", grasp, main)
	mvsrv = rospy.ServiceProxy("position_service", move)
	IR_sub = rospy.Subscriber("/robot/range/right_hand_range/state", Range, range_callback)
	# mvpub = rospy.Publisher("/pos_cmd", moveto, queue_size=1)
	rospy.spin()
	# main()
