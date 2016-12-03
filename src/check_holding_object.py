#!/usr/bin/env python
import argparse
 
import rospy
 
import baxter_interface
import baxter_external_devices
 
from baxter_interface import CHECK_VERSION
from final_project.srv import grasp_status

def main():
	rospy.init_node("check_holding_object")
	s = rospy.Service("graspstatus",grasp_status,srv_callback)
	freq = 10.0
	rate = rospy.Rate(freq)
	# rospy.sleep(1)
	rospy.spin()

def srv_callback(ss):
	#set object size force parameters
	right = baxter_interface.Gripper('right')
	object_size = 42
	force_set = 70
	error_size = 5
	close_command = ss.start_grasp
	if close_command==1:
		rospy.sleep(0.8)
		grab_force = right.force()
		grip_pos = right.position()
		print('position: ',grip_pos)
		print('force: ', grab_force)
		if grab_force>10 and object_size-error_size<grip_pos<object_size+error_size:
			print("I got it")
			return 1
		else:
			print("I missed")
			return 0
	# print('position: ',grip_pos)
	# print('force: ', grab_force)
	# if grab_force>10.0:
	# 	print("i'm grabbing objects")
	# if block_width-error_size < grip_pos < block_width+error_size:
	# 	if not grab_force>force_set/2:


if __name__ == '__main__':
	main()