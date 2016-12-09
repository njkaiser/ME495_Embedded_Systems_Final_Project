#!/usr/bin/env python
import argparse
 
import rospy
 
import baxter_interface
import baxter_external_devices
 
from baxter_interface import CHECK_VERSION
from final_project.srv import grasp_status


def main():
	rospy.init_node("gripper_test")
	right = baxter_interface.Gripper('right')
	right.calibrate()
	freq = 1.0
	rate = rospy.Rate(freq)
	rospy.sleep(2)
	print("calibrated")
	right.set_moving_force(30)
	right.set_holding_force(60)
	right.set_dead_band(20)
	rospy.sleep(2)
	right.close()
	# using service to check if the gripper successfully get the object
	req = rospy.ServiceProxy("graspstatus",grasp_status)
	rep = req(1)
	success = rep.success_grasp
	print(success)

	# rospy.sleep(0.8)
	grab_force = right.force()
	grip_pos = right.position()
	print('position: ',grip_pos)
	print('force: ', grab_force)
	# object_size = 48
	# error_size = 5	
	# if grab_force>10 and object_size-error_size<grip_pos<object_size+error_size:
	# 	print("I got it")
	# else:
	# 	print("I missed")
	# rospy.sleep(2)
	# right.open()



	# rospy.sleep(2)
	# right.open()
	# while not rospy.is_shutdown():
	# 	right.set_moving_force(30)
	# 	right.set_holding_force(50)
	# 	right.close()
	# 	grab_force = right.force()
 #    	grip_pos = right.position()
 #    	print("check b")
 #    	print(grab_force)
 #    	rate.sleep()

if __name__ == '__main__':
	main()