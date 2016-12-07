#!/usr/bin/env python
import rospy
import baxter_interface
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

def main():

	rospy.init_node("left_testing")
	the_srv = rospy.ServiceProxy("/left_limb_command",SetBool)
	try:
		rospy.wait_for_service("/left_limb_command",5)
		resp = the_srv(True)
	except (rospy.ServiceException, rospy.ROSException), e:
		rospy.logerr("Service call failed: %s" % (e,))
		return

	
	rospy.sleep(3)
	try:
		resp2 = the_srv(False)
	except (rospy.ServiceException, rospy.ROSException), e:
		rospy.logerr("Service call failed: %s" % (e,))
		return
	

if __name__ == '__main__':
	main()