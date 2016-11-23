#!/usr/bin/env python

import sys
sys.dont_write_bytecode = True
import rospy
# import tf
import baxter_interface
# import os
# import time
# from moveit_commander import conversions
# from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
# from std_msgs.msg import Header
# import std_srvs.srv
# from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest


def initialize():
    # initialize Baxter
    baxter_interface.RobotEnable().enable()



    ### END initialize()


if __name__ == '__main__':
    try:
        rospy.init_node('baxter_initialize')

        initialize()
        rospy.spin()

    except:
        print "\n/baxter_initialize node killed, DISABLING BAXTER\n"
        baxter_interface.RobotEnable().disable()

### END OF SCRIPT
