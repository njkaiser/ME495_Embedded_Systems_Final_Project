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
    baxter_interface.RobotEnable().enable()



    ### END initialize()


if __name__ == '__main__':
    initialize()

### END OF SCRIPT
