#!/usr/bin/env python

import sys
sys.dont_write_bytecode = True
import rospy
import baxter_interface



if __name__ == '__main__':
    try:
        rospy.init_node('baxter_initialize')

        # enable robot
        baxter_interface.RobotEnable().enable()

        # calibrate grippers so we can use them later
        for side in ['right', 'left']:
            side = baxter_interface.Gripper(side)
            side.calibrate()

    except:
        # print "\n/baxter_initialize node killed, DISABLING BAXTER\n"
        # baxter_interface.RobotEnable().disable()
        pass

### END OF SCRIPT
