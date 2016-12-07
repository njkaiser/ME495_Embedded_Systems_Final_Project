#!/usr/bin/env python

import rospy
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
import roslaunch

from final_project.srv import sweep, grasp
from std_srvs.srv import SetBool


if __name__ == '__main__':
    rospy.init_node("master_node")
    grasp = rospy.ServiceProxy("/grasp", grasp)
    rospy.wait_for_service("/grasp", 3.0)
    pour = rospy.ServiceProxy("/pour", SetBool)
    rospy.wait_for_service("/pour", 3.0)
    left_srv = rospy.ServiceProxy("/left_limb_command",SetBool)
    rospy.wait_for_service("/left_limb_command",3.0)


    # run initial sweep looking for ingredients, return y (horizontal) coordinates
    sweep = rospy.ServiceProxy("/initial_sweep", sweep)
    rospy.wait_for_service("/initial_sweep", 3.0)
    object_y = sweep(True)
    print "RETURNED VALUE FROM SWEEP NODE"
    print object_y


    # pass object positions to grasp node, 1 by 1
    obj = Point()
    x = 0.9
    z = -0.06
    obj.x = x
    obj.z = z
    for i, y in enumerate(object_y.positions):
        obj.y = y
        print "object ", i, "position : ", obj.x, obj.y, obj.z

        # head to first object and grab it
        grasp_return = grasp('right', obj, 'get')
        print grasp_return

        left_return = left_srv(1) # bring mixer to pour position

        pour_return = pour(True)
        print pour_return

    left_return = left_srv(0) # return mixer to table


    # main()
