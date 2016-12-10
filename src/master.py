#!/usr/bin/env python

import rospy
# from geometry_msgs.msg import (
#     PoseStamped,
#     Pose,
#     Point,
#     Quaternion,
# )
from final_project.srv import sweep, grasp
from std_srvs.srv import SetBool


if __name__ == '__main__':
    # initialize node
    rospy.init_node("master_node")

    # set up subscribers / publishers / services
    grasp = rospy.ServiceProxy("/grasp", grasp)
    rospy.wait_for_service("/grasp", 3.0)

    pour = rospy.ServiceProxy("/pour", SetBool)
    rospy.wait_for_service("/pour", 3.0)

    left_srv = rospy.ServiceProxy("/left_limb_command", SetBool)
    rospy.wait_for_service("/left_limb_command",3.0)

    sweep = rospy.ServiceProxy("/initial_sweep", sweep)
    rospy.wait_for_service("/initial_sweep", 3.0)


    # run initial sweep looking for ingredients, return y (horizontal) coordinates
    object_y = sweep(True)
    print "RETURNED VALUE(S) FROM SWEEP NODE"
    print object_y

    # now take the order from the terminal:
    max_num = len(object_y.positions)
    print "Ingredient positions found and stored:", max_num
    while(1):
        order = raw_input("ENTER INGREDIENT POSITIONS AS INTEGERS: ")
        try:
            indices = order.split()
            indices = [int(e)-1 for e in indices]
            for i in indices:
                if not i == int(i):
                    raise ValueError
                if i >= max_num:
                    raise ValueError
        except ValueError:
            print "INVALID ENTRY, TRY AGAIN"
            continue
        break
    print indices

    # now iterate through object positions, 1 by 1
    obj = Point()
    obj.x = 0.9
    obj.z = -0.06
    max_indices = len(indices)
    for i, index in enumerate(indices):
        # update y-coordinate and fetch the next ingredient
        y = object_y.positions[index]
        obj.y = y
        print "object ", index, "position : ", obj.x, obj.y, obj.z

        # head to first object and grab it
        grasp_return = grasp('right', obj, 'get')
        print grasp_return

        # bring mixer to pour position
        left_return = left_srv(1)
        print left_return

        # do the pour
        pour_return = pour(True)
        print pour_return

    left_return = left_srv(0) # return mixer to table when finished
