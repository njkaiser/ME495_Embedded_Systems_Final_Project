#!/usr/bin/env python

import rospy
import baxter_interface
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
import numpy as np
from baxter_core_msgs.msg import EndpointState
from final_project.srv import move
from final_project.msg import moveto



class Position(object):
    def __init__(self):
        self.x = 1.0 # FORWARD/BACK, + is Baxter's forward
        self.y = -0.5 # LEFT/RIGHT, + is Baxter's left
        self.z = 0.25 # VERTICAL, + is up



def move_callback(data):
	#set object size force parameters
    side = data.side
    print side
    point = data.point
    print point
    speed = data.speed
    print speed
    npoints = data.npoints
    print npoints


    ns = "ExternalTools/" + side + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    arm = baxter_interface.Limb(side)
    arm.set_joint_position_speed(0.5 * speed)

    x, y, z = point.x, point.y, point.z

    dx = x - position_now.x
    dy = y - position_now.y
    dz = z - position_now.z

    xs = np.linspace(x - dx, x, npoints, endpoint=True)
    ys = np.linspace(y - dy, y, npoints, endpoint=True)
    zs = np.linspace(z - dz, z, npoints, endpoint=True)

    for i in np.arange(npoints):
        pose = {
            side: PoseStamped(
                header = Header(stamp=rospy.Time.now(), frame_id='base'),
                pose = Pose(
                    position = Point(
                        # x = x[i],
                        # y = y[i],
                        # z = z[i],
                        x = xs[i],
                        y = ys[i],
                        z = zs[i],
                    ),
                    # e-e must be maintained in this orientation
                    orientation = Quaternion(
                        x = 0.0,
                        y = 0.71,
                        z = 0.0,
                        w = 0.71,
                    ),
                ),
            ),
        }

        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp = [pose[side]]

        try:
            resp = iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return
        if (resp.isValid[0]):
            # print("LEFT_SUCCESS - Valid Joint Solution Found:")
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            # arm.set_joint_positions(limb_joints)
            if i == npoints - 1:
                arm.move_to_joint_positions(limb_joints)
            else:
                arm.set_joint_positions(limb_joints)
                rospy.sleep(0.025/speed)

        else:
            print("INVALID POSE - No Valid Joint Solution Found.")
            return -1

    return 1



def get_present_state(data, position_now):
        x = data.pose.position.x
        y = data.pose.position.y
        z = data.pose.position.z
        position_now.x = x
        position_now.y = y
        position_now.z = z



if __name__ == '__main__':
    rospy.init_node("move_node")
    position_now = Position()
    ### THIS NEEDS TO USE THE SIDE, NOT HARDCODED FOR RIGHT
    rospy.Subscriber("/robot/limb/right/endpoint_state", EndpointState, get_present_state, position_now, queue_size=1)
    rospy.Subscriber("/pos_cmd", moveto, move_callback, queue_size=1)
    rospy.Service("/position_service", move, move_callback)
    print "position service initiated"
    rospy.spin()
