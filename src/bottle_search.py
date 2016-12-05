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
    # initialize to a neutral-ish postion, in case we accidentally call without overwriting the values
    def __init__(self, x=0.9, y=0.0, z=0.3):
        self.x = x # FORWARD/BACK, + is Baxter's forward
        self.y = y # LEFT/RIGHT, + is Baxter's left
        self.z = z # VERTICAL, + is up

    def get_point(self):
        p = Point()
        p.x = self.x
        p.y = self.y
        p.z = self.z
        return p


rospy.init_node("joint_move_cartesian")

side = 'right'
arm = baxter_interface.Limb(side)

xyz = Position(0., 0., 0.)

xyz_pres = Position(0., 0., 0.)

ns = "ExternalTools/" + side + "/PositionKinematicsNode/IKService"
iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
ikreq = SolvePositionIKRequest()

mvsrv = rospy.ServiceProxy("position_service", move)
mvpub = rospy.Publisher("pos_cmd", moveto, queue_size=1)



def bottle_search(start, end, side):
    y_start = start.y
    y_end = end.y
    dy = abs(y_start - y_end)
    npoints = int(dy/0.01) # SWAG
    x = start.x
    z = start.z
    # for i, y in enumerate(np.linspace(y_start, y_end, npoints, endpoint=True)):
    # rospy.wait_for_service(move, 5.0)
    # x = 0.9
    # y = -0.5
    # z = 0.0
    start = Point()
    start.x, start.y, start.z = x, y_start, z
    print "start position requested: ", start
    retval = mvsrv('right', start, 0.5, npoints)



    end = Point()
    end.x, end.y, end.z = x, y_end, z
    print "end position requested", end
    mvpub.publish('right', end, 0.2, npoints)


    return 0



def get_present_state(data, xyz_pres):
        x = data.pose.position.x
        y = data.pose.position.y
        z = data.pose.position.z
        xyz_pres.x = x
        xyz_pres.y = y
        xyz_pres.z = z



def move_callback(data, xyz):
    xyz.x = data.x
    xyz.y = data.y
    xyz.z = data.z
    return 0


def main():
    #Subscribe to image
    rospy.Subscriber("/robot/limb/right/endpoint_state", EndpointState, get_present_state, xyz_pres, queue_size=1)
    rospy.Subscriber("/object_image_command", Point, move_callback, xyz, queue_size=1)
    rospy.Publisher("pos_cmd", moveto, queue_size=1)

    rospy.wait_for_service(ns, 5.0)
    rospy.sleep(0.05)

    x = 0.9
    y_start = -0.5
    y_end = 0.2
    z = 0.0
    start_pos = Position(x, y_start, z)
    end_pos = Position(x, y_end, z)

    bottle_positions = []
    # for i in np.arange(3):
    bp = bottle_search(start_pos, end_pos, side)
    # print bp
    # quit()
    # bottle_positions.append(bp)
    # print "type(start_pos)", type(start_pos)
    # print start_pos
    # print "type(bp)", type(bp)
    # print bp
    # start_pos.y = bp.y + 0.07
    # start_pos.y += 0.07 # guess

    # print "FINAL POSITIONS APPENDED:"
    # print bottle_positions



    # move outward more 5 cm in x-dir ----> Sill
    # z-axis and orientation must be kept in this form
    # pose = {
    #     'left': PoseStamped(
    #         header=hdr,
    #         pose=Pose(
    #             position=Point(
    #                 x= x_b4+0.05,
    #                 y= 0.278189662246,
    #                 z= -0.07,
    #             ),
    #             orientation=Quaternion(
    #                 x= 0.71,
    #                 y= 0.0,
    #                 z= 0.71,
    #                 w=-0.0,
    #             ),
    #         ),
    #     ),
    # }
    # ikreq_l = SolvePositionIKRequest()
    #
    # ikreq_l.pose_stamp.append(pose['left'])
    # try:
    #     rospy.wait_for_service(ns_l, 5.0)
    #     resp_l = iksvc_l(ikreq_l)
    # except (rospy.ServiceException, rospy.ROSException), e:
    #     rospy.logerr("Service call failed: %s" % (e,))
    #     return
    #
    # if (resp_l.isValid[0]):
    #     print("LEFT_SUCCESS - Valid Joint Solution Found:")
    #     # Format solution into Limb API-compatible dictionary
    #     limb_joints_l = dict(zip(resp_l.joints[0].name, resp_l.joints[0].position))
    #     print limb_joints_l
    # else:
    #     print("LEFT_INVALID POSE - No Valid Joint Solution Found.")
    #
    # left.move_to_joint_positions(limb_joints_l)

if __name__ == '__main__':
    main()
