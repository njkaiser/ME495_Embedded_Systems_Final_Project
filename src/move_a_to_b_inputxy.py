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



class Position(object):
    def __init__(self):
        self.x = 1.0 # FORWARD/BACK, + is Baxter's forward
        self.y = -0.5 # LEFT/RIGHT, + is Baxter's left
        self.z = 0.25 # VERTICAL, + is up



# def move_function(side, start, speed):
#     # x0, y0, z0 = start.x, start.y, start.z
#     # xf, yf, zf = end.x, end.y, end.z
#
#     # dx = xf - x0
#     # dy = yf - y0
#     # dz = zf - z0
#
#     # npoints = int(np.sqrt(dx**2 + dy**2 + dz**2) / 0.003) # SWAG
#     npoints = 2
#
#     # xs = np.linspace(x0, xf, npoints, endpoint=True)
#     # ys = np.linspace(y0, yf, npoints, endpoint=True)
#     # zs = np.linspace(z0, zf, npoints, endpoint=True)
#     # xs = np.linspace(x0, xf, 2, endpoint=True)
#     # ys = np.linspace(y0, yf, 2, endpoint=True)
#     # zs = np.linspace(z0, zf, 2, endpoint=True)
#
#     x, y, z = start.x, start.y, start.z
#
#     for i in np.arange(npoints):
#         pose = {
#             side: PoseStamped(
#                 header = Header(stamp=rospy.Time.now(), frame_id='base'),
#                 pose = Pose(
#                     position = Point(
#                         # x = x[i],
#                         # y = y[i],
#                         # z = z[i],
#                         x = x,
#                         y = y,
#                         z = z,
#                     ),
#                     # e-e must be maintained in this orientation
#                     orientation = Quaternion(
#                         x = 0.0,
#                         y = 0.71,
#                         z = 0.0,
#                         w = 0.71,
#                     ),
#                 ),
#             ),
#         }
#
#         ikreq = SolvePositionIKRequest()
#         ikreq.pose_stamp = [pose[side]]
#
#         try:
#             resp = iksvc(ikreq)
#         except (rospy.ServiceException, rospy.ROSException), e:
#             rospy.logerr("Service call failed: %s" % (e,))
#             return
#         if (resp.isValid[0]):
#             # print("LEFT_SUCCESS - Valid Joint Solution Found:")
#             # Format solution into Limb API-compatible dictionary
#             limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
#             if i == 0:
#                 arm.set_joint_position_speed(0.5) # % of max speed
#                 arm.move_to_joint_positions(limb_joints)
#             else:
#                 arm.set_joint_position_speed(0.05 * max([xyz.x/320, 0.3]))# % of max speed
#                 arm.set_joint_positions(limb_joints)
#
#                 # print "value", xyz.x
#                 if xyz.x <= 30 and xyz.x >= -5:
#                     return xyz_pres.get_point()
#
#                 # rospy.sleep(0.001)
#         else:
#             print("INVALID POSE - No Valid Joint Solution Found.")
#
#     print "SHOULDN'T MAKE IT HERE", xyz_pres.get_point(), end.get_point()
#     return xyz_pres.get_point()



def service_callback(data):
	#set object size force parameters
    side = data.side
    print side
    start = data.start
    print start
    end = data.end
    print end
    speed = data.speed
    print speed
    npoints = data.npoints
    print npoints


    ns = "ExternalTools/" + side + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    arm = baxter_interface.Limb(side)
    arm.set_joint_position_speed(0.5 * speed)

    # val = move_function(side, start, speed)

    # npoints = 2

    x, y, z = start.x, start.y, start.z

    dx = start.x - position_now.x
    dy = start.y - position_now.y
    dz = start.z - position_now.z

    xs = np.linspace(x - dx, x, npoints, endpoint=True)
    ys = np.linspace(y - dy, y, npoints, endpoint=True)
    zs = np.linspace(z - dz, z, npoints, endpoint=True)
    # xs = np.linspace(x0, xf, 2, endpoint=True)
    # ys = np.linspace(y0, yf, 2, endpoint=True)
    # zs = np.linspace(z0, zf, 2, endpoint=True)

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
            arm.set_joint_positions(limb_joints)
            rospy.sleep(0.05)
            # if i == 0:
            #     arm.set_joint_position_speed(0.5) # % of max speed
            #     arm.move_to_joint_positions(limb_joints)
            # else:
            #     arm.set_joint_position_speed(0.05 * max([xyz.x/320, 0.3]))# % of max speed
            #     arm.set_joint_positions(limb_joints)

                # print "value", xyz.x
    #             if xyz.x <= 30 and xyz.x >= -5:
    #                 return position_now.get_point()
    #
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
    rospy.init_node("position_service_node")
    print "position service initiated"
    position_now = Position()
    ### THIS NEEDS TO USE THE SIDE, NOT HARDCODED FOR RIGHT
    rospy.Subscriber("/robot/limb/right/endpoint_state", EndpointState, get_present_state, position_now, queue_size=1)
    rospy.Service("/position_service", move, service_callback)
    rospy.spin()
