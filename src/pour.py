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
# import ang_calc_lib as angcalc
from final_project.srv import move, sweep
from final_project.msg import moveto
from baxter_core_msgs.msg import EndpointState
from std_srvs.srv import SetBool

q0 = Quaternion()
q0.x, q0.y, q0.z, q0.w = 0.0, 0.71, 0.0, 0.71


class Position(object):
    # initialize to a neutral-ish postion, in case we accidentally call without overwriting the values
    def __init__(self, x=0.9, y=0.0, z=0.3):
        self.x = x # FORWARD/BACK, + is Baxter's forward
        self.y = y # LEFT/RIGHT, + is Baxter's left
        self.z = z # VERTICAL, + is up

    def get_point(self):
        # return coordinates as Point()
        p = Point()
        p.x = self.x
        p.y = self.y
        p.z = self.z
        return p



def main(data):
    # don't need to use data at the moment, just ignore it

    side = 'right'

    origin = xyz_pres.get_point() # store for later - we need to go back after pour

    # move up a bit to clear objects on table
    goto = xyz_pres.get_point()
    goto.z += 0.05
    retval = mvsrv(side, goto, q0, 0.5, 0) # 0 = let node determine npoints

    # move to pour position
    goto = xyz_pres.get_point()
    goto.x = 0.95
    goto.y = -0.1
    goto.z = 0.4
    retval = mvsrv(side, goto, q0, 0.6, 0) # 0 = let node determine npoints


    # now that we're in the correct position, pour stuff
    arm = baxter_interface.Limb(side)
    gripper = baxter_interface.Gripper(side)
    ns = "ExternalTools/" + side + "/PositionKinematicsNode/IKService"
    rospy.wait_for_service(ns, 5)
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()

    # tilt to pour
    q_pour = Quaternion()
    q_pour.x = 0.622736455639
    q_pour.y = -0.280677044169
    q_pour.z = 0.631915418651
    q_pour.w = -0.366200228517
    retval = mvsrv(side, goto, q_pour, 0.6, 1) # 1 = only 1 point needed to move between

    # hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    # pose = {
    #     side: PoseStamped(
    #         header=hdr,
    #         pose=Pose(
    #             position=Point(
    #                 x = goto.x,
    #                 y = goto.y,
    #                 z = goto.z,
    #             ),
    #             orientation=Quaternion(
    #                 x = 0.622736455639,
    #                 y = -0.280677044169,
    #                 z = 0.631915418651,
    #                 w = -0.366200228517,
    #             ),
    #         ),
    #     ),
    # }
    #
    # ikreq.pose_stamp = [pose[side]]
    #
    # try:
    #     # rospy.wait_for_service(ns, 5.0)
    #     resp = iksvc(ikreq)
    # except (rospy.ServiceException, rospy.ROSException), e:
    #     rospy.logerr("Service call failed: %s" % (e,))
    #     return
    #
    # if (resp.isValid[0]):
    #     # print("LEFT_SUCCESS - Valid Joint Solution Found:")
    #     # Format solution into Limb API-compatible dictionary
    #     limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
    #     arm.move_to_joint_positions(limb_joints)
    #     # print limb_joints
    # else:
    #     print("LEFT_INVALID POSE - No Valid Joint Solution Found.")

    # hold for a sec
    # print "MADE IT HERE, SLEEPING FOR 8 SECONDS"
    # rospy.sleep(8)

    # tilt back to vertical position
    retval = mvsrv(side, goto, q0, 0.6, 1) # 1 = only 1 point needed to move between
    # hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    # pose = {
    #     side: PoseStamped(
    #         header=hdr,
    #         pose=Pose(
    #             position=Point(
    #                 x = goto.x,
    #                 y = goto.y,
    #                 z = goto.z,
    #             ),
    #             orientation=Quaternion(
    #                 x = 0.0,
    #                 y = 0.71,
    #                 z = 0.0,
    #                 w = 0.71,
    #             ),
    #         ),
    #     ),
    # }
    #
    # ikreq.pose_stamp = [pose[side]]
    #
    # try:
    #     # rospy.wait_for_service(ns, 5.0)
    #     resp = iksvc(ikreq)
    # except (rospy.ServiceException, rospy.ROSException), e:
    #     rospy.logerr("Service call failed: %s" % (e,))
    #     return
    #
    # if (resp.isValid[0]):
    #     # print("LEFT_SUCCESS - Valid Joint Solution Found:")
    #     # Format solution into Limb API-compatible dictionary
    #     limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
    #     arm.move_to_joint_positions(limb_joints)
    #     # print limb_joints
    # else:
    #     print("LEFT_INVALID POSE - No Valid Joint Solution Found.")



    # now move back to original position to
    check_point = Point()
    check_point.x = origin.x
    check_point.y = origin.y
    check_point.z = goto.z
    retval = mvsrv(side, check_point, q0, 0.6, 0) # 0 = let node determine npoints
    origin.z -= 0.3
    retval = mvsrv(side, origin, q0, 0.3, 0) # 0 = let node determine npoints

    rospy.sleep(0.2)

    # drop it like it's hot
    gripper.open()
    rospy.sleep(0.5)
    origin.x -= 0.08
    origin.z += 0.02
    retval = mvsrv(side, origin, q0, 0.6, 0) # 0 = let node determine npoints



    return True, "pour.py executed successfully"



def get_present_state(data, xyz_pres):
    x = data.pose.position.x
    y = data.pose.position.y
    z = data.pose.position.z
    xyz_pres.x = x
    xyz_pres.y = y
    xyz_pres.z = z
    return 0



if __name__ == '__main__':
    rospy.init_node("pour")
    mvsrv = rospy.ServiceProxy("/position_service", move)
    # rospy.Service("/pour", sweep, main)
    xyz_pres = Position()
    rospy.Subscriber("/robot/limb/right/endpoint_state", EndpointState, get_present_state, xyz_pres, queue_size=1)
    rospy.Service("/pour", SetBool, main)
    rospy.spin()
