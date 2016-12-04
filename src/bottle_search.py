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
# from ~/ME495_Final_Project/ws/src/baxter_interface/src/joint_trajectory_action/joint_trajectory_action.py import JointTrajectoryActionServer
from final_project.srv import move



class Position(object):
    def __init__(self, x, y, z):
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

# mvsrv = rospy.ServiceProxy("position_service", move)



def bottle_search(start, end, side):
    # stop = False
    # hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    y_start = start.y
    y_end = end.y
    dy = abs(y_start - y_end)
    npoints = int(dy/0.03) # SWAG
    x = start.x
    z = start.z
    # for i, y in enumerate(np.linspace(y_start, y_end, npoints, endpoint=True)):
    mvsrv = rospy.ServiceProxy("/position_service", move)
    # rospy.wait_for_service(move, 5.0)
    # x = 0.9
    # y = -0.5
    # z = 0.0
    start = Point()
    end = Point()
    start.x, start.y, start.z = x, y_start, z
    end.x, end.y, end.z = x, y_end, z
    retval = mvsrv('right', start, end, 0.5, npoints)
    status = retval.status
    print(status)

    start.x, start.y, start.z = x, y_end, z
    retval = mvsrv('right', start, end, 0.5, npoints)
    status = retval.status
    print(status)

    return 0
    #     # print "X = ", xyz.x
    #     # if not stop:
    #     pose = {
    #         side: PoseStamped(
    #             header=hdr,
    #             pose=Pose(
    #                 position=Point(
    #                     x= x,
    #                     y= y,
    #                     z= z,
    #                 ),
    #                 # z-axis and orientation must be kept in this form for baxter to reach outward
    #                 orientation=Quaternion(
    #                     x = 0.0,
    #                     y = 0.71,
    #                     z = 0.0,
    #                     w = 0.71,
    #                 ),
    #             ),
    #         ),
    #     }
    #     # else :
    #     #    # print "not hereeeee"
    #     #     pose = {
    #     #         side: PoseStamped(
    #     #             header=hdr,
    #     #             pose=Pose(
    #     #                 position=Point(
    #     #                     x= xyz_pres.x,
    #     #                     y= xyz_pres.y,
    #     #                     z= xyz_pres.z,
    #     #                 ),
    #     #                 orientation=Quaternion(
    #     #                     x = 0.0,
    #     #                     y = 0.71,
    #     #                     z = 0.0,
    #     #                     w = 0.71,
    #     #                 ),
    #     #             ),
    #     #         ),
    #     #     }
    #
    #     ikreq.pose_stamp = [pose[side]]
    #
    #     try:
    #         resp = iksvc(ikreq)
    #     except (rospy.ServiceException, rospy.ROSException), e:
    #         rospy.logerr("Service call failed: %s" % (e,))
    #         return
    #         # print resp
    #     if (resp.isValid[0]):
    #         # print("LEFT_SUCCESS - Valid Joint Solution Found:")
    #         # Format solution into Limb API-compatible dictionary
    #         limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
    #         if i == 0:
    #             arm.set_joint_position_speed(0.5) # % of max speed
    #             arm.move_to_joint_positions(limb_joints)
    #         else:
    #             arm.set_joint_position_speed(0.05 * max([xyz.x/320, 0.3]))# % of max speed
    #             arm.set_joint_positions(limb_joints)
    #             # if stop:
    #                 # x_is_zero = True
    #                 # if x_is_zero:
    #                 # bottle_xyz.append(xyz_pres.get_point())
    #                 # x_is_zero = True
    #
    #                 # can_count == False
    #                 # if
    #                 # print "Bottle_xyz : ", bottle_xyz
    #                 # return xyz_pres.get_point()
    #
    #                 # ROSPY.SLEEP(5)
    #
    #             # print "value", xyz.x
    #             if xyz.x <= 30 and xyz.x >= -5:
    #                 # stop = True
    #                 return xyz_pres.get_point()
    #
    #             # print limb_joints
    #             rospy.sleep(0.001)
    #     else:
    #         print("LEFT_INVALID POSE - No Valid Joint Solution Found.")
    #
    # print "SHOULDN'T MAKE IT HERE", xyz_pres.get_point(), end.get_point()
    # return xyz_pres.get_point()


def get_present_state(data, xyz_pres):
        x = data.pose.position.x
        y = data.pose.position.y
        z = data.pose.position.z
        # print x,y,z
        xyz_pres.x = x
        xyz_pres.y = y
        xyz_pres.z = z
        # print "ENTERED CALLBACK 2", x, y, z


def move_callback(data, xyz):
    xyz.x = data.x
    xyz.y = data.y
    xyz.z = data.z
    # print "THIS IS X : ", x
    return 0



def main():




    # print "XYZ_PRES : ", xyz_pres
    # bottle_xyz = []
    # if len(bottle_xyz) == 3:
    #     return
    #Create a publisher



    #Subscribe to image
    rospy.Subscriber("/robot/limb/right/endpoint_state", EndpointState, get_present_state, xyz_pres, queue_size=1)
    rospy.Subscriber("/object_image_command", Point, move_callback, xyz, queue_size=1)

    # Call service
    rospy.wait_for_service(ns, 5.0)

    # jtas = JointTrajectoryActionServer(side, reconfig_server, rate=100.0,
    #          mode='position_w_id'):

    arm.set_joint_position_speed(0.2) # % of max speed

    x = 0.9
    y_start = -0.5
    y_end = 0.2
    z = 0.0
    start_pos = Position(x, y_start, z)
    end_pos = Position(x, y_end, z)

    bottle_positions = []
    # for i in np.arange(3):
    print "start position requested: ", start_pos.get_point()
    print "end position requested", end_pos.get_point()
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
