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
# from std_srvs.srv import SetBool
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
import numpy as np
from baxter_core_msgs.msg import EndpointState
from final_project.srv import move, sweep
from final_project.msg import moveto

# development only:
import matplotlib.pyplot as plt



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


rospy.init_node("joint_move_cartesian")

side = 'right'
arm = baxter_interface.Limb(side)

object_camera_position = Position()
xyz_pres = Position()

ns = "ExternalTools/" + side + "/PositionKinematicsNode/IKService"
iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
ikreq = SolvePositionIKRequest()

mvsrv = rospy.ServiceProxy("position_service", move)
mvpub = rospy.Publisher("pos_cmd", moveto, queue_size=1)


def bottle_search(start, end, side):

    startpoint = Point()
    startpoint = start.get_point()
    print "start position requested: ", start.x, start.y, start.z
    retval = mvsrv('right', start, 0.7, 0) # 0 = let node determine npoints

    endpoint = end.get_point()
    print "end position requested", end.x, end.y, end.z
    mvpub.publish('right', end, 0.12, 0) # 0 = let node determine npoints

    # x, y, z = end.x, end.y, end.z

    # dx = x - xyz_pres.x
    # dy = y - xyz_pres.y
    # dz = z - xyz_pres.z
    # # print 'dx, dy, dz', dx, dy, dz
    # dist = np.sqrt(dx**2 + dy**2 + dz**2)
    # npoints = int(dist / 0.01)
    #
    # xs = np.linspace(x - dx, x, npoints, endpoint=True)
    # ys = np.linspace(y - dy, y, npoints, endpoint=True)
    # zs = np.linspace(z - dz, z, npoints, endpoint=True)
    #
    # print "npoints", npoints

    # for i in np.arange(npoints):
    #     print i
    #     ppp = Point()
    #     ppp.x = xs[i]
    #     ppp.y = ys[i]
    #     ppp.z = zs[i]
    #     mvpub.publish('right', ppp, 0.2, 1)
    #     rospy.sleep(0.025/0.2)
    #     if object_camera_position.x < 20 and object_camera_position.x !=0:
    #         print "THE CAMERA VALUE EXITED THE BOTTLE SEARCH LOOP"
    #         return 0

    # post process and determine where the objects are
    object_positions = []
    while(abs(xyz_pres.y - end.y) > 0.02):
        tmp = object_camera_position.x
        threshold = 35
        if tmp < threshold and tmp > -threshold and tmp != 0:
            print object_camera_position.x
            print xyz_pres.x, xyz_pres.y, xyz_pres.z
            object_positions.append([object_camera_position.x, xyz_pres.x, xyz_pres.y, xyz_pres.z])
        rospy.sleep(0.1)

    # n_objects = 3 # number of objects we're searching for
    separation = 0.05 # distance between object detections before they're not the same object
    ys = [row[2] for row in object_positions]
    diffs = np.diff(ys)
    diffs = np.insert(diffs, 0, 0)
    # print 'diffs'
    # print diffs
    # print type(diffs)
    count = 0
    tot = 0
    output = []
    for i, y in enumerate(ys):
        if diffs[i] > separation or i == len(ys) - 1:
            output.append(tot/count)
            tot = 0
            count = 0
        tot += y
        count += 1

    print 'OUTPUT'
    print output

    zs = [row[3] for row in object_positions]

    # fig = plt.figure('x-y-z of robot end-effector')
    # ax = fig.gca(projection='3d')
    # ax.scatter(xs, ys, zs)
    plt.figure()
    for v in output:
        plt.axvline(x=v)
    plt.scatter(ys, zs)
    plt.show()

    # while(object_camera_position.x > 20 or object_camera_position.x == 0):
    #     print object_camera_position.x
    #     rospy.sleep(0.15)
    #
    # while(True):
    #     if object_camera_position.x != 0:
    #         obj_xyz_pos = xyz_pres
    #         print "THE CAMERA VALUE EXITED THE BOTTLE SEARCH LOOP WITH VALUE", obj_xyz_pos.x, obj_xyz_pos.y, obj_xyz_pos.z
    #         return 0
    #     else:
    #         rospy.sleep(0.1)
    #         pass

    # mvpub.publish('right', xyz_pres.get_point(), 0.1, 0) # 0 = let node determine npoints

    # return object_positions
    return output


def get_present_state(data, xyz_pres):
    x = data.pose.position.x
    y = data.pose.position.y
    z = data.pose.position.z
    xyz_pres.x = x
    xyz_pres.y = y
    xyz_pres.z = z
    return 0


def move_callback(data, object_camera_position):
    object_camera_position.x = data.x
    object_camera_position.y = data.y
    object_camera_position.z = data.z
    return 0





def main(status):
    go = status.data
    #Subscribe to image
    rospy.Subscriber("/robot/limb/right/endpoint_state", EndpointState, get_present_state, xyz_pres, queue_size=1)
    rospy.Subscriber("/object_image_command", Point, move_callback, object_camera_position, queue_size=1)
    rospy.Publisher("/pos_cmd", moveto, queue_size=1)

    rospy.wait_for_service(ns, 5.0)
    rospy.sleep(0.05)

    x = 0.9
    y_start = -0.5
    y_end = 0.2
    z = -0.05
    start_pos = Position(x, y_start, z)
    end_pos = Position(x, y_end, z)

    bottle_positions = []
    # for i in np.arange(3):
    bp = bottle_search(start_pos, end_pos, side)
    # print bp

    # obj = Point()
    # obj.x = x
    # obj.z = z
    # for i, y in enumerate(bp):
    #     obj.y = y
    #     print "object ", i, "position : ", obj.x, obj.y, obj.z
    #     retval = mvsrv('right', obj, 0.6, 0) # 0 = let node determine npoints
    #     inpt = raw_input("holding, press any key to continue: ")
        # rospy.sleep(5)

    return bp, True # not sure if True is necessary, but I included it in the service prototype anyway in case we want to return False for a failed attempt?



    # xs = [row[1] for row in bp]
    # ys = [row[2] for row in bp]
    # zs = [row[3] for row in bp]

    # fig = plt.figure('x-y-z of robot end-effector')
    # ax = fig.gca(projection='3d')
    # ax.scatter(xs, ys, zs)
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
# def main(status):
#     rospy.Service("/initial_sweep", SetBool, initialize())
#     rospy.spin()

if __name__ == '__main__':
    rospy.Service("/initial_sweep", sweep, main)
    rospy.spin()
