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
from baxter_core_msgs.msg import EndpointState



class Position(object):
    def __init__(self):
        self.x = 0.9
        self.y = -0.5
        self.z = 0.25


# GLOBALS (SORRY JARVIS)
# instantiate required classes
rospy.init_node("move_cam_at_obj_center")
xyz_pres = Position()
xyz_next = Position()
right = baxter_interface.Limb('right')
# # Call service
ns_r = "ExternalTools/" + 'right' + "/PositionKinematicsNode/IKService"
iksvc_r = rospy.ServiceProxy(ns_r, SolvePositionIK)
ikreq_r = SolvePositionIKRequest()



def get_present_state(data, xyz_pres):
        x = data.pose.position.x
        y = data.pose.position.y
        z = data.pose.position.z
        # print x,y,z
        xyz_pres.x = x
        xyz_pres.y = y
        xyz_pres.z = z
        # print "ENTERED CALLBACK 2", x, y, z


def move_callback(data):
    # iksvc_r = rospy.ServiceProxy(ns_r, SolvePositionIK)
    # ikreq_r = SolvePositionIKRequest()

    # print "got here 2"
    # try:
    # fetch Point data from topic
    x = data.x
    y = data.y
    # print "input from camera:     ", x, y
    # print "input from Subscriber2:", xyz_pres.x, xyz_pres.y, xyz_pres.z
    # print "(x, y) from vision node:", x, y

    # if (x > 5):
    #     xyz_next.y = xyz_pres.y + 0.03#+ x/10000
    # if (x < -5):
    #     xyz_next.y = xyz_pres.y - 0.03#- x/10000
    # if (y > 5):
    #     xyz_next.z = xyz_pres.z - 0.03#- y/10000
    # if (y < -5):
    #     xyz_next.z = xyz_pres.z + 0.03#+ y/10000

    if (abs(x) > 5):
        xyz_next.y = xyz_pres.y + x/5000
    if (abs(y) > 5):
        xyz_next.z = xyz_pres.z - y/5000
    # print "xyz_next.x, xyz_next.y, xyz_next.z", xyz_next.x, xyz_next.y, xyz_next.z

    # print "BEFORE", xyz_pres.x, xyz_pres.y, xyz_pres.z, "|", xyz_next.x, xyz_next.y, xyz_next.z,

    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
        'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x= xyz_next.x,
                    y= xyz_next.y,
                    z= xyz_next.z,
                ),
                orientation=Quaternion(
                    x= 0.0,
                    y= 0.71,
                    z= 0.0,
                    w=0.71,
                ),
            ),
        ),
    }

    # print poses

    ikreq_r.pose_stamp = [poses['right']]
    # print "ikreq_r.pose_stamp", ikreq_r.pose_stamp

    # # try:
    rospy.wait_for_service(ns_r, 5.0)
    resp_r = iksvc_r(ikreq_r)
    # print resp_r
    # except (rospy.ServiceException, rospy.ROSException), e:
    #     rospy.logerr("Service call failed: %s" % (e,))
    #     return

    if (resp_r.isValid[0]):
        print("LEFT_SUCCESS - Valid Joint Solution Found:")
        # Format solution into Limb API-compatible dictionary
        limb_joints_r = dict(zip(resp_r.joints[0].name, resp_r.joints[0].position))
        right.move_to_joint_positions(limb_joints_r)
        #print limb_joints_r
    else:
        print("LEFT_INVALID POSE - No Valid Joint Solution Found.")
    #     print xyz_pres.x, xyz_pres.y, "|", xyz_next.x, xyz_next.y
    # # try:
    # # except UnboundLocalError:
    # #     print "NO SOLUTION FOUND"
    # rospy.wait_for_service(ns_r, 5.0)
    # print "AFTER ",  xyz_pres.x, xyz_pres.y, xyz_pres.z, "|", xyz_next.x, xyz_next.y, xyz_next.z,
    # rospy.sleep(0.2)
    # rospy.spin()


def main():
    #Create a subscriber to /Point msg
    rospy.Subscriber("/object_command", Point, move_callback)
    rospy.Subscriber("/robot/limb/right/endpoint_state", EndpointState, get_present_state, xyz_pres)
    rospy.sleep(0.5)
    rospy.spin()



if __name__ == '__main__':
    main()
