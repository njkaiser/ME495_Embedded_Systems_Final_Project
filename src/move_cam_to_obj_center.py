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
xy = Position()
xyz_pres = Position()
xyz_next = Position()
right = baxter_interface.Limb('right')
# Call service
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


def move_callback(data):
    # iksvc_r = rospy.ServiceProxy(ns_r, SolvePositionIK)
    # ikreq_r = SolvePositionIKRequest()

    # print "got here 2"
    # try:
    # fetch Point data from topic
    x = data.x
    y = data.y

    rospy.Subscriber("/robot/limb/right/endpoint_state", EndpointState, get_present_state, xyz_pres)

    # if (x != 0) && (y != 0):
    if (x > 5):
        xyz_next.y = xyz_pres.y + 0.02
    if (x < -5):
        xyz_next.y = xyz_pres.y - 0.02
    # if (y > 5):
    #     xyz_next.z = xyz_pres.z - 0.02
    # if (y < -5):
    #     xyz_next.z = xyz_pres.z + 0.02

    # xyz_pres.x = xyz_next.x
    # xyz_pres.y = xyz_next.y
    # xyz_pres.z = xyz_next.z

    hdr = Header(stamp=rospy.Time.now(), frame_id='base')

    print "BEFORE", xyz_pres.x, xyz_pres.y, xyz_pres.z, "|", xyz_next.x, xyz_next.y, xyz_next.z,

    poses = {
        'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x= xyz_next.x,
                    y= xyz_pres.y,
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

    # print "poses['right'].pose.position.x =", poses['right'].pose.position.x
    # print type(poses['right'])
    ikreq_r.pose_stamp = []
    ikreq_r.pose_stamp.append(poses['right'])
    # ikreq_r.pose_stamp[0].pose.position.x = xyz_next.x
    # ikreq_r.pose_stamp[0].pose.position.y = xyz_next.y
    # ikreq_r.pose_stamp[0].pose.position.z = xyz_next.z
    # ikreq_r.pose_stamp[0].pose.orientation.x = 0.0
    # ikreq_r.pose_stamp[0].pose.orientation.y = 0.71
    # ikreq_r.pose_stamp[0].pose.orientation.z = 0.0
    # ikreq_r.pose_stamp[0].pose.orientation.w = 0.71
    # print len
    # print ikreq_r.pose_stamp
    # try:
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
        print xyz_pres.x, xyz_pres.y, "|", xyz_next.x, xyz_next.y
    # try:
    # except UnboundLocalError:
    #     print "Fuck u"

    # except:
    #     pass
    # print "AFTER ",  xyz_pres.x, xyz_pres.y, xyz_pres.z, "|", xyz_next.x, xyz_next.y, xyz_next.z,
    rospy.sleep(0.5)


def main():

    while not rospy.is_shutdown():
        #Create a subscriber to /Point msg
        print "got here 1"
        rospy.Subscriber("/object_command", Point, move_callback)
        rospy.sleep(0.5)
        # rospy.Subscriber("/robot/limb/right/endpoint_state", EndpointState, get_present_state(EndpointState,xyz_pres))



if __name__ == '__main__':
    main()
