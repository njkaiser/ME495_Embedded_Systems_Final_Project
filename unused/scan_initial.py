#!/usr/bin/env python
import rospy
import baxter_interface
import random
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
import numpy as np
import ang_calc_lib as angcalc

def random_seed():
    seed = JointState()
    seed.name = ['right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']
    seed.position = [random.uniform(-1,1),random.uniform(-1,1),random.uniform(-1,1),random.uniform(-1,1),random.uniform(-1,1),random.uniform(-1,1),random.uniform(-1,1)]
    # seed[right_s0] = random.uniform(-1,1)
    # seed[right_s1] = random.uniform(-1,1)
    # seed[right_w0] = random.uniform(-1,1)
    # seed[right_w1] = random.uniform(-1,1)
    # seed[right_w2] = random.uniform(-1,1)
    # seed[right_e0] = random.uniform(-1,1)
    # seed[right_e1] = random.uniform(-1,1)
    # seed = {'right_e0':random.uniform(-1,1), 'right_e1':random.uniform(-1,1), 'right_s0':random.uniform(-1,1), 'right_s1':random.uniform(-1,1), 'right_w0':random.uniform(-1,1), 'right_w1':random.uniform(-1,1), 'right_w2':random.uniform(-1,1)}
    # seed = [random.uniform(-1,1), random.uniform(-1,1), random.uniform(-1,1), random.uniform(-1,1), random.uniform(-1,1), random.uniform(-1,1), random.uniform(-1,1)]
    # seed.name = ['right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']
    # seed.position = [random.uniform(-1,1),random.uniform(-1,1),random.uniform(-1,1),random.uniform(-1,1),random.uniform(-1,1),random.uniform(-1,1)]
    return seed



def main():


    rospy.init_node("scan_initial")
    #Create a publisher
    right = baxter_interface.Limb('right')

    # Call service
    ns_r = "ExternalTools/" + 'right' + "/PositionKinematicsNode/IKService"
    iksvc_r = rospy.ServiceProxy(ns_r, SolvePositionIK)
    ikreq_r = SolvePositionIKRequest()

    # Customize message
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    #z-axis and orientation must be kept in this form for baxter to reach outward
    poses = {
        'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x= 0.85,
                    y= -0.39,
                    z= -0.00,
                ),
                orientation=Quaternion(
                    x= 0.00,
                    y= 0.71,
                    z= 0.0,
                    w= 0.71,
                ),
            ),
        ),
    }

    ikreq_r.pose_stamp = [poses['right']]
    print ikreq_r
    try:
        rospy.wait_for_service(ns_r, 5.0)
        resp_r = iksvc_r(ikreq_r)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 
    print resp_r
    if (resp_r.isValid[0]):
        print("RIGHT_SUCCESS - Valid Joint Solution Found:")
        # Format solution into Limb API-compatible dictionary
        limb_joints_r = dict(zip(resp_r.joints[0].name, resp_r.joints[0].position))
        print limb_joints_r
    else:
        print("RIGHT_INVALID POSE - No Valid Joint Solution Found.")
    right.move_to_joint_positions(limb_joints_r)


    rospy.sleep(5)

    # move outward more 5 cm in x-dir ----> Sill
    # z-axis and orientation must be kept in this form

    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
        'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x= 0.85,
                    y= 0.00,
                    z= 0.00,
                ),
                orientation=Quaternion(
                    x= 0.0,
                    y= 0.71,
                    z= 0.0,
                    w=-0.71,
                ),
            ),
        ),
    }

    ikreq_r = SolvePositionIKRequest()
    ikreq_r.pose_stamp = [poses['right']]
    seed = random_seed()
    ikreq_r.seed_angles = seed
    ikreq_r.seed_mode = 1
    print "IKREQ_FIXED = ", ikreq_r
    #print "IKREQ_R : ",ikreq_r

    try:
        rospy.wait_for_service(ns_r, 5.0)
        resp_r = iksvc_r(ikreq_r)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 
    
    if (resp_r.isValid[0]):
        print("RIGHT_SUCCESS - Valid Joint Solution Found:")
        # Format solution into Limb API-compatible dictionary
        limb_joints_r = dict(zip(resp_r.joints[0].name, resp_r.joints[0].position))
        print limb_joints_r
    else:
        print("RIGHT_INVALID POSE - No Valid Joint Solution Found.")

    right.move_to_joint_positions(limb_joints_r)

if __name__ == '__main__':
    main()
