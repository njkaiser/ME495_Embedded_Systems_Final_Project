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

def main():

    rospy.init_node("joint_move_cartesian")

    ns_l = "ExternalTools/" + 'left' + "/PositionKinematicsNode/IKService"
    ns_r = "ExternalTools/" + 'right' + "/PositionKinematicsNode/IKService"
    iksvc_l = rospy.ServiceProxy(ns_l, SolvePositionIK)
    iksvc_r = rospy.ServiceProxy(ns_r, SolvePositionIK)
    ikreq_l = SolvePositionIKRequest()
    ikreq_r = SolvePositionIKRequest()   
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')

    poses = {
        'left': PoseStamped(
            header=hdr,
            pose=Pose(
                # position=Point(
                #     x=0.657579481614,
                #     y=0.851981417433,
                #     z=0.0388352386502,
                # ),
                # orientation=Quaternion(
                #     x=-0.366894936773,
                #     y=0.885980397775,
                #     z=0.108155782462,
                #     w=0.262162481772,
                # ),

                position=Point(
                    x=0.95,
                    y=1.3,
                    z=0.0388352386502,
                ),
                orientation=Quaternion(
                    x=-0.366894936773,
                    y=0.885980397775,
                    z=0.108155782462,
                    w=0.262162481772,
                ),

            ),
        ),
        'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=0.656982770038,
                    y=-0.852598021641,
                    z=0.0388609422173,
                ),
                orientation=Quaternion(
                    x=0.367048116303,
                    y=0.885911751787,
                    z=-0.108908281936,
                    w=0.261868353356,
                ),
            ),
        ),
    }
    ikreq_l.pose_stamp.append(poses['left'])
    ikreq_r.pose_stamp.append(poses['right'])

    try:
        rospy.wait_for_service(ns_l, 5.0)
        resp_l = iksvc_l(ikreq_l)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 

    try:
        rospy.wait_for_service(ns_r, 5.0)
        resp_r = iksvc_r(ikreq_r)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 

    print "resp_l > ", resp_l
    if (resp_l.isValid[0]):
        print("LEFT_SUCCESS - Valid Joint Solution Found:")
        # Format solution into Limb API-compatible dictionary
        limb_joints_l = dict(zip(resp_l.joints[0].name, resp_l.joints[0].position))
        print limb_joints_l
    else:
        print("LEFT_INVALID POSE - No Valid Joint Solution Found.")

    if (resp_r.isValid[0]):
        print("RIGHT_SUCCESS - Valid Joint Solution Found:")
        # Format solution into Limb API-compatible dictionary
        limb_joints_r = dict(zip(resp_r.joints[0].name, resp_r.joints[0].position))
        print limb_joints_r
    else:
        print("RIGHT_INVALID POSE - No Valid Joint Solution Found.")

    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    # angles_l = left.joint_angles()
    # angles_r = right.joint_angles()
    # angles_l['left_s0']=0.0
    # angles_l['left_s1']=0.0
    # angles_l['left_e0']=0.0
    # angles_l['left_e1']=0.0
    # angles_l['left_w0']=0.0
    # angles_l['left_w1']=0.0
    # angles_l['left_w2']=0.0
    # angles_r['right_s0']=0.0
    # angles_r['right_s1']=0.0
    # angles_r['right_e0']=0.0
    # angles_r['right_e1']=0.0
    # angles_r['right_w0']=0.0
    # angles_r['right_w1']=0.0
    # angles_r['right_w2']=0.0

    left.move_to_joint_positions(limb_joints_l)
    right.move_to_joint_positions(limb_joints_r)


    rospy.signal_shutdown("Finished!!")


if __name__ == '__main__':
    main()
