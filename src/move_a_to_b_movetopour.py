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
import ang_calc_lib as angcalc

def main():


    rospy.init_node("joint_move_cartesian")
    #Create a publisher
    left = baxter_interface.Limb('left')
    left_gripper = baxter_interface.Gripper('left')

    # Call service
    ns_l = "ExternalTools/" + 'left' + "/PositionKinematicsNode/IKService"
    iksvc_l = rospy.ServiceProxy(ns_l, SolvePositionIK)
    ikreq_l = SolvePositionIKRequest()

    # Customize message
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')

    # Input A (Start) and B (End)
    # x_a, y_a, z_a = 0,0,0
    # x_b, y_b, z_b = 1,1,1

    # R = np.array([[1,0,0],[0,1,0],[0,0,1]])
    # Quat = angcalc.so3_to_quat(R)

    # Input into pose
    # poses = {
    #     'left': PoseStamped(
    #         header=hdr,
    #         pose=Pose(
    #             position=Point(
    #                 x= 1.03357902807,
    #                 y= 0.460848115622,
    #                 z= -0.0384131810517,
    #             ),
    #             orientation=Quaternion(
    #                 x= 0.692725624165,
    #                 y= 0.12322448771,
    #                 z= 0.699150393326,
    #                 w=-0.127026228658,
    #             ),
    #         ),
    #     ),
    # }
    
    # subscribe to /robot/limb/left/endpoint_state to get present position x of end-effector >>>> haven't done
    # The approximately possible bound of x and y for left arm at z = -0.7
    # ----> x : [0.77,1.03]
    # ----> y : [0.05,0.53]

    #z-axis and orientation must be kept in this form for baxter to reach outward

    poses = {
        'left': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x= 1.15954199591,
                    y= 0.0335153870923,
                    z= 0.36371966232,
                ),
                orientation=Quaternion(
                    x= 0.71,
                    y= 0.0,
                    z= 0.71,
                    w=-0.0,
                ),
            ),
        ),
    }
    x_b4 = 0.839724445993
    ikreq_l.pose_stamp.append(poses['left'])
    try:
        rospy.wait_for_service(ns_l, 5.0)
        resp_l = iksvc_l(ikreq_l)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 
    
    if (resp_l.isValid[0]):
        print("LEFT_SUCCESS - Valid Joint Solution Found:")
        # Format solution into Limb API-compatible dictionary
        limb_joints_l = dict(zip(resp_l.joints[0].name, resp_l.joints[0].position))
        print limb_joints_l
    else:
        print("LEFT_INVALID POSE - No Valid Joint Solution Found.")

    left.move_to_joint_positions(limb_joints_l)
    rospy.sleep(2.0)
    left_gripper.close()

if __name__ == '__main__':
    main()
