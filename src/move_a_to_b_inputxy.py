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
# from ~/ME495_Final_Project/ws/src/baxter_interface/src/joint_trajectory_action/joint_trajectory_action.py import JointTrajectoryActionServer



def main():

    side = 'right'

    rospy.init_node("joint_move_cartesian")
    #Create a publisher
    arm = baxter_interface.Limb(side)

    # Call service
    ns = "ExternalTools/" + side + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()

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

    # specify x and y
    # The approximately possible bound of x and y for left arm at z = -0.7
    # ----> x : [0.64,1.06]
    # ----> y : [0.01,0.53]
    # y = 0.1

    # jtas = JointTrajectoryActionServer(side, reconfig_server, rate=100.0,
    #          mode='position_w_id'):

    arm.set_joint_position_speed(0.1) # % of max speed

    x = 0.9
    y = -0.45
    z = 0.0

    for i, y in enumerate(np.linspace(-0.45, 0.2, 1000, endpoint=True)):#[-0.4, 0.2]:
        # for z in [0.0, 0]:
            print "x, y, z", x, y, z
            #z-axis and orientation must be kept in this form for baxter to reach outward
            poses = {
                side: PoseStamped(
                    header=hdr,
                    pose=Pose(
                        position=Point(
                            x= x,
                            y= y,
                            z= z,
                        ),
                        orientation=Quaternion(
                            x = 0.0,
                            y = 0.71,
                            z = 0.0,
                            w = 0.71,
                        ),
                    ),
                ),
            }
            # x_b4 = 0.839724445993

            ikreq.pose_stamp = [poses[side]]

            try:
                # rospy.wait_for_service(ns, 5.0)
                resp = iksvc(ikreq)
            except (rospy.ServiceException, rospy.ROSException), e:
                rospy.logerr("Service call failed: %s" % (e,))
                return
            # print resp
            if (resp.isValid[0]):
                # print("LEFT_SUCCESS - Valid Joint Solution Found:")
                # Format solution into Limb API-compatible dictionary
                limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
                if i == 0:
                    arm.set_joint_position_speed(0.5) # % of max speed
                    arm.move_to_joint_positions(limb_joints)
                else:
                    arm.set_joint_position_speed(0.1) # % of max speed
                    arm.set_joint_positions(limb_joints)
                # print limb_joints
                rospy.sleep(0.005)
            else:
                print("LEFT_INVALID POSE - No Valid Joint Solution Found.")




    # move outward more 5 cm in x-dir ----> Sill
    # z-axis and orientation must be kept in this form
    # poses = {
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
    # ikreq_l.pose_stamp.append(poses['left'])
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
