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


# class xyposition(object):
#     def __init__(self):
#         self.x = 0
#         self.y = 0

    # def update(self, x, y):
    #     self.x = x
    #     self.y = y

    # def get(self):
    #     return self.x, self.y  
        
class Position(object):
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0

def get_present_state(data,xyz_pres):
        x = data.pose.position.x
        y = data.pose.position.y
        z = data.pose.position.z
        # print x,y,z
        xyz_pres.x = x
        xyz_pres.y = y
        xyz_pres.z = z
        # print x,y,z



def callback(data):
    try:
        x = data.x
        y = data.y
        xy.x = x
        xy.y = y
        # print x,y
    except:
        pass


# def initnode():



def main():
    rospy.init_node("move_cam_at_obj_center")
    right = baxter_interface.Limb('right')

    # Call service
    ns_r = "ExternalTools/" + 'right' + "/PositionKinematicsNode/IKService"
    iksvc_r = rospy.ServiceProxy(ns_r, SolvePositionIK)
    ikreq_r = SolvePositionIKRequest()
    

    # initnode()
    xy = Position()
    xyz_pres = Position()
    next_xyz = Position()
    while not rospy.is_shutdown():
        #Create a subscriber to /Point msg
        x = xy.x                
        y = xy.y
        print x, y
        rospy.Subscriber("/object_command", Point, callback)
        # rospy.Subscriber("/robot/limb/right/endpoint_state", EndpointState, get_present_state(EndpointState,xyz_pres))
        rospy.Subscriber("/robot/limb/right/endpoint_state", EndpointState, get_present_state,xyz_pres)

        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        # if (x != 0) && (y != 0):
        if (x > 0):
            next_xyz.x = xyz_pres.x - 0.02
        else:
            next_xyz.x = xyz_pres.x + 0.02
        if (y > 0):
            next_xyz.z = xyz_pres.z + 0.02
        else :
            next_xyz.z = xyz_pres.z - 0.02


        poses = {
            'right': PoseStamped(
                header=hdr,
                pose=Pose(
                    position=Point(
                        x= next_xyz.x,
                        y= xyz_pres.y,
                        z= next_xyz.z,
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
        
        ikreq_r.pose_stamp.append(poses['right'])
        try:
            rospy.wait_for_service(ns_r, 5.0)
            resp_r = iksvc_r(ikreq_r)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return 
                
        if (resp_r.isValid[0]):
            print("LEFT_SUCCESS - Valid Joint Solution Found:")
            # Format solution into Limb API-compatible dictionary
            limb_joints_r = dict(zip(resp_r.joints[0].name, resp_r.joints[0].position))
            #print limb_joints_r
        else:
            print("LEFT_INVALID POSE - No Valid Joint Solution Found.")
        try:
            right.move_to_joint_positions(limb_joints_r)
        except UnboundLocalError:
            print "Fuck u"
            
            


if __name__ == '__main__':
    main()
