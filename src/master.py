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
from final_project.srv import move
from final_project.msg import moveto

# development only:
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D



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


rospy.init_node("master_node")

mvsrv = rospy.ServiceProxy("position_service", move)
mvpub = rospy.Publisher("pos_cmd", moveto, queue_size=1)



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


def main():
    #Subscribe to image
    rospy.Subscriber("/robot/limb/right/endpoint_state", EndpointState, get_present_state, xyz_pres, queue_size=1)
    rospy.Subscriber("/object_image_command", Point, move_callback, object_camera_position, queue_size=1)
    rospy.Publisher("/pos_cmd", moveto, queue_size=1)

    rospy.wait_for_service(ns, 5.0)
    rospy.sleep(0.05)



if __name__ == '__main__':
    main()
