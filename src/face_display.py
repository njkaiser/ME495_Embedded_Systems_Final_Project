#!/usr/bin/env python

import sys
sys.dont_write_bytecode = True
import rospy
from sensor_msgs.msg import Image


def callback(data):
    pub = rospy.Publisher('/robot/xdisplay', Image, queue_size=2)
    pub.publish(data)


if __name__ == '__main__':
    try:
        rospy.init_node('port_camera_to_display')
        # rospy.Subscriber("/usb_cam/image_raw", Image, callback) # testing with webcam
        rospy.Subscriber("/cameras/left_hand_camera/image", Image, callback)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
