#!/usr/bin/env python

# import os
import sys
sys.dont_write_bytecode = True
import rospy
import cv2
import cv_bridge
# import trackbar as tb
# from cv_bridge import CvBridge, CvBridgeError
import numpy as np

from sensor_msgs.msg import (
    Image,
)



# def initial_setup():
#     bridge = cv_bridge.CvBridge()
#     val1 = 100
#     val2 = 100
#     cv2.createTrackbar('val1', 'cv_image_canny', val1, 1000, nothing)




class Canny(object):
    def __init__(self, window_name):
        # create named window for appending trackbars, fill with image later
        self.window_name = window_name
        cv2.namedWindow(self.window_name, cv2.CV_WINDOW_AUTOSIZE)

        # initial values for trackbars:
        self.val1 = 200
        self.val2 = 120

        # create trackbars
        cv2.createTrackbar('val1', self.window_name, self.val1, 1000, self.nothing)
        cv2.createTrackbar('val2', self.window_name, self.val2, 1000, self.nothing)

    def nothing(self, x):
        # 'do nothing' callback needed for trackbars operation
        # may as well print out the trackbar value while I'm here
        print "value of trackbar is:", x

    def get_val1(self):
        return cv2.getTrackbarPos('val1', self.window_name)

    def get_val2(self):
        return cv2.getTrackbarPos('val2', self.window_name)


def callback(ros_image):
    bridge = cv_bridge.CvBridge()

    # convert ROS image to openCV image for processing
    cv_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")
    cv_image_blur = cv2.GaussianBlur(cv_image, (5,5), 1)
    cv_image_canny = cv2.Canny(cv_image_blur, canny.get_val1(), canny.get_val2())

    cv2.imshow("cv_image_blur", cv_image_blur)
    cv2.imshow("cv_image_canny", cv_image_canny)
    cv2.waitKey(10)

    # pub = rospy.Publisher('/robot/xdisplay', Image, queue_size=2)
    # pub.publish(data)

    ### END OF main()


if __name__ == '__main__':
    try:
        # set up initial variables:
        rospy.init_node('object_detect')
        canny = Canny('cv_image_canny')

        # subscribe to whatever image topic we want to use for object detection
        rospy.Subscriber("/usb_cam/image_raw", Image, callback) # testing with webcam
        # rospy.Subscriber("/cameras/left_hand_camera/image", Image, callback)

        # keep program open until we're good and done with it
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

### END OF SCRIPT
