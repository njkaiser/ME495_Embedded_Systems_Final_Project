#!/usr/bin/env python

# import os
import sys
sys.dont_write_bytecode = True
import rospy
import cv2
import cv_bridge
# from cv_bridge import CvBridge, CvBridgeError
import numpy as np

from sensor_msgs.msg import (
    Image,
)

def initialize():
    bridge = cv_bridge.CvBridge()

def callback(ros_image):
    # pub = rospy.Publisher('/robot/xdisplay', Image, queue_size=2)
    # pub.publish(data)
    bridge = cv_bridge.CvBridge()

    cv_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")

    cv_image_canny = cv2.Canny(cv_image, 150, 300)

    # create a canny edge detection map of the greyscale image
    # cv2.Canny(gray, cv_image, 45, 150, 3)

    # display the canny transformation
    # cv2.ShowImage("Canny Edge Detection", cv_image)

    # cv2.imshow("cv_image", cv_image)
    cv2.imshow("cv_image_canny", cv_image_canny)
    # cv2.waitKey(1000)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # img = cv2.imread('messi5.jpg',0)
    # edges = cv2.Canny(img,100,200)


    ### END OF main()


if __name__ == '__main__':
    try:
        initialize()
        rospy.init_node('object_detect')
        rospy.Subscriber("/usb_cam/image_raw", Image, callback) # testing with webcam
        # rospy.Subscriber("/cameras/left_hand_camera/image", Image, callback)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
### END OF SCRIPT
