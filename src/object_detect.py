#!/usr/bin/env python

import sys
sys.dont_write_bytecode = True
import rospy
import cv2
import cv_bridge
import numpy as np
from collections import deque
from geometry_msgs.msg import Point

from sensor_msgs.msg import (
    Image,
)



class window_with_trackbars(object):
    def __init__(self, window_name, tb_defaults=[], tb_highs=[]):
        # initial variables & setup
        self.window_name = window_name
        self.tb_defaults = tb_defaults
        self.tb_highs = tb_highs
        self.N = len(tb_defaults)
        assert (len(tb_defaults) == len(tb_highs)), "trackbar default and high value input vector lengths are not the same"

        # create named window for appending trackbars, fill with image later
        cv2.namedWindow(self.window_name, cv2.CV_WINDOW_AUTOSIZE)

        # add trackbars for easy development
        for i in range(self.N):
            cv2.createTrackbar('val'+str(i), self.window_name, self.tb_defaults[i], self.tb_highs[i], self.nothing)

    def nothing(self, x):
        # 'do nothing' callback needed for trackbars operation
        # might as well print out the trackbar value
        print "value of trackbar is:", x

    def get_vals(self):
        vals = []
        for i in range(self.N):
            val = cv2.getTrackbarPos('val'+str(i), self.window_name)
            vals.append(val)
        return vals



def filter_red(img_hsv, img_raw):
    # filter non-red objects from camera feed
    v = redwin.get_vals()
    low_vals = np.array([v[0], v[1], v[2]])
    high_vals = np.array([v[3], v[4], v[5]])

    red_lo = cv2.inRange(img_hsv, low_vals, high_vals)
    red_hi = cv2.inRange(img_hsv, np.array([v[6], v[7], v[8]]), np.array([v[9], v[10], v[11]]))
    red = red_lo + red_hi

    # smooth the image a bit
    n = 2
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(n,n))
    red = cv2.morphologyEx(red, cv2.MORPH_OPEN, kernel)
    red = cv2.morphologyEx(red, cv2.MORPH_CLOSE, kernel)

    # now mask onto original image and return
    red = cv2.bitwise_and(img_raw, img_raw, mask=red)
    return red


def object_position(img):

    greyscale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    contours = cv2.findContours(greyscale,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)[0]

    for i, c in enumerate(contours):
        M = cv2.moments(c)
        if M["m00"] > 2500:
            try:
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            except ZeroDivisionError:
                continue
            # print center
            # print center[0], center[1]
            epsilon = 0.003 * cv2.arcLength(c, True)
            c = cv2.approxPolyDP(c, epsilon, True)
            cv2.drawContours(img, [c], -1, (0, 255, 0), thickness=2, maxLevel=0)
            # cv2.drawContours(image, contours, contourIdx, color[, thickness[, lineType[, hierarchy[, maxLevel[, offset]]]]])
            cv2.circle(img, center, 5, (0, 255, 0), -1)
            # cv2.putText(img, str(i), (center[0] - 20, center[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        img_height, img_width, img_depth = img.shape

        try:
            x = center[0] - img_width / 2
            y = center[1] - img_height / 2
            if abs(x) < 50:
                x = 0
            if abs(y) < 50:
                y = 0
        except UnboundLocalError:
            continue
        print "COORDINATES:", x, y

        coords = Point()
        coords.x = x
        coords.y = y
        ocpub.publish(coords)

        # if len(contours) > 0:
        #     c = max(contours,key=cv2.contourArea)
        #     ((x,y), r) = cv2.minEnclosingCircle(c)
        #     M = cv2.moments(c)
        #     center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        #
        #     if r > 5:
        #         #send message
        #         # sumelements = [0, 0]
        #         # pts.appendleft(center)
        #         # deliminator = 5
        #         # if len(pts) < deliminator:
        #         #     deliminator = len(pts)
        #         # for i in np.arange(0, deliminator):
        #         #     sumelements[0] += pts[i][0]
        #         #     sumelements[1] += pts[i][1]
        #         # sumelements[0] = sumelements[0] / deliminator
        #         # sumelements[1] = sumelements[1] / deliminator
        #         # moto_logic(sumelements)
        #
        #         print c
        #
        #         #draw
        #         # cv2.circle(img,(int(x), int(y)), int(r), (0, 255, 0), 2)
        #         # cv2.circle(img,center, 5, (0, 0, 255), -1)
        #         # pts.appendleft(center)
        #
        #         # for i in np.arange(1, len(pts)):
        #         #     thickness = int(np.sqrt(32 / float(i + 1)) * 2.5)
        #         #     cv2.line(img, pts[i-1], pts[i], (0,0,255), thickness)

    return img


def callback(ros_img):
    bridge = cv_bridge.CvBridge()

    # convert ROS image to openCV image for processing
    img_raw = cv2.flip(bridge.imgmsg_to_cv2(ros_img, desired_encoding="bgr8"), 1)
    # img_width, img_height, img_depth = img_raw.shape
    # print img_width, img_height, img_depth
    img_blur = cv2.GaussianBlur(img_raw, (9,9), 1)
    img_hsv = cv2.cvtColor(img_blur, cv2.COLOR_BGR2HSV)
    img_canny = cv2.Canny(img_blur, cannywin.get_vals()[0], cannywin.get_vals()[1])

    img_red = filter_red(img_hsv, img_raw)
    tracked_red = object_position(img_red)

    # cv2.imshow("image_blur", img_blur)
    cv2.imshow("image_canny", img_canny)
    # cv2.imshow("image_hsv", img_hsv)
    cv2.imshow("image_red", img_red)
    cv2.imshow("tracked_red", tracked_red)
    cv2.waitKey(10)




    ### END OF main()



if __name__ == '__main__':
    try:
        # set up initial variables:
        rospy.init_node('object_detect')
        # rospy.init_node('object_command')
        ocpub = rospy.Publisher('object_command', Point, queue_size=10)

        pts = deque(maxlen=32)

        # create window and set up necessary thresholds:
        canny_tb_highs = [1000, 1000]
        # canny_tb_defaults = [200, 120] # for camera
        canny_tb_defaults = [110, 110] # for Baxter hand camera
        cannywin = window_with_trackbars('image_canny', canny_tb_defaults, canny_tb_highs)

        # these values tuned on baxter near fume hood, right hand camera
        red_tb_highs = [179, 255, 255, 179, 255, 255, 179, 255, 255, 179, 255, 255]
        red_tb_defaults = [0, 130, 70, 10, 230, 255, 170, 0, 0, 179, 255, 255]
        redwin = window_with_trackbars('image_red', red_tb_defaults, red_tb_highs)

        # subscribe to whatever image topic we want to use for object detection
        # rospy.Subscriber("/usb_cam/image_raw", Image, callback) # testing with webcam
        rospy.Subscriber("/cameras/right_hand_camera/image", Image, callback)

        # keep program open until we're good and done with it
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

### END OF SCRIPT
