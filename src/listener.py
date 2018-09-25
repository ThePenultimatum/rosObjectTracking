#!/usr/bin/env python
# -*- coding: utf-8 -*-

#Created on Thu Sep 20 10:50:36 2018

#@author: vnoelifant


import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
from math import sqrt

bridge = CvBridge()
img_original = np.zeros((480,640,3))
mask = np.zeros((480,640,3))
res = np.zeros((480,640,3))
resg = np.zeros((480,640,3))
cimg = np.zeros((480,640,3))
center_x = 320
center_y = 240

def callback(data):

    global bridge
    global img_original
    global mask
    global res

    #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
        # Convert Image message to CV image with blue-green-red color order (bgr8)
    try:
        img_original = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        #print("==[CAMERA MANAGER]==", e)
        print("error")

    # Convert BGR to HSV
    hsv = cv2.cvtColor(img_original, cv2.COLOR_BGR2HSV)

    # define range of blue color in HSV
    lower_blue = np.array([105,100,100])
    upper_blue = np.array([135,255,255])
    lower_red = np.array([0, 0, 150])
    upper_red = np.array([50, 50 ,255])

    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    #mask = cv2.inRange(hsv, lower_red, upper_red)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(img_original,img_original, mask= mask)

    # get the contours
    _, contours, _ = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)

    hasGreaterThan5000 = False
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 5000:
            hasGreaterThan5000 = True
            # get moments
            moments = cv2.moments(contour)
            global center_x
            global center_y
            center_x = int(moments["m10"]/moments["m00"])
            center_y = int(moments["m01"]/moments["m00"])
            # draw contours and center
            cv2.drawContours(res, contours, -1, (0,255,0), 3)
            cv2.circle(res,(center_x,center_y),7,(0,0,255), -1)
            cv2.drawContours(img_original, contours, -1, (0, 255, 0), 3)
            cv2.circle(img_original,(center_x,center_y),7,(0,0,255), -1)
    if not hasGreaterThan5000:
        center_x = 320
        center_y = 240

def eucDist(p0, p1):
    return sqrt((p1[0] - p0[0])**2 + (p1[1] - p0[1])**2)

def pointCalc():
    global center_x
    global center_y
    globalCenter = [320, 240]
    minDist = 1
    dist = eucDist([center_x, center_y], globalCenter)
    xProp = 0 if dist == 0 else float(center_x - globalCenter[0]) / dist
    yProp = 0 if dist == 0 else float(center_y - globalCenter[1]) / dist
    if dist > minDist:
        return str(int(xProp)) + " " + str(int(yProp))
    else:
        return ""

def publishOrNot(s):
    return s != ""

def main():

    global bridge
    global img_original
    global mask
    global res
    global center_x
    global center_y

    # Initialise ROS node
    rospy.init_node('listener')

    rospy.Subscriber('/usb_cam/image_raw', Image, callback)
    pub = rospy.Publisher('/processing/servo_commands', String, queue_size = 10)

    while not rospy.is_shutdown():
        cv2.imshow('frame',img_original)
        #cv2.imshow('mask',mask)
        #cv2.imshow('res',res)
        messageToPublish = pointCalc()
        print "message to publish:", messageToPublish
        if publishOrNot(messageToPublish):
            print "ACTUALLY PUBLISHING:", messageToPublish
            pub.publish(messageToPublish)
        k = cv2.waitKey(5) & 0xFF
        rospy.sleep(0.05)


    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()

if __name__ == '__main__':
    main()
