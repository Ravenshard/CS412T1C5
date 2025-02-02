#!/usr/bin/env python
import signal
import rospy
import smach
import smach_ros
import cv2
import cv_bridge
import numpy
import time
import v2
from sensor_msgs.msg import Image
from enum import Enum

bridge = cv_bridge.CvBridge()
shutdown_requested = False

def request_shutdown(sig, frame):
    global shutdown_requested
    shutdown_requested = True

def main():
    global symbol_green_mask_orig, symbol_green_mask_good, h, w, d
    rospy.init_node('green')
    # image_sub = rospy.Subscriber('cv_camera/image_raw',
                                  # Image, logitech_callback)
    image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                      Image, cam1green_callback)
    # image_sub = rospy.Subscriber('usb_cam/image_raw',
                                    # Image, cam1green_callback)
    while not shutdown_requested:
        rospy.spin()
    return

def logitechRed_callback(msg):
    # print("callback")
    global bridge, red_mask, cntR2
    image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    h, w, d = image.shape
    upper_red_a = numpy.array([20, 255, 255])
    lower_red_a = numpy.array([0, 150, 50])
    red_mask_a = cv2.inRange(hsv, lower_red_a, upper_red_a)

    upper_red_b = numpy.array([255, 255, 255])
    lower_red_b = numpy.array([150, 150, 50])
    red_mask_b = cv2.inRange(hsv, lower_red_b, upper_red_b)
    red_mask = cv2.bitwise_or(red_mask_a, red_mask_b)

    blur = cv2.medianBlur(red_mask, 7)
    blur[0:h, 0:w/10] = 0
    blur[0:h, 9*w/10:w] = 0
    thresh = cv2.threshold(blur, 250, 255, 0)[1]
    image2, contours, hierarchy = cv2.findContours(thresh, 2, 1)
    if len(contours) > 0:
        cntR2 = contours[0]
    cv2.imshow("red window",blur)
    cv2.waitKey(3)
    return

def cam1red_callback(msg):
    # print("callback")
    global bridge, red_mask, cntR1
    image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    h, w, d = image.shape

    upper_red = numpy.array([177,255, 255])
    lower_red = numpy.array([0, 150, 50])
    red_mask = cv2.inRange(hsv, lower_red, upper_red)
    blur = cv2.medianBlur(red_mask, 7)
    thresh = cv2.threshold(blur, 250, 255, 0)[1]
    image2, contours, hierarchy = cv2.findContours(thresh, 2, 1)
    if len(contours) > 0:
        cntR1 = contours[0]
    cv2.imshow("red window",blur)
    cv2.waitKey(3)
    return

def cam1green_callback(msg):
    global bridge, green_mask, h, w, d, cntG
    image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    h, w, d = image.shape
    upper_green = numpy.array([149, 255, 255])
    lower_green = numpy.array([57, 80, 0])
    green_mask = cv2.inRange(hsv, lower_green, upper_green)
    blur = cv2.medianBlur(green_mask, 7)
    blurcopy = blur.copy()
    # blur[0:h/2, 0:w] = 0
    # print("pre 0: {}".format(blurcopy[10]))
    blurcopy[0:(0/10)*h, 0:w] = 0
    # print("after 0: {}".format(blurcopy[10]))
    cv2.imshow("green window",blurcopy)
    cv2.waitKey(3)

    thresh = cv2.threshold(blurcopy, 250, 255, 0)[1]
    image2, contours, hierarchy = cv2.findContours(thresh, 2, 1)
    # cntG = contours[0]
    if len(contours) > 0:
        cntG = contours[0]
    del blur, blurcopy
    return

if __name__ == '__main__':
    main()
