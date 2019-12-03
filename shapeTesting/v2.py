#!/usr/bin/env python

import signal
import rospy
import smach
import smach_ros
import cv2
import cv_bridge
import numpy
import time
import math
from sensor_msgs.msg import Image
from enum import Enum

green_mask = [['a']]
red_mask = [['a']]
bridge = cv_bridge.CvBridge()
shutdown_requested = False
h, w, d = 0, 0, 0
# c1Triange = cv2.imread('./shapeTesting/c1Triangle.png', 0)
# c2Triange = cv2.imread('./shapeTesting/c2Triangle.png', 0)
# c1Square = cv2.imread('./shapeTesting/c1Square.png', 0)
# c2Square = cv2.imread('./shapeTesting/c2Square.png', 0)
# c1Circle = cv2.imread('./shapeTesting/c1Circle.png', 0)
# c2Circle = cv2.imread('./shapeTesting/c2Circle.png', 0)
c1Triange = cv2.imread('c1Triangle.png', 0)
c2Triange = cv2.imread('./shapeTesting/c2Triangle.png', 0)
c1Square = cv2.imread('c1Square.png', 0)
c2Square = cv2.imread('./shapeTesting/c2Square.png', 0)
c1Circle = cv2.imread('c1Circle.png', 0)
c2Circle = cv2.imread('./shapeTesting/c2Circle.png', 0)
cntG = None
cntR1 = None
cntR2 = None


def request_shutdown(sig, frame):
    global shutdown_requested
    shutdown_requested = True

def initC1Images():
    ''' Initialize the model images for triangle, square and circle.
        Use the shapes for c1 (camera 1, 3d sensors)
    '''
    threshT = cv2.threshold(c1Triange, 250, 255,0)[1]
    threshS = cv2.threshold(c1Square, 250, 255,0)[1]
    threshC = cv2.threshold(c1Circle, 250, 255,0)[1]
    _, contoursT, hierarchyT = cv2.findContours(threshT,2,1)
    _, contoursS, hierarchyS = cv2.findContours(threshS,2,1)
    _, contoursC, hierarchyC = cv2.findContours(threshC,2,1)
    cntT = contoursT[0]
    cntS = contoursS[0]
    cntC = contoursC[0]
    return (cntT, cntS, cntC)

def initC2Images():
    ''' Initialize the model images for triangle, square and circle.
        Use the shapes for c2 (camera 2, logitech)
    '''
    threshT = cv2.threshold(c1Triange, 250, 255,0)[1]
    threshS = cv2.threshold(c2Square, 250, 255,0)[1]
    threshC = cv2.threshold(c2Circle, 250, 255,0)[1]
    _, contoursT, hierarchyT = cv2.findContours(threshT,2,1)
    _, contoursS, hierarchyS = cv2.findContours(threshS,2,1)
    _, contoursC, hierarchyC = cv2.findContours(threshC,2,1)
    cntT = contoursT[0]
    cntS = contoursS[0]
    cntC = contoursC[0]
    return (cntT, cntS, cntC)

def compareImages(cntT, cntS, cntC, cntI):
    ''' Compare cntI with the other shapes and return the one which cntI is
        the closest to
    Parameters:
        cntT (contour): contours as found with cv2. findContours()
        cntS (contour): contours as found with cv2. findContours()
        cntC (contour): contours as found with cv2. findContours()
        cntI (contour): contours as found with cv2. findContours()

    Returns:
        string: whichever cnt matches best, last letter of cnt
                matches the name of the shape
    '''
    retT = cv2.matchShapes(cntI,cntT,1,0.0)
    retS = cv2.matchShapes(cntI,cntS,1,0.0)
    retC = cv2.matchShapes(cntI,cntC,1,0.0)
    best = min(retT, retS, retC)
    if best == retC: return "circle"
    if best == retS: return "square"
    if best == retT: return "triangle"
    return "triangle"


def shapeDetection(colour, camera, p=False):
    ''' detect a shape given the colour and which camera we are using
    Parameters:
        colour (string): either red or green
        camera (int):    either 1 or 2

    Returns:
        string:         the shape we detected

    NOTE: cntG and cntR are global variables that will need to be addressed
          Look at the 3 callback functions below to see how the variable is
          created
    '''
    global cntG, cntR1, cntR2
    maxCount = 10
    loop = 1000
    count = 0
    while count < maxCount:
        results = {"triangle":0, "square":0, "circle":0}
        for _ in range(loop):
            if camera == 1:
                (cntT, cntS, cntC) = initC1Images()
                cntR = cntR1
            elif camera == 2:
                (cntT, cntS, cntC) = initC2Images()
                cntR = cntR2
            if colour == "green": result = compareImages(cntT, cntS, cntC, cntG)
            elif colour == "red": result = compareImages(cntT, cntS, cntC, cntR)
            results[result] += 1
        if p: print("results:\n{}".format(results))
        confidence = max(results.values())
        if confidence > 900:
            if confidence == results["circle"]: return "circle"
            if confidence == results["square"]: return "square"
            if confidence == results["triangle"]: return "triangle"
        count += 1
    return "No Shape found"


def logitechRed_callback(msg):
    # print("callback")
    global bridge, red_mask, h, w, d, cntR2
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
    blur[0:h, 9*w/10*w:w] = 0
    thresh = cv2.threshold(blur, 250, 255, 0)[1]
    image2, contours, hierarchy = cv2.findContours(thresh, 2, 1)
    if len(contours) > 0:
        cntR2 = contours[0]
    # cv2.imshow("red window",blur)
    # cv2.waitKey(3)
    return

def cam1red_callback(msg):
    # print("callback")
    global bridge, red_mask, h, w, d, cntR1
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
    # cv2.imshow("red window",blur)
    # cv2.waitKey(3)
    return

def cam1green_callback(msg):
    global bridge, green_mask, cntG
    image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    h, w, d = image.shape
    upper_green = numpy.array([149, 255, 255])
    lower_green = numpy.array([57, 80, 0])
    green_mask = cv2.inRange(hsv, lower_green, upper_green)
    blur = cv2.medianBlur(green_mask, 7)
    blur[0:h / 4, 0:w] = 0
    blur[3 * h / 4:h, 0:w] = 0
    green_mask = blur
    thresh = cv2.threshold(blur, 250, 255, 0)[1]
    image2, contours, hierarchy = cv2.findContours(thresh, 2, 1)
    # cntG = contours[0]
    if len(contours) > 0:
        cntG = contours[0]
    return

def count_objects(mask, threshold=1000, canvas=None):
    """Count the number of distinct objects in the boolean image."""
    _, contours, _ = cv2.findContours(mask, 1, 2)
    moments = [cv2.moments(cont) for cont in contours]
    big_moments = [m for m in moments if m["m00"] > threshold]
    if canvas is not None:
        for moment in big_moments:
            cx = int(moment["m10"] / moment["m00"])
            cy = int(moment["m01"] / moment["m00"])
            cv2.circle(canvas, (cx, cy), 20, (0, 0, 255), -1)
    return len(big_moments)


def main():
    rospy.init_node('attempt')

    # image_sub = rospy.Subscriber('cv_camera/image_raw',
                                      # Image, logitechRed_callback)
    image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                      Image, cam1green_callback)

    # image_sub = rospy.Subscriber('usb_cam/image_raw',
                                      # Image, logitechRed_callback)
    colour = "green"
    camera = 1
    x = shapeDetection(colour, camera, p=True)
    print(x)


if __name__ == '__main__':
    main()
