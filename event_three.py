import signal
import rospy
import smach
import smach_ros
import math
import time
import cv2
from math import tanh
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import numpy as np
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Joy
from kobuki_msgs.msg import Sound
import event_two
import traceback

import sys
sys.path.insert(1, '/home/malcolm/Documents/CMPUT_412/Competition/CS412T1C4/shapeTesting')
import v2

global shutdown_requested
global number_of_checks


class OdomFollow(smach.State):

    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['done3', 'rotate_left'])
        self.callbacks = callbacks
        self.twist = Twist()
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.prev_error = None
        self.Kp = 1.0 / 200.0
        self.Ki = 1.0 / 200.0
        self.Kd = 1.0 / 200.0
        self.speed = 0.6

    def execute(self, userdata):
        global shutdown_requested
        global number_of_checks

        if number_of_checks == 0:
            distance = 0.8
        else:
            distance = 0.3

        while self.callbacks.bot_odom_position is None:
            time.sleep(1)

        sp = self.callbacks.bot_odom_position
        ep = sp

        while math.sqrt((sp.x - ep.x) ** 2 + (sp.y - ep.y) ** 2) < distance:
            if shutdown_requested:
                return 'done3'
            h = self.callbacks.secondary_h
            w = self.callbacks.secondary_w
            search_top = 3 * h / 4
            search_bot = h
            bottom_white_mask = self.callbacks.line_white_mask.copy()
            bottom_white_mask[0:search_top, 0:w] = 0
            bottom_white_mask[search_bot:h, 0:w] = 0

            M = cv2.moments(bottom_white_mask)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                # BEGIN CONTROL
                if self.prev_error is None:
                    error = cx - w / 2
                    rotation = -(self.Kp * float(error))
                    self.prev_error = error
                else:
                    error = cx - w / 2
                    rotation = -(self.Kp * float(error) + self.Kd * (error - self.prev_error))
                    self.prev_error = error
                self.twist.linear.x = self.speed
                self.twist.angular.z = rotation
                self.cmd_vel_pub.publish(self.twist)
                # END CONTROL
                ep = self.callbacks.bot_odom_position

        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.cmd_vel_pub.publish(self.twist)

        return 'rotate_left'


class RotateLeft(smach.State):
    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['done3', 'check'])
        self.callbacks = callbacks
        self.twist = Twist()
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)

    def execute(self, userdata):
        global shutdown_requested
        while not shutdown_requested:

            target_heading = (self.callbacks.bot_odom_heading + 90) % 360

            turning = True
            previous_difference = None
            while turning:
                if shutdown_requested:
                    return 'done3'
                difference = minimum_angle_between_headings(target_heading, self.callbacks.bot_odom_heading)
                #print(difference)

                if previous_difference is None:
                    self.twist.angular.z = 0.4
                    self.cmd_vel_pub.publish(self.twist)
                else:
                    if difference < 1:
                        turning = False
                        self.twist.angular.z = 0
                        self.cmd_vel_pub.publish(self.twist)
                    else:
                        self.twist.angular.z = 0.4
                        self.cmd_vel_pub.publish(self.twist)

                if previous_difference != difference:
                    previous_difference = difference

            return 'check'

        return 'done3'


class Check(smach.State):

    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['done3', 'rotate_right'])
        self.callbacks = callbacks
        self.twist = Twist()
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.sound_pub = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size=1)

    def execute(self, userdata):
        global shutdown_requested
        global number_of_checks
        while not shutdown_requested:
            if self.callbacks.line_white_mask is not None and self.callbacks.red_mask is not None:
                number_of_checks += 1
                h = self.callbacks.main_h
                w = self.callbacks.main_w
                symbol_red_mask = self.callbacks.symbol_red_mask.copy()
                symbol_red_mask[0:h / 2, 0:w] = 0
                try:
                    shape = v2.shapeDetection('red', 2)
                    print("red shape detected:" + shape)
                except Exception as e:
                    print(e)
                    traceback.print_exc()

                if shape is None:
                    time.sleep(1)
                    return "rotate_right"

                if shape == event_two.previous_shape:
                    self.sound_pub.publish(1)

            time.sleep(1)
            return "rotate_right"
        return 'done3'


class RotateRight(smach.State):

    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['done3', 'success3', 'odom_follow'])
        self.callbacks = callbacks
        self.twist = Twist()
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)

    def execute(self, userdata):
        global shutdown_requested
        global number_of_checks
        while not shutdown_requested:

            target_heading = self.callbacks.bot_odom_heading - 90
            if target_heading < 0:
                target_heading = target_heading + 360

            turning = True
            previous_difference = None
            while turning:
                if shutdown_requested:
                    return 'done3'
                difference = minimum_angle_between_headings(target_heading, self.callbacks.bot_odom_heading)

                if previous_difference is None:
                    self.twist.angular.z = -0.4
                    self.cmd_vel_pub.publish(self.twist)
                else:
                    if difference < 0.5:
                        turning = False
                        self.twist.angular.z = 0
                        self.cmd_vel_pub.publish(self.twist)
                    else:
                        self.twist.angular.z = -0.4
                        self.cmd_vel_pub.publish(self.twist)

                if previous_difference != difference:
                    previous_difference = difference

            if number_of_checks >= 3:
                return 'success3'
            else:
                return 'odom_follow'
        return 'done3'


def minimum_angle_between_headings(a, b):
    heading_difference = a - b
    if heading_difference < 0:
        heading_difference += 360
    if heading_difference > 180:
        heading_difference = b - a
        if heading_difference < 0:
            heading_difference += 360
    return heading_difference


def get_state_machine(callbacks):
    global number_of_checks
    number_of_checks = 0

    sm_event_3 = smach.StateMachine(outcomes=['DONE3', 'SUCCESS3'])
    with sm_event_3:
        smach.StateMachine.add('ODOM_FOLLOW', OdomFollow(callbacks),
                               transitions={'done3': 'DONE3', 'rotate_left': 'ROTATE_LEFT'})
        smach.StateMachine.add('ROTATE_LEFT', RotateLeft(callbacks),
                               transitions={'done3': 'DONE3', 'check': 'CHECK'})
        smach.StateMachine.add('CHECK', Check(callbacks),
                               transitions={'done3': 'DONE3', 'rotate_right': 'ROTATE_RIGHT'})
        smach.StateMachine.add('ROTATE_RIGHT', RotateRight(callbacks),
                               transitions={'done3': 'DONE3', 'success3': 'SUCCESS3', 'odom_follow': 'ODOM_FOLLOW'})
    return sm_event_3
