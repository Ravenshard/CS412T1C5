import signal
import rospy
import smach
import smach_ros
import math
from math import tanh
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import numpy as np
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Joy
from kobuki_msgs.msg import Sound
import time
import sys
sys.path.insert(1, '/home/malcolm/Documents/CMPUT_412/Competition/CS412T1C4/shapeTesting')
import v2



global shutdown_requested

class RotateLeft(smach.State):

    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['done1', 'count'])
        self.callbacks = callbacks
        self.twist = Twist()
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)

    def execute(self, userdata):
        global shutdown_requested
        while not shutdown_requested:

            target_heading = (self.callbacks.bot_odom_heading + 90) % 360

            turning = True
            previous_difference = None
            angularSpeed = 0.4
            while turning:
                if shutdown_requested: # Basically ctrl+C
                    return 'done1'
                difference = minimum_angle_between_headings(target_heading, self.callbacks.bot_odom_heading)

                if previous_difference is None:
                    self.twist.angular.z = angularSpeed
                    self.cmd_vel_pub.publish(self.twist)
                else:
                    if difference < 1:
                        turning = False
                        self.twist.angular.z = 0
                        self.cmd_vel_pub.publish(self.twist)
                    else:
                        self.twist.angular.z = angularSpeed
                        self.cmd_vel_pub.publish(self.twist)

                if previous_difference != difference:
                    previous_difference = difference
            return 'count'
        return 'done1'


class Count(smach.State):

    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['done1', 'rotate_right'])
        self.callbacks = callbacks
        self.twist = Twist()
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.sound_pub = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size=1)

    def execute(self, userdata):
        global shutdown_requested
        while not shutdown_requested:
            symbol_red_mask = self.callbacks.symbol_red_mask.copy()
            symbol_red_mask[0:self.callbacks.main_h / 4, 0:self.callbacks.main_w] = 0
            count = 0
            loopTotal = 10
            for i in range(loopTotal):
                count += v2.count_objects(symbol_red_mask)
            real_count = math.ceil(count/loopTotal)
            print(real_count)
            for i in range(int(real_count)):
                self.sound_pub.publish(1)
                time.sleep(1)
            return 'rotate_right'
        return 'done1'


class RotateRight(smach.State):

    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['done1', 'success1'])
        self.callbacks = callbacks
        self.twist = Twist()
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)

    def execute(self, userdata):
        global shutdown_requested
        while not shutdown_requested:

            target_heading = self.callbacks.bot_odom_heading - 90
            if target_heading < 0:
                target_heading = target_heading + 360

            turning = True
            previous_difference = None
            angularSpeed = -0.4
            while turning:
                if shutdown_requested:
                    return 'done1'
                difference = minimum_angle_between_headings(target_heading, self.callbacks.bot_odom_heading)

                if previous_difference is None:
                    self.twist.angular.z = angularSpeed
                    self.cmd_vel_pub.publish(self.twist)
                else:
                    if difference < 1:
                        turning = False
                        self.twist.angular.z = 0
                        self.cmd_vel_pub.publish(self.twist)
                    else:
                        self.twist.angular.z = angularSpeed
                        self.cmd_vel_pub.publish(self.twist)

                if previous_difference != difference:
                    previous_difference = difference
            return 'success1'
        return 'done1'


def minimum_angle_between_headings(a, b):
    ''' return the difference in angle between where we are and where we want to be
    Parameters:
        a (int):    the target heading of where we want to be
        b (int):    the current heading of where we are
    Returns:
        heading_difference (int):   the difference in angle between where we are
                                    and where we want to be
    '''
    heading_difference = a - b
    if heading_difference < 0:
        heading_difference += 360
    if heading_difference > 180:
        heading_difference = b - a
        if heading_difference < 0:
            heading_difference += 360
    return heading_difference


def get_state_machine(callbacks):
    sm_event_1 = smach.StateMachine(outcomes=['DONE1', 'SUCCESS1'])
    with sm_event_1:
        smach.StateMachine.add('ROTATE_LEFT', RotateLeft(callbacks),
                               transitions={'done1': 'DONE1', 'count': 'COUNT'})
        smach.StateMachine.add('COUNT', Count(callbacks),
                               transitions={'done1': 'DONE1', 'rotate_right': 'ROTATE_RIGHT'})
        smach.StateMachine.add('ROTATE_RIGHT', RotateRight(callbacks),
                               transitions={'done1': 'DONE1', 'success1': 'SUCCESS1'})
    return sm_event_1
