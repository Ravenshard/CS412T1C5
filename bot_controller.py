#!/usr/bin/env python

import signal
import rospy
import smach
import smach_ros
import math
from math import tanh
from geometry_msgs.msg import Twist
import numpy as np
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseWithCovarianceStamped
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import time

global shutdown_requested
global linear_velocity
global rotation_velocity

"""
pose: 
    position: 
      x: 3.18032880923
      y: 2.36354110942
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: 0.86541182624
      w: 0.501061244763

"""


class Control(smach.State):

    def __init__(self, linear_velocity_multiplier=1.0, rotation_velocity_multiplier=1.0):
        smach.State.__init__(self, outcomes=['done'])

        self.linear_velocity_multiplier = linear_velocity_multiplier
        self.rotation_velocity_multiplier = rotation_velocity_multiplier

        # Publish movement commands to the turtlebot's base
        self.cmd_vel_pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=1)
        self.initial_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)  # <3>
        #self.client.wait_for_server()

        initial = PoseWithCovarianceStamped()
        initial.pose.pose.position.x = -1.3
        initial.pose.pose.position.y = -0
        initial.pose.pose.position.z = 0.0
        initial.pose.pose.orientation.x = 0.0
        initial.pose.pose.orientation.y = 0.0
        initial.pose.pose.orientation.z = -0.372922294621
        initial.pose.pose.orientation.w = 0.927862577203
        print("waiting")
        time.sleep(1)
        self.initial_pub.publish(initial)
        print("initial pose sent")

        # Create a Twist message, and fill in the fields.  We're also
        # going to fill in the linear.x field, even though we think
        # we're going to set it later, just to be sure we know its
        # value.
        self.command = Twist()
        self.command.linear.x = 0.0
        self.command.linear.y = 0.0
        self.command.linear.z = 0.0
        self.command.angular.x = 0.0
        self.command.angular.y = 0.0
        self.command.angular.z = 0.0
        self.hit = 0
        return

    def execute(self, userdata):
        global shutdown_requested
        global linear_velocity
        global rotation_velocity

        test = MoveBaseGoal()
        test.target_pose.header.frame_id = "map"
        test.target_pose.header.stamp = rospy.Time.now()
        test.target_pose.pose.position.x = 0
        test.target_pose.pose.position.y = 0
        test.target_pose.pose.orientation.z = -0.372922294621
        test.target_pose.pose.orientation.w = 0.927862577203
        #self.goal_pub.publish(test)
        #self.client.send_goal(test)
        #self.client.wait_for_result()
        # print("Goal sent")

        while not shutdown_requested:
            self.command.linear.x = linear_velocity * self.linear_velocity_multiplier
            self.command.angular.z = rotation_velocity * self.rotation_velocity_multiplier
            self.cmd_vel_pub.publish(self.command)
            #self.client.send_goal(test)
            #self.client.wait_for_result()
            # self.initial_pub.publish(initial)


        self.command.linear.x = 0
        self.command.angular.z = 0
        self.cmd_vel_pub.publish(self.command)
        return 'done'


def controller_callback(event):
    global linear_velocity
    global rotation_velocity

    if event.buttons[8] == 1:
        global shutdown_requested
        shutdown_requested = True

    linear_velocity = event.axes[3]
    rotation_velocity = event.axes[2]


def request_shutdown(sig, frame):
    global shutdown_requested
    shutdown_requested = True


def main():
    global shutdown_requested
    global linear_velocity
    global rotation_velocity

    shutdown_requested = False
    linear_velocity = 0
    rotation_velocity = 0

    rospy.init_node('cop_bot')
    controller_sub = rospy.Subscriber('joy', Joy, controller_callback)

    # Create done outcome which will stop the state machine
    sm_turtle = smach.StateMachine(outcomes=['DONE'])

    with sm_turtle:
        smach.StateMachine.add('CONTROL', Control(linear_velocity_multiplier=0.5),
                               transitions={'done': 'DONE'})

    # Create and start the instrospection server - needed for smach_viewer
    sis = smach_ros.IntrospectionServer('TRAVELLER_server', sm_turtle, 'FOLLOWER')
    sis.start()

    # Start state machine and run until SIGINT received
    signal.signal(signal.SIGINT, request_shutdown)
    sm_turtle.execute()

    # Stop server
    sis.stop()


if __name__ == '__main__':
    main()