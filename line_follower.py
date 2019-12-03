#!/usr/bin/env python

import cv2
import cv_bridge
import numpy
import time
import event_one
import event_two
import event_three
import event_four
import signal
import rospy
import smach
import smach_ros
import math
import trig
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from sensor_msgs.msg import Joy
from tf.transformations import euler_from_quaternion
from ar_track_alvar_msgs.msg import AlvarMarkers
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import Led
from kobuki_msgs.msg import Sound
from actionlib_msgs.msg import GoalStatusArray
from kobuki_msgs.msg import BumperEvent
import sys
sys.path.insert(1, '/home/malcolm/Documents/CMPUT_412/Competition/CS412T1C4/shapeTesting')
import v2

global shutdown_requested
global red_count


class Wait(smach.State):

    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['follow_line', 'done', 'event_four'])
        self.callbacks = callbacks
        self.led1_pub = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=1)
        self.led2_pub = rospy.Publisher('/mobile_base/commands/led2', Led, queue_size=1)

    def execute(self, userdata):
        global shutdown_requested
        self.led1_pub.publish(0)
        self.led2_pub.publish(0)
        return 'follow_line'  # TODO: take out debug line
        while not shutdown_requested:
            if self.callbacks.stopWaiting:
                return 'follow_line'
            time.sleep(1)
        return 'done'


class End(smach.State):

    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['done'])
        self.callbacks = callbacks
        self.led1_pub = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=1)
        self.led2_pub = rospy.Publisher('/mobile_base/commands/led2', Led, queue_size=1)
        self.sound_pub = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size=1)

    def execute(self, userdata):
        self.led1_pub.publish(1)  # green
        self.led2_pub.publish(1)  # green
        self.sound_pub.publish(1)

        time.sleep(5)

        self.led1_pub.publish(0)
        self.led2_pub.publish(0)

        return 'done'


class Stop(smach.State):
    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['follow_line', 'done', 'event_one', 'event_two', 'event_three', 'event_four', 'end'])
        self.callbacks = callbacks
        self.twist = Twist()
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.prev_error = None
        self.Kp = 1.0 / 50.0
        self.Ki = 1.0 / 50.0
        self.Kd = 1.0 / 50.0
        self.speed = 0.8

    def execute(self, userdata):
        global button_start
        global shutdown_requested
        global red_count

        red_count = (red_count + 1)

        red_events = [1, 3]
        red_stops = [0, 2, 4, 5, 6]
        red_end = [6]
        if red_count in red_stops:
            distance = 0.4
        else:
            distance = 0.6

        while self.callbacks.bot_odom_position is None:
            time.sleep(1)

        sp = self.callbacks.bot_odom_position
        ep = sp

        while math.sqrt((sp.x - ep.x) ** 2 + (sp.y - ep.y) ** 2) < distance:
            if shutdown_requested:
                return 'done'
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

        if red_count in red_events:
            if red_count == 1:
                #return 'follow_line'
                return 'event_one'
            elif red_count == 3:
                #return 'follow_line'
                return 'event_two'

        if red_count in red_end:
            return 'end'

        start = time.time()
        while time.time() - start < 5:
            if shutdown_requested:
                return 'done'

        if red_count == 5:
            # return 'follow_line'
            return 'event_three'

        if red_count == 4:
            # return 'follow_line'
            return 'event_four'

        return 'follow_line'


class FollowLine(smach.State):

    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['stop', 'done'])
        self.callbacks = callbacks
        self.prev_error = None
        self.Kp = 1.0 / 50.0
        self.Ki = 1.0 / 50.0
        self.Kd = 1.0 / 50.0
        self.speed = 0.8
        self.twist = Twist()
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)

    def execute(self, userdata):
        global button_start
        global shutdown_requested
        while not shutdown_requested:
            if self.callbacks.line_white_mask is not None and self.callbacks.red_mask is not None:

                bottom_white_mask = self.callbacks.line_white_mask.copy()
                bottom_red_mask = self.callbacks.red_mask.copy()

                # Check if a long red strip has been detected
                h = self.callbacks.secondary_h
                w = self.callbacks.secondary_w
                search_top = 3 * h / 4
                search_bot = h
                bottom_white_mask[0:search_top, 0:w] = 0
                bottom_white_mask[search_bot:h, 0:w] = 0
                bottom_red_mask[h*1/2:h, 0:w] = 0
                red_pixel_count = cv2.sumElems(bottom_red_mask)[0] / 255

                # Check if a half red strip, on the left, has been detected
                left_red_mask = bottom_red_mask.copy()
                right_red_mask = bottom_red_mask.copy()
                left_red_mask[0:h, w / 2: w] = 0
                right_red_mask[0:h, 0: w / 2] = 0

                RM = cv2.moments(bottom_red_mask)
                if RM['m00'] > 0:
                    ry = int(RM['m01'] / RM['m00'])
                    print(str(red_pixel_count)+" "+str(ry))

                    # Check special case of
                    if red_pixel_count > 1000 and ry > 100 and red_count == 3:
                        print(red_pixel_count)
                        print(ry)
                        print("Red count 3 line found")
                        return 'stop'

                    if red_pixel_count > 1000 and ry > 190 and red_count == 4:
                        print(red_pixel_count)
                        print(ry)
                        print("Red count 4 line found")
                        return 'stop'

                    if red_pixel_count > 1000 and ry > 200:
                        print(red_pixel_count)
                        print(ry)
                        print("Red line found")
                        return 'stop'


                # If there is no significant red line, follow white line
                WM = cv2.moments(bottom_white_mask)

                if WM['m00'] > 0:
                    cx = int(WM['m10'] / WM['m00'])
                    cy = int(WM['m01'] / WM['m00'])

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
                    print("test")
                    self.cmd_vel_pub.publish(self.twist)
        return 'done'


def request_shutdown(sig, frame):
    global shutdown_requested
    event_one.shutdown_requested = True
    event_two.shutdown_requested = True
    event_three.shutdown_requested = True
    event_four.shutdown_requested = True
    shutdown_requested = True


class Callbacks:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.Kp = 1.0/200.0
        self.Ki = 1.0/200.0
        self.Kd = 1.0/200.0
        self.prev_error = None
        self.past_error = []

        self.twist = Twist()

        self.red_mask = None  # Gotten with secondary camera
        self.line_white_mask = None  # Gotten with secondary camera
        self.symbol_red_mask = None  # Gotten with Main camera
        self.symbol_green_mask = None  # Gotten with Main

        self.main_h = None
        self.main_w = None
        self.main_d = None

        self.secondary_h = None
        self.secondary_w = None
        self.secondary_d = None

        self.bot_odom_position = None
        self.bot_odom_heading = None

        self.bot_map_pose = None
        self.bot_map_position = None
        self.bot_map_heading = None

        self.box_marker_id = 2
        self.box_target_marker_id = 20
        self.box_position = None
        self.box_target_position = None

        self.stopWaiting = False

        self.left_bumper_pressed = False
        self.middle_bumper_pressed = False
        self.right_bumper_pressed = False

        self.move_base_status = None

    def odometry_callback(self, msg):
        self.bot_odom_position = msg.pose.pose.position
        yaw = euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])[2]
        self.bot_odom_heading = (yaw + math.pi) * (180 / math.pi)
        return

    def controller_callback(self, msg):
        if msg.buttons[7] == 1:
            self.stopWaiting = True

    def marker_pose_callback(self, msg):
        if len(msg.markers) > 0:
            for marker in msg.markers:
                if marker.id == self.box_marker_id:
                    self.box_position = marker.pose.pose.position
                elif marker.id == self.box_target_marker_id:
                    yaw = euler_from_quaternion([0.0, 0.0, 0.0889745806541, 0.996033897012])[2]
                    heading = (yaw + math.pi) * (180 / math.pi)
                    self.box_target_position = trig.get_point(marker.pose.pose.position, 0.30, heading)

    def bot_map_pose_callback(self, msg):
        self.bot_map_pose = msg
        self.bot_map_position = msg.pose.pose.position
        yaw = euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])[2]
        self.bot_map_heading = (yaw + math.pi) * (180 / math.pi)

    def bumper_callback(self, msg):
        # bumper = 0 = Left
        # bumper = 1 = Middle
        # bumper = 2 = Right
        if msg.bumper == 0:
            self.left_bumper_pressed = bool(msg.state)
        elif msg.bumper == 1:
            self.middle_bumper_pressed = bool(msg.state)
        else:
            self.right_bumper_pressed = bool(msg.state)

    def move_base_status_callback(self, msg):
        # 1 == active
        # 3 == success
        # 4 == aborted
        if len(msg.status_list) > 0:
            self.move_base_status = msg.status_list[0].status

    def main_image_callback(self, msg):
        v2.cam1green_callback(msg)  # For shape detection
        v2.cam1red_callback(msg)  # For shape detection
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        self.main_h, self.main_w, self.main_d = image.shape

        upper_red_a = numpy.array([20, 255, 255])
        lower_red_a = numpy.array([0, 200, 60])
        red_mask_a = cv2.inRange(hsv, lower_red_a, upper_red_a)

        upper_red_b = numpy.array([255, 255, 255])
        lower_red_b = numpy.array([150, 200, 60])
        red_mask_b = cv2.inRange(hsv, lower_red_b, upper_red_b)

        self.symbol_red_mask = cv2.bitwise_or(red_mask_a, red_mask_b)

        upper_green = numpy.array([136, 255, 255])
        lower_green = numpy.array([56, 43, 90])
        self.symbol_green_mask = cv2.inRange(hsv, lower_green, upper_green)

        # TESTING STUFF BELOW
        '''
        h = self.h
        w = self.w
        symbol_red_mask = self.symbol_red_mask.copy()
        symbol_red_mask[0:h/4, 0:w] = 0
        symbol_red_mask[3*h/4:h, 0:w] = 0

        symbol_green_mask = self.symbol_green_mask.copy()
        symbol_green_mask[0:h / 4, 0:w] = 0
        symbol_green_mask[3 * h / 4:h, 0:w] = 0

        #shapes = detect_shape.detect_shape(symbol_green_mask)[0]

        cv2.imshow("red symbol window", self.red_mask)
        cv2.imshow("green symbol window", self.line_white_mask)
        '''
        cv2.waitKey(3)

    def secondary_image_callback(self, msg):
        v2.logitechRed_callback(msg) # For shape detection =
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        self.secondary_h, self.secondary_w, self.secondary_d = image.shape

        #upper_white = numpy.array([360, 20, 255])
        #lower_white = numpy.array([0, 0, 250])
        upper_white = numpy.array([360, 50, 255])
        lower_white = numpy.array([0, 0, 200])
        self.line_white_mask = cv2.inRange(hsv, lower_white, upper_white)

        upper_red_a = numpy.array([20, 255, 255])
        lower_red_a = numpy.array([0, 100, 100])
        red_mask_a = cv2.inRange(hsv, lower_red_a, upper_red_a)

        upper_red_b = numpy.array([255, 255, 255])
        lower_red_b = numpy.array([150, 100, 100])
        red_mask_b = cv2.inRange(hsv, lower_red_b, upper_red_b)
        self.red_mask = cv2.bitwise_or(red_mask_a, red_mask_b)

        # TESTING STUFF BELOW
        '''
        bottom_white_mask = self.line_white_mask.copy()
        bottom_red_mask = self.red_mask.copy()
        h = self.h
        w = self.w
        search_top = 3 * h / 4
        search_bot = h
        bottom_white_mask[0:search_top, 0:w] = 0
        bottom_white_mask[search_bot:h, 0:w] = 0
        bottom_red_mask[0:search_top, 0:w] = 0
        bottom_red_mask[search_bot:h, 0:w] = 0
        red_pixel_count = cv2.sumElems(bottom_red_mask)[0] / 255
        white_pixel_count = cv2.sumElems(bottom_white_mask)[0] / 255

        # self.line_white_mask = white_mask
        # self.red_mask = red_mask
        '''
        bottom_red_mask = self.red_mask.copy()
        h = self.secondary_h
        w = self.secondary_w
        search_top = 3 * h / 4
        search_bot = h
        bottom_red_mask[h*1/2:h, 0:w] = 0
        #bottom_red_mask[search_bot:h, 0:w] = 0

        # print(cv2.sumElems(red_mask)[0] / 255)
        #cv2.imshow("red window", bottom_red_mask)
        #cv2.imshow("white window", self.line_white_mask)
        cv2.waitKey(3)


def main():
    global button_start
    global shutdown_requested
    global red_count

    # TESTING STUFF BELOW
    event_two.previous_shape = "triangle"

    #red_count = 0
    #red_count = 1
    red_count = 2  # Event 2
    #red_count = 3  # Event four
    #red_count = 4  # Event 3
    #red_count = 6  # Last stop

    button_start = False

    shutdown_requested = False
    event_one.shutdown_requested = False
    event_two.shutdown_requested = False
    event_three.shutdown_requested = False
    event_four.shutdown_requested = False

    rospy.init_node('line_follow_bot')

    callbacks = Callbacks()
    rospy.Subscriber('/move_base/status', GoalStatusArray, callbacks.move_base_status_callback, queue_size=1)
    rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, callbacks.bumper_callback, queue_size=1)
    rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, callbacks.bot_map_pose_callback)
    rospy.Subscriber('/usb_cam/image_raw', Image, callbacks.secondary_image_callback)
    rospy.Subscriber('camera/rgb/image_raw', Image, callbacks.main_image_callback)
    rospy.Subscriber('ar_pose_marker', AlvarMarkers, callbacks.marker_pose_callback)
    rospy.Subscriber("odom", Odometry, callbacks.odometry_callback)
    rospy.Subscriber('joy', Joy, callbacks.controller_callback)

    # Create done outcome which will stop the state machine
    sm_turtle = smach.StateMachine(outcomes=['DONE'])

    with sm_turtle:
        smach.StateMachine.add('WAIT', Wait(callbacks),
                               transitions={'follow_line': 'FOLLOW_LINE', 'done': 'DONE', 'event_four': 'EVENT_FOUR'})
        smach.StateMachine.add('FOLLOW_LINE', FollowLine(callbacks),
                               transitions={'stop': 'STOP', 'done': 'DONE'})
        smach.StateMachine.add('STOP', Stop(callbacks),
                               transitions={'follow_line': 'FOLLOW_LINE', 'done': 'DONE',
                                            'event_one': 'EVENT_ONE',
                                            'event_two': 'EVENT_TWO',
                                            'event_three': 'EVENT_THREE',
                                            'event_four': 'EVENT_FOUR',
                                            'end': 'END'})
        smach.StateMachine.add('END', End(callbacks), transitions={'done': 'DONE'})

        sm_event_1 = event_one.get_state_machine(callbacks)
        smach.StateMachine.add('EVENT_ONE', sm_event_1,
                               transitions={'DONE1': 'DONE', 'SUCCESS1': 'FOLLOW_LINE'})

        sm_event_2 = event_two.get_state_machine(callbacks)
        smach.StateMachine.add('EVENT_TWO', sm_event_2,
                               transitions={'DONE2': 'DONE', 'SUCCESS2': 'FOLLOW_LINE'})

        sm_event_3 = event_three.get_state_machine(callbacks)
        smach.StateMachine.add('EVENT_THREE', sm_event_3,
                               transitions={'DONE3': 'DONE', 'SUCCESS3': 'FOLLOW_LINE'})

        sm_event_4 = event_four.get_state_machine(callbacks)
        smach.StateMachine.add('EVENT_FOUR', sm_event_4,
                               transitions={'DONE4': 'DONE', 'SUCCESS4': 'FOLLOW_LINE'})



    # Create and start the instrospection server - needed for smach_viewer
    sis = smach_ros.IntrospectionServer('TRAVELLER_server', sm_turtle, 'STATEMACHINE')
    sis.start()

    # Start state machine and run until SIGINT received
    signal.signal(signal.SIGINT, request_shutdown)
    sm_turtle.execute()

    # Stop server
    sis.stop()


if __name__ == '__main__':
    main()
