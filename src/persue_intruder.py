#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
This script makes MiRo patrol and pursue a red-collared intruder robot using HSV-based red detection.
Inspired by the modular state machine structure of kick_blue_ball.py
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import TwistStamped
from cv_bridge import CvBridge

# State Constants
STATE_DETECT = 0
STATE_LOCK = 1
STATE_CHASE = 2
STATE_PATROL = 3

class MiRoPatrol:
    TICK = 0.1  # seconds
    CAM_FREQ = 5
    LINEAR_SPEED = 0.05
    ANGULAR_GAIN = 0.002
    DEBUG = False

    def __init__(self):
        rospy.init_node("persue_intruder")

        self.state = STATE_PATROL
        self.bridge = CvBridge()
        self.image = None
        self.cmd_pub = rospy.Publisher("/miro/rob02/platform/control/cmd_vel", TwistStamped, queue_size=1)
        rospy.Subscriber("/miro/rob02/camera/primary/compressed", CompressedImage, self.callback_image)

        # Predefined patrol pattern (list of (linear, angular) commands)
        self.patrol_commands = [(0.05, 0.0)] * 20 + [(0.0, 0.5)] * 10 + [(0.05, 0.0)] * 20 + [(0.0, -0.5)] * 10
        self.patrol_index = 0

    def callback_image(self, msg):
        try:
            self.image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logwarn("Could not convert image: %s", e)

    def detect_red_object(self):
        if self.image is None:
            return None

        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        moments = cv2.moments(mask)
        if moments['m00'] > 0:
            cx = int(moments['m10'] / moments['m00'])
            return cx, mask
        return None, mask

    def tick(self):
        result = self.detect_red_object()
        if result:
            cx, _ = result
            width = self.image.shape[1]
            error = cx - width // 2

            if self.state != STATE_CHASE:
                rospy.loginfo("Red detected. Locking on...")
                self.state = STATE_LOCK

            if self.state == STATE_LOCK:
                if abs(error) < 30:
                    rospy.loginfo("Target centered. Chasing...")
                    self.state = STATE_CHASE
                self.publish_movement(0.0, -self.ANGULAR_GAIN * error)

            elif self.state == STATE_CHASE:
                self.publish_movement(self.LINEAR_SPEED, -self.ANGULAR_GAIN * error)
        else:
            if self.state != STATE_PATROL:
                rospy.loginfo("No red detected. Resuming patrol.")
                self.state = STATE_PATROL
            self.patrol()

    def patrol(self):
        if self.patrol_index >= len(self.patrol_commands):
            self.patrol_index = 0  # loop
        lin, ang = self.patrol_commands[self.patrol_index]
        self.publish_movement(lin, ang)
        self.patrol_index += 1

    def publish_movement(self, linear, angular):
        msg = TwistStamped()
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        self.cmd_pub.publish(msg)

    def run(self):
        rate = rospy.Rate(1 / self.TICK)
        while not rospy.is_shutdown():
            self.tick()
            rate.sleep()

if __name__ == '__main__':
    try:
        node = MiRoPatrol()
        node.run()
    except rospy.ROSInterruptException:
        pass
