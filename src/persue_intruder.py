#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
MiRo patrol robot that searches for a red-collared intruder using HSV detection.
Uses a modular state machine and MIRO_ROBOT_NAME for topic resolution.
"""

import os
import subprocess
from math import radians  # This is used to reset the head pose
import numpy as np  # Numerical Analysis library
import cv2  # Computer Vision library

import rospy  # ROS Python interface
from sensor_msgs.msg import CompressedImage  # ROS CompressedImage message
from sensor_msgs.msg import JointState  # ROS joints state message
from cv_bridge import CvBridge, CvBridgeError  # ROS -> OpenCV converter
from geometry_msgs.msg import TwistStamped  # ROS cmd_vel (velocity control) message

import miro2 as miro  # Import MiRo Developer Kit library

try:  # For convenience, import this util separately
    from miro2.lib import wheel_speed2cmd_vel  # Python 3
except ImportError:
    from miro2.utils import wheel_speed2cmd_vel  # Python 2


# State Constants
STATE_DETECT = 0
STATE_LOCK = 1
STATE_CHASE = 2
STATE_PATROL = 3

class MiRoPatrol:
    TICK = 0.1  # seconds
    LINEAR_SPEED = 0.05
    ANGULAR_GAIN = 0.002
    CENTER_TOLERANCE = 30

    def callback_caml(self, ros_image):  # Left camera
        self.callback_cam(ros_image, 0)

    def callback_camr(self, ros_image):  # Right camera
        self.callback_cam(ros_image, 1)

    def callback_cam(self, ros_image, index):
        """
        Callback function executed upon image arrival
        """
        # Silently(-ish) handle corrupted JPEG frames
        try:
            # Convert compressed ROS image to raw CV image
            image = self.image_converter.compressed_imgmsg_to_cv2(ros_image, "rgb8")
            # Convert from OpenCV's default BGR to RGB
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            # Store image as class attribute for further use
            self.input_camera[index] = image
            # Get image dimensions
            self.frame_height, self.frame_width, channels = image.shape
            self.x_centre = self.frame_width / 2.0
            self.y_centre = self.frame_height / 2.0
            # Raise the flag: A new frame is available for processing
            self.new_frame[index] = True
        except CvBridgeError as e:
            # Ignore corrupted frames
            pass

    def __init__(self):
        rospy.init_node("persue_intruder", anonymous=True)
        rospy.sleep(2.0)

        # Initialise CV Bridge
        self.image_converter = CvBridge()
        self.robot_name = "/" + os.getenv("MIRO_ROBOT_NAME", "miro02")
        
        self.sub_caml = rospy.Subscriber(
            topic_base_name + "/sensors/caml/compressed",
            CompressedImage,
            self.callback_caml,
            queue_size=1,
            tcp_nodelay=True,
        )
        self.sub_camr = rospy.Subscriber(
            topic_base_name + "/sensors/camr/compressed",
            CompressedImage,
            self.callback_camr,
            queue_size=1,
            tcp_nodelay=True,
        )
        self.cmd_pub = rospy.Publisher(cmd_topic, TwistStamped, queue_size=1)

        # Create handle to store images
        self.input_camera = [None, None]

        # Set the default frame width (gets updated on receiving an image)
        self.frame_width = 640

        self.state = STATE_PATROL
        self.image = None
        
        rospy.Subscriber(cam_topic, CompressedImage, self.callback_image)

        print(f"[INFO] Patrol node started for {self.robot_name}")
        print(f"[INFO] Subscribed to: {self.sub_caml}")
        print(f"[INFO] Subscribed to: {self.sub_camr}")
        print(f"[INFO] Publishing to: {self.cmd_pub}")

        self.patrol_pattern = [(0.05, 0.0)] * 25 + [(0.0, 0.5)] * 10 + [(0.05, 0.0)] * 25 + [(0.0, -0.5)] * 10
        self.step = 0

    def callback_image(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def tick(self):
        if self.image is None:
            print("[WAIT] Waiting for camera image...")
            return

        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        M = cv2.moments(mask)
        if M["m00"] > 10000:
            cx = int(M["m10"] / M["m00"])
            width = mask.shape[1]
            error = cx - width // 2

            if abs(error) < self.CENTER_TOLERANCE:
                self.state = STATE_CHASE
                print(f"[CHASE] Aligned to target → error: {error}")
                self.publish_movement(self.LINEAR_SPEED, 0.0)
            else:
                self.state = STATE_LOCK
                turn = -self.ANGULAR_GAIN * error
                print(f"[LOCK] Centering target → error: {error}, turn: {turn:.3f}")
                self.publish_movement(0.0, turn)
        else:
            self.state = STATE_PATROL
            lin, ang = self.patrol_pattern[self.step]
            print(f"[PATROL] No red found → step {self.step}, lin={lin}, ang={ang}")
            self.publish_movement(lin, ang)
            self.step = (self.step + 1) % len(self.patrol_pattern)

    def publish_movement(self, linear, angular):
        msg = TwistStamped()
        msg.header.stamp = rospy.Time.now()
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        self.cmd_pub.publish(msg)

if __name__ == "__main__":
    node = MiRoPatrol()
    rate = rospy.Rate(1.0 / node.TICK)
    while not rospy.is_shutdown():
        node.tick()
        rate.sleep()
