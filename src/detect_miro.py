#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
This script makes MiRo spin around, detect another MiRo using YOLOv8, and move toward it.
"""

# Imports
import os
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import TwistStamped
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO  # YOLOv8 library

try:  # For convenience, import this util separately
    from miro2.lib import wheel_speed2cmd_vel  # Python 3
except ImportError:
    from miro2.utils import wheel_speed2cmd_vel  # Python 2

class MiRoYOLOClient:
    def __init__(self):
        # Initialise ROS node
        rospy.init_node("miro_yolo_detect", anonymous=True)

        # YOLO model path
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.model_path = os.path.join(script_dir, "yolo_model/best.pt")
        self.model = YOLO(self.model_path)

        # Initialise CV Bridge
        self.image_converter = CvBridge()

        # ROS topic base name
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME", "miro")

        # Camera subscribers
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

        # Velocity publisher
        self.vel_pub = rospy.Publisher(
            topic_base_name + "/control/cmd_vel", TwistStamped, queue_size=0
        )

        # Variables
        self.input_camera_left = None
        self.input_camera_right = None
        self.new_frame_left = False
        self.new_frame_right = False
        self.detected_miro = False
        self.miro_position_left = None
        self.miro_position_right = None

    def callback_caml(self, ros_image):
        """
        Callback function for the left camera.
        """
        try:
            # Convert ROS image to OpenCV format
            image = self.image_converter.compressed_imgmsg_to_cv2(ros_image, "rgb8")
            self.input_camera_left = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            self.new_frame_left = True
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {e}")

    def callback_camr(self, ros_image):
        """
        Callback function for the right camera.
        """
        try:
            # Convert ROS image to OpenCV format
            image = self.image_converter.compressed_imgmsg_to_cv2(ros_image, "rgb8")
            self.input_camera_right = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            self.new_frame_right = True
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {e}")
            
    def drive(self, speed_l=0.1, speed_r=0.1):  # (m/sec, m/sec)
            """
            Wrapper to simplify driving MiRo by converting wheel speeds to cmd_vel
            """
            # Prepare an empty velocity command message
            msg_cmd_vel = TwistStamped()

            # Desired wheel speed (m/sec)
            wheel_speed = [speed_l, speed_r]

            # Convert wheel speed to command velocity (m/sec, Rad/sec)
            (dr, dtheta) = wheel_speed2cmd_vel(wheel_speed)

            # Update the message with the desired speed
            msg_cmd_vel.twist.linear.x = dr
            msg_cmd_vel.twist.angular.z = dtheta

            # Publish message to control/cmd_vel topic
            self.vel_pub.publish(msg_cmd_vel)

    def detect_miro(self, input_camera):
        """
        Use YOLOv8 to detect another MiRo in the camera frame.
        """
        if input_camera is None:
            return None

        # Run YOLOv8 inference
        results = self.model(input_camera)

        # Parse results
        for result in results[0].boxes:
            if int(result.cls) == 0:  # Assuming class 0 is "MiRo"
                # Get bounding box center
                detected_miro = result.xyxy[0]
                x_center = (detected_miro[0] + detected_miro[2]) / 2
                y_center = (detected_miro[1] + detected_miro[3]) / 2
                return (x_center, y_center)

        return None
    
    def micro_adjust(self, left_x, right_x, width, base_speed=0.3, adjustment=0.1):
        """
        Perform micro adjustments to keep MiRo aligned with the detected target.
        """
        frame_half = width / 2

        if left_x < frame_half:
            self.drive(speed_l=base_speed, speed_r=base_speed+adjustment)
        elif right_x > frame_half:
            self.drive(speed_l=base_speed+adjustment, speed_r=base_speed)  
        else:
            # Move forward if aligned
            self.drive(speed_l=base_speed, speed_r=base_speed)

    def loop(self):
        """
        Main control loop.
        """
        rospy.loginfo("MiRo is searching for another MiRo...")
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            if self.new_frame_left:
                # Detect MiRo in the left camera frame
                self.miro_position_left = self.detect_miro(self.input_camera_left)
                self.new_frame_left = False

            if self.new_frame_right:
                # Detect MiRo in the right camera frame
                self.miro_position_right = self.detect_miro(self.input_camera_right)
                self.new_frame_right = False
            
            if self.miro_position_left or self.miro_position_right:
                # Move forward if MiRo is detected in either frame
                # If MiRo is not detected in the left frame, turn anticlockwise
                if not self.miro_position_left:
                    self.drive(speed_l=0.1, speed_r=-0.1)  # Anticlockwise rotation
                # If MiRo is not detected in the right frame, turn clockwise
                elif not self.miro_position_right:
                    self.drive(speed_l=-0.1, speed_r=0.1)  # Clockwise rotation
                else:
                    self.drive(speed_l=0.4, speed_r=0.4)  # Move forward
                    # Perform micro adjustments if MiRo is detected in both cameras
                    left_x, _ = self.miro_position_left
                    right_x, _ = self.miro_position_right
                    self.micro_adjust(left_x, right_x, width=self.input_camera_left.shape[1])
            else:
                self.drive(speed_l=-0.1, speed_r=0.1)  # Rotate in place
            

            rate.sleep()


if __name__ == "__main__":
    try:
        client = MiRoYOLOClient()
        client.loop()
    except rospy.ROSInterruptException:
        pass