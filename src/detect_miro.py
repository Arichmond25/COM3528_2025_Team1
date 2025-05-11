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

        # Camera subscriber
        self.sub_caml = rospy.Subscriber(
            topic_base_name + "/sensors/caml/compressed",
            CompressedImage,
            self.callback_caml,
            queue_size=1,
            tcp_nodelay=True,
        )

        # Velocity publisher
        self.vel_pub = rospy.Publisher(
            topic_base_name + "/control/cmd_vel", TwistStamped, queue_size=0
        )

        # Variables
        self.input_camera = None
        self.new_frame = False
        self.detected_miro = False
        self.miro_position = None

    def callback_caml(self, ros_image):
        """
        Callback function for the left camera.
        """
        try:
            # Convert ROS image to OpenCV format
            image = self.image_converter.compressed_imgmsg_to_cv2(ros_image, "rgb8")
            self.input_camera = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            self.new_frame = True
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {e}")

    def detect_miro(self):
        """
        Use YOLOv8 to detect another MiRo in the camera frame.
        """
        if self.input_camera is None:
            return None

        # Run YOLOv8 inference
        results = self.model(self.input_camera)

        # Parse results
        for result in results[0].boxes:
            if int(result.cls) == 0:  # Assuming class 0 is "MiRo"
                # Get bounding box center
                detected_miro = result.xyxy[0]
                x_center = (detected_miro[0] + detected_miro[2]) / 2
                y_center = (detected_miro[1] + detected_miro[3]) / 2
                return (x_center, y_center)

        return None

    def spin(self, speed=0.5):
        """
        Make MiRo spin in place.
        """
        msg_cmd_vel = TwistStamped()
        msg_cmd_vel.twist.angular.z = speed
        self.vel_pub.publish(msg_cmd_vel)

    def move_toward(self, x_center, frame_width, angular_speed=0.2, linear_speed=1):
        """
        Rotate MiRo to position the detected MiRo on the right side of the frame
        and drive forward toward it.
        """
        msg_cmd_vel = TwistStamped()

        # Calculate error from the right side of the frame
        target_position = frame_width * 0.75  # Adjust this factor to control how far right (e.g., 75% of frame width)
        error = x_center - target_position

        # Adjust angular velocity to position the detected MiRo on the right
        if abs(error) > 20:  # Allow a small margin of error (e.g., 20 pixels)
            msg_cmd_vel.twist.angular.z = -0.002 * error  # Rotate to position the MiRo
            msg_cmd_vel.twist.linear.x = 0  # Stop forward motion while rotating
        else:
            msg_cmd_vel.twist.angular.z = 0  # Stop rotating when positioned
            msg_cmd_vel.twist.linear.x = linear_speed  # Drive forward

        # Publish the velocity command
        self.vel_pub.publish(msg_cmd_vel)

    def loop(self):
        """
        Main control loop.
        """
        rospy.loginfo("MiRo is searching for another MiRo...")
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            if self.new_frame:
                # Detect MiRo in the frame
                self.miro_position = self.detect_miro()
                self.new_frame = False

                if self.miro_position:
                    rospy.loginfo("MiRo detected! Moving toward it...")
                    self.detected_miro = True
                else:
                    self.detected_miro = False

            if self.detected_miro and self.miro_position:
                # Move toward the detected MiRo
                x_center, _ = self.miro_position
                self.move_toward(x_center, self.input_camera.shape[1])
            else:
                # Spin to search for MiRo
                self.spin()

            rate.sleep()


if __name__ == "__main__":
    try:
        client = MiRoYOLOClient()
        client.loop()
    except rospy.ROSInterruptException:
        pass