#!/usr/bin/env python3

import os
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

class MiRoDetector:
    def __init__(self):
        rospy.init_node("miro_color_detector", anonymous=True)
        self.bridge = CvBridge()

        # Red color range in HSV (both ends of the hue spectrum)
        self.lower_red1 = np.array([0, 120, 70])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([170, 120, 70])
        self.upper_red2 = np.array([180, 255, 255])

        self.input_camera = [None, None]

        # Subscribe to left and right MiRo cameras
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        rospy.Subscriber(topic_base_name + "/sensors/caml/compressed", CompressedImage, self.callback_caml)
        rospy.Subscriber(topic_base_name + "/sensors/camr/compressed", CompressedImage, self.callback_camr)

        rospy.loginfo("🔍 MiRo Color Detector node started. Looking for red markers...")

    def callback_caml(self, ros_image):
        self.process_frame(ros_image, camera_index=0)

    def callback_camr(self, ros_image):
        self.process_frame(ros_image, camera_index=1)

    def process_frame(self, ros_image, camera_index):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(ros_image, "rgb8")
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
        except CvBridgeError as e:
            rospy.logwarn("CvBridgeError: {}".format(e))
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Combine two red ranges
        mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # Find contours and check for a large enough red object
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            if area > 200:  # You can tweak this threshold
                rospy.loginfo(f"[Camera {camera_index}] 🔴 Red marker detected (area={int(area)}) — Possible MiRo in view!")

        # Optional: Visual debug
        cv2.imshow(f"Camera {camera_index}", mask)
        cv2.waitKey(1)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        detector = MiRoDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
