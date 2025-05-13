#!/usr/bin/env python3
import os
import rospy
import cv2
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import time

class MiRoImageCapture:
    """
    A program to capture images from MiRo's cameras in the sim and save them to a directory.
    """
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("miro_image_capture", anonymous=True)

        # Get MiRo's robot name from the environment
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        print(f"MiRo's robot name: {topic_base_name}")

        # Subscribers for left and right camera feeds
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

        # Initialize CV Bridge
        self.bridge = CvBridge()

        # State variables for camera frames
        self.left_image = None
        self.right_image = None

        # Create the output directory if it doesn't exist
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.output_dir = os.path.join(script_dir, "data/new_images")
        os.makedirs(self.output_dir, exist_ok=True)

    def callback_caml(self, data):
        try:
            self.left_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"Left camera error: {e}")

    def callback_camr(self, data):
        try:
            self.right_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"Right camera error: {e}")

    def save_image(self, image, filename):
        if image is not None:
            filepath = os.path.join(self.output_dir, filename)
            cv2.imwrite(filepath, image)
            print(f"Image saved: {filepath}")
        else:
            print("No image available to save.")

    def loop(self):
        """
        Main loop to continuously capture images every 10 seconds.
        """
        time.sleep(1)
        print("Capturing images every 10 seconds. Press CTRL+C to exit.")
        counter = 20  # Start the counter for image filenames
        rate = rospy.Rate(0.1)  # Set the loop rate to 1 Hz (1 second)

        while not rospy.is_shutdown():
            # Save images from both cameras with sequential filenames
            self.save_image(self.left_image, f"image{counter}_left.jpg")
            self.save_image(self.right_image, f"image{counter}_right.jpg")
            counter += 1  # Increment the counter
            rate.sleep()  # Sleep to maintain the 1 Hz rate

if __name__ == "__main__":
    try:
        capture = MiRoImageCapture()
        capture.loop()
    except rospy.ROSInterruptException:
        pass