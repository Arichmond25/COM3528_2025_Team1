#!/usr/bin/env python3
import os
import rospy
import cv2
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

class MiRoImageCapture:
    """
    A program to capture images from MiRo's cameras and save them to a directory.
    """
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("miro_image_capture", anonymous=True)

        # Get MiRo's robot name from the environment
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

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
        self.output_dir = "data/images"
        os.makedirs(self.output_dir, exist_ok=True)

    def callback_caml(self, data):
        """
        Callback function for the left camera.
        """
        try:
            self.left_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"Left camera error: {e}")

    def callback_camr(self, data):
        """
        Callback function for the right camera.
        """
        try:
            self.right_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"Right camera error: {e}")

    def save_image(self, image, filename):
        """
        Save an image to the specified filename.
        """
        if image is not None:
            filepath = os.path.join(self.output_dir, filename)
            cv2.imwrite(filepath, image)
            print(f"Image saved: {filepath}")
        else:
            print("No image available to save.")

    def loop(self):
        """
        Main loop to capture images on user input.
        """
        print("Type 'capture' to save images from MiRo's cameras. Press CTRL+C to exit.")
        while not rospy.is_shutdown():
            user_input = input("Command: ").strip().lower()
            if user_input == "capture":
                # Save images from both cameras
                self.save_image(self.left_image, "left_camera.jpg")
                self.save_image(self.right_image, "right_camera.jpg")
            else:
                print("Unknown command. Type 'capture' to save images.")

if __name__ == "__main__":
    try:
        capture = MiRoImageCapture()
        capture.loop()
    except rospy.ROSInterruptException:
        pass