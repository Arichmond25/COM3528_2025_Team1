#!/usr/bin/env python3
import os
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import TwistStamped
from cv_bridge import CvBridge, CvBridgeError

class MazeNavigator:
    """
    A program to control MiRo to traverse a maze using its cameras
    to detect walls and avoid collisions.
    """
    FORWARD_SPEED = 0.2  # Speed when driving forward (m/s)
    TURN_SPEED = 0.3     # Speed when turning (rad/s)
    WALL_THRESHOLD = 100  # Pixel intensity threshold to detect walls

    def __init__(self):
        # Initialize ROS node
        rospy.init_node("maze_navigator", anonymous=True)

        # Get MiRo's robot name from the environment
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

        # Publisher for velocity commands
        self.vel_pub = rospy.Publisher(
            topic_base_name + "/control/cmd_vel", TwistStamped, queue_size=0
        )

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

    def process_image(self, image):
        """
        Process the camera image to detect walls.
        """
        if image is None:
            return False

        # Convert the image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Apply a threshold to detect bright areas (walls)
        _, thresh = cv2.threshold(gray, self.WALL_THRESHOLD, 255, cv2.THRESH_BINARY)

        # Count the number of white pixels (indicating walls)
        wall_pixels = cv2.countNonZero(thresh)

        # If there are enough wall pixels, return True
        return wall_pixels > 500

    def drive_forward(self):
        """
        Command MiRo to drive forward.
        """
        vel_cmd = TwistStamped()
        vel_cmd.twist.linear.x = self.FORWARD_SPEED
        vel_cmd.twist.angular.z = 0.0
        self.vel_pub.publish(vel_cmd)

    def turn(self, direction="left"):
        """
        Command MiRo to turn left or right.
        """
        vel_cmd = TwistStamped()
        vel_cmd.twist.linear.x = 0.0
        vel_cmd.twist.angular.z = self.TURN_SPEED if direction == "left" else -self.TURN_SPEED
        self.vel_pub.publish(vel_cmd)

    def stop(self):
        """
        Command MiRo to stop.
        """
        vel_cmd = TwistStamped()
        vel_cmd.twist.linear.x = 0.0
        vel_cmd.twist.angular.z = 0.0
        self.vel_pub.publish(vel_cmd)

    def loop(self):
        """
        Main control loop.
        """
        rospy.sleep(1.0)  # Wait for initialization
        print("Maze navigation started. Press CTRL+C to stop.")

        while not rospy.is_shutdown():
            # Process the left and right camera images
            left_wall = self.process_image(self.left_image)
            right_wall = self.process_image(self.right_image)

            if left_wall and right_wall:
                # Walls on both sides, stop and turn left
                print("Walls detected on both sides! Turning left...")
                self.stop()
                rospy.sleep(0.5)
                self.turn(direction="left")
                rospy.sleep(1.0)
                self.stop()
            elif left_wall:
                # Wall on the left, turn right
                print("Wall detected on the left! Turning right...")
                self.turn(direction="right")
                rospy.sleep(0.5)
            elif right_wall:
                # Wall on the right, turn left
                print("Wall detected on the right! Turning left...")
                self.turn(direction="left")
                rospy.sleep(0.5)
            else:
                # No walls detected, drive forward
                print("Driving forward...")
                self.drive_forward()

            rospy.sleep(0.1)  # Loop at 10 Hz

if __name__ == "__main__":
    try:
        navigator = MazeNavigator()
        navigator.loop()
    except rospy.ROSInterruptException:
        pass