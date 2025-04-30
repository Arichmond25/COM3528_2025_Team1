# Imports
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

class MiroClient:
    """
    Script settings below
    """
    ##########################
    TICK = 0.02  # This is the update interval for the main control loop in secs
    CAM_FREQ = 1  # Number of ticks before camera gets a new frame, increase in case of network lag
    SLOW = 0.1  # Radial speed when turning on the spot (rad/s)
    FAST = 0.4  # Linear speed when kicking the ball (m/s)
    DEBUG = False # Set to True to enable debug views of the cameras
    TRANSLATION_ONLY = False # Whether to rotate only
    IS_MIROCODE = False  # Set to True if running in MiRoCODE

    # formatting order
    PREPROCESSING_ORDER = ["edge", "smooth", "color", "gaussian"]
        # set to empty to not preprocess or add the methods in the order you want to implement.
        # "edge" to use edge detection, "gaussian" to use difference gaussian
        # "color" to use color segmentation, "smooth" to use smooth blurring,

    # color segmentation format
    HSV = True  # if true select a color which will convert to hsv format with a range of its own, else you can select your own rgb range
    f = lambda x: int(0) if (x < 0) else (int(255) if x > 255 else int(x))
    COLOR_HSV = [f(0), f(0), f(255)]     # target color which will be converted to hsv for processing, format BGR
    COLOR_LOW = (f(0), f(0), f(180))         # low color segment, format BGR
    COLOR_HIGH = (f(255), f(255), f(255))  # high color segment, format BGR

    # edge detection format
    INTENSITY_LOW = 50   # min 0, max 500
    INTENSITY_HIGH = 50  # min 0, max 500

    # smoothing_blurring
    GAUSSIAN_BLURRING = False
    KERNEL_SIZE = 15         # min 3, max 15
    STANDARD_DEVIATION = 0  # min 0.1, max 4.9

    # difference gaussian
    DIFFERENCE_SD_LOW = 1.5 # min 0.00, max 1.40
    DIFFERENCE_SD_HIGH = 0 # min 0.00, max 1.40
    ##########################
    """
    End of script settings
    """

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

    """ ----------------------------
    # !!!! ALL FUNCTION BELOW ARE NOT FINAL AND WILL BE DUE TO CHANGE
     ---------------------------- """

    def look_for_invader(self):
        """
        [1 of 3] Rotate MiRo if it doesn't see a ball in its current
        position, until it sees one.
        """

        """
        TO DO:
        - Change all variable names and others to coincide with what you are 
        trying to look for
        """
        if self.just_switched:  # Print once
            print("MiRo is looking for the ball...")
            self.just_switched = False
        for index in range(2):  # For each camera (0 = left, 1 = right)
            # Skip if there's no new image, in case the network is choking
            if not self.new_frame[index]:
                continue
            image = self.input_camera[index]
            # Run the detect ball procedure
            self.ball[index] = self.detect_invader(image, index)
        # If no ball has been detected
        if not self.ball[0] and not self.ball[1]:
            self.drive(self.SLOW, -self.SLOW)
        else:
            self.status_code = 2  # Switch to the second action
            self.just_switched = True

    def lock_onto_invader(self, error=25):
        """
        [2 of 3] Once a ball has been detected, turn MiRo to face it
        """

        """
        TO DO:
        - Change all variable names and others to coincide with what you are 
        trying to lock onto
        - Possibly make it more animalistic (stalking, walking from behind, etx.)
        """
        if self.just_switched:  # Print once
            print("MiRo is locking on to the intruder")
            self.just_switched = False
        for index in range(2):  # For each camera (0 = left, 1 = right)
            # Skip if there's no new image, in case the network is choking
            if not self.new_frame[index]:
                continue
            image = self.input_camera[index]
            # Run the detect ball procedure
            self.ball[index] = self.detect_invader(image, index)
        # If only the right camera sees the ball, rotate clockwise
        if not self.ball[0] and self.ball[1]:
            self.drive(self.SLOW, -self.SLOW)
        # Conversely, rotate counter-clockwise
        elif self.ball[0] and not self.ball[1]:
            self.drive(-self.SLOW, self.SLOW)
        # Make the MiRo face the ball if it's visible with both cameras
        elif self.ball[0] and self.ball[1]:
            error = 0.05  # 5% of image width
            # Use the normalised values
            left_x = self.ball[0][0]  # should be in range [0.0, 0.5]
            right_x = self.ball[1][0]  # should be in range [-0.5, 0.0]
            rotation_speed = 0.03  # Turn even slower now
            if abs(left_x) - abs(right_x) > error:
                self.drive(rotation_speed, -rotation_speed)  # turn clockwise
            elif abs(left_x) - abs(right_x) < -error:
                self.drive(-rotation_speed, rotation_speed)  # turn counter-clockwise
            else:
                # Successfully turned to face the ball
                self.status_code = 3  # Switch to the third action
                self.just_switched = True
                self.bookmark = self.counter
        # Otherwise, the ball is lost :-(
        else:
            self.status_code = 0  # Go back to square 1...
            print("MiRo has lost the ball...")
            self.just_switched = True

    # Persue target
    def persue(self):
        """
        [3 of 3] Once MiRO is in position, this function should drive the MiRo
        forward until it kicks the ball!
        """

        """
        TO DO:
        - Change all variable names and others to coincide with what you are 
        trying to persue
        """
        if self.just_switched:
            print("MiRo is kicking the ball!")
            self.just_switched = False
        if self.counter <= self.bookmark + 2 / self.TICK and not self.TRANSLATION_ONLY:
            self.drive(self.FAST, self.FAST)
        else:
            self.status_code = 0  # Back to the default state after the kick
            self.just_switched = True

    def detect_invader(self, frame, index):
        """
        Image processing operations, fine-tuned to detect a small,
        toy blue ball in a given frame.
        """

        """
        TO DO:
        - Change detection to find largest red rectangle (or 
        whatever you are trying to detect)
            - Change from circle to rectangle
            - Change HSV colour to red
        - Change all variable names and others to coincide with what you are 
        trying to detect
        """
        if frame is None:  # Sanity check
            return

        # Debug window to show the frame
        if self.DEBUG:
            cv2.imshow("camera" + str(index), frame)
            cv2.waitKey(1)

        # Flag this frame as processed, so that it's not reused in case of lag
        self.new_frame[index] = False

        processed_img = frame

        for method in self.PREPROCESSING_ORDER:
            if method == "color":
                if self.HSV == True:
                    # Get image in HSV (hue, saturation, value) colour format
                    im_hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

                    # Specify target ball colour
                    rgb_colour = np.uint8([[self.COLOR_HSV]])  # e.g. Blue (Note: BGR)
                    # Convert this colour to HSV colour model
                    hsv_colour = cv2.cvtColor(rgb_colour, cv2.COLOR_RGB2HSV)

                    # Extract colour boundaries for masking image
                    # Get the hue value from the numpy array containing target colour
                    target_hue = hsv_colour[0, 0][0]
                    hsv_lo_end = np.array([target_hue - 20, 70, 70])
                    hsv_hi_end = np.array([target_hue + 20, 255, 255])

                    # Generate the mask based on the desired hue range
                    mask = cv2.inRange(im_hsv, hsv_lo_end, hsv_hi_end)
                    processed_img = cv2.bitwise_and(processed_img, processed_img, mask=mask)
                else:
                    mask = cv2.inRange(frame, self.COLOR_LOW, self.COLOR_HIGH)
                    processed_img = cv2.bitwise_and(processed_img, processed_img, mask=mask)

            elif method == "gaussian":
                sigma1 = self.DIFFERENCE_CHECK(self.DIFFERENCE_SD_LOW)
                sigma2 = self.DIFFERENCE_CHECK(self.DIFFERENCE_SD_HIGH)
                img_gauss1 = cv2.GaussianBlur(processed_img, (0, 0), sigma1)
                img_gauss2 = cv2.GaussianBlur(processed_img, (0, 0), sigma2)
                processed_img = img_gauss1 - img_gauss2

            elif method == "smooth":
                kernel_size = (self.KERNEL_SIZE_CHECK(self.KERNEL_SIZE), self.KERNEL_SIZE_CHECK(self.KERNEL_SIZE))

                if not self.GAUSSIAN_BLURRING:
                    # average smoothing
                    kernel = np.ones(kernel_size, np.float32) / kernel_size[0]**2
                    processed_img = cv2.filter2D(processed_img, -1, kernel)

                else:
                    # Gaussian blurring
                    sigma = self.STANDARD_DEVIATION_PROCESS(self.STANDARD_DEVIATION)
                    processed_img = cv2.GaussianBlur(processed_img, (0,0), sigma) # kernel size computed as: [(sigma - 0.8)/0.3 + 1] / 0.5 + 1
                                                                    # see opencv documentation

            elif method == "edge":
                processed_img = cv2.Canny(processed_img, self.INTENSITY_LOW, self.INTENSITY_HIGH)

        # Debug window to show the mask
        if self.DEBUG:
            cv2.imshow("mask" + str(index), processed_img)
            cv2.waitKey(1)

        if len(processed_img.shape) == 3:
            processed_img = cv2.cvtColor(processed_img, cv2.COLOR_BGR2GRAY)

        # Debug window to show the mask
        if self.DEBUG:
            cv2.imshow("gray" + str(index), processed_img)
            cv2.waitKey(1)

        # Fine-tune parameters
        ball_detect_min_dist_between_cens = 40  # Empirical
        canny_high_thresh = 10  # Empirical
        ball_detect_sensitivity = 10  # Lower detects more circles, so it's a trade-off
        ball_detect_min_radius = 5  # Arbitrary, empirical
        ball_detect_max_radius = 50  # Arbitrary, empirical

        # Find circles using OpenCV routine
        # This function returns a list of circles, with their x, y and r values
        circles = cv2.HoughCircles(
            processed_img,
            cv2.HOUGH_GRADIENT,
            1,
            ball_detect_min_dist_between_cens,
            param1=canny_high_thresh,
            param2=ball_detect_sensitivity,
            minRadius=ball_detect_min_radius,
            maxRadius=ball_detect_max_radius,
        )

        if circles is None:
            # If no circles were found, just display the original image
            return

        # Get the largest circle
        max_circle = None
        self.max_rad = 0
        circles = np.uint16(np.around(circles))
        for c in circles[0, :]:
            if c[2] > self.max_rad:
                self.max_rad = c[2]
                max_circle = c
        # This shouldn't happen, but you never know...
        if max_circle is None:
            return

        # Append detected circle and its centre to the frame
        cv2.circle(frame, (max_circle[0], max_circle[1]), max_circle[2], (0, 255, 0), 2)
        cv2.circle(frame, (max_circle[0], max_circle[1]), 2, (0, 0, 255), 3)
        if self.DEBUG:
            cv2.imshow("circles" + str(index), frame)
            cv2.waitKey(1)

        # Normalise values to: x,y = [-0.5, 0.5], r = [0, 1]
        max_circle = np.array(max_circle).astype("float32")
        max_circle[0] -= self.x_centre
        max_circle[0] /= self.frame_width
        max_circle[1] -= self.y_centre
        max_circle[1] /= self.frame_width
        max_circle[1] *= -1.0
        max_circle[2] /= self.frame_width

        # Return a list values [x, y, r] for the largest circle
        return [max_circle[0], max_circle[1], max_circle[2]]
    """ ----------------------------
    # !!!! ALL FUNCTION ABOVE ARE NOT FINAL AND WILL BE DUE TO CHANGE
     ---------------------------- """
