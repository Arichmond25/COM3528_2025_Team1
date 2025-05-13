#!/usr/bin/env python3

"""
MiRo Intruder Detection and Avoidance System
This script detects audio alerts and makes MiRo move away from the sound source.
"""

import os
import numpy as np
import rospy
import miro2 as miro
import time
import math
from std_msgs.msg import Int16MultiArray, String
from geometry_msgs.msg import TwistStamped
from node_detect_audio_engine import DetectAudioEngine

try:  # For convenience, import this util separately
    from miro2.lib import wheel_speed2cmd_vel  # Python 3
except ImportError:
    from miro2.utils import wheel_speed2cmd_vel  # Python 2


class SoundDetector:
    def __init__(self):
        # Initialize audio engine
        self.audio_engine = DetectAudioEngine()

        # Initialize variables for sound localization
        self.audio_event = None
        self.frame = None
        self.frame_p = None
        self.thresh = 0.03  # Base threshold for sound detection
        self.tmp = []  # Buffer for dynamic thresholding

        # Variables to store the intruder's location
        self.intruder_direction = None  # Direction in radians
        self.intruder_distance = None  # Distance estimate based on sound level

        # Variables for movement control
        self.escape_time = 2.0  # seconds to run away
        self.start_pose = None  # Starting position before escape
        self.alert_count = 0  # Number of alerts detected

        # Audio buffer for keyword detection
        self.audio_buffer = []
        self.buffer_size = (
            20  # Buffer 20 frames (about 2 seconds at 500 samples per buffer)
        )

        # Target keywords to detect in audio
        self.target_phrases = ["intruder", "danger", "alert", "warning"]

        # which miro - get robot name from environment variable
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

        # subscribers
        self.sub_mics = rospy.Subscriber(
            topic_base_name + "/sensors/mics",
            Int16MultiArray,
            self.callback_mics,
            queue_size=1,
            tcp_nodelay=True,
        )

        # self.sub_speech = rospy.Subscriber(
        #     "/speech_recognition/result", String, self.callback_speech, queue_size=10
        # )

        # publishers
        self.pub_wheels = rospy.Publisher(
            topic_base_name + "/control/cmd_vel", TwistStamped, queue_size=0
        )

        # Messages for controlling MiRo
        self.msg_wheels = TwistStamped()

        # Status flags
        self.status_code = 0
        self.escape_time = 2.0  # seconds to run away

        rospy.loginfo("Intruder initialized, waiting for audio events...")

    def callback_mics(self, data):
        """
        Process incoming microphone data
        """
        # Process data for sound localization
        self.audio_event = self.audio_engine.process_data(data.data)

        # Process data for dynamic thresholding
        data_t = np.asarray(data.data, "float32") * (1.0 / 32768.0)
        data_t = data_t.reshape((4, 500))
        head_data = data_t[2][:]

        # Update buffer for dynamic thresholding
        if self.tmp is None:
            self.tmp = np.hstack((self.tmp, np.abs(head_data)))
        elif len(self.tmp) < 10500:
            self.tmp = np.hstack((self.tmp, np.abs(head_data)))
        else:
            # When the buffer is full
            self.tmp = np.hstack((self.tmp[-10000:], np.abs(head_data)))
            # Dynamic threshold is calculated and updated when new signal comes
            self.thresh = 0.03 + self.audio_engine.non_silence_thresh(self.tmp)

        # Add audio data to buffer for keyword spotting
        self.update_audio_buffer(data_t)

        # Check for keywords in audio
        if len(self.audio_buffer) >= self.buffer_size:
            self.detect_keywords_in_audio()

    def update_audio_buffer(self, data_t):
        """
        Update the audio buffer with new microphone data
        """
        # Use the highest quality microphone data (typically head mic)
        head_data = data_t[2][:]

        # Add to buffer
        self.audio_buffer.append(head_data)

        # Keep buffer at fixed size
        if len(self.audio_buffer) > self.buffer_size:
            self.audio_buffer.pop(0)

    def detect_keywords_in_audio(self):
        """
        Simple audio event detection based on energy levels
        In a production system, you would replace this with a proper keyword detection model
        """
        # Combine buffer into a single array
        audio_segment = np.concatenate(self.audio_buffer)

        # Calculate energy
        energy = np.mean(np.abs(audio_segment))

        # Simple energy threshold detection (simulation of keyword detection)
        if energy > self.energy_threshold:
            self.consecutive_energy_frames += 1

            if self.consecutive_energy_frames >= self.consecutive_frames_threshold:
                rospy.loginfo(f"High energy audio event detected: {energy:.3f}")
                self.consecutive_energy_frames = 0

                # Simulate keyword detection
                # In a real implementation, you would use a proper speech recognition or
                # keyword spotting system here
                if self.audio_event and self.audio_event[0] is not None:
                    # Randomly "detect" a keyword for demonstration purposes
                    # You would replace this with actual keyword detection
                    if energy > 0.15:  # Higher threshold for "keyword detection"
                        detected_phrase = np.random.choice(self.target_phrases)
                        rospy.loginfo(f"Simulated keyword detected: {detected_phrase}")
                        self.target_detected(f"Detected: {detected_phrase}")
        else:
            self.consecutive_energy_frames = 0

    def target_detected(self, text):
        """
        Called when target text is detected
        """
        rospy.loginfo(f"Target phrase detected in: '{text}'")

        # Check if we have valid audio event data
        if self.audio_event and self.audio_event[0] is not None:
            # Get audio event data
            ae = self.audio_event[0]
            self.frame = self.audio_event[1]

            # Calculate mean audio level across left and right microphones
            mean_level = (self.audio_event[2][0] + self.audio_event[2][1]) / 2

            # Set direction (in radians) from the audio event
            self.intruder_direction = ae.azim
            self.intruder_direction_degrees = math.degrees(self.intruder_direction)

            # Estimate distance based on audio level
            # This is a rough estimation that needs calibration
            if mean_level > 0.3:
                self.intruder_distance = 0.5  # Very close (0.5 meters)
            elif mean_level > 0.2:
                self.intruder_distance = 1.0  # Close (1 meter)
            elif mean_level > 0.1:
                self.intruder_distance = 2.0  # Medium distance (2 meters)
            else:
                self.intruder_distance = 3.0  # Far (3+ meters)

            rospy.loginfo(
                f"Intruder detected at direction: {self.intruder_direction_degrees:.2f} degress, {self.intruder_distance} radians,  estimated distance: ~{self.intruder_distance:.1f} meters"
            )

            # Escape from the sound source (move in opposite direction)
            self.escape_from_intruder()

        else:
            rospy.logwarn("Target detected but could not localize sound source")

    def escape_from_intruder(self):
        """
        Move MiRo in the opposite direction of the detected sound
        """
        if self.intruder_direction is None:
            rospy.logwarn("Cannot escape: direction unknown")
            return

        # Calculate opposite direction (180 degrees / π radians from source)
        escape_direction = self.intruder_direction + np.pi
        if escape_direction > np.pi:
            escape_direction -= 2 * np.pi

        escape_direction_degrees = math.degrees(escape_direction)

        rospy.loginfo(
            f"Escaping in direction: {escape_direction_degrees:.2f} degrees, {escape_direction} radians"
        )

        # First rotate to face escape direction
        self.rotate_to_direction(escape_direction)

        # Then move forward to escape
        self.move_forward(self.escape_time)

        rospy.loginfo("Escape complete")

    def rotate_to_direction(self, direction):
        """
        Rotate MiRo to face the specified direction
        """
        # MiRo completes rotation in 0.5s
        Tf = 0.5
        T1 = 0

        while T1 <= Tf:
            # Set angular velocity to turn toward the direction
            self.msg_wheels.twist.linear.x = 0.0
            self.msg_wheels.twist.angular.z = direction * 2

            # Publish command
            self.pub_wheels.publish(self.msg_wheels)

            # Small sleep to control timing
            time.sleep(0.02)
            T1 += 0.02

    def move_forward(self, duration):
        """
        Move MiRo forward for the specified duration
        """
        T1 = 0

        while T1 <= duration:
            # Set linear velocity for forward movement, no rotation
            self.msg_wheels.twist.linear.x = 0.1  # Forward speed
            self.msg_wheels.twist.angular.z = 0.0

            # Publish command
            self.pub_wheels.publish(self.msg_wheels)

            # Small sleep to control timing
            time.sleep(0.02)
            T1 += 0.02

        # Stop MiRo after movement
        self.msg_wheels.twist.linear.x = 0.0
        self.pub_wheels.publish(self.msg_wheels)

    def run(self):
        """
        Main loop for the intruder detector
        """

        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():

            # Main loop logic goes here if needed
            rate.sleep()


# if __name__ == "__main__":
#     rospy.init_node("intruder_detector", anonymous=True)
#     detector = SoundDetector()
#     detector.run()
