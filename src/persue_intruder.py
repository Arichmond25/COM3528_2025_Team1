#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
MiRo patrol robot that searches for a red-collared intruder using HSV detection.
Uses a modular state machine and MIRO_ROBOT_NAME for topic resolution.
"""

import os
import subprocess
import math  # This is used to reset the head pose
import numpy as np  # Numerical Analysis library
import cv2  # Computer Vision library
from tf.transformations import euler_from_quaternion

import rospy  # ROS Python interface
from sensor_msgs.msg import CompressedImage  # ROS CompressedImage message
from sensor_msgs.msg import JointState  # ROS joints state message
from cv_bridge import CvBridge, CvBridgeError  # ROS -> OpenCV converter
from geometry_msgs.msg import TwistStamped  # ROS cmd_vel (velocity control) message
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

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
STATE_RECOVER = 4
STATE_START = 5

class MiRoPatrol:
    TICK = 0.02  # seconds
    LINEAR_SPEED = 0.2  # Speed when driving forward (m/s)
    ANGULAR_SPEED = 0.01 # Speed when turning (rad/s)
    ANGULAR_GAIN = 0.002
    TURN_DURATION = 1.5
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

            # Set the image for processing
            self.image = image
        except CvBridgeError as e:
            # Ignore corrupted frames
            pass

    def __init__(self):
        rospy.init_node("persue_intruder", anonymous=True)
        rospy.sleep(1.0)

        # Initialise CV Bridge
        self.image_converter = CvBridge()
        self.robot_name = "/" + os.getenv("MIRO_ROBOT_NAME", "miro02")
        topic_base_name = self.robot_name
        print(f"MiRo's robot name: {topic_base_name}")
        cmd_topic = topic_base_name + "/control/cmd_vel"
        cam_topic = topic_base_name + "/camera/primary/compressed"

        self.new_frame = [False, False]

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

        #Publisher for velocity commands
        self.cmd_pub = rospy.Publisher(
            cmd_topic, TwistStamped, queue_size=0)
        
        # self.wheel_speed_l_pub = rospy.Publisher(
        #     self.robot_name + "/control/wheel_speed_l", Float32, queue_size=1
        # )
        # self.wheel_speed_r_pub = rospy.Publisher(
        #     self.robot_name + "/control/wheel_speed_r", Float32, queue_size=1
        # )
        
        # Create a new publisher to move the robot head
        self.pub_kin = rospy.Publisher(
            topic_base_name + "/control/kinematic_joints", JointState, queue_size=0
        )

        # Create handle to store images
        self.input_camera = [None, None]

        # Set the default frame width (gets updated on receiving an image)
        self.frame_width = 640

        print(f"[INFO] Patrol node started for {self.robot_name}")
        print(f"[INFO] Subscribed to: {self.sub_caml}")
        print(f"[INFO] Subscribed to: {self.sub_camr}")
        print(f"[INFO] Publishing to: {self.cmd_pub}")

        self.pose_sub = rospy.Subscriber(
            self.robot_name + '/sensors/odom', Odometry, self.callback_pose)

        # For PID control
        self.yaw_target = None
        self.pid_integral = 0.0
        self.pid_last_error = 0.0
        self.pid_kp = 2.0    # proportional gain
        self.pid_ki = 0.0    # integral gain
        self.pid_kd = 0.2    # derivative gain

        #Patrol steps
        self.image = None
        self.pose_received = False
        self.yaw = 0
        self.state = STATE_START
        self.last_state = STATE_PATROL

        self.start_patrol_steps = [
            ("forward", 60), ("turn", 90), ("forward", 60), ("turn", 90),]

        self.patrol_steps = [("forward", 120), ("turn", 90), ("forward", 120), ("turn", 90),
                             ("forward", 120), ("turn", 90), ("forward", 120), ("turn", 90)]
        self.current_step = 0
        self.step_timer = rospy.Time.now()

        self.recovery_path = []
        self.recovery_index = 0
        self.recovery_timer = None

        rospy.loginfo("[WAIT] Waiting for first camera frame...")
        while self.image is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
        rospy.loginfo("[OK] First camera image received.")

        # Move the head to default pose
        self.reset_head_pose()

    # --- ADDED: Callback for pose updates ---
    def callback_pose(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        self.yaw = yaw
        self.pose_received = True

    def pid_correction(self, target_yaw, current_yaw, dt):
        # Calculate shortest angle difference
        error = math.atan2(math.sin(target_yaw - current_yaw), math.cos(target_yaw - current_yaw))
        
        # PID components
        self.pid_integral += error * dt
        derivative = (error - self.pid_last_error) / dt if dt > 0 else 0.0
        self.pid_last_error = error

        # PID output (angular correction)
        correction = (
            self.pid_kp * error +
            self.pid_ki * self.pid_integral +
            self.pid_kd * derivative
        )
        return correction


    def execute_patrol_step(self, patrol_steps):
        action, value = patrol_steps[self.current_step]
        if action == "forward":
            self.yaw_target = self.yaw  # Set current heading as target
            duration = value  # seconds to move forward
            rate = rospy.Rate(50)  # 50 Hz control loop
            start_time = rospy.Time.now()
            last_time = rospy.Time.now()

            while (rospy.Time.now() - start_time).to_sec() < duration and not rospy.is_shutdown():
                now = rospy.Time.now()
                dt = (now - last_time).to_sec()
                last_time = now
                correction = self.pid_correction(self.yaw_target, self.yaw, dt)
                # Convert angular correction into wheel speed differential
                wheel_base = 0.14  # meters
                speed_l = self.LINEAR_SPEED - correction
                speed_r = self.LINEAR_SPEED + correction
                self.publish_movement(speed_l, speed_r, 0.02)  # small step
                rate.sleep()

            self.current_step = (self.current_step + 1) % len(patrol_steps)
            self.step_timer = rospy.Time.now()
            self.stop()
            rospy.sleep(0.5)
        elif action == "turn":
            self.turn_90()
            self.current_step = (self.current_step + 1) % len(patrol_steps)
            self.step_timer = rospy.Time.now()
            self.stop()
            rospy.sleep(0.5)

    def execute_recovery_step(self):
        if self.recovery_index >= len(self.recovery_path):
            self.state = STATE_PATROL
            self.recovery_path.clear()
            return

        action, value = self.recovery_path[self.recovery_index]
        elapsed = (rospy.Time.now() - self.recovery_timer).to_sec()
        if action == "backward":
            if elapsed < value:
                self.publish_movement(-self.LINEAR_SPEED, -self.LINEAR_SPEED)
            else:
                self.recovery_index += 1
                self.recovery_timer = rospy.Time.now()
                self.publish_movement(0.0, 0.0)
                rospy.sleep(0.5)
        elif action == "turn":
            if elapsed < self.TURN_DURATION:
                direction = np.sign(value)
                self.publish_movement(-direction * self.ANGULAR_SPEED, direction * self.ANGULAR_SPEED)
            else:
                self.recovery_index += 1
                self.recovery_timer = rospy.Time.now()
                self.publish_movement(0.0, 0.0)
                rospy.sleep(0.5)

    # --- ADDED: Pose-based patrol navigation ---
    # def patrol_to_coords(self):
    #     rospy.sleep(0.1)
    #     if not self.pose_received:
    #         print("[WAIT] Waiting for initial pose...")
    #         return
    #     target = self.patrol_coords[self.current_patrol_index]
    #     dx = target[0] - self.position[0]
    #     dy = target[1] - self.position[1]
    #     distance = math.hypot(dx, dy)
    #     angle_to_target = math.atan2(dy, dx)
    #     angle_error = math.atan2(math.sin(angle_to_target - self.yaw), math.cos(angle_to_target - self.yaw))
    #     print(abs(angle_error))

    #     if distance < 0.05:
    #         self.publish_movement(0.0, 0.0)
    #         self.current_patrol_index = (self.current_patrol_index + 1) % len(self.patrol_coords)
    #         print(f"moving to patrol point {self.current_patrol_index}")
    #         print(f"current patrol point: {self.patrol_coords[self.current_patrol_index-1]}")
    #         print(f"current position: {self.position}")
    #     elif abs(angle_error) > 0.02:
    #         angular_speed = max(min(1.5 * angle_error, 0.8), -0.8)  # Clamp to avoid spinning too fast
    #         # Convert angular velocity to differential wheel speeds
    #         wheel_base = 0.14  # Approximate wheelbase in meters
    #         speed_l = -angular_speed * wheel_base / 2.0
    #         speed_r = angular_speed * wheel_base / 2.0
    #         self.publish_movement(speed_l, speed_r)
    #         print(f"[INFO] Adjusting heading to target: {angular_speed:.3f}")
    #         rospy.sleep(1)
    #     else:
    #         self.publish_movement(0.3, 0.3)
    #         # print("[INFO] Moving to patrol point")
    #     if self.image is None:
    #         print("[WAIT] Waiting for camera image...")
    #         return
        
    # --- ADDED: Find closest patrol point ---
    # def find_closest_patrol_index(self):
    #     min_dist = float('inf')
    #     min_index = 0
    #     for i, (x, y) in enumerate(self.patrol_coords):
    #         dist = math.hypot(x - self.position[0], y - self.position[1])
    #         if dist < min_dist:
    #             min_dist = dist
    #             min_index = i
    #     return min_index
    

    # def tick(self):
    #     if self.image is None:
    #         print("[WAIT] Waiting for camera image...")
    #         return

    #     hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
    #     lower_red1 = np.array([0, 100, 100])
    #     upper_red1 = np.array([10, 255, 255])
    #     lower_red2 = np.array([160, 100, 100])
    #     upper_red2 = np.array([180, 255, 255])
    #     mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    #     mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    #     mask = cv2.bitwise_or(mask1, mask2)

    #     M = cv2.moments(mask)
    #     if 10 > 10000:
    #         cx = int(M["m10"] / M["m00"])
    #         width = mask.shape[1]
    #         error = cx - width // 2

    #         if abs(error) < self.CENTER_TOLERANCE:
    #             self.state = STATE_CHASE
    #             print(f"[CHASE] Aligned to target → error: {error}")
    #             self.publish_movement(self.LINEAR_SPEED, self.LINEAR_SPEED)
    #             rospy.sleep(0.5)
    #         else:
    #             self.state = STATE_LOCK
    #             turn = -self.ANGULAR_GAIN * error
    #             print(f"[LOCK] Centering target → error: {error}, turn: {turn:.3f}")
    #             # Convert angular turn into differential wheel speeds
    #             wheel_base = 0.14
    #             speed_l = -turn * wheel_base / 2.0
    #             speed_r = turn * wheel_base / 2.0
    #             self.publish_movement(speed_l, speed_r)
    #             rospy.sleep(0.5)
    #     else:
    #         self.state = STATE_PATROL
    #         # --- ADDED: Reset to nearest patrol point if switching from CHASE/LOCK ---
    #         if self.last_state != STATE_PATROL:
    #             self.current_patrol_index = self.find_closest_patrol_index()
    #         self.patrol_to_coords()
    #         self.last_state = STATE_PATROL

    def tick(self):
        if self.image is None:
            return

        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([0,100,100]), np.array([10,255,255])) |                cv2.inRange(hsv, np.array([160,100,100]), np.array([180,255,255]))
        M = cv2.moments(mask)

        print(f"[INFO] State: {self.state}, Last state: {self.last_state}, Current step: {self.current_step}")

        if self.state == STATE_START and self.current_step <= 3:
            print("[INFO] Exiting start box...")
            self.execute_patrol_step(self.start_patrol_steps)
            if self.current_step == 0:
                self.state = STATE_PATROL
                self.current_step = 0
                self.step_timer = rospy.Time.now()
        elif 10 > 10000:
            cx = int(M["m10"] / M["m00"])
            error = cx - (self.image.shape[1] // 2)
            if abs(error) < self.CENTER_TOLERANCE:
                self.publish_movement(0.3, 0.3)
                self.recovery_path.append(("backward", 0.3))
            else:
                turn = -self.ANGULAR_SPEED * np.sign(error)
                self.publish_movement(-turn, turn)
                self.recovery_path.append(("turn", -90 * np.sign(error)))
            self.state = STATE_CHASE
        elif self.state == STATE_CHASE and self.last_state == STATE_CHASE:
            # Begin recovery
            self.state = STATE_RECOVER
            self.recovery_path.reverse()
            self.recovery_index = 0
            self.recovery_timer = rospy.Time.now()
        elif self.state == STATE_RECOVER:
            self.execute_recovery_step()
        else:
            self.state = STATE_PATROL
            if self.last_state != STATE_PATROL:
                self.step_timer = rospy.Time.now()
            print(f"[INFO] Executing patrol step {self.current_step}...")
            self.execute_patrol_step(self.patrol_steps)

        self.last_state = self.state

    def reset_head_pose(self):
        """
        Reset MiRo head to default position, to avoid having to deal with tilted frames
        """
        print("[INFO] Resetting head pose...")
        self.kin_joints = JointState()  # Prepare the empty message
        self.kin_joints.name = ["tilt", "lift", "yaw", "pitch"]
        self.kin_joints.position = [0.0, math.radians(34.0), 0.0, 0.0]
        t = 0
        while not rospy.core.is_shutdown():  # Check ROS is running
            # Publish state to neck servos for 1 sec
            self.pub_kin.publish(self.kin_joints)
            rospy.sleep(self.TICK)
            t += self.TICK
            if t > 1:
                break
        self.INTENSITY_CHECK = lambda x: int(0) if (x < 0) else (int(500) if x > 500 else int(x))
        self.KERNEL_SIZE_CHECK = lambda x: int(3) if (x < 3) else (int(15) if x > 15 else int(x))
        self.STANDARD_DEVIATION_PROCESS = lambda x: 0.1 if (x < 0.1) else (4.9 if x > 4.9 else round(x, 1))
        self.DIFFERENCE_CHECK = lambda x: 0.01 if (x < 0.01) else (1.40 if x > 1.40 else round(x,2))

    # def publish_movement(self, speed_l = 0.1, speed_r = 0.1):
    #     # Prepare an empty velocity command message
    #     msg_cmd_vel = TwistStamped()
    #     # Desired wheel speed (m/sec)
    #     wheel_speed = [speed_l, speed_r]
    #     # Convert wheel speed to command velocity (m/sec, Rad/sec)
    #     (dr, dtheta) = wheel_speed2cmd_vel(wheel_speed)
    #     # Update the message with the desired speed
    #     msg_cmd_vel.twist.linear.x = dr
    #     msg_cmd_vel.twist.angular.z = dtheta
    #     # Publish message to control/cmd_vel topic
    #     self.cmd_pub.publish(msg_cmd_vel)

    def publish_movement(self, speed_l, speed_r, duration):
        msg = TwistStamped()
        (dr, dtheta) = wheel_speed2cmd_vel([speed_l, speed_r])
        msg.twist.linear.x = dr
        msg.twist.angular.z = dtheta
        print(f"Publishing: linear={msg.twist.linear.x:.3f}, angular={msg.twist.angular.z:.3f}")

        end_time = rospy.Time.now() + rospy.Duration(duration)
        rate = rospy.Rate(20)
        while rospy.Time.now() < end_time and not rospy.is_shutdown():
            self.cmd_pub.publish(msg)
            rate.sleep()

    # def publish_wheel_speeds(self, speed_l, speed_r, duration):
    #     """
    #     Publish velocity commands to the left and right wheels separately.
    #     """
    #     print(f"Publishing wheel speeds: left={speed_l}, right={speed_r}, duration={duration}")
    #     end_time = rospy.Time.now() + rospy.Duration(duration)
    #     rate = rospy.Rate(20)
    #     while rospy.Time.now() < end_time and not rospy.is_shutdown():
    #         self.wheel_speed_l_pub.publish(speed_l)
    #         self.wheel_speed_r_pub.publish(speed_r)
    #         rate.sleep()
    #     print("Wheel speed publishing complete.")

    def stop(self):
        msg = TwistStamped()
        (dr, dtheta) = wheel_speed2cmd_vel([0.0, 0.0])
        msg.twist.linear.x = dr
        msg.twist.angular.z = dtheta
        self.cmd_pub.publish(msg)
        rospy.sleep(0.5)

    def turn_90(self):
        if not self.pose_received:
            rospy.logwarn("No pose received. Skipping turn.")
            return

        # Calculate target yaw 90 degrees from current (in radians)
        self.yaw_target = self.yaw + math.radians(90)
        self.yaw_target = math.atan2(math.sin(self.yaw_target), math.cos(self.yaw_target))  # Normalize angle

        rospy.loginfo("Starting PID turn to 90 degrees...")

        rate = rospy.Rate(50)
        last_time = rospy.Time.now()

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = (now - last_time).to_sec()
            last_time = now

            # Compute correction from current to target yaw
            correction = self.pid_correction(self.yaw_target, self.yaw, dt)

            # Stop condition
            if abs(self.yaw - self.yaw_target) < math.radians(0.5):  # ~0.5 degree tolerance
                break

            # Turn in place using angular correction
            wheel_base = 0.14
            speed_l = -correction * wheel_base / 2.0
            speed_r = correction * wheel_base / 2.0
            self.publish_movement(speed_l, speed_r, 0.02)

            rate.sleep()
        self.stop()
        rospy.loginfo("Turn complete.")
        rospy.sleep(1.0)



if __name__ == "__main__":
    node = MiRoPatrol()
    rate = rospy.Rate(1.0 / node.TICK)
    while not rospy.is_shutdown():
        node.tick()
        rate.sleep()
