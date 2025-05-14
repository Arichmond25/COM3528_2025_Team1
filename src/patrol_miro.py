#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
MiRo patrol robot that searches for a red-collared intruder using HSV detection.
Uses a modular state machine and MIRO_ROBOT_NAME for topic resolution.
"""

import os
import math  # This is used to reset the head pose
import numpy as np  # Numerical Analysis library
import cv2  # Computer Vision library

import time
from tf.transformations import euler_from_quaternion

import rospy  # ROS Python interface
from sensor_msgs.msg import CompressedImage  # ROS CompressedImage message
from sensor_msgs.msg import JointState  # ROS joints state message
from cv_bridge import CvBridge, CvBridgeError  # ROS -> OpenCV converter
from geometry_msgs.msg import TwistStamped  # ROS cmd_vel (velocity control) message
from nav_msgs.msg import Odometry

from ultralytics import YOLO  # YOLOv8 library

try:  # For convenience, import this util separately
    from miro2.lib import wheel_speed2cmd_vel  # Python 3
except ImportError:
    from miro2.utils import wheel_speed2cmd_vel  # Python 2


# State Constants
STATE_DETECT = "detect"
STATE_LOCK = "lock"
STATE_CHASE = "chase"
STATE_PATROL = "patrol"
STATE_RECOVER = "recover"
STATE_START = "start"

class MiRoPatrol:
    TICK = 0.02  # seconds
    LINEAR_SPEED = 0.2  # Speed when driving forward (m/s)
    ANGULAR_SPEED = 0.01 # Speed when turning (rad/s)
    ANGULAR_GAIN = 0.002
    TURN_DURATION = 1.5
    CENTER_TOLERANCE = 30

    def __init__(self):
        rospy.init_node("persue_intruder", anonymous=True)
        rospy.sleep(1.0)

        # Initialise CV Bridge
        self.image_converter = CvBridge()
        self.robot_name = "/" + os.getenv("MIRO_ROBOT_NAME","miro01")
        topic_base_name = self.robot_name
        print(f"MiRo's robot name: {topic_base_name}")
        cmd_topic = topic_base_name + "/control/cmd_vel"
        
        # Load YOLO model
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.model_path = os.path.join(script_dir, "yolo_model/best.pt")
        self.model = YOLO(self.model_path)

        # Initialise CV Bridge
        self.image_converter = CvBridge()

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

        #Publisher for velocity commands
        self.cmd_pub = rospy.Publisher(
            cmd_topic, TwistStamped, queue_size=0)
        
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
        self.start_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        self.yaw_target = 0
        self.pid_integral = 0.0
        self.pid_last_error = 0.0
        self.pid_kp = 2.0    # proportional gain
        self.pid_ki = 0.0    # integral gain
        self.pid_kd = 0.2    # derivative gain

        # For Detection 
        self.input_camera_left = None
        self.input_camera_right = None
        self.new_frame_left = False
        self.new_frame_right = False
        self.detected_miro = False
        self.miro_position_left = None
        self.miro_position_right = None

        #Patrol steps
        self.pose_received = False
        self.yaw = 0
        self.state = STATE_START
        self.last_state = STATE_PATROL

        self.start_patrol_steps = [
            ("forward", 40), ("turn", 90), ("forward", 35), ("turn", 90),]

        self.patrol_steps = [("forward", 85), ("turn", 90), ("forward", 85), ("turn", 90),
                             ("forward", 85), ("turn", 90), ("forward", 85), ("turn", 90)]
        self.current_step = 0
        self.step_timer = rospy.Time.now()

        self.recovery_path = []
        self.recovery_index = 0
        self.recovery_timer = None

        # Move the head to default pose
        self.reset_head_pose()
        print("[INFO] Head pose reset.")

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
            if int(result.cls) == 0:  # Class 0 is "MiRo"
                # Get bounding box center
                detected_miro = result.xyxy[0]
                x_center = (detected_miro[0] + detected_miro[2]) / 2
                y_center = (detected_miro[1] + detected_miro[3]) / 2
                return (x_center, y_center)

        return None
    
    def look_for_miro(self):
        if self.new_frame_left:
            # Detect MiRo in the left camera frame
            self.miro_position_left = self.detect_miro(self.input_camera_left)
            self.new_frame_left = False

        if self.new_frame_right:
            # Detect MiRo in the right camera frame
            self.miro_position_right = self.detect_miro(self.input_camera_right)
            self.new_frame_right = False

        if self.miro_position_left or self.miro_position_right:
            self.state = STATE_CHASE
        else:
            self.state = STATE_PATROL
        

    def micro_adjust(self, left_x, right_x, width, base_speed=0.3, adjustment=0.05):
        """
        Perform micro adjustments to keep MiRo aligned with the detected target.
        """
        frame_half = width / 2

        if left_x < frame_half:
            self.drive(speed_l=base_speed, speed_r=base_speed+adjustment)
            rospy.loginfo("adijusting for left")
        else:
            # Move forward if aligned
            self.drive(speed_l=base_speed, speed_r=base_speed)
            
        if right_x > frame_half:
            self.drive(speed_l=base_speed+adjustment, speed_r=base_speed)  
            rospy.loginfo("adijusting for right")
        else:
            # Move forward if aligned
            self.drive(speed_l=base_speed, speed_r=base_speed)

    def drive(self, speed_l=0.1, speed_r=0.1):  # (m/sec, m/sec)
        """
        Wrapper to simplify driving MiRo by converting wheel speeds to cmd_vel
        """
        # Prepare an empty velocity command message
        msg_cmd_vel = TwistStamped()

        # Desired wheel speed (m/sec)
        wheel_speed = [speed_l, speed_r]

        #Add velocity command to recovery path when in chase state
        if self.state == STATE_CHASE:
            self.recovery_path.insert(0,[-speed_l,-speed_r])

        # Convert wheel speed to command velocity (m/sec, Rad/sec)
        (dr, dtheta) = wheel_speed2cmd_vel(wheel_speed)

        # Update the message with the desired speed
        msg_cmd_vel.twist.linear.x = dr
        msg_cmd_vel.twist.angular.z = dtheta

        # Publish message to control/cmd_vel topic
        self.cmd_pub.publish(msg_cmd_vel)

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
        # Check if action is "forward" or "turn"
        if action == "forward":
            self.yaw_target = self.yaw  # Set current heading as target
            duration = value  # seconds to move forward
            rate = rospy.Rate(50)  # 50 Hz control loop
            self.start_time = rospy.Time.now()
            self.last_time = rospy.Time.now()

            while (rospy.Time.now() - self.start_time).to_sec() < duration and not rospy.is_shutdown():
                # Look for MiRo in the camera frames
                if self.state != STATE_START:
                    self.look_for_miro()
                if self.state == STATE_CHASE:
                    # If MiRo is in chase state, don't publish velocity commands and add them to recovery path
                    self.recovery_path.insert(0,[self.LINEAR_SPEED,self.LINEAR_SPEED])
                else:
                    self.publish_movement(self.LINEAR_SPEED, self.LINEAR_SPEED, 0.02)  # small step
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

    def execute_recovery_step(self,speed_l, speed_r):
        # Check if recovery index is more than or equal to the length of the recovery path
        if self.recovery_index >= len(self.recovery_path):
            self.state = STATE_PATROL
            self.recovery_path.clear()
            self.recovery_index = -1
            return
        
        msg = TwistStamped()
        (dr, dtheta) = wheel_speed2cmd_vel([speed_l, speed_r])
        msg.twist.linear.x = dr
        msg.twist.angular.z = dtheta
        self.cmd_pub.publish(msg)

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

    def publish_movement(self, speed_l, speed_r, duration = None, pid=True):
        msg = TwistStamped()
        if pid:
            now = rospy.Time.now()
            dt = (now - self.last_time).to_sec()
            self.last_time = now
            correction = self.pid_correction(self.yaw_target, self.yaw, dt)
            # Convert angular correction into wheel speed differential
            speed_l = self.LINEAR_SPEED - correction
            speed_r = self.LINEAR_SPEED + correction

        if self.state == STATE_CHASE:
            self.recovery_path.insert(0,[-speed_l,-speed_r])
        (dr, dtheta) = wheel_speed2cmd_vel([speed_l, speed_r])
        msg.twist.linear.x = dr
        msg.twist.angular.z = dtheta
        # print(f"Publishing: linear={msg.twist.linear.x:.3f}, angular={msg.twist.angular.z:.3f}")
        
        if duration is None:
            self.cmd_pub.publish(msg)
            return
        
        end_time = rospy.Time.now() + rospy.Duration(duration)
        rate = rospy.Rate(20)
        while rospy.Time.now() < end_time and not rospy.is_shutdown():
            self.cmd_pub.publish(msg)
            rate.sleep()

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
            # Look for MiRo in the camera frames
            #COULD LOOK FOR MIRO WHILE TURNING, BUT LOGIC IS NOT IMPLEMENTED
            # self.look_for_miro()

            if self.state == STATE_CHASE:
                # If MiRo is detected, stop moving go back to rate
                self.stop()
                break
            
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
            self.publish_movement(speed_l, speed_r, 0.02, pid=False)

            rate.sleep()
        self.stop()
        rospy.loginfo("Turn complete.")
        rospy.sleep(1.0)


    def tick(self):
        print(f"[INFO] [PATROL] I am currently doing: {self.state}, Last thing I did was: {self.last_state},")
        
        # Start state is get out of the start box
        if self.state == STATE_START:
            print("[INFO] Exiting start box...")
            self.execute_patrol_step(self.start_patrol_steps)
            if self.current_step == 0:
                self.state = STATE_PATROL
                self.current_step = 0
                self.step_timer = rospy.Time.now()

        # Add Chase state functionality when ditected an intruder
        elif self.state == STATE_CHASE:
            # If MiRo is not detected in the left frame, turn anticlockwise
            if not self.miro_position_left:
                self.drive(speed_l=0.1, speed_r=-0.1)  # Anticlockwise rotation
            # If MiRo is not detected in the right frame, turn clockwise
            elif not self.miro_position_right:
                self.drive(speed_l=-0.1, speed_r=0.1)  # Clockwise rotation
            else:
                self.drive(speed_l=0.3, speed_r=0.3)  # Move forward
                # Micro adjustments if MiRo is detected in both cameras
                left_x, left_y = self.miro_position_left
                right_x, right_y = self.miro_position_right
                self.micro_adjust(left_x, right_x, width=self.input_camera_left.shape[1])

        # If no MiRo was chasing intruder now need to recover to patrol route 
        elif self.last_state == STATE_CHASE and self.state != STATE_CHASE:
            # Begin recovery
            self.state = STATE_RECOVER
            # TODO: Change state to STATE_PATROL once recovery is done
            self.recovery_index = 0
            self.recovery_timer = rospy.Time.now()
        elif self.state == STATE_RECOVER:
            self.execute_recovery_step(self.recovery_path[self.recovery_index][0],
                                        self.recovery_path[self.recovery_index][1])
            self.recovery_index += 1
        else:
            print(f"[INFO] [PATROL] I am in patrol mode, current step: {self.current_step}")
            # Detect MiRo in the camera frame
            self.look_for_miro()
            if self.last_state != STATE_PATROL:
                self.step_timer = rospy.Time.now()
            print(f"[INFO] [PATROL] Executing patrol step {self.current_step}...")
            self.execute_patrol_step(self.patrol_steps)

        self.last_state = self.state



if __name__ == "__main__":
    node = MiRoPatrol()
    rate = rospy.Rate(1.0 / node.TICK)
    while not rospy.is_shutdown():
        node.tick()
        rate.sleep()