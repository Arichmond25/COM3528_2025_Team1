#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
import os
from geometry_msgs.msg import TwistStamped
from random import choice

try:
    from miro2.lib import wheel_speed2cmd_vel  # Python 3
except ImportError:
    from miro2.utils import wheel_speed2cmd_vel  # Python 2


class MiRoMovement:
    def __init__(self):
        # Initialise ROS node
        rospy.init_node("miro_movement", anonymous=True)

        # Velocity publisher
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME", "miro")
        self.vel_pub = rospy.Publisher(
            topic_base_name + "/control/cmd_vel", TwistStamped, queue_size=0
        )

    def drive(self, speed_l=0.0, speed_r=0.0, duration=0):
        """
        Drive MiRo with specified wheel speeds for a given duration.
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

        # Publish the velocity command for the specified duration
        start_time = time.time()
        while time.time() - start_time < duration and not rospy.is_shutdown():
            self.vel_pub.publish(msg_cmd_vel)
            rospy.sleep(0.1)

    def loop(self):
        """
        Main control loop.
        """
        rospy.loginfo("MiRo is alternating between turning and moving forward...")
        rate = rospy.Rate(1)  # 1 Hz

        while not rospy.is_shutdown():
            # Randomly choose to turn left or right
            turn_direction = choice(["left", "right"])
            if turn_direction == "left":
                rospy.loginfo("Turning left...")
                self.drive(speed_l=-0.1, speed_r=0.1, duration=0.4)  # Turn left for 2 seconds
            else:
                rospy.loginfo("Turning right...")
                self.drive(speed_l=0.1, speed_r=-0.1, duration=0.4)  # Turn right for 2 seconds

            # Move forward for 5 seconds
            rospy.loginfo("Moving forward...")
            self.drive(speed_l=0.2, speed_r=0.2, duration=20)

            rate.sleep()


if __name__ == "__main__":
    try:
        miro_movement = MiRoMovement()
        miro_movement.loop()
    except rospy.ROSInterruptException:
        pass