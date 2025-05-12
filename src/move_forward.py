#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import TwistStamped

class MiRoMoveForward:
    def __init__(self):
        # Initialise a new ROS node
        rospy.init_node("move_forward", anonymous=True)
        # Create a publisher to send velocity commands
        self.vel_pub = rospy.Publisher(
            "/" + os.getenv("MIRO_ROBOT_NAME") + "/control/cmd_vel", TwistStamped, queue_size=1
        )
        # Set the forward speed
        self.forward_speed = 0.2  # m/s

    def move_forward(self):
        # Create a TwistStamped message
        msg_cmd_vel = TwistStamped()
        msg_cmd_vel.twist.linear.x = self.forward_speed  # Set forward linear speed
        msg_cmd_vel.twist.angular.z = 0.0  # No rotation

        # Publish the velocity command
        self.vel_pub.publish(msg_cmd_vel)

    def loop(self):
        # Main loop to keep moving forward
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.move_forward()
            rate.sleep()

if __name__ == "__main__":
    try:
        miro = MiRoMoveForward()
        miro.loop()
    except rospy.ROSInterruptException:
        pass