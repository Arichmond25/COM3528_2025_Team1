#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
This script makes MiRo act as an intruder wandering a maze in a predictable patrol-like pattern.
Structured similarly to the patrol robot script for consistency.
"""

import rospy
from geometry_msgs.msg import TwistStamped

STATE_WANDER = 0

class MiRoIntruder:
    TICK = 0.1  # seconds
    LINEAR_SPEED = 0.05

    def __init__(self):
        rospy.init_node("wander_intruder")
        self.cmd_pub = rospy.Publisher("/miro/rob01/platform/control/cmd_vel", TwistStamped, queue_size=1)

        self.state = STATE_WANDER
        self.wander_pattern = [(0.05, 0.0)] * 30 + [(0.0, -0.5)] * 10 + [(0.05, 0.0)] * 25 + [(0.0, 0.5)] * 10
        self.step = 0

    def tick(self):
        if self.step >= len(self.wander_pattern):
            self.step = 0  # loop
        lin, ang = self.wander_pattern[self.step]
        self.publish_movement(lin, ang)
        self.step += 1

    def publish_movement(self, linear, angular):
        msg = TwistStamped()
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        self.cmd_pub.publish(msg)

    def run(self):
        rate = rospy.Rate(1 / self.TICK)
        while not rospy.is_shutdown():
            self.tick()
            rate.sleep()

if __name__ == '__main__':
    try:
        bot = MiRoIntruder()
        bot.run()
    except rospy.ROSInterruptException:
        pass
