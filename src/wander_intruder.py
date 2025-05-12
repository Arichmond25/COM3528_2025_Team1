#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
MiRo intruder robot that wanders the maze in a looping pattern.
Automatically resolves topic namespace from MIRO_ROBOT_NAME environment variable.
"""

import rospy
import os
from geometry_msgs.msg import TwistStamped

STATE_WANDER = 0

class MiRoIntruder:
    TICK = 0.1  # seconds
    LINEAR_SPEED = 0.05

    def __init__(self):
        rospy.init_node("wander_intruder")
        self.robot_name = os.getenv("MIRO_ROBOT_NAME", "miro01")
        topic = f"/{self.robot_name}/platform/control/cmd_vel"
        self.cmd_pub = rospy.Publisher(topic, TwistStamped, queue_size=1)

        print(f"[INFO] Intruder node started for {self.robot_name} → publishing to {topic}")

        self.state = STATE_WANDER
        self.wander_pattern = [(0.05, 0.0)] * 30 + [(0.0, -0.5)] * 10 + [(0.05, 0.0)] * 25 + [(0.0, 0.5)] * 10
        self.step = 0

    def tick(self):
        if self.step >= len(self.wander_pattern):
            self.step = 0
        lin, ang = self.wander_pattern[self.step]
        self.publish_movement(lin, ang)
        # print(f"[DEBUG] step {self.step}: lin={lin}, ang={ang}")
        self.step += 1

    def publish_movement(self, linear, angular):
        msg = TwistStamped()
        msg.header.stamp = rospy.Time.now()
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        self.cmd_pub.publish(msg)

if __name__ == "__main__":
    node = MiRoIntruder()
    rate = rospy.Rate(1.0 / node.TICK)
    while not rospy.is_shutdown():
        node.tick()
        rate.sleep()
