#!/usr/bin/env python3
# -*- coding: utf-8 -*-


# Imports
import rospy
from sound_play.libsoundplay import SoundClient
import os



class MiRoSpeaker:
    """
    A simple class to make MiRo speak
    """

    def __init__(self):
        rospy.init_node("miro_speaker", anonymous=True)
        self.robot_name = "/" + os.getenv("MIRO_ROBOT_NAME", "miro01")
        self.sound_client = SoundClient()
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME", "miro")
        rospy.sleep(1)
        print("MiRo speaker initialized. Ready to speak!")

    def say(self, text):
        """
        Make MiRo say the provided text
        """
        print(f"MiRo says: {text}")
        self.sound_client.say(text)
        rospy.sleep(3)


if __name__ == "__main__":
    speaker = MiRoSpeaker()
    rate = rospy.Rate(1.0 /0.02 )
    while not rospy.is_shutdown():
        speaker.say("hello im a miro")
        rate.sleep()


        


