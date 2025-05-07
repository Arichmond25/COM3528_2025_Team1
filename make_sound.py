#!/usr/bin/env python3


# Imports
import rospy
from sound_play.libsoundplay import SoundClient
import os
import sys


class MiRoSpeaker:
    """
    A simple class to make MiRo speak
    """

    def __init__(self):
        rospy.init_node("miro_speaker", anonymous=True)
        self.sound_client = SoundClient()
        rospy.sleep(1)
        print("MiRo speaker initialized. Ready to speak!")

    def say(self, text):
        """
        Make MiRo say the provided text
        """
        print(f"MiRo says: {text}")
        self.sound_client.say(text)
        rospy.sleep(3)


# if __name__ == "__main__":
#     try:

#         # speaker = MiRoSpeaker()


#         # # Make MiRo say the text
#         # speaker.say(text)

#     except rospy.ROSInterruptException:
#         pass
