#!/usr/bin/env python

from sound_play.libsoundplay import SoundClient
from sound_play.msg import SoundRequest
import rospy
import time

rospy.init_node('sound')

soundthing = SoundClient()
time.sleep(1)

soundthing.play(SoundRequest.NEEDS_UNPLUGGING)
time.sleep(1)
soundthing.say("I found mandy and michele", "voice_kal_diphone")
time.sleep(2)
