#!/usr/bin/env python

from sound_play.libsoundplay import SoundClient
# from sound_play.msg import SoundRequest
import rospy
import time

rospy.init_node('sound')

soundthing = SoundClient()
time.sleep(1)
# soundthing.play(SoundRequest.NEEDS_UNPLUGGING)
# soundthing.voiceSound("Testing the new A P I")
soundthing.playWave("/home/mandy/winter18/Robotics/Competitions/Comp4/meow.ogg")
print("meow sound started")

time.sleep(3)

soundthing.playWave("/home/mandy/winter18/Robotics/Competitions/Comp4/moo.ogg")
print("woof sound started")