#!/usr/bin/env python

from sound_play.libsoundplay import SoundClient
from sound_play.msg import SoundRequest
import rospy

def sound_ready(msg):
    print("ready")

rospy.init_node('sound')

ready = rospy.Subscriber("robotsound", SoundRequest, sound_ready)


soundthing = SoundClient()
soundthing.playWave("~/winter18/Robotics/Competitions/Comp4/meow.mp3")