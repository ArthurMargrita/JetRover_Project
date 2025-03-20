#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
import os

def callback(msg):
    rospy.loginfo("Lecture du message : {}".format(msg.data))
    os.system(f'espeak "{}"'.format(msg.data))

def text_to_speech_node():
    rospy.init_node('text_to_speech')
    rospy.Subscriber('speech', String, callback)
    rospy.spin()

if __name__ == '__main__':
    text_to_speech_node()
