#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import os, subprocess, shlex

import speech_recognition as sr
#from __future__ import unicode_literals


INFILE = 'listener.wav'
RATE = '48000'

LANGUAGE = 'it'
KEY = "AIzaSyCmwVC1h5TasF6NhCV2CwZUq4bFLRrCx9A"
ALT_KEY="AIzaSyAcalCzUvPmmJ7CZBFOEWx2Z1ZSn4Vs1gg"
BROWSE_KEY="AIzaSyBQj_nPHkFfL1ovmlwLQUU7si5dbnX_6io"
REC = "rec -r " + RATE + " -c 1 " + INFILE + " silence 0 1 0:00:01 2% silence 0 1 00:00:03 0%"
#RESULT = 'wget -O - --post-file '+INFILE+ ' --header="Content-Type: audio/wav; rate=16000" "'+URL+'" > '+OUTFILE


def callback(data):
    rospy.loginfo("Recording audio...")
    os.system(REC)
    rospy.loginfo("Audio Recorded!")
    rospy.loginfo("Sample stored into %s" %INFILE)
    r = sr.Recognizer()
    with sr.WavFile(INFILE) as source:
        audioBuffer = r.record(source)
    try:
        decodedString = r.recognize_google(audioBuffer, language = LANGUAGE)
    except:
        decodedString = False
    if decodedString:
        print(decodedString)
    else:
        print("niente ce")
    
    
def listener():
    rospy.init_node('qbo_listener')
    rospy.Subscriber("key_pressed", String, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    
if __name__ == "__main__":
    start = listener()
