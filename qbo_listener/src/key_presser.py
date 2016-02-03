#!/usr/bin/env python

import rospy
import roslib
import os
import subprocess
from std_msgs.msg import String

#Speech Recognition libs
import speech_recognition as sr

#Constants definition
INFILE_WAV = 'presser.wav'
REC = "rec -r 48000 -c 1 " + INFILE_WAV + " silence 0 1 0:00:01 2% silence 0 1 00:00:03 0%"


def listener():
    pub = rospy.Publisher("key_pressed",String,queue_size=1)
    rospy.init_node("key_logger",anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        currentString = recordAndListenAudio()
        if keywordEn(currentString):
            rospy.loginfo("Keyword Trovata!")
            pub.publish(currentString)
        rate.sleep()
        
        
def recordAndListenAudio():
    rospy.loginfo("Recording audio...")
    os.system(REC)
    rospy.loginfo("Audio Recorded!")
    rospy.loginfo("Sample stored into %s" %INFILE_WAV)
    r = sr.Recognizer()
    with sr.WavFile(INFILE_WAV) as source:
        audioBuffer = r.record(source)
    try:
        decodedString = r.recognize_google(audioBuffer, language = 'it')
    except:
        decodedString = "no keyword"
    return decodedString
    
def keywordEn(bufString):
    if "ciao" in bufString:
        return True
    else:
        return False

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
    

