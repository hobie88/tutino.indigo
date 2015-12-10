#!/usr/bin/env python
# coding: utf-8
# Author: Vincent FOUCAULT
# Here is a text-to-speech wrapper for Espeak
# It's part of QBO robot ros softwares


import rospy
import roslib
from qbo_talk.srv import Text2Speach # read service content (words we want to be spoken)
import os
import subprocess
from std_msgs.msg import Bool
from random import choice

# Some possibilities of different languages
#fr1_speak = "espeak -a 70 -s 140 -p50 -v mb/mb-fr1 \"%s\" | mbrola -e -C \"n n2\" /usr/share/mbrola/voices/fr1 - -.au | paplay"
#en1_speak = "espeak -a 70 -s 140 -p50 -v mb/mb-en1 \"%s\" | mbrola -e -C \"n n2\" /usr/share/mbrola/voices/en1 - -.au | paplay"
#fr1_speak = "padsp swift \"%s\""
fr1_speak = "pico2wave -l fr-FR -w test.wav \"%s\" && play test.wav"
en1_speak = "pico2wave -l en-GB -w test.wav \"%s\" && play test.wav"
it1_speak = "pico2wave -l it-IT -w test.wav \"%s\" && play test.wav"


def run_process(command = ""):
    if command != "":
        return os.system(command)
    else:
        return -1
            
            
class talk():

    def __init__(self):

        rospy.init_node('talk', anonymous=True)
        fr1 = rospy.Service('say_fr1', Text2Speach, self.fr1_talk)
        en1 = rospy.Service('say_en1', Text2Speach, self.en1_talk)
    	it1 = rospy.Service('say_it1', Text2Speach, self.it1_talk)
	rospy.Subscriber('/face_detected',Bool,self.callback)
        

    #TODO  Dic. words fo gain better words sounds ! 

    def callback(self, msg):
	sentence=choice(["Tanti auguri di buon natale","ciao, come va?","heilà, come stai?","ciao umano, come va?","salve, come sta?","ciao io sono tutìno ", "ciao collega!"])
	run_process("amixer -c 1 sset Mic,1 0%")
	os.system(it1_speak % sentence)
	run_process("amixer -c 1 sset Mic,1 75%")

    def it1_talk(self, speak):
        run_process("amixer -c 1 sset Mic,1 0%")
        os.system(it1_speak % speak.sentence) 
        run_process("amixer -c 1 sset Mic,1 75%")
    
    def fr1_talk(self, speak): 
        # si speak.sentence.find
        run_process("amixer -c 1 sset Mic,1 0%")
        os.system(fr1_speak % speak.sentence) 
        run_process("amixer -c 1 sset Mic,1 75%")
        return []

    def en1_talk(self, speak):
	run_process("amixer -c 1 sset Mic,1 0%")
        os.system(en1_speak % speak.sentence)
	run_process("amixer -c 1 sset Mic,1 75%")
        return []

if __name__ == '__main__':
    try:
        talk = talk()
        rospy.spin()
    except rospy.ROSInterruptException: pass


