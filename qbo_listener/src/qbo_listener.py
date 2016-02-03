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
    pub = rospy.Publisher("vocal_move",String,queue_size=1)
    rospy.init_node("qbo_listener",anonymous=True)
    r = sr.Recognizer()
    command = ""
    with sr.Microphone() as source:
        r.adjust_for_ambient_noise(source,2)    #calibrate recognizer
        
#   file = open("phrases.txt","w")    
    
    while not rospy.is_shutdown():
        with sr.Microphone() as source:
            audio = r.record(source, 3) # record for 3 seconds
        print("Recognizing...")
        try:
            buffString = r.recognize_google(audio, language='it')
        except sr.UnknownValueError:    #Audio cannot be recognized by the used SRE
            buffString = "No audio"
            pass
        except sr.RequestError: #key is invalid
            buffString = "Used Key is invalid!"
            pass
#        file.write(buffString+"\n")
#        file.flush()
        buffString = buffString.lower() #lower case to all the chars
        print(buffString)
        if "tutino" in buffString:
            if analyze(buffString):
                pub.publish(command)
                print ("publishing: " + command)
#                file.close()
            

def analyze(buffString):    
    if ("gira" in buffString):
        if ("destra" in buffString):
            #gira a destra
            command = "destra"
            return True
        elif ("sinistra" in buffString):
            #gira a sinistra
            command = "sinistra"
            return True
    elif ("girati" in buffString) or ("voltati" in buffString):
        #gira di 180
        command = "180"
        return True
    elif ("indietro" in buffString):
        #vai indietro
        command = "indietro"
        return True
    elif ("avanti" in buffString):
        #vai avanti
        command = "avanti"
        return True
    else:
        #non e' un comando valido
        return False
   

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
    

