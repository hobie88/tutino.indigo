#!/usr/bin/env python

#ROS Import
import ros
import roslib
import rospy
from std_msgs.msg import String
from std_msgs.msg import UInt8

from sys import byteorder
import sys
from array import array
from struct import pack

import pyaudio
import wave
import speech_recognition as sr
import time
from collections import  deque
#import threading
import thread
import signal
import os


THRESHOLD = 500
CHUNK_SIZE = 1024
RATE = 48000
LANGUAGE = "it"
maxAudioLen = 2
GOOGLE_KEY="AIzaSyApW6YIzdZzA5b2Ttc4qp0goVLPQg5z_xQ"
GREEN = 2
RED = 1
OFF = 0


class listener(object):
    
    audioQueue = deque()
    stringList = []
    recordFlag = True
    recogFlag = True
        
    def __init__(self,language="en-US"):
        self.lang = language
        rospy.init_node("qbo_listener",anonymous=True)
        self.pub = rospy.Publisher("vocal_move",String,queue_size=3)
        self.nose_pub = rospy.Publisher("/cmd_nose",UInt8,queue_size=2)
        self.r = sr.Recognizer()
        with sr.Microphone() as source:
            print("Adjusting Mic Level")
            self.r.adjust_for_ambient_noise(source,2)
            print("Adjustment Done!")
        #startTime = time.time()
        thread.start_new_thread(self.record,(self.r,))
        while True:
            try:
                self.recognizeAudio(self.r)
            except KeyboardInterrupt:
                sys.exit(0)

#    def isSilent(self,sndData):
#        return max(sndData) < THRESHOLD
    
#    def normalize(self,sndData):
#        MAXIMUM = 16384
#        times = float(MAXIMUM) / max(abs(i) for i in sndData)
#        r = array('h')
#        for i in sndData:
#            r.append(int(i*times))
#        return r



    def record(self,recog):
        while self.recordFlag:
            if (len(self.audioQueue) < maxAudioLen):
                #print("\nRecording:")
                with sr.Microphone() as source:
                    try:
                             #audio = recog.record(source,3)
                        audio = recog.record(source,8)    #Better than listen
                        print("Recording end \n")
                        self.audioQueue.append(audio)
                    except:
                        pass
            else:
                continue
    #        else:
    #            print("I am not listening")

    def noseCmd(self,color):
        msg=UInt8()
        msg.data=color
        self.nose_pub.publish(msg)        

    def recognizeAudio(self,recog):
        while self.recogFlag:
            if len(self.audioQueue) > 0:
                print("Trying to Understand")
                try:
                    audioBuffer = self.audioQueue.popleft()
                    #print("queue len: " + str(len(audioQueue)))
                    decodedString = recog.recognize_google(audioBuffer, language = self.lang, key=GOOGLE_KEY)   #LANGUAGE global var was used before
                except sr.UnknownValueError:    #Audio not recognized
                    decodedString = ""
                    self.noseCmd(OFF)
                except sr.RequestError:    #Key or Internet-bound error
                    decodedString = ""
                    self.noseCmd(OFF)
                except IndexError:    #AudioQueue is empty but for some reason we got here
                    time.sleep(1)    #Wait for the Record thread to give some data
                    decodedString = ""
                    self.noseCmd(OFF)
                if decodedString:
                    self.stringList.append(decodedString)    #Store decoded Strings into a list
                    rospy.logerr("decoded: "+decodedString)
                    self.noseCmd(GREEN)
                cmd = self.analyze(decodedString)    #If decoded String is meaningful
                self.pub.publish(cmd)
            
    def stopAll(self):
        '''
        It is basically a loop that stops the execution of the overall node when some keywords are recognized
        '''
        counter = 0
        while self.recordFlag or self.recogFlag:
            for element in self.stringList[counter:]:
                counter += 1    #So this thread checks only unchecked elements
               # if ("fermati" in element) or ("cancaro" in element) or ("stop" in element):
                if "cancaro" in element:
                    self.recordFlag = False    #Stop Recording man
                    while len(self.audioQueue) > 0:    #Until buffer is > 0, work!
                        pass
                    self.recogFlag = False
                with open("strings.txt", "a") as buffText:
                    for element in self.stringList:
                        buffText.write("%s - %d \n" %(element,time.time()))
               
            
    def analyze(self,buffString):   
        '''
        Input:
            String buffString: It is the decoded string given by the SRE.
        Return:
            String being the cmd that is going to be published on ROS
        ''' 
        if ("gira" in buffString):
            if ("destra" in buffString):
                #gira a destra
                command = "destra"
            elif ("sinistra" in buffString):
                #gira a sinistra
                command = "sinistra"
        elif ("girati" in buffString) or ("voltati" in buffString):
            #gira di 180
            command = "180"
        elif ("dietro" in buffString):
            #vai indietro
            command = "indietro"
        elif ("avanti" in buffString):
            #vai avanti
            command = "avanti"
        elif (("stop" in buffString)or("fermati" in buffString)):
            command = "stop"
        else:
            #non e' un comando valido
            return ""
        return command
   
    
if __name__ == "__main__":
    #system cannot start jack server autonoumously
    os.system("pulseaudio --kill")
    os.system("jack_control start")
    a = listener()
    signal.signal(signal.SIGINT, sys.exit(0))
