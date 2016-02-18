#!/usr/bin/env python

#ROS Import
import ros
import roslib
import rospy
from std_msgs.msg import String
from std_msgs.msg import UInt8

from sys import byteorder
from array import array
from struct import pack

import pyaudio
import wave
import speech_recognition as sr
import time
from collections import  deque
import threading
import signal
import sys
from gi.overrides.Gdk import Color


THRESHOLD = 500
CHUNK_SIZE = 1024
RATE = 48000
LANGUAGE = "it"
maxAudioLen = 2
GOOGLE_KEY="AIzaSyApW6YIzdZzA5b2Ttc4qp0goVLPQg5z_xQ"

def isSilent(sndData):
    return max(sndData) < THRESHOLD
    
def normalize(sndData):
    MAXIMUM = 16384
    times = float(MAXIMUM) / max(abs(i) for i in sndData)
    r = array('h')
    for i in sndData:
        r.append(int(i*times))
    return r


#
#
#
GREEN = 2
RED = 1
OFF = 0
audioQueue = deque()
stringList = []
recordFlag = True
recogFlag = True

def record(recog):
    global audioQueue
    while recordFlag:
        if (len(audioQueue) < maxAudioLen):
            #print("\nRecording:")
            with sr.Microphone() as source:
                try:
                         #audio = recog.record(source,3)
                    audio = recog.record(source,8)    #Better than listen
                    print("Recording end \n")
                    audioQueue.append(audio)
                except:
                    pass
        else:
            continue
#        else:
#            print("I am not listening")

def noseCmd(color):
    msg=UInt8()
    msg.data=color
    nose_pub.publish(msg)        

def recognizeAudio(recog):
    global audioQueue
    global stringList
    while recogFlag:
        if len(audioQueue) > 0:
            print("Trying to Understand")
            try:
                audioBuffer = audioQueue.popleft()
                #print("queue len: " + str(len(audioQueue)))
                decodedString = recog.recognize_google(audioBuffer, language = LANGUAGE, key=GOOGLE_KEY)
            except sr.UnknownValueError:    #Audio not recognized
                decodedString = ""
                noseCmd(OFF)
            except sr.RequestError:    #Key or Internet-bound error
                decodedString = ""
                noseCmd(OFF)
            except IndexError:    #AudioQueue is empty but for some reason we got here
                time.sleep(1)    #Wait for the Record thread to give some data
                decodedString = ""
                noseCmd(OFF)
            if decodedString:
                stringList.append(decodedString)    #Store decoded Strings into a list
                rospy.logerr("decoded: "+decodedString)
                noseCmd(GREEN)
            cmd = analyze(decodedString)    #If decoded String is meaningful
            pub.publish(cmd)
            
def stopAll():
    '''
    It is basically a loop that stops the execution of the overall node when some keywords are recognized
    '''
    global recogFlag
    global recordFlag
    counter = 0
    while recordFlag or recogFlag:
        for element in stringList[counter:]:
            counter += 1    #So this thread checks only unchecked elements
           # if ("fermati" in element) or ("cancaro" in element) or ("stop" in element):
            if "cancaro" in element:
                recordFlag = False    #Stop Recording man
                while len(audioQueue) > 0:    #Until buffer is > 0, work!
                    pass
                recogFlag = False
            with open("strings.txt", "a") as buffText:
                for element in stringList:
                    buffText.write("%s - %d \n" %(element,time.time()))
               
            
def analyze(buffString):   
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
    rospy.init_node("qbo_listener",anonymous=True)
    pub = rospy.Publisher("vocal_move",String,queue_size=3)
    nose_pub = rospy.Publisher("/cmd_nose",UInt8,queue_size=2)
    r = sr.Recognizer()
    with sr.Microphone() as source:
        print("Adjusting Mic Level")
        r.adjust_for_ambient_noise(source,2)
        print("Adjustment Done!")
    #startTime = time.time()
    recordThread = threading.Thread(target=record, args=(r,))
    recordThread.start()
    recognizerThread = threading.Thread(target=recognizeAudio, args=(r,))
    recognizerThread.start()
    stopAllThread = threading.Thread(target=stopAll)
    stopAllThread.start()
    signal.signal(signal.SIGINT, sys.exit(0))
    
