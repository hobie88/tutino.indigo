#!/usr/bin/env python

#ROS Import
import ros
import roslib
import rospy
from std_msgs.msg import String

from sys import byteorder
from array import array
from struct import pack

import pyaudio
import wave
import speech_recognition as sr
import time
from collections import  deque
import threading


THRESHOLD = 500
CHUNK_SIZE = 1024
RATE = 48000
LANGUAGE = "it"

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

audioQueue = deque()
stringList = []
recordFlag = True
recogFlag = True

def record(recog):
    while recordFlag:
        print("Recording:")
        if (len(audioQueue) < 4):
            with sr.Microphone() as source:
                try:
                    #audio = recog.record(source,3)
                    audio = recog.record(source,5)    #Better than listen
                    audioQueue.append(audio)
                except:
                    pass
        else:
            print("I am not listening")
        

def recognizeAudio(recog):
    while recogFlag:
        print("Trying to Understand")
        try:
#            audioBuffer = audioQueue.pop()
            audioBuffer = audioQueue.popleft()
            decodedString = recog.recognize_google(audioBuffer, language = LANGUAGE)
        except sr.UnknownValueError:    #Audio not recognized
            decodedString = False
        except sr.RequestError:    #Key or Internet-bound error
            decodedString = False
        except IndexError:    #AudioQueue is empty but for some reason we got here
            time.sleep(1)    #Wait for the Record thread to give some data
            decodedString = False
        if decodedString:
            stringList.append(decodedString)    #Store decoded Strings into a list
            cmd = analyze(decodedString)    #If decoded String is meaningful
            pub.publish(cmd)
            
def stopAll():
    global recogFlag
    global recordFlag
    counter = 0
    while recordFlag or recogFlag:
        for element in stringList[counter:]:
            counter += 1    #So this thread checks only unchecked elements
            if ("fermati" in element) or ("cancaro" in element) or ("stop" in element):
                recordFlag = False    #Stop Recording man
                while len(audioQueue) > 0:    #Until buffer is > 0, work!
                    pass
                recogFlag = False
                with open("strings.txt", "a") as buffText:
                    for element in stringList:
                        buffText.write("%s - %d \n" %(element,time.time()))
               
            
def analyze(buffString):    
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
    elif ("indietro" in buffString):
        #vai indietro
        command = "indietro"
    elif ("avanti" in buffString):
        #vai avanti
        command = "avanti"
    else:
        #non e' un comando valido
        return ""
    return command
   
    
if __name__ == "__main__":
    pub = rospy.Publisher("vocal_move",String,queue_size=3)
    rospy.init_node("qbo_listener",anonymous=True)
    r = sr.Recognizer()
    with sr.Microphone() as source:
        print("Adjusting Mic Level")
        r.adjust_for_ambient_noise(source,2)
        print("Adjustment Done!")
    startTime = time.time()
    recordThread = threading.Thread(target=record, args=(r,))
    recordThread.start()
    recognizerThread = threading.Thread(target=recognizeAudio, args=(r,))
    recognizerThread.start()
    stopAllThread = threading.Thread(target=stopAll)
    stopAllThread.start()
    
