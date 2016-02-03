#!/usr/bin/env python3

# NOTE: this example requires PyAudio because it uses the Microphone class

import speech_recognition as sr
from espeak import espeak

import pyaudio


if espeak.set_voice("mb-it4"):
    canSpeak = True
else:
    canSpeak = False
    
def speakTutino(string):
    if canSpeak:
        espeak.synth(buffString)
    else:
        print(buffString)

r = sr.Recognizer()
print("mic and recognizer initialized")
with sr.Microphone() as source:
    r.adjust_for_ambient_noise(source,2) # we only need to calibrate once, before we start listening
    
buffString = ""  
i=0
while i<5:
    with sr.Microphone() as source:
        audio = r.record(source,3)
    
    print("Recognizing...")
    try:
        buffString = r.recognize_google(audio, language='it')
    except sr.UnknownValueError:    #Audio cannot be recognized by the used SRE
        buffString = "No audio"
        pass
    except sr.RequestError: #key is invalid
        buffString = "Used Key is invalid!"
        pass
   
    if "ciao" in buffString:
        print(buffString)
        break
    else:
        print(buffString)

