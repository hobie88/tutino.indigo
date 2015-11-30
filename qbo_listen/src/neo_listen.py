#!/usr/bin/env python
# -*- coding: utf-8 -*-


#********************************************************************
#
#
#                 QBO LISTEN USING POCKETSPHINX
#
#                REALLY USEFULL FOR VOCAL ORDERS
#
#              USE LISTEN_GOOGLE NODE FOR BETTER DIC
#
#                     elpimous12@orange.fr
#
#
#********************************************************************

import rospy
import subprocess
import pocketsphinx.pocketsphinx as ps
import sphinxbase
from std_msgs.msg import String
import commands
from os import path
import sys


# record location
audio_file = '/tmp/recording.wav' 

# recording process, use of SOX, 1 channel, rate 16000, default qbo audio config
# silence used to create blank sounds before and after record
record = 'sox -c 1 -r 48000 -t alsa default "'+audio_file+'" silence 1 0.1 1% 1 1.5 1%' 
MODELDIR = "/opt/ros/indigo/catkin_ws/src/qbo_listen/model"

class recognizer(object):
    
    def __init__(self):

        rospy.init_node("qbo_listen")
        config = ps.Decoder.default_config()
        config.set_string('-hmm', path.join(MODELDIR, 'en-us/en-us'))
        config.set_string('-lm', path.join(MODELDIR, 'en-us/en-us.lm.bin'))
        config.set_string('-dict', path.join(MODELDIR, 'en-us/cmudict-en-us.dict'))

        while not rospy.is_shutdown():

            def decodeSpeech(decoder_config, audio_file):

                # pocketsphinx wav recognition process. Do not modify !
                
                
                speechRec = ps.Decoder(decoder_config)

                subprocess.call(record, shell=True)
                stream = open(audio_file, 'rb')
                in_speech_bf = True
                speechRec.start_utt()
                
                while True:
                    buf = stream.read(1024)
                    #audio_file2 = file(audio_file,'rb')
                    #audio_file2.seek(44)
                    #if audio_file2:
                    #    speechRec.process_raw(audio_file2,False,False)
                    #    #speechRec.decode_raw(audio_file2)
                    if buf:
                        speechRec.process_raw(buf,False,False)
                        try:
                            if speechRec.hyp().hypstr != '':
                                print "**************PARTIAL decoding reult:", speechRec.hyp().hypstr
                        except AttributeError:
                            pass
                        if speechRec.get_in_speech():
                            sys.stdout.write('.')
                            sys.stdout.flush()
                        if speechRec.get_in_speech() != in_speech_bf:
                            in_speech_bf = speechRec.get_in_speech()
                            if not in_speech_bf:
                                speechRec.end_utt()
                                try:
                                    if speechRec.hyp().hypstr != '':
                                        print 'Stream decoding result:', speechRec.hyp().hypstr
                                except AttributeError:
                                    pass
                                speechRec.start_utt()
                                     
                    else:
                        break
                speechRec.end_utt()
                result = speechRec.hyp()
                return result 

            recognised = decodeSpeech(config,audio_file)
            recognised = str(recognised) # to avoid python error on printing "none object"
            print""
            print'        Ai-je bien compris : "'+recognised+'" ?...'
            print""

            # publishing sentence in "/listened" topic
            pub = rospy.Publisher('listened', String, queue_size=10)
            pub.publish(recognised)
            continue

if __name__ == "__main__":
    start = recognizer()

