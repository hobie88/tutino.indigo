#!/usr/bin/env python
# -*- coding: utf-8 -*-


#********************************************************************
#
#
#        ARTIFICIAL INTELLIGENCY WITH PY-AIML (ALICE STYLE)
#
#                 KEYBOARD INPUTS FOR QUESTIONS
#
#                   ANSWER SPOKEN WITH ESPEAK
#
#                    elpimous12@orange.fr
#
#
#********************************************************************

"""
	For english :
		- l41 (/say_en1)
		- l63 (ai_en)

"""

import rospy
from qbo_talk.srv import Text2Speach
from std_msgs.msg import String
import aiml
import sys

def speak_this(text): # for qbo_talk
    global client_speak
    client_speak(str(text))

class Neo_Chat():

    def __init__(self):
	global client_speak
        rospy.Subscriber('listened', String, self.listen_callback)
        client_speak = rospy.ServiceProxy("/say_en1", Text2Speach)
        rospy.init_node('neo_chat_node', anonymous=True)
        self.NEO = aiml.Kernel()# charge le moteur de l'IA
        self.brainLoaded = False
        self.forceReload = False

	# Données personnelles du robot, comme son nom, ses préférences...
        self.NEO.setBotPredicate('name', "Tutino")
        self.NEO.setBotPredicate('age' ,"18 years old")
        self.NEO.setBotPredicate('compleanno', "october 2015")
        self.NEO.setBotPredicate('gender', "male")
        self.NEO.setBotPredicate('master', "Marco")
        self.NEO.setBotPredicate('favoritebook', "Do androids dream of electronic ships")
        self.NEO.setBotPredicate('favcolor', "bleu")
        self.NEO.setBotPredicate('location' ,"Turin")
        self.NEO.setBotPredicate('sign', "taureau")
        self.NEO.setBotPredicate('favoritefood', "circuits")
        self.NEO.setBotPredicate('favmovie', "Modern times")
        self.NEO.setBotPredicate('favoriteband', "rush")
        self.NEO.setBotPredicate('kindmusic', "electronic")
        self.NEO.setBotPredicate('birthplace' ,"Italy, turin tech")

        while not self.brainLoaded:
	    if self.forceReload or (len(sys.argv) >= 2 and sys.argv[1] == "reload"):
		    self.NEO.bootstrap(learnFiles="/home/neo/catkin_ws/src/qbo_ai/ai_en/*.aiml")# charge tous les fichiers aiml
		    self.brainLoaded = True
		    # backup des differents fichiers aiml dans un fichier compressé, pour un chargement éclair les fois suivantes
		    self.NEO.saveBrain("/home/neo/catkin_ws/src/qbo_ai/NEO.brn")
	    else:

		    try:
			    # chargement rapide des fichiers aiml, si existance du fichier compressé
			    self.NEO.bootstrap(brainFile = "/home/neo/catkin_ws/src/qbo_ai/NEO.brn")
			    self.brainLoaded = True
		    except:
			    self.forceReload = True

            while True:
	        reponse = self.NEO.respond(raw_input("Ask me something : "))# A commenter pour tester la reco. vocale
	        #reponse = self.NEO.respond(msg.data)# récupère la séquence ententue via qbo_listen et interroge le moteur d'IA
                speak_this(reponse)# si une réponse est trouvée, elle est exécutée oralement
	        print "réponse : ",reponse


    def listen_callback(self, msg):
        print "question = ",msg.data



if __name__ == '__main__':
    try:
        Neo_Chat()
        rospy.spin()
    except rospy.ROSInterruptException: pass
llo
