#!/usr/bin/env python
# coding: utf-8
"""
A ROS-HYDRO/INDIGO CATKIN FACE LEARNING PROGRAM :
Vincent FOUCAULT / Avril-2015
"""

import rospy
import smach # smach ready for use
import smach_ros
import os
import subprocess
import random
from actionlib import *
from actionlib.msg import *
from random import choice
from qbo_arduqbo.msg import Nose # for nose color modif.
from std_msgs.msg import String
from std_msgs.msg import UInt8
from qbo_talk.srv import Text2Speach
from time import sleep
from face_recognition.msg import * #FaceRecognitionAction, FaceRecognitionGoal, FaceRecognitionResult # for actionlib

def run_process(command = ""):
    if command != "":
        return subprocess.Popen(command.split())
    else:
        return -1

# to recover informations in terminal view
def runCmdOutput(cmd, timeout=None):
    ph_out = None # process output
    ph_err = None # stderr
    ph_ret = None # return code
    p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if not timeout:
        ph_ret = p.wait()
    else:
        fin_time = time.time() + timeout
        while p.poll() == None and fin_time > time.time():
            time.sleep(1)
        if fin_time < time.time():
            os.kill(p.pid, signal.SIGKILL)
            raise OSError("Process timeout has been reached")
        ph_ret = p.returncode
    ph_out, ph_err = p.communicate()
    return ph_out


# international alphabet used for name
class SAY_YOUR_NAME_AND_VALIDATE(smach.State):
  heardname = ""
  old_voice = True
  def __init__(self):
    smach.State.__init__(self, outcomes=['next_state'])
    self.client_speak = rospy.ServiceProxy("/say_fr1", Text2Speach)
    rospy.Subscriber('listened', String, self.listen_callback)
    self.goal_states = ['RECALLING', 'REJECTED', 'ABORTED', 'SUCCEEDED']
    self.nose_pub = rospy.Publisher('/cmd_nose',UInt8 , queue_size=1)
    self.nose = 0
    self.noseLearn = False
    self.heard = ""
    self.state = "none"
    self.first_try = True
    self.learningFinished = False
    self.learning = actionlib.SimpleActionClient("qbo_face_recognition", FaceRecognitionAction)

# robot voice
  def speak_this(self,text):
    self.client_speak(str(text))


# robot listening
  def listen_callback(self, msg):
      self.heard = msg.data
      rospy.loginfo("*****Call back called******")
      rospy.loginfo( self.heard)


# name creation with international alphabet listening
  def myName(self):
          self.state = "myName"
          conversion = {'alpha':'a','bravo':'b','charly':'c','delta':'d','echo':'e','foxtrot':'f','golf':'g','hotel':'h','india':'i','juliet':'j','kilo':'k','lima':'l','mike':'m','novembre':'n','oscar':'o','papa':'p','quebec':'q','roméo':'r','sierra':'s','tango':'t','uniform':'u','victor':'v','whisky':'w','x_ray':'x','yankee':'y','zoulou':'z',}

#          for i, j in conversion.iteritems():
#	    SAY_YOUR_NAME_AND_VALIDATE.heardname = (self.heard).replace('alpha','a').replace('bravo','b').replace('charly','c').replace('delta','d').replace('echo','e').replace('foxtrot','f').replace('golf','g').replace('hotel','h').replace('india','i').replace('juliet','j').replace('kilo','k').replace('lima','l').replace('mike','m').replace('november','n').replace('oscar','o').replace('papa','p').replace('quebec','q').replace('roméo','r').replace('sierra','s').replace('tango','t').replace('uniform','u').replace('victor','v').replace('wkisky','w').replace('x-ray','x').replace('yankee','y').replace('zoulou','z').replace(' ','').replace('terminé','') # replace exemple : "alpha x-ray echo lima" => "axel"
	  SAY_YOUR_NAME_AND_VALIDATE.heardname = self.heard
#            self.speak_this(choice(["Alors, ton nom est, "+SAY_YOUR_NAME_AND_VALIDATE.heardname+"? Correct?","Tu tapelle "+SAY_YOUR_NAME_AND_VALIDATE.heardname+"? C'est ça?","Il me faut un peu de temps, ça y est, "+SAY_YOUR_NAME_AND_VALIDATE.heardname+". J'ai bon?"]))
#            break
          rospy.loginfo(SAY_YOUR_NAME_AND_VALIDATE.heardname)


  def Apprentissage(self):

        self.state = "Apprentissage"
        self.noseYellow()
        if self.first_try == True: # tuto for first time
          if SAY_YOUR_NAME_AND_VALIDATE.old_voice == True:
#            voix_synthetique = ("voix_synthetique_1","voix_synthetique_2","voix_synthetique_3")
#            os.system("play /opt/ros/indigo/catkin_ws/src/qbo_face_operations/wav/"+(random.choice(voix_synthetique))+".wav")#TODO os.system ???
            sleep(0.5)
#          self.speak_this("épelle ton prénom, avec l'alphabet international, puis dit, terminé. ")
          self.speak_this(choice(["Tocca a te.","Quando dici tu.","Io sono pronto.","Andiamo.","ti ascolto.","aspetto te."]))

        if self.first_try == False: # no more tuto, direct loop
          self.speak_this(choice(["Oh, cavolo, si ricomincia,","Non ho capito . ripeti,","Riproviamoci"]))
          self.speak_this(choice(["e non dimenticare di dire finito","e cerca di dire pure finito alla fine"]))



  def non(self):
        self.first_try = False
        self.Apprentissage()


# NOSE fonctions, for some visual help
  def noseLearn(self):
     while not rospy.is_shutdown():
        self.nose= 1
        self.nose_pub.publish (self.nose)
        sleep(1)
        self.nose= 0
        self.nose_pub.publish (self.nose)
        sleep(0.3)
  def noseRed(self):
        self.nose= 1
        self.nose_pub.publish (self.nose)
  def noseGreen(self):
        self.nose= 2
        self.nose_pub.publish (self.nose)
  def noseYellow(self):
        self.nose= 3
        self.nose_pub.publish (self.nose)
  def noseOff(self):
        self.nose.data= 0
        self.nose_pub.publish (self.nose)


  def execute(self, userdata):
    rospy.loginfo('Executing state SAY_NAME') 

    rosnode_list = runCmdOutput("rosnode list")
    if rosnode_list.find("qbo_listen") > -1:
      run_process("rosnode kill /qbo_listen")
      rospy.sleep(1)
#    run_process("roslaunch qbo_listen face_learning.launch")
    run_process("rosrun qbo_face_operations keyboard_input.py") 

    self.Apprentissage()
    while not rospy.is_shutdown():
      if self.state == "Apprentissage":
        if self.heard.find("finito") > -1: # reconstruct name with internat. alphabet
            self.noseGreen()
            self.myName() 
            self.heard =""
      
      if self.state == "myName":
        if self.heard.find("si") > -1: # first learning
          self.speak_this(choice(["Ok "+SAY_YOUR_NAME_AND_VALIDATE.heardname+", lerning in corso, tieni d'occhio il mio naso, e aspetta il messaggio finale",SAY_YOUR_NAME_AND_VALIDATE.heardname+", bel nome. Guardami negli occhi e attendi che ti parli",SAY_YOUR_NAME_AND_VALIDATE.heardname+", Credo che sei stato già registrato, Guarda il mio naso e attendi."]))
          self.heard =""
          while not rospy.is_shutdown():
              self.learningGoal = FaceRecognitionGoal()
              self.learningGoal.order_id = 2
              self.learningGoal.order_argument=SAY_YOUR_NAME_AND_VALIDATE.heardname
              self.learning.send_goal(self.learningGoal)
	      self.learning.wait_for_result() ############################### marco edit (added)
              state = self.learning.get_state()

              while not str(self.goal_states[state]) == "SUCCEEDED":
                self.nose= 1
                self.nose_pub.publish (self.nose)
                sleep(1)
                self.nose= 0
                self.nose_pub.publish (self.nose)
                sleep(0.3)
                state = self.learning.get_state()

#              self.learning.wait_for_result()
              state = self.learning.get_state()
              if str(self.goal_states[state]) == "SUCCEEDED":
                self.noseGreen()
                self.speak_this(choice(["Bene, beh, ecco, Ho finito, "+SAY_YOUR_NAME_AND_VALIDATE.heardname+".",SAY_YOUR_NAME_AND_VALIDATE.heardname+", fatto, è stato facile, infatti.","Apprendimento terminato, mi puoi chiedere di riconoscerti in futuro, "+SAY_YOUR_NAME_AND_VALIDATE.heardname]))
                self.speak_this(choice(["Vuoi che ti memorizzi di nuovo, "+SAY_YOUR_NAME_AND_VALIDATE.heardname+"?","Vuoi che ricomincio ?"]))
                SAY_YOUR_NAME_AND_VALIDATE.first_try = ""
                self.first_try = True
                return 'next_state'



      if self.state == "myName":
        if self.heard.find("no") > -1: # wrong recognized name, next try
            self.noseYellow()
            self.non()
            self.heard =""

# PROCESS listening
class CONTINUE_OR_NOT(smach.State):
  def __init__(self):
      smach.State.__init__(self, outcomes=['next_state','return','again'])
      self.client_speak = rospy.ServiceProxy("/say_fr1", Text2Speach)
      rospy.Subscriber('listened', String, self.listen_callback)
      self.nose_pub = rospy.Publisher('/cmd_nose', UInt8 , queue_size=1)
      self.nose = 0
      self.learning = actionlib.SimpleActionClient("qbo_face_recognition", FaceRecognitionAction)
      self.goal_states = ['RECALLING', 'REJECTED', 'ABORTED', 'SUCCEEDED']
      self.heard = ""
      self.learningFinished = False
      self.quit = False
      self.improve = True

# robot voice
  def speak_this(self,text):
    self.client_speak(str(text))

 
# robot listening
  def listen_callback(self, msg):
        self.heard = msg.data


# NOSE fonctions, for some visual help
  def noseLearn(self):
     while not rospy.is_shutdown():
        self.nose= 1
        self.nose_pub.publish (self.nose)
        sleep(1)
        self.nose= 0
        self.nose_pub.publish (self.nose)
        sleep(0.3)
  def noseRed(self):
        self.nose= 1
        self.nose_pub.publish (self.nose)
  def noseGreen(self):
        self.nose= 2
        self.nose_pub.publish (self.nose)
  def noseYellow(self):
        self.nose= 3
        self.nose_pub.publish (self.nose)
  def noseOff(self):
        self.nose= 0
        self.nose_pub.publish (self.nose)


  def execute(self, userdata):

    rospy.loginfo('Executing state CONTINUE_OR_NOT')
    self.heard =""
    self.learningFinished = False
    self.learning = actionlib.SimpleActionClient("qbo_face_recognition", FaceRecognitionAction)
    self.goal_states = ['RECALLING', 'REJECTED', 'ABORTED', 'SUCCEEDED']
    while not rospy.is_shutdown():

        if self.heard.find("si") > -1 and self.improve: # more learning
            self.speak_this(choice(["Attenzione, "+SAY_YOUR_NAME_AND_VALIDATE.heardname+", si va.","Bene, "+SAY_YOUR_NAME_AND_VALIDATE.heardname+", partito"])) 
            self.heard = ""
            SAY_YOUR_NAME_AND_VALIDATE().noseRed()
            while not rospy.is_shutdown():
              if self.learningFinished == False: 
                self.learningGoal = FaceRecognitionGoal()
                self.learningGoal.order_id = 2
                self.learningGoal.order_argument= SAY_YOUR_NAME_AND_VALIDATE.heardname
                self.learning.send_goal(self.learningGoal)
                state = self.learning.get_state()

                while not str(self.goal_states[state]) == "SUCCEEDED":
                  self.nose= 1
                  self.nose_pub.publish (self.nose)
                  sleep(1)
                  self.nose= 0
                  self.nose_pub.publish (self.nose)
                  sleep(0.3)
                  state = self.learning.get_state()

                self.learning.wait_for_result()
                state = self.learning.get_state()
                self.learningFinished = True 
                SAY_YOUR_NAME_AND_VALIDATE().noseGreen()
                self.speak_this(choice(["Bene, beh, ecco, Ho finito, "+SAY_YOUR_NAME_AND_VALIDATE.heardname+".",SAY_YOUR_NAME_AND_VALIDATE.heardname+", fatto, è stato facile, infatti.","Apprendimento terminato, mi puoi chiedere di riconoscerti in futuro, "+SAY_YOUR_NAME_AND_VALIDATE.heardname]))
                rospy.sleep(0.5)
                self.speak_this(choice(["Vuoi ricominciare, "+SAY_YOUR_NAME_AND_VALIDATE.heardname+", Rispondi, sì o no"]))
                self.heard =""
                return 'again'
            else:
                pass


        if self.heard.find("si") > -1 and not self.improve: # more learning 
          self.speak_this(choice(["Bene","Daccordo","ok"])) 
          SAY_YOUR_NAME_AND_VALIDATE.old_voice = False
          self.heard =""
          self.learningFinished = False
          return 'return'                


        if self.heard.find("no") > -1 and not self.quit: # start a new name reco ?
          SAY_YOUR_NAME_AND_VALIDATE().noseOff()
          self.speak_this(choice(["Ah, daccordo "+SAY_YOUR_NAME_AND_VALIDATE.heardname+". E tu vuoi che io ricominci con un nuovo nome ?","Ok "+SAY_YOUR_NAME_AND_VALIDATE.heardname+". Vuoi che ricominci con un nuovo nome ?"]))
          self.quit = True
          self.improve = False
          self.heard =""
          return 'again'

        if self.heard.find("non") > -1 and self.quit: # quit face learning.
          SAY_YOUR_NAME_AND_VALIDATE().noseRed()
          print "************* suite  *********************"
          os.remove("/opt/ros/indigo/catkin_ws/src/qbo_face_recognition/facedata.xml")
          self.learningGoal = FaceRecognitionGoal()
          self.learningGoal.order_id = 3
          self.learningGoal.order_argument="none"
          self.speak_this("OK, sto aggiornando la base di dati,")
          self.learning.send_goal(self.learningGoal) 
          fin_de_voix_synthetique = ("fin_voix_synthetique_1","fin_voix_synthetique_2","fin_voix_synthetique_3")
          os.system("play /opt/ros/indigo/catkin_ws/src/qbo_face_operations/wav/"+(random.choice(fin_de_voix_synthetique))+".wav")
          run_process("rosnode kill /qbo_listen")
          run_process("rosnode kill /neo_face_learning")
          self.heard =""
          return 'next_state'


# main function
def main():
    rospy.init_node('neo_face_learning')
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['EXIT'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('SAY_YOUR_NAME_AND_VALIDATE', SAY_YOUR_NAME_AND_VALIDATE(), 
                               transitions={'next_state':'CONTINUE_OR_NOT'})

        smach.StateMachine.add('CONTINUE_OR_NOT', CONTINUE_OR_NOT(), 
                               transitions={'next_state':'EXIT','return':'SAY_YOUR_NAME_AND_VALIDATE','again':'CONTINUE_OR_NOT'})


    # Create and start the introspection server (to see on smach_viewer)
    sis = smach_ros.IntrospectionServer('server_name', sm, 'FACE_LEARNING')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()
