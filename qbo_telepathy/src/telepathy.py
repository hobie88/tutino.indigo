#!/usr/bin/env python

import time
import signal
# ROS import
import ros
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt8
# import for url managing
import urllib2
import json

# param definition
providerPath = "http://www.nightfox32000.altervista.org/QboTelepathy/"
fileName = "cmds.txt"
userAgent = 'Mozilla/5.0 (Windows; U; Windows NT 5.1; en-US; rv:1.9.0.7) Gecko/2009021910 Firefox/3.0.7'
headerReq = {'User-Agent': userAgent}

TEST = False # if True read a local file
ROS = True # if True publish on ros, else print results on the std output

class Telepathy(object):    #Required for classes employing decorators
    def __init__(self):    #Handle kwargs and set Publishers/DataType,queue_size
        '''
        After creating ROS node, this class must be initialized.
        Its properties are defined as follows: getters tries to get the value from the Http-retrieved JSON.
                                                setters copy the value directly to the msg(stored inside self.msgs)
        '''
        
        self.timeThreshold = time.time()
        self.request = urllib2.Request(providerPath + fileName, headers=headerReq)
        self.pubs = {"body":rospy.Publisher("/cmd_vel",Twist,queue_size=2),
                    "head":rospy.Publisher("/cmd_joints",JointState,queue_size=2),
                    "nose":rospy.Publisher("/cmd_nose",UInt8,queue_size=2)
                    }
        self.resetMsgs()    #Initalize self.msgs and set all the values to 0
        
    def resetMsgs(self):
        bodyMsg = Twist()
        bodyMsg.linear.x = float(0)
        bodyMsg.angular.z = float(0)
        headMsg = JointState()
        headMsg.name = ["head_pan_joint", "head_tilt_joint"]
        headMsg.position = [float(0), float(0)]
        noseMsg = UInt8()
        noseMsg.data = int(0)
        self.msgs = {"body": bodyMsg,
                     "head": headMsg,
                     "nose": noseMsg
                     }
        
        
    #
    # PROPERTIES
    #
        
    @property
    def x(self):
        try:
            value = self.getValues("linear")
        except:
            value = 0
        return value
    
    @x.setter
    def x(self,value):
        '''
        Sets linear x to value
        '''
        self.msgs["body"].linear.x = float(value)
   
    @property
    def z(self):
        try:
            value = self.getValues("angular")
        except:
            value = 0
        return value
    
    @z.setter
    def z(self,value):
        '''
        Sets angular z to value
        '''
        self.msgs["body"].angular.z = float(value)
        
    @property
    def headJoints(self):
        try:
            values = [float(self.getValues("pan")), float(self.getValues("tilt")) ]
        except:
            values = [0, 0]
        return values
    
    @headJoints.setter
    def headJoints(self,values):
        '''
        Input:
            list values = [pan, tilt]
        '''
        if (type(values) == type([])):
            self.msgs["head"].position = values
        else:
            print("List of PAN,TILT is accepted!!!")
            self.msgs["head"].position = [float(0), float(0)]
            
    @property
    def noseColor(self):
        '''
        '''
        try:
            value = self.getValues("nose")
        except:
            value = 0
        return value
    
    @noseColor.setter
    def noseColor(self,value):
        '''
        Sets nose color
        '''
        self.msgs["nose"].data = int(value)
        
    @property
    def lastTime(self):
        '''
        Set last valid timestamp msg decoded
        '''
        return self.timeThreshold
        
    @lastTime.setter
    def lastTime(self,value):
        '''
        '''
        try:
            self.timeThreshold = self.getValues("timestamp")
        except:
            pass
                     
    def retrieveData(self):
        '''
        Open the self.request and decode JSON-coded data
        '''
        self.data = urllib2.urlopen(self.request)
        self.data = self.data.read()
        try:
            self.data = json.loads(self.data)
        except ValueError:
            print("JSON Decoding failed")
           # pass
        #DEBUG
        #for key in self.data:
        #    print("%s -> %s" %(key,self.data[key]))
        
    
    def getValues(self,key):
        '''
        Write values to msgs
        '''
        try:
            value = self.data[key]
        except:
            value = 0
        return value
    
    def publishMsgs(self):
        '''
        Publish all the msg inside self.msgs using Publishers inside self.pubs
        '''
        #print("Publishing") #DEBUG
        for key in self.pubs:
            if "head" in key:
                self.pubs[key].header = rospy.get_rostime()
            try:
                self.pubs[key].publish(self.msgs[key])
            except:
                print("Error occurred when trying to publish %s" %key)
        
        

def close():
    sys.exit(0)


if __name__ == '__main__':
    signal.signal(signal.SIGINT, close) #to close properly the node
    rospy.init_node("telepathy", anonymous=True)
    #Initialize class
    qboSkill = Telepathy()
    #Handle class
    while True:
        
        try:
            qboSkill.retrieveData()
        except ValueError: #In case it goes wrong, restarts execution until it goes right
            print("RetrieveData failed! Trying again...")
            continue
        except KeyboardInterrupt:
            sys.exit(0)
            
        #Get data from JSON-decoded attribute
        qboSkill.x = qboSkill.x
        qboSkill.z = qboSkill.z
        qboSkill.headJoints = qboSkill.headJoints
        qboSkill.noseColor = qboSkill.noseColor
        qboSkill.lastTime = qboSkill.lastTime
        #Publish
        qboSkill.publishMsgs()


